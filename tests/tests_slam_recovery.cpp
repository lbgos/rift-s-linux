// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
// Actual-backend motion, queue-stall recovery and latency check. No hardware required.
#include "tracking/t_tracking.h"
#include "xrt/xrt_tracking.h"
#include "xrt/xrt_frame.h"
#include "os/os_time.h"
#include "util/u_frame.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

struct Motion
{
	double x = 0, yaw = 0, ax = 0, wy = 0;
};

static Motion
motion(double t)
{
	if (t <= 1)
		return {};
	t -= 1;
	static const double yaw_amplitude = std::getenv("SLAM_TEST_FAST_MOTION") ? 0.65 : 0.25;
	return {0.10 * (1 - std::cos(3 * t)), yaw_amplitude * (1 - std::cos(5 * t)), 0.9 * std::cos(3 * t),
	        5 * yaw_amplitude * std::sin(5 * t)};
}

// Textured points at distinct depths. KB4 projection matches the supplied cameras.
static void
render(xrt_frame *frame, double baseline, double t)
{
	std::memset(frame->data, 20, frame->size);
	const auto m = motion(t);
	const double c = std::cos(m.yaw), s = std::sin(m.yaw);
	uint32_t rng = 42;
	auto random = [&] {
		rng = rng * 1664525u + 1013904223u;
		return double(rng >> 8) / 16777216;
	};
	for (int i = 0; i < 1200; ++i) {
		double x = (random() - .5) * 7 - m.x;
		double y = (random() - .5) * 5;
		double z = 1.5 + random() * 3;
		double cx = c * x - s * z - baseline;
		double cz = s * x + c * z;
		std::array<double, 49> texture;
		for (auto &v : texture)
			v = random() > .5 ? 220 : 50;
		if (cz < .5)
			continue;
		double r = std::hypot(cx, y), theta = std::atan2(r, cz);
		double u = 320 + 200 * theta * cx / r, v = 240 + 200 * theta * y / r;
		int ix = int(std::floor(u)), iy = int(std::floor(v));
		if (ix < 5 || ix >= 635 || iy < 5 || iy >= 475)
			continue;
		// Bilinear translation keeps feature motion continuous across pixel boundaries.
		for (int py = -3; py <= 3; ++py)
			for (int px = -3; px <= 3; ++px) {
				double a = u - ix, b = v - iy;
				auto tex = [&](int dx, int dy) {
					return texture[std::clamp(dy, 0, 6) * 7 + std::clamp(dx, 0, 6)];
				};
				double value = (1 - a) * (1 - b) * tex(px + 3, py + 3) +
				               a * (1 - b) * tex(px + 2, py + 3) + (1 - a) * b * tex(px + 3, py + 2) +
				               a * b * tex(px + 2, py + 2);
				frame->data[(iy + py) * frame->stride + ix + px] = uint8_t(value);
			}
	}
}

int
main()
{
	const char *library = std::getenv("VIT_SYSTEM_LIBRARY_PATH");
	if (!library)
		return 77;
	t_slam_calibration calibration{};
	const char *camera_count = std::getenv("SLAM_TEST_CAM_COUNT");
	const int cameras = camera_count && std::strcmp(camera_count, "5") == 0 ? 5 : 2;
	calibration.cam_count = cameras;
	calibration.imu.frequency = 200;
	for (auto *c : {&calibration.imu.base.accel, &calibration.imu.base.gyro})
		for (int i = 0; i < 3; ++i) {
			c->transform[i][i] = 1;
			c->bias_std[i] = .001;
			c->noise_std[i] = .01;
		}
	for (int i = 0; i < cameras; ++i) {
		auto &c = calibration.cams[i];
		c.frequency = 30;
		c.base.image_size_pixels = {640, 480};
		c.base.distortion_model = T_DISTORTION_FISHEYE_KB4;
		c.base.intrinsics[0][0] = c.base.intrinsics[1][1] = 200;
		c.base.intrinsics[0][2] = 320;
		c.base.intrinsics[1][2] = 240;
		c.base.intrinsics[2][2] = 1;
		for (int k = 0; k < 4; ++k)
			c.T_imu_cam.v[k * 5] = 1;
		c.T_imu_cam.v[12] = cameras == 2 ? (i ? .05 : -.05) : (i - 2) * .05;
	}
	t_slam_tracker_config config{};
	t_slam_fill_default_config(&config);
	config.vit_system_library_path = library;
	config.cam_count = cameras;
	config.submit_from_start = true;
	config.slam_calib = &calibration;
	config.slam_ui = false;
	const char *directory = std::getenv("SLAM_TEST_BENCH_DIR");
	config.write_csvs = directory != nullptr;
	if (directory)
		config.csv_path = directory;
	config.timing_stat = true;
	config.features_stat = true;
	xrt_frame_context context{};
	xrt_tracked_slam *tracker = nullptr;
	xrt_slam_sinks *sinks = nullptr;
	if (t_slam_create(&context, &config, &tracker, &sinks) || t_slam_start(tracker))
		return 1;
	struct Cleanup
	{
		xrt_frame_context &context;
		~Cleanup()
		{
			xrt_frame_context_destroy_nodes(&context);
		}
	} cleanup{context};
	std::vector<xrt_frame *> frames(cameras);
	for (auto &frame : frames)
		u_frame_create_one_off(XRT_FORMAT_L8, 640, 480, &frame);
	struct FrameCleanup
	{
		std::vector<xrt_frame *> &frames;
		~FrameCleanup()
		{
			for (auto &f : frames)
				xrt_frame_reference(&f, nullptr);
		}
	} frame_cleanup{frames};
	auto stereo = [&](int64_t ts, double t) {
		for (int eye = 0; eye < cameras; ++eye) {
			render(frames[eye], calibration.cams[eye].T_imu_cam.v[12], t);
			frames[eye]->timestamp = ts;
			xrt_sink_push_frame(sinks->cams[eye], frames[eye]);
		}
	};
	int64_t last_sample = 0;
	std::vector<double> lag;
	auto stream = [&](double seconds) {
		const int64_t begin = os_monotonic_get_ns();
		int64_t imu_ts = begin;
		int count = 0;
		for (int i = 0; i < int(seconds * 30); ++i) {
			const int64_t ts = begin + i * 33333333LL;
			const int64_t delay = ts - int64_t(os_monotonic_get_ns());
			if (delay > 0)
				os_nanosleep(delay);
			while (imu_ts <= ts + 10000000) {
				auto m = motion(double(imu_ts - begin) / 1e9);
				xrt_imu_sample imu{};
				imu.timestamp_ns = imu_ts;
				imu.accel_m_s2 = {std::cos(m.yaw) * m.ax, 9.81, std::sin(m.yaw) * m.ax};
				imu.gyro_rad_secs.y = m.wy;
				xrt_sink_push_imu(sinks->imu, &imu);
				imu_ts += 5000000;
			}
			stereo(ts, double(i) / 30);
			int64_t sample = 0;
			xrt_space_relation relation{}, query{};
			if (t_slam_get_latest_sample(tracker, os_monotonic_get_ns(), &sample, &relation, &query) &&
			    sample != last_sample && std::isfinite(relation.pose.position.x) &&
			    std::isfinite(relation.pose.position.y) && std::isfinite(relation.pose.position.z)) {
				last_sample = sample;
				if (sample >= begin) {
					++count;
					lag.push_back(double(int64_t(os_monotonic_get_ns()) - sample) / 1e6);
				}
			}
		}
		return count;
	};
	if (stream(6) < 90) {
		std::fputs("Motion stream did not stay live\n", stderr);
		return 2;
	}
	std::sort(lag.begin(), lag.end());
	std::printf("MOTION samples=%zu lag_median_ms=%.3f lag_p95_ms=%.3f lag_max_ms=%.3f\n", lag.size(),
	            lag[lag.size() / 2], lag[lag.size() * 95 / 100], lag.back());
	if (directory)
		return 0; // Benchmark mode measures the intact pipeline only.

	// A frame waiting for an unreachable future IMU wedges the frontend. Estimator-only
	// reset cannot clear it; this models the persistent stale pipeline after a sensor gap.
	stereo(os_monotonic_get_ns() + 3600000000000LL, 0);
	os_nanosleep(100000000);
	if (t_slam_reset(tracker))
		return 3;
	last_sample = 0;
	const int stuck = stream(3);
	std::printf("STUCK estimator_reset_fresh_samples=%d\n", stuck);
	if (stuck > 1)
		return 4;
	const int64_t restart_begin = os_monotonic_get_ns();
	// Deliberately use the default runtime policy, without SLAM_BACKEND_RECREATE=true.
	if (tracker->restart(tracker)) {
		std::fputs("Default policy leaves backend stuck\n", stderr);
		return 5;
	}
	const double restart_ms = double(os_monotonic_get_ns() - restart_begin) / 1e6;
	last_sample = 0;
	lag.clear();
	const int recovered = stream(3);
	std::printf("RECOVERY restart_ms=%.3f fresh_samples=%d first_pose_lag_ms=%.3f\n", restart_ms, recovered,
	            lag.empty() ? -1 : lag.front());
	if (restart_ms > 2000 || recovered < 45)
		return 6;
	return 0;
}
