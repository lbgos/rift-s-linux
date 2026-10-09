// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
// Hardware-free lifecycle check of the actual VIT backend through Monado.
#include "tracking/t_tracking.h"
#include "xrt/xrt_tracking.h"
#include "xrt/xrt_frame.h"
#include "os/os_time.h"
#include "util/u_frame.h"
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <atomic>
#include <thread>
#ifdef SLAM_TEST_HAVE_TBB
#include <tbb/global_control.h>
#endif
int
main()
{
	if (!std::getenv("VIT_SYSTEM_LIBRARY_PATH")) {
		puts("Set VIT_SYSTEM_LIBRARY_PATH to run the backend lifecycle check");
		return 77;
	}
	t_slam_calibration calibration{};
	calibration.cam_count = 2;
	// Use the default Basalt pipeline IMU rate for this lifecycle-only check.
	calibration.imu.frequency = 200;
	for (auto *c : {&calibration.imu.base.accel, &calibration.imu.base.gyro}) {
		for (int i = 0; i < 3; ++i) {
			c->transform[i][i] = 1;
			c->bias_std[i] = 0.001;
			c->noise_std[i] = 0.01;
		}
	}
	for (int i = 0; i < 2; ++i) {
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
		c.T_imu_cam.v[12] = i ? 0.05 : -0.05;
	}
	// Exercise the production default. The backend must advertise safe recreation.
	t_slam_tracker_config config{};
	t_slam_fill_default_config(&config);
	config.vit_system_library_path = std::getenv("VIT_SYSTEM_LIBRARY_PATH");
	config.cam_count = 2;
	config.submit_from_start = true;
	config.slam_calib = &calibration;
	config.slam_ui = false;
	config.write_csvs = false;
	config.features_stat = config.timing_stat = false;
#ifdef SLAM_TEST_HAVE_TBB
	const auto previous_worker_limit = tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism);
	auto worker_limit_ok = [&] {
		const char *expected = std::getenv("SLAM_TEST_WORKER_LIMIT");
		if (!expected)
			return true;
		const auto actual = tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism);
		if (actual == std::strtoul(expected, nullptr, 10))
			return true;
		std::fprintf(stderr, "Expected Basalt TBB worker limit %s, got %zu\n", expected, actual);
		return false;
	};
#else
	if (std::getenv("SLAM_TEST_WORKER_LIMIT")) {
		std::fputs("TBB is required to check the backend worker limit\n", stderr);
		return 1;
	}
#endif
	xrt_frame_context context{};
	xrt_tracked_slam *tracker = nullptr;
	xrt_slam_sinks *sinks = nullptr;
	if (t_slam_create(&context, &config, &tracker, &sinks) != 0 || t_slam_start(tracker) != 0)
		return 1;
#ifdef SLAM_TEST_HAVE_TBB
	if (!worker_limit_ok()) {
		xrt_frame_context_destroy_nodes(&context);
		return 7;
	}
#endif
	std::atomic<bool> reading{true};
	std::thread reader([&] {
		while (reading) {
			int count;
			timepoint_ns timestamp;
			t_slam_get_feature_count(tracker, &count, &timestamp);
			xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
			tracker->get_tracked_pose(tracker, os_monotonic_get_ns(), &relation);
			os_nanosleep(1000000);
		}
	});
	// Join the reader on early return as well as successful teardown.
	struct ReaderGuard
	{
		std::atomic<bool> &reading;
		std::thread &reader;
		~ReaderGuard()
		{
			reading = false;
			if (reader.joinable())
				reader.join();
		}
	} reader_guard{reading, reader};
	for (int i = 0; i < 3; ++i) {
		xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
		tracker->get_tracked_pose(tracker, os_monotonic_get_ns(), &relation);
		if (relation.relation_flags != 0 || tracker->restart(tracker) != 0)
			return 2;
	}
	auto push_stereo = [&](timepoint_ns timestamp) {
			for (int eye = 0; eye < 2; ++eye) {
				xrt_frame *frame = nullptr;
				u_frame_create_one_off(XRT_FORMAT_L8, 640, 480, &frame);
				frame->timestamp = timestamp;
				for (unsigned y = 0; y < frame->height; ++y)
					for (unsigned x = 0; x < frame->width; ++x)
						frame->data[y * frame->stride + x] =
						    ((x + eye * 5) / 16 + y / 16) % 2 ? 220 : 30;
				xrt_sink_push_frame(sinks->cams[eye], frame);
				xrt_frame_reference(&frame, nullptr);
			}
	};

	// Cameras without IMUs can fill flow/VIO queues; submissions and restart must stay live.
	const timepoint_ns camera_begin = os_monotonic_get_ns();
	for (int i = 0; i < 32; ++i)
		push_stereo(camera_begin + i * 33333333);
	if (tracker->restart(tracker) != 0)
		return 5;

	// An in-flight frame waiting for another IMU must survive the shutdown sentinel.
	xrt_imu_sample sparse_imu{};
	sparse_imu.timestamp_ns = os_monotonic_get_ns();
	sparse_imu.accel_m_s2.z = 9.81;
	xrt_sink_push_imu(sinks->imu, &sparse_imu);
	push_stereo(sparse_imu.timestamp_ns + 20000000);
	os_nanosleep(200000000);
	if (tracker->restart(tracker) != 0)
		return 6;

	// More than the backend's 300-sample capacity, with no images.
	const timepoint_ns imu_begin = os_monotonic_get_ns() - 3000000000;
	for (int i = 0; i < 4000; ++i) {
		xrt_imu_sample imu{};
		imu.timestamp_ns = imu_begin + i * 5000000LL;
		imu.accel_m_s2.z = 9.81;
		xrt_sink_push_imu(sinks->imu, &imu);
	}
	std::fputs("No-image IMU submissions finished; restarting\n", stderr);
	if (tracker->restart(tracker) != 0)
		return 4;

	// Exercise teardown with real frontend work and a camera backlog, not just sentinels.
	for (int delay_ms : {0, 10, 100}) {
		const timepoint_ns begin = os_monotonic_get_ns() - 1000000000;
		for (int i = 0; i < 160; ++i) {
			xrt_imu_sample imu{};
			imu.timestamp_ns = begin + i * 5000000LL;
			imu.accel_m_s2.z = 9.81;
			xrt_sink_push_imu(sinks->imu, &imu);
		}
		for (int i = 0; i < 14; ++i) {
			push_stereo(begin + 20000000 + i * 33333333);
		}
		os_nanosleep(delay_ms * 1000000);
		if (tracker->restart(tracker) != 0)
			return 3;
	}
#ifdef SLAM_TEST_HAVE_TBB
	if (!worker_limit_ok())
		return 7;
#endif
	reading = false;
	reader.join();
	xrt_frame_context_destroy_nodes(&context);
#ifdef SLAM_TEST_HAVE_TBB
	if (tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism) != previous_worker_limit)
		return 8;
#endif
	puts("Actual Basalt lifecycle: empty, saturated no-image IMU and queued stereo/IMU restarts passed");
}
