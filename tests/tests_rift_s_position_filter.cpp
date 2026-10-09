// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
extern "C" {
#include "rift_s/rift_s_position_filter.h"
}
#include <cmath>
#include <memory>
#include <random>

static constexpr timepoint_ns MS = U_TIME_1MS_IN_NS;
static constexpr timepoint_ns IMU_STEP = 2 * MS; // 500 Hz
// Optical fixes every 16 IMU samples: 32 ms, about 30 Hz.
static const xrt_vec3 OPTICAL_VARIANCE = {3e-3f * 3e-3f, 3e-3f * 3e-3f, 3e-3f * 3e-3f};

static std::unique_ptr<rift_s_position_filter>
make_filter()
{
	auto f = std::make_unique<rift_s_position_filter>();
	rift_s_position_filter_reset(f.get());
	return f;
}

static xrt_vec3
position_at(const rift_s_position_filter &f, timepoint_ns t)
{
	xrt_vec3 p, v;
	rift_s_position_filter_predict(&f, t, 100 * MS, &p, &v);
	return p;
}

TEST_CASE("Rift S loss ladder matches the Windows defaults")
{
	time_duration_ns p, v;
	rift_s_tracking_loss_durations(0, 0, &p, &v);
	CHECK(p == 0);
	rift_s_tracking_loss_durations(1, 0, &p, &v);
	CHECK(p == 250 * MS);
	CHECK(v == 250 * MS);
	rift_s_tracking_loss_durations(3, 0, &p, &v);
	CHECK(p == 500 * MS);
	CHECK(v == 500 * MS);
	rift_s_tracking_loss_durations(11, 0, &p, &v);
	CHECK(p == 2000 * MS);
	CHECK(v == 4000 * MS);
	rift_s_tracking_loss_durations(30, 21, &p, &v);
	CHECK(p == 3000 * MS);
	CHECK(v == 5000 * MS);
}

TEST_CASE("Rift S position filter smooths optical noise at rest and learns the accelerometer bias")
{
	auto f = make_filter();
	std::mt19937 rng(1);
	std::normal_distribution<float> noise(0, 3e-3f);
	const xrt_vec3 truth = {0.1f, 1.2f, -0.3f};
	const xrt_vec3 biased_accel = {0.2f, -0.3f, 0.1f}; // tilt/bias error seen as acceleration
	double sq = 0, raw_sq = 0;
	int n = 0;
	for (timepoint_ns t = MS; t < 6000 * MS; t += IMU_STEP) {
		rift_s_position_filter_imu(f.get(), t, &biased_accel);
		if ((t / IMU_STEP) % 16 == 0) {
			xrt_vec3 z = {truth.x + noise(rng), truth.y + noise(rng), truth.z + noise(rng)};
			rift_s_position_filter_fix(f.get(), t - 20 * MS, &z, &OPTICAL_VARIANCE, 0.15f);
			if (t > 3000 * MS) {
				xrt_vec3 p = position_at(*f, t);
				sq += pow(p.x - truth.x, 2) + pow(p.y - truth.y, 2) + pow(p.z - truth.z, 2);
				raw_sq += pow(z.x - truth.x, 2) + pow(z.y - truth.y, 2) + pow(z.z - truth.z, 2);
				n++;
			}
		}
	}
	double rms = sqrt(sq / n), raw_rms = sqrt(raw_sq / n);
	INFO("rms=" << rms << " raw_rms=" << raw_rms);
	CHECK(rms < 0.75 * raw_rms);
	CHECK(rms < 4.5e-3);
	CHECK(f->state.x[0][2] == Catch::Approx(biased_accel.x).margin(0.05));
	CHECK(f->state.x[1][2] == Catch::Approx(biased_accel.y).margin(0.05));
	CHECK(f->resets == 1);
	CHECK(f->gated == 0);
}

TEST_CASE("Rift S position filter follows motion between delayed optical fixes")
{
	auto f = make_filter();
	const double w = 2 * M_PI * 1.5, amplitude = 0.2; // a brisk 1.5 Hz hand swing
	auto truth = [&](timepoint_ns t) { return amplitude * sin(w * (double)t / U_TIME_1S_IN_NS); };
	auto accel = [&](timepoint_ns t) { return -amplitude * w * w * sin(w * (double)t / U_TIME_1S_IN_NS); };
	const timepoint_ns latency = 40 * MS;
	double filter_sq = 0, held_sq = 0;
	int n = 0;
	double held = 0;
	for (timepoint_ns t = MS; t < 4000 * MS; t += IMU_STEP) {
		xrt_vec3 a = {(float)accel(t), 0, 0};
		rift_s_position_filter_imu(f.get(), t, &a);
		// A fix captured at t - latency is delivered now.
		if ((t / IMU_STEP) % 16 == 0 && t > latency) {
			xrt_vec3 z = {(float)truth(t - latency), 0, 0};
			rift_s_position_filter_fix(f.get(), t - latency, &z, &OPTICAL_VARIANCE, 0.15f);
			held = z.x;
		}
		if (t > 1000 * MS) {
			// The compositor asks for 20 ms in the future.
			timepoint_ns query = t + 20 * MS;
			filter_sq += pow(position_at(*f, query).x - truth(query), 2);
			held_sq += pow(held - truth(query), 2);
			n++;
		}
	}
	double filter_rms = sqrt(filter_sq / n), held_rms = sqrt(held_sq / n);
	INFO("filter_rms=" << filter_rms << " held_rms=" << held_rms);
	CHECK(filter_rms < 0.005);
	CHECK(filter_rms < 0.05 * held_rms);
	CHECK(f->gated == 0);
}

TEST_CASE("Rift S position filter coasts with bounded drift, holds when frozen and resets on reacquire")
{
	auto f = make_filter();
	const xrt_vec3 zero = {0, 0, 0};
	const xrt_vec3 here = {0.3f, 1.0f, -0.4f};
	timepoint_ns t = MS;
	for (; t < 1000 * MS; t += IMU_STEP) {
		rift_s_position_filter_imu(f.get(), t, &zero);
		if ((t / IMU_STEP) % 16 == 0) {
			rift_s_position_filter_fix(f.get(), t, &here, &OPTICAL_VARIANCE, 0.15f);
		}
	}
	// Optical loss with a 0.5 m/s^2 uncorrected acceleration error: coast for the 0.5 s ladder step.
	f->coasting = true;
	const xrt_vec3 error = {0.5f, 0, 0};
	for (timepoint_ns end = t + 500 * MS; t < end; t += IMU_STEP) {
		rift_s_position_filter_imu(f.get(), t, &error);
	}
	xrt_vec3 coasted = {(float)f->state.x[0][0], (float)f->state.x[1][0], (float)f->state.x[2][0]};
	// The 500 ms bridge with decaying velocity: a(tau t - tau^2 (1 - e^-t/tau)) = 4.6 cm.
	CHECK(fabsf(coasted.x - here.x) < 0.05f);
	CHECK(fabsf(coasted.x - here.x) > 0.0f);

	rift_s_position_filter_freeze(f.get());
	for (timepoint_ns end = t + 2000 * MS; t < end; t += IMU_STEP) {
		rift_s_position_filter_imu(f.get(), t, &error);
	}
	xrt_vec3 held = position_at(*f, t + 50 * MS);
	CHECK(held.x == Catch::Approx(coasted.x).margin(1e-6));
	xrt_vec3 p, v;
	rift_s_position_filter_predict(f.get(), t, 100 * MS, &p, &v);
	CHECK(v.x == 0.0f);

	// Reacquire somewhere else: three consistent fixes re-initialize.
	const xrt_vec3 there = {0.6f, 1.1f, -0.2f};
	CHECK(rift_s_position_filter_fix(f.get(), t - 96 * MS, &there, &OPTICAL_VARIANCE, 0.15f) ==
	      RIFT_S_POSITION_FIX_PENDING);
	CHECK(rift_s_position_filter_fix(f.get(), t - 63 * MS, &there, &OPTICAL_VARIANCE, 0.15f) ==
	      RIFT_S_POSITION_FIX_PENDING);
	CHECK(f->frozen);
	CHECK(rift_s_position_filter_fix(f.get(), t - 30 * MS, &there, &OPTICAL_VARIANCE, 0.15f) ==
	      RIFT_S_POSITION_FIX_RESET);
	CHECK_FALSE(f->frozen);
	CHECK(position_at(*f, t).x == Catch::Approx(there.x).margin(0.01));
}

TEST_CASE("Rift S position filter resets on a confirmed large innovation and rejects fixes older than its history")
{
	auto f = make_filter();
	const xrt_vec3 zero = {0, 0, 0};
	const xrt_vec3 here = {0, 1, 0};
	timepoint_ns t = MS;
	for (; t < 500 * MS; t += IMU_STEP) {
		rift_s_position_filter_imu(f.get(), t, &zero);
		if ((t / IMU_STEP) % 16 == 0) {
			rift_s_position_filter_fix(f.get(), t, &here, &OPTICAL_VARIANCE, 0.15f);
		}
	}
	const xrt_vec3 far = {0.5f, 1, 0};
	CHECK(rift_s_position_filter_fix(f.get(), t - 74 * MS, &far, &OPTICAL_VARIANCE, 0.15f) ==
	      RIFT_S_POSITION_FIX_PENDING);
	CHECK(position_at(*f, t).x == Catch::Approx(0.0f).margin(1e-3));
	CHECK(rift_s_position_filter_fix(f.get(), t - 42 * MS, &far, &OPTICAL_VARIANCE, 0.15f) ==
	      RIFT_S_POSITION_FIX_PENDING);
	CHECK(rift_s_position_filter_fix(f.get(), t - 10 * MS, &far, &OPTICAL_VARIANCE, 0.15f) ==
	      RIFT_S_POSITION_FIX_RESET);
	CHECK(f->gated == 3);
	CHECK(f->rejected == 2);
	CHECK(position_at(*f, t).x == Catch::Approx(0.5f).margin(1e-3));

	for (timepoint_ns end = t + 2000 * MS; t < end; t += IMU_STEP) {
		rift_s_position_filter_imu(f.get(), t, &zero);
	}
	CHECK(rift_s_position_filter_fix(f.get(), t - 1500 * MS, &far, &OPTICAL_VARIANCE, 0.15f) ==
	      RIFT_S_POSITION_FIX_STALE);
	CHECK(f->stale == 1);
}

TEST_CASE("Rift S position filter stays close to optical under a time-varying acceleration error")
{
	// Tilt error during rotation and accelerometer scale error show up as a slowly varying
	// world acceleration error; the filter must not lag or overshoot the optical fixes.
	auto f = make_filter();
	std::mt19937 rng(2);
	std::normal_distribution<float> optical_noise(0, 3e-3f), accel_noise(0, 0.05f);
	const double w = 2 * M_PI * 1.0, amplitude = 0.25;
	auto truth = [&](timepoint_ns t) { return amplitude * sin(w * (double)t / U_TIME_1S_IN_NS); };
	auto accel = [&](timepoint_ns t) { return -amplitude * w * w * sin(w * (double)t / U_TIME_1S_IN_NS); };
	auto error = [&](timepoint_ns t) { return 0.5 * sin(2 * M_PI * 0.7 * (double)t / U_TIME_1S_IN_NS + 1); };
	const timepoint_ns latency = 40 * MS;
	double sq = 0;
	int n = 0;
	for (timepoint_ns t = MS; t < 6000 * MS; t += IMU_STEP) {
		xrt_vec3 a = {(float)(accel(t) + error(t)) + accel_noise(rng), 0, 0};
		rift_s_position_filter_imu(f.get(), t, &a);
		if ((t / IMU_STEP) % 16 == 0 && t > latency) {
			xrt_vec3 z = {(float)truth(t - latency) + optical_noise(rng), 0, 0};
			rift_s_position_filter_fix(f.get(), t - latency, &z, &OPTICAL_VARIANCE, 0.15f);
		}
		if (t > 1000 * MS) {
			timepoint_ns query = t + 20 * MS;
			sq += pow(position_at(*f, query).x - truth(query), 2);
			n++;
		}
	}
	double rms = sqrt(sq / n);
	INFO("rms=" << rms);
	CHECK(rms < 0.006);
	CHECK(f->gated == 0);
}

// r13 worn log, left hand behind the back: after a long track was held, single wrong fits ~1 m
// away re-initialized the filter, and coasting on their velocity drifted it further.
TEST_CASE("Rift S held position ignores isolated wrong fits and needs a consistent reacquire")
{
	auto f = make_filter();
	const xrt_vec3 zero = {0, 0, 0};
	const xrt_vec3 here = {0.15f, 0.0f, 0.2f};
	timepoint_ns t = MS;
	for (; t < 2000 * MS; t += IMU_STEP) {
		rift_s_position_filter_imu(f.get(), t, &zero);
		if ((t / IMU_STEP) % 16 == 0) {
			rift_s_position_filter_fix(f.get(), t - 20 * MS, &here, &OPTICAL_VARIANCE, 0.15f);
		}
	}
	rift_s_position_filter_freeze(f.get());
	// Scattered single fits, each one frame apart, never two in agreement.
	const xrt_vec3 wrong[] = {{-0.99f, 0.29f, 0.40f},
	                          {-1.02f, -0.18f, 0.71f},
	                          {-0.63f, 0.17f, 0.07f},
	                          {-0.57f, 0.28f, 0.40f},
	                          {-1.06f, -0.24f, 0.81f}};
	for (const auto &w : wrong) {
		t += 33 * MS;
		rift_s_position_filter_imu(f.get(), t, &zero);
		CHECK(rift_s_position_filter_fix(f.get(), t - 20 * MS, &w, &OPTICAL_VARIANCE, 0.15f) ==
		      RIFT_S_POSITION_FIX_PENDING);
		xrt_vec3 p = position_at(*f, t);
		CHECK(p.x == Catch::Approx(here.x).margin(1e-3));
		CHECK(p.z == Catch::Approx(here.z).margin(1e-3));
	}
	CHECK(f->frozen);
	// The hand comes back: three consistent fixes take over.
	const xrt_vec3 back = {0.25f, 0.05f, 0.1f};
	enum rift_s_position_fix_result r = RIFT_S_POSITION_FIX_PENDING;
	for (int i = 0; i < RIFT_S_POSITION_FILTER_CONFIRM_FIXES; i++) {
		t += 33 * MS;
		rift_s_position_filter_imu(f.get(), t, &zero);
		xrt_vec3 z = {back.x + 0.01f * i, back.y, back.z};
		r = rift_s_position_filter_fix(f.get(), t - 20 * MS, &z, &OPTICAL_VARIANCE, 0.15f);
	}
	CHECK(r == RIFT_S_POSITION_FIX_RESET);
	CHECK(position_at(*f, t).x == Catch::Approx(back.x + 0.02f).margin(0.01));
}

TEST_CASE("Rift S young track holds instead of coasting on velocity from a few fixes")
{
	auto f = make_filter();
	const xrt_vec3 zero = {0, 0, 0};
	timepoint_ns t = MS;
	// Two fixes 33 ms apart and 3 cm apart (a wrong pair implies ~1 m/s), then nothing.
	const xrt_vec3 a = {0.07f, 0.17f, 0.59f}, b = {0.10f, 0.17f, 0.59f};
	rift_s_position_filter_fix(f.get(), t, &a, &OPTICAL_VARIANCE, 0.15f);
	for (timepoint_ns end = t + 33 * MS; t < end; t += IMU_STEP)
		rift_s_position_filter_imu(f.get(), t, &zero);
	rift_s_position_filter_fix(f.get(), t, &b, &OPTICAL_VARIANCE, 0.15f);
	CHECK(f->track_updates == 2);
	const double fused = f->state.x[0][0];
	f->coasting = true;
	for (timepoint_ns end = t + 250 * MS; t < end; t += IMU_STEP)
		rift_s_position_filter_imu(f.get(), t, &zero);
	CHECK(f->state.x[0][0] == Catch::Approx(fused).margin(1e-6));
	CHECK(f->state.x[0][1] == 0.0);
}

TEST_CASE("Optical loss bounds IMU displacement and freezes after the bridge")
{
	auto f = make_filter();
	xrt_vec3 here = {0.5f, 1.2f, -0.9f}, error = {-20, 0, 0};
	REQUIRE(rift_s_position_filter_fix(f.get(), MS, &here, &OPTICAL_VARIANCE, 0.15f) == RIFT_S_POSITION_FIX_RESET);
	f->track_updates = 30;
	for (timepoint_ns t = 3 * MS; t <= 1000 * MS; t += IMU_STEP) {
		f->coasting = t > 121 * MS;
		rift_s_position_filter_imu(f.get(), t, &error);
		if (t > 121 * MS)
			CHECK(fabs(position_at(*f, t).x - here.x) <= 0.101);
	}
	CHECK(f->frozen);
	xrt_vec3 held = position_at(*f, 1000 * MS);
	for (timepoint_ns t = 1001 * MS; t < 2000 * MS; t += IMU_STEP)
		rift_s_position_filter_imu(f.get(), t, &error);
	CHECK(position_at(*f, 2000 * MS).x == held.x);
}

TEST_CASE("Optical-loss queries bound prediction during a radio gap")
{
	auto f = make_filter();
	xrt_vec3 here = {0.5f, 1.2f, -0.9f}, zero = {0, 0, 0};
	REQUIRE(rift_s_position_filter_fix(f.get(), MS, &here, &OPTICAL_VARIANCE, 0.15f) == RIFT_S_POSITION_FIX_RESET);
	f->track_updates = 30;
	rift_s_position_filter_imu(f.get(), 100 * MS, &zero);
	f->state.x[0][0] = here.x - 0.09;
	f->state.x[0][1] = -4;
	xrt_vec3 p, v;
	rift_s_position_filter_predict(f.get(), 150 * MS, 50 * MS, &p, &v);
	CHECK(p.x == Catch::Approx(here.x - 0.1).margin(1e-5));
	CHECK(v.x == 0);
	// Past the bridge the held estimate is not extrapolated.
	rift_s_position_filter_predict(f.get(), RIFT_S_POSITION_FILTER_BRIDGE_NS + 100 * MS, 50 * MS, &p, &v);
	CHECK(p.x == Catch::Approx(here.x - 0.09).margin(1e-5));
	CHECK(v.x == 0);
}
