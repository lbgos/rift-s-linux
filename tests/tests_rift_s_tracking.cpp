// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#include "rift_s_tracker.h"
#include "rift_s_util.h"
#include "rift_s_controller.h"
#include "rift_s_hmd.h"
#include "math/m_clock_tracking.h"
#ifdef TEST_RIFT_S_STEAMVR_POSE
#include "ovrd_pose.hpp"
#endif
#include <algorithm>
#include <array>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <limits>
#include <random>

TEST_CASE("Rift S optical yaw takes ten percent of the shortest world yaw")
{
	const xrt_vec3 axis = {0, 1, 0};
	xrt_quat optical;
	for (float angle : {170.f, -170.f}) {
		math_quat_from_angle_vector(DEG_TO_RAD(angle), &axis, &optical);
		for (float sign : {1.f, -1.f}) {
			xrt_quat imu = {0, 0, 0, 1};
			xrt_quat equivalent = {sign * optical.x, sign * optical.y, sign * optical.z, sign * optical.w};
			CHECK(rift_s_apply_optical_yaw(&imu, &equivalent) == Catch::Approx(DEG_TO_RAD(angle * 0.1f)));
			CHECK(2 * atan2f(imu.y, imu.w) == Catch::Approx(DEG_TO_RAD(angle * 0.1f)));
		}
	}
}

TEST_CASE("Rift S optical yaw preserves gravity alignment while tilted")
{
	const xrt_vec3 tilt_axis = {1, 0, 0};
	const xrt_vec3 yaw_axis = {0, 1, 0};
	xrt_quat imu, yaw, optical;
	math_quat_from_angle_vector(DEG_TO_RAD(80), &tilt_axis, &imu);
	math_quat_from_angle_vector(DEG_TO_RAD(90), &yaw_axis, &yaw);
	math_quat_rotate(&yaw, &imu, &optical);
	xrt_quat inverse;
	xrt_vec3 before, after;
	math_quat_invert(&imu, &inverse);
	math_quat_rotate_vec3(&inverse, &yaw_axis, &before);
	CHECK(rift_s_apply_optical_yaw(&imu, &optical) == Catch::Approx(DEG_TO_RAD(9)));
	math_quat_invert(&imu, &inverse);
	math_quat_rotate_vec3(&inverse, &yaw_axis, &after);
	CHECK(after.x == Catch::Approx(before.x).margin(1e-6));
	CHECK(after.y == Catch::Approx(before.y).margin(1e-6));
	CHECK(after.z == Catch::Approx(before.z).margin(1e-6));
}

TEST_CASE("Rift S optical yaw applies small corrections and ignores the deadband")
{
	const xrt_vec3 axis = {0, 1, 0};
	xrt_quat imu = {0, 0, 0, 1}, optical;
	math_quat_from_angle_vector(DEG_TO_RAD(2), &axis, &optical);
	CHECK(rift_s_apply_optical_yaw(&imu, &optical) == Catch::Approx(DEG_TO_RAD(2)));
	CHECK(rift_s_apply_optical_yaw(&imu, &optical) == 0);
}

enum u_logging_level rift_s_log_level = U_LOGGING_ERROR;

static xrt_quat test_head_orientation = XRT_QUAT_IDENTITY;

static xrt_result_t
get_test_head_pose(xrt_device *, xrt_input_name, int64_t, xrt_space_relation *out)
{
	*out = XRT_SPACE_RELATION_ZERO;
	out->pose.orientation = test_head_orientation;
	out->pose.position = {1, 1.6f, 2};
	out->relation_flags = static_cast<xrt_space_relation_flags>(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
	return XRT_SUCCESS;
}

TEST_CASE("Rift S controller coordinates hold after loss and stay invalid before acquisition")
{
	rift_s_hmd hmd = {};
	hmd.base.get_tracked_pose = get_test_head_pose;
	rift_s_system sys = {};
	sys.hmd = &hmd;
	os_mutex_init(&sys.dev_mutex);
	rift_s_controller ctrl = {};
	ctrl.sys = &sys;
	ctrl.gravity_initialized = true;
	ctrl.pose.orientation.w = 1;
	ctrl.P_imu_device = XRT_POSE_IDENTITY;
	ctrl.P_device_aim = ctrl.P_device_grip = XRT_POSE_IDENTITY;
	os_mutex_init(&ctrl.mutex);

	for (auto hand : {XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER, XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER}) {
		ctrl.base.device_type = hand;
		for (auto input : {XRT_INPUT_TOUCH_GRIP_POSE, XRT_INPUT_TOUCH_AIM_POSE}) {
			for (auto optical_ts : {uint64_t{0}, uint64_t{1000000000}, uint64_t{1999000000}}) {
				// One optical fix at optical_ts (0: never fixed).
				const xrt_vec3 fix = {0.3f, 1.2f, 1.5f};
				rift_s_position_filter_reset(&ctrl.position_filter);
				ctrl.track_vision_updates = ctrl.track_reliable_updates = 0;
				ctrl.last_fix_ns = 0;
				if (optical_ts != 0) {
					const xrt_vec3 variance = {1e-5f, 1e-5f, 1e-5f};
					rift_s_position_filter_fix(&ctrl.position_filter, optical_ts, &fix, &variance,
					                           0.15f);
					ctrl.track_vision_updates = 1;
					ctrl.last_fix_ns = optical_ts;
				}
				xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
				REQUIRE(rift_s_controller_get_tracked_pose(&ctrl.base, input, 2000000000, &rel) ==
				        XRT_SUCCESS);
				CHECK((rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0);
				CHECK((rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) != 0);
				CHECK(((rel.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0) ==
				      (optical_ts != 0));
				// Optical confidence expires, but the held position remains usable.
				const bool tracked = optical_ts == 1999000000;
				const bool held = optical_ts == 1000000000;
				CHECK(((rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0) == tracked);
				CHECK(rel.pose.position.z == Catch::Approx(tracked || held ? 1.5f : 0));
				CHECK(rel.pose.position.x == Catch::Approx(tracked || held ? 0.3f : 0));
#ifdef TEST_RIFT_S_STEAMVR_POSE
				vr::DriverPose_t pose = {};
				pose.poseIsValid = true;
				pose.result = vr::TrackingResult_Running_OK;
				apply_pose(&rel, &pose, XRT_SPACE_RELATION_POSITION_VALID_BIT, true);
				CHECK(pose.poseIsValid == (tracked || held));
				CHECK(pose.result == (tracked || held ? vr::TrackingResult_Running_OK
				                                      : vr::TrackingResult_Running_OutOfRange));
				CHECK(pose.vecPosition[0] == Catch::Approx(tracked || held ? rel.pose.position.x : 0));
				CHECK(pose.vecPosition[2] == Catch::Approx(tracked || held ? rel.pose.position.z : 0));
				CHECK(pose.qRotation.w == Catch::Approx(1));
				// HMD call sites retain their original tracked-position requirement.
				vr::DriverPose_t head_pose = {};
				apply_pose(&rel, &head_pose);
				CHECK(head_pose.vecPosition[2] == Catch::Approx(tracked ? rel.pose.position.z : 0));
#endif
			}
		}
	}
	if (ctrl.imu_clock != nullptr)
		m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
	os_mutex_destroy(&ctrl.mutex);
	os_mutex_destroy(&sys.dev_mutex);
}

TEST_CASE("Rift S mature track bridges a short optical gap and clears optical confidence after loss")
{
	// r18 worn log: positional validity cleared 250 ms after every fix, and fixes came about every
	// 0.6 s, so both hands were invisible 60% of the time. A track with more than ten fixes keeps
	// its IMU-bridged position through a 400 ms gap; a young track keeps the 250 ms ladder step.
	rift_s_hmd hmd = {};
	hmd.base.get_tracked_pose = get_test_head_pose;
	rift_s_system sys = {};
	sys.hmd = &hmd;
	os_mutex_init(&sys.dev_mutex);
	rift_s_controller ctrl = {};
	ctrl.sys = &sys;
	ctrl.gravity_initialized = true;
	ctrl.pose.orientation.w = 1;
	ctrl.P_imu_device = ctrl.P_device_aim = ctrl.P_device_grip = XRT_POSE_IDENTITY;
	os_mutex_init(&ctrl.mutex);
	const timepoint_ns fix_ns = 1000000000;
	const xrt_vec3 fix = {0.3f, 1.2f, 1.5f}, variance = {1e-5f, 1e-5f, 1e-5f};
	auto valid_after = [&](uint32_t updates, time_duration_ns gap) {
		rift_s_position_filter_reset(&ctrl.position_filter);
		rift_s_position_filter_fix(&ctrl.position_filter, fix_ns, &fix, &variance, 0.15f);
		ctrl.track_vision_updates = ctrl.track_reliable_updates = updates;
		ctrl.last_fix_ns = fix_ns;
		xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
		REQUIRE(rift_s_controller_get_tracked_pose(&ctrl.base, XRT_INPUT_TOUCH_GRIP_POSE, fix_ns + gap, &rel) ==
		        XRT_SUCCESS);
		return (rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0;
	};
	CHECK(valid_after(30, 400 * U_TIME_1MS_IN_NS));
	CHECK(valid_after(30, 490 * U_TIME_1MS_IN_NS));
	CHECK_FALSE(valid_after(30, 510 * U_TIME_1MS_IN_NS));
	CHECK_FALSE(valid_after(1, 300 * U_TIME_1MS_IN_NS));
	os_mutex_destroy(&ctrl.mutex);
	os_mutex_destroy(&sys.dev_mutex);
}

TEST_CASE("Rift S stationary startup calibrates low-magnitude gravity and gyro bias")
{
	for (const xrt_vec3 accel : {xrt_vec3{7.6148f, 3.9714f, 1.6924f}, xrt_vec3{-6.9321f, 6.7900f, 1.8497f}}) {
		rift_s_controller ctrl = {};
		m_imu_3dof_init(&ctrl.fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
		os_mutex_init(&ctrl.mutex);
		ctrl.accel = accel;
		xrt_space_relation relation;
		CHECK(rift_s_controller_get_tracked_pose(&ctrl.base, XRT_INPUT_GENERIC_TRACKER_POSE, 1000000000,
		                                         &relation) == XRT_ERROR_POSE_NOT_ACTIVE);
		ctrl.gyro = {-0.014f, 0.003f, 0.017f};
		for (int i = 0; i < 125; i++) {
			rift_s_controller_update_fusion(&ctrl, 1000000000ULL + i * 2000000ULL);
			rift_s_controller_record_attitude(&ctrl, 1000000000ULL + i * 2000000ULL,
			                                  1000000000ULL + i * 2000000ULL);
		}
		CHECK_FALSE(ctrl.gravity_initialized);
		rift_s_controller_update_fusion(&ctrl, 1250000000ULL);
		rift_s_controller_record_attitude(&ctrl, 1250000000ULL, 1250000000ULL);
		REQUIRE(ctrl.gravity_initialized);
		CHECK(rift_s_controller_get_tracked_pose(&ctrl.base, XRT_INPUT_GENERIC_TRACKER_POSE, 1250000000,
		                                         &relation) == XRT_SUCCESS);
		for (int i = 126; i < 15000; i++) {
			rift_s_controller_update_fusion(&ctrl, 1000000000ULL + i * 2000000ULL);
			rift_s_controller_record_attitude(&ctrl, 1000000000ULL + i * 2000000ULL,
			                                  1000000000ULL + i * 2000000ULL);
		}
		xrt_vec3 world_accel;
		math_quat_rotate_vec3(&ctrl.pose.orientation, &accel, &world_accel);
		CHECK(world_accel.x == Catch::Approx(0).margin(0.15));
		CHECK(world_accel.z == Catch::Approx(0).margin(0.15));
		CHECK(world_accel.y * ctrl.gravity_accel_scale == Catch::Approx(MATH_GRAVITY_M_S2).margin(1e-3));
		// A reading this far from gravity (8.75 m/s^2) is an offset accelerometer: the fusion does
		// not level to it until the offset is known (see the optical levelling test). The 9.88 m/s^2
		// reading keeps gravity levelling.
		const bool offset = fabsf(m_vec3_len(accel) - (float)MATH_GRAVITY_M_S2) > 0.4f;
		CHECK(ctrl.accel_offset == offset);
		CHECK(((ctrl.fusion.flags & M_IMU_3DOF_USE_GRAVITY_DUR_20MS) == 0) == offset);
		if (ctrl.imu_clock != nullptr)
			m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
		os_mutex_destroy(&ctrl.mutex);
		m_imu_3dof_close(&ctrl.fusion);
	}
}

TEST_CASE("Rift S rest statistics expose an orientation-dependent accelerometer error")
{
	// A 1 m/s^2 bias along the IMU y axis: 8.8 m/s^2 lying one way up, 10.8 the other way.
	rift_s_rest_stats rest = {};
	const xrt_vec3 bias = {0, -1.0f, 0}, still = {0.01f, 0, 0}, turning = {0.5f, 0, 0};
	timepoint_ns t = 1000000000;
	auto hold = [&](xrt_vec3 up, int samples, xrt_vec3 gyro) {
		for (int i = 0; i < samples; i++, t += 2000000) {
			xrt_vec3 accel = m_vec3_add(m_vec3_mul_scalar(up, MATH_GRAVITY_M_S2), bias);
			accel.x += (i % 2 ? 0.05f : -0.05f);
			rift_s_rest_stats_push(&rest, t, &accel, &gyro);
		}
	};
	hold({0, 1, 0}, 100, still);
	CHECK(rest.windows == 0); // 200 ms is not yet a rest
	hold({0, 1, 0}, 100, still);
	CHECK(rest.windows == 1);
	hold({0, 0, 1}, 200, turning); // moving: ignored
	CHECK(rest.windows == 1);
	hold({0, -1, 0}, 200, still);
	hold({1, 0, 0}, 200, still);
	CHECK(rest.windows == 3);
	CHECK(rest.min_mps2 == Catch::Approx(MATH_GRAVITY_M_S2 - 1).margin(0.01));
	CHECK(rest.max_mps2 == Catch::Approx(MATH_GRAVITY_M_S2 + 1).margin(0.01));
	CHECK(rest.min_dir.y == Catch::Approx(1).margin(0.01));
	CHECK(rest.max_dir.y == Catch::Approx(-1).margin(0.01));
}

TEST_CASE("Rift S accelerometer bias from rest windows explains the right controller's worn readings")
{
	// Rest windows of the right controller in the r6 worn log: 8.802 m/s^2 along (0.87,0.43,0.23)
	// at startup, and the extremes 7.876 along (-0.29,0.95,-0.15) and 9.533 along (0.94,0.06,0.33).
	// The left read 9.79..9.98. One bias explains all three exactly.
	const xrt_vec3 logged[3] = {{7.678f, 3.795f, 2.030f}, {-2.274f, 7.448f, -1.176f}, {8.979f, 0.573f, 3.152f}};
	const xrt_vec3 bias = {-0.067f, -1.983f, 0.355f};
	for (const auto &m : logged)
		CHECK(m_vec3_len(m_vec3_sub(m, bias)) == Catch::Approx(MATH_GRAVITY_M_S2).margin(0.01));
	auto window = [&](xrt_vec3 dir) {
		math_vec3_normalize(&dir);
		return m_vec3_add(m_vec3_mul_scalar(dir, MATH_GRAVITY_M_S2), bias);
	};

	rift_s_accel_bias estimate = {};
	// One orientation, however often, observes nothing.
	for (int i = 0; i < 40; i++)
		rift_s_accel_bias_add_window(&estimate, &logged[0]);
	CHECK(estimate.count == 1);
	CHECK_FALSE(estimate.valid);
	rift_s_accel_bias_add_window(&estimate, &logged[1]);
	CHECK_FALSE(estimate.valid); // two directions
	rift_s_accel_bias_add_window(&estimate, &logged[2]);
	REQUIRE(estimate.valid);
	CHECK(estimate.rms_before_mps2 > 1.0f);
	CHECK(estimate.rms_after_mps2 < 0.05f);
	// Three windows leave some of the bias to the prior; more orientations pin it down.
	CHECK(m_vec3_len(m_vec3_sub(estimate.bias, bias)) < 0.5f);
	for (xrt_vec3 dir : {xrt_vec3{0.3f, 0.7f, 0.65f}, xrt_vec3{-0.6f, 0.5f, 0.6f}, xrt_vec3{0.2f, 0.4f, -0.9f}}) {
		xrt_vec3 mean = window(dir);
		rift_s_accel_bias_add_window(&estimate, &mean);
	}
	REQUIRE(estimate.valid);
	CHECK(m_vec3_len(m_vec3_sub(estimate.bias, bias)) < 0.1f);
	// The corrected gravity direction is within half a degree in every stored orientation, where
	// the raw reading was off by up to 12 degrees.
	float worst_raw = 0, worst = 0;
	for (unsigned i = 0; i < estimate.count; i++) {
		xrt_vec3 truth = m_vec3_sub(estimate.means[i], bias), raw = estimate.means[i],
		         fixed = m_vec3_sub(estimate.means[i], estimate.bias);
		worst_raw = std::max(worst_raw, m_vec3_angle(truth, raw));
		worst = std::max(worst, m_vec3_angle(truth, fixed));
	}
	CHECK(RAD_TO_DEG(worst_raw) > 8);
	CHECK(RAD_TO_DEG(worst) < 0.5);

	// A calibrated accelerometer gets (almost) no correction.
	rift_s_accel_bias good = {};
	for (xrt_vec3 dir : {xrt_vec3{0, 1, 0}, xrt_vec3{1, 0, 0}, xrt_vec3{0, 0.7f, 0.7f}, xrt_vec3{-0.7f, 0.7f, 0}}) {
		math_vec3_normalize(&dir);
		xrt_vec3 mean = m_vec3_mul_scalar(dir, MATH_GRAVITY_M_S2);
		rift_s_accel_bias_add_window(&good, &mean);
	}
	REQUIRE(good.valid);
	CHECK(m_vec3_len(good.bias) < 0.01f);
}

TEST_CASE("Rift S fusion uses the estimated accelerometer bias")
{
	rift_s_controller ctrl = {};
	m_imu_3dof_init(&ctrl.fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
	const xrt_vec3 bias = {-0.067f, -1.983f, 0.355f}, still = {0, 0, 0};
	timepoint_ns t = 1000000000;
	auto hold = [&](xrt_vec3 dir, int samples) {
		math_vec3_normalize(&dir);
		ctrl.accel = m_vec3_add(m_vec3_mul_scalar(dir, MATH_GRAVITY_M_S2), bias);
		ctrl.gyro = still;
		for (int i = 0; i < samples; i++, t += 2000000)
			rift_s_controller_update_fusion(&ctrl, t);
	};
	hold({0.87f, 0.43f, 0.23f}, 200);
	REQUIRE(ctrl.gravity_initialized);
	CHECK_FALSE(ctrl.accel_bias.valid);
	// The startup scale normalizes only the startup orientation.
	hold({0.94f, 0.06f, 0.33f}, 200);
	CHECK(m_vec3_len(ctrl.fusion.last.accel) > MATH_GRAVITY_M_S2 + 0.5f);
	hold({-0.29f, 0.95f, -0.15f}, 200);
	REQUIRE(ctrl.accel_bias.valid);
	hold({0.94f, 0.06f, 0.33f}, 10);
	CHECK(m_vec3_len(ctrl.fusion.last.accel) == Catch::Approx(MATH_GRAVITY_M_S2).margin(0.1));
	m_imu_3dof_close(&ctrl.fusion);
}

TEST_CASE("Rift S moving startup does not wait indefinitely for stationary samples")
{
	rift_s_controller ctrl = {};
	m_imu_3dof_init(&ctrl.fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
	ctrl.accel = {0, 9.82f, 0};
	ctrl.gyro = {0.4f, 0, 0};
	for (int i = 0; i < 200; i++) {
		rift_s_controller_update_fusion(&ctrl, 1000000000ULL + i * 2000000ULL);
	}
	CHECK(ctrl.gravity_initialized);
	ctrl.gyro = {0, 0, 0};
	for (int i = 200; i < 400; i++) {
		ctrl.accel.x = i % 2 ? 1 : -1;
		rift_s_controller_update_fusion(&ctrl, 1000000000ULL + i * 2000000ULL);
	}
	CHECK(ctrl.gravity_initialized);
	m_imu_3dof_close(&ctrl.fusion);
}

static void
record_test_attitude(rift_s_controller &ctrl, int sample, float yaw)
{
	const xrt_vec3 axis = {0, 1, 0};
	math_quat_from_angle_vector(DEG_TO_RAD(yaw), &axis, &ctrl.fusion.rot);
	ctrl.pose.orientation = ctrl.fusion.rot;
	const timepoint_ns device_ns = 1000000000LL + sample * 2000000LL;
	REQUIRE(rift_s_controller_record_attitude(&ctrl, device_ns, device_ns + 10000000000LL));
}

TEST_CASE("Rift S counts repeated IMU reports apart from clock errors")
{
	rift_s_controller ctrl = {};
	ctrl.device_type = RIFT_S_DEVICE_LEFT_CONTROLLER;
	os_mutex_init(&ctrl.mutex);
	auto imu_report = [&](uint32_t device_us, timepoint_ns arrival_ns) {
		rift_s_controller_report_t report = {};
		report.num_info = 1;
		report.info[0].block_id = RIFT_S_CTRL_IMU;
		report.info[0].imu.timestamp = device_us;
		rift_s_controller_handle_report(&ctrl, arrival_ns, &report);
	};
	imu_report(1000, 1000000000);
	imu_report(3000, 1002000000);
	imu_report(3000, 1002100000); // radio retransmit
	imu_report(3000, 1002200000);
	CHECK(ctrl.diagnostics.reject_duplicate == 2);
	CHECK(ctrl.diagnostics.reject_clock == 0);
	imu_report(2000, 1004000000); // device time running backwards
	CHECK(ctrl.diagnostics.reject_clock == 1);
	CHECK(ctrl.diagnostics.reject_clock_by[RIFT_S_CLOCK_BACKWARDS] == 1);
	CHECK(ctrl.diagnostics.backwards_run == 1);
	CHECK(ctrl.diagnostics.last_step_us == 2000);
	imu_report(5000, 1004000000);
	CHECK(ctrl.last_imu_device_time_ns == 5000000);
	CHECK(ctrl.diagnostics.backwards_run == 0);
	CHECK(ctrl.diagnostics.last_step_us == 2000);
	if (ctrl.imu_clock != nullptr)
		m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
	os_mutex_destroy(&ctrl.mutex);
}

TEST_CASE("Rift S IMU clock resync needs a run or a large repeated backwards step")
{
	CHECK_FALSE(rift_s_imu_clock_needs_resync(2000, 1)); // a late resend
	CHECK_FALSE(rift_s_imu_clock_needs_resync(50000, RIFT_S_IMU_RESYNC_RUN - 1));
	CHECK(rift_s_imu_clock_needs_resync(50000, RIFT_S_IMU_RESYNC_RUN));
	CHECK_FALSE(rift_s_imu_clock_needs_resync(1521000000u, 1)); // one stray sample is not enough
	CHECK(rift_s_imu_clock_needs_resync(1521000000u, 2));
}

TEST_CASE("Rift S IMU clock re-anchors after a stale first timestamp instead of rejecting for minutes")
{
	// r3 worn log after a VM reset without a headset power cycle: each controller's first
	// accepted sample was ~1521 s ahead, and every later sample was rejected as clock_backwards
	// (backwards_us shrinking by 1e6 per second, ~25 minutes of a hand glued to the head).
	rift_s_controller ctrl = {};
	ctrl.device_type = RIFT_S_DEVICE_RIGHT_CONTROLLER;
	os_mutex_init(&ctrl.mutex);
	auto imu_report = [&](uint32_t device_us, timepoint_ns arrival_ns) {
		rift_s_controller_report_t report = {};
		report.num_info = 1;
		report.info[0].block_id = RIFT_S_CTRL_IMU;
		report.info[0].imu.timestamp = device_us;
		rift_s_controller_handle_report(&ctrl, arrival_ns, &report);
	};
	const uint32_t base_us = 2690011380u, stale_us = base_us + 1521000000u;
	timepoint_ns arrival = 5000000000LL;
	imu_report(stale_us, arrival);
	REQUIRE(ctrl.imu_time_valid);
	// Dependent state captured against the stale anchor.
	const xrt_vec3 here = {0.2f, 1.1f, -0.3f}, variance = {1e-5f, 1e-5f, 1e-5f};
	rift_s_position_filter_reset(&ctrl.position_filter);
	rift_s_position_filter_fix(&ctrl.position_filter, arrival, &here, &variance, 0.15f);
	ctrl.attitude_history_count = 3;
	ctrl.clock_stable_samples = 32;
	ctrl.optical_heading_valid = true;
	ctrl.imu_to_host_ns = 123;
	timepoint_ns previous = ctrl.last_imu_device_time_ns;
	uint32_t accepted = 0;
	for (int i = 1; i <= 500; i++) {
		arrival += 2000000;
		imu_report(base_us + i * 2000u, arrival);
		// The 64-bit device time never runs backwards, so fusion time keeps moving forward.
		CHECK(ctrl.last_imu_device_time_ns >= previous);
		if (ctrl.last_imu_device_time_ns > previous)
			accepted++;
		previous = ctrl.last_imu_device_time_ns;
	}
	CHECK(ctrl.diagnostics.clock_resyncs == 1);
	CHECK(ctrl.diagnostics.reject_clock_by[RIFT_S_CLOCK_BACKWARDS] == 2); // the second sample, then resync
	CHECK(accepted == 499);
	CHECK(ctrl.diagnostics.backwards_run == 0);
	CHECK(ctrl.imu_timestamp32 == base_us + 500 * 2000u);
	CHECK(ctrl.diagnostics.last_step_us == 2000);
	// Reset like a clock discontinuity: mapping, capture-time history and heading prior.
	CHECK(ctrl.imu_clock == nullptr);
	CHECK(ctrl.imu_to_host_ns == 0);
	CHECK(ctrl.attitude_history_count == 0);
	CHECK(ctrl.clock_stable_samples == 0);
	CHECK_FALSE(ctrl.optical_heading_valid);
	CHECK(ctrl.position_filter.frozen);
	// After the resync, ordinary late resends are still rejected without another resync.
	imu_report(base_us + 499 * 2000u, arrival + 1000000);
	CHECK(ctrl.diagnostics.clock_resyncs == 1);
	CHECK(ctrl.diagnostics.reject_clock_by[RIFT_S_CLOCK_BACKWARDS] == 3);
	if (ctrl.imu_clock != nullptr)
		m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
	os_mutex_destroy(&ctrl.mutex);
}

TEST_CASE("Rift S IMU clock re-anchors after a run of small backwards steps")
{
	rift_s_controller ctrl = {};
	ctrl.device_type = RIFT_S_DEVICE_LEFT_CONTROLLER;
	os_mutex_init(&ctrl.mutex);
	auto imu_report = [&](uint32_t device_us, timepoint_ns arrival_ns) {
		rift_s_controller_report_t report = {};
		report.num_info = 1;
		report.info[0].block_id = RIFT_S_CTRL_IMU;
		report.info[0].imu.timestamp = device_us;
		rift_s_controller_handle_report(&ctrl, arrival_ns, &report);
	};
	timepoint_ns arrival = 1000000000;
	imu_report(100000, arrival);
	// The device clock restarts 50 ms earlier: every sample is behind the anchor.
	for (int i = 1; i < RIFT_S_IMU_RESYNC_RUN; i++)
		imu_report(50000 + i * 2000, arrival += 2000000);
	CHECK(ctrl.diagnostics.clock_resyncs == 0);
	CHECK(ctrl.diagnostics.backwards_run == RIFT_S_IMU_RESYNC_RUN - 1);
	imu_report(50000 + RIFT_S_IMU_RESYNC_RUN * 2000, arrival += 2000000);
	CHECK(ctrl.diagnostics.clock_resyncs == 1);
	const timepoint_ns anchored = ctrl.last_imu_device_time_ns;
	// Continued by the host time since the last accepted sample (the rejected run), never backwards.
	CHECK(anchored == 100000000LL + RIFT_S_IMU_RESYNC_RUN * 2000000LL);
	imu_report(50000 + (RIFT_S_IMU_RESYNC_RUN + 1) * 2000, arrival += 2000000);
	CHECK(ctrl.last_imu_device_time_ns == anchored + 2000000);
	if (ctrl.imu_clock != nullptr)
		m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
	os_mutex_destroy(&ctrl.mutex);
}

TEST_CASE("Rift S reports use the firmware TrackedObject calibration in sensor units")
{
	rift_s_controller ctrl = {};
	os_mutex_init(&ctrl.mutex);
	m_imu_3dof_init(&ctrl.fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
	ctrl.have_calibration = ctrl.have_config = true;
	ctrl.config.accel_scale = 1.0f / 1024;
	ctrl.config.gyro_scale = 0.1220703125f;
	// Recorded right-controller factory arrays.
	ctrl.calibration.accel_calibration = {{{.99574226f, .04186834f, -.03562105f, -.03175851f, .00306439f,
	                                        -1.00078183f, -.04157540f, 1.00271152f, .00852858f}},
	                                      {-.20133353f, .05943762f, .07639897f},
	                                      12};
	ctrl.calibration.gyro_calibration = {{{.99724316f, .04099295f, -.03803329f, -.02534426f, -.00046619f,
	                                       -.99736829f, -.04205823f, .99727071f, .00495981f}},
	                                     {.00554654f, -.00502796f, .00356880f},
	                                     12};
	ctrl.gyro_calibration = ctrl.calibration.gyro_calibration;
	ctrl.imu_descriptor_done = true;
	// Deliberately different legacy fields must not select the calibration frame.
	ctrl.calibration.accel.rectification = ctrl.calibration.gyro.rectification = {{1, 0, 0, 0, 1, 0, 0, 0, 1}};
	rift_s_controller_report_t report = {};
	report.num_info = 1;
	report.info[0].block_id = RIFT_S_CTRL_IMU;
	report.info[0].imu.timestamp = 123000;
	const int16_t accel[3] = {321, -456, 789}, gyro[3] = {-100, 200, 300};
	for (int i = 0; i < 3; i++) {
		report.info[0].imu.accel[i] = accel[i];
		report.info[0].imu.gyro[i] = gyro[i];
	}
	rift_s_controller_handle_report(&ctrl, 123000000, &report);
	CHECK(ctrl.accel.x == Catch::Approx(2.809778870).margin(1e-6));
	CHECK(ctrl.accel.y == Catch::Approx(-7.603138362).margin(1e-6));
	CHECK(ctrl.accel.z == Catch::Approx(-4.510852398).margin(1e-6));
	CHECK(ctrl.gyro.x == Catch::Approx(-0.224496913).margin(1e-6));
	CHECK(ctrl.gyro.y == Catch::Approx(-0.628577923).margin(1e-6));
	CHECK(ctrl.gyro.z == Catch::Approx(0.442303355).margin(1e-6));
	m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
	m_imu_3dof_close(&ctrl.fusion);
	os_mutex_destroy(&ctrl.mutex);
}

TEST_CASE("Rift S controller IMU waits for the descriptor, then falls back to factory gyro calibration")
{
	rift_s_controller ctrl = {};
	os_mutex_init(&ctrl.mutex);
	m_imu_3dof_init(&ctrl.fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
	ctrl.have_calibration = ctrl.have_config = true;
	ctrl.config.accel_scale = 1.0f / 1024;
	ctrl.config.gyro_scale = 0.1220703125f;
	ctrl.calibration.accel_calibration = {{{1, 0, 0, 0, 1, 0, 0, 0, 1}}, {}, 12};
	ctrl.calibration.gyro_calibration = {{{1, 0, 0, 0, 1, 0, 0, 0, 1}}, {}, 12};
	ctrl.gyro_calibration = ctrl.calibration.gyro_calibration;
	ctrl.imu_descriptor_requested = true;
	ctrl.imu_descriptor_deadline_ns = 3000000000;
	rift_s_controller_report_t report = {};
	report.num_info = 1;
	report.info[0].block_id = RIFT_S_CTRL_IMU;
	report.info[0].imu.timestamp = 1000;
	report.info[0].imu.gyro[0] = 100;

	rift_s_controller_handle_report(&ctrl, 1000000000, &report);
	CHECK_FALSE(ctrl.imu_descriptor_done);
	CHECK(ctrl.gyro.x == 0.0f);

	report.info[0].imu.timestamp = 5000;
	rift_s_controller_handle_report(&ctrl, 3000000000, &report);
	CHECK(ctrl.imu_descriptor_done);
	CHECK(ctrl.gyro.x == Catch::Approx(DEG_TO_RAD(100 * 0.1220703125)).margin(1e-6));

	m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
	m_imu_3dof_close(&ctrl.fusion);
	os_mutex_destroy(&ctrl.mutex);
}

TEST_CASE("Rift S head and camera output invert the firmware transform from IMU")
{
	rift_s_tracker tracker = {};
	os_mutex_init(&tracker.mutex);
	tracker.pose.position = {1, 1.6f, -2};
	const xrt_vec3 y = {0, 1, 0}, x = {1, 0, 0};
	math_quat_from_angle_vector(DEG_TO_RAD(70), &y, &tracker.pose.orientation);
	// Without SLAM the position is the guard's held position.
	rift_s_slam_guard_init(&tracker.slam_guard);
	tracker.slam_guard.have_good = true;
	tracker.slam_guard.good.position = tracker.pose.position;
	tracker.device_from_imu.position = {-0.000295f, -0.0190975f, -0.068692f};
	math_quat_from_angle_vector(DEG_TO_RAD(15), &x, &tracker.device_from_imu.orientation);
	tracker.left_cam_from_imu.position = {0.04f, -0.02f, 0.08f};
	math_quat_from_angle_vector(DEG_TO_RAD(-30), &y, &tracker.left_cam_from_imu.orientation);
	for (auto pose : {RIFT_S_TRACKER_POSE_DEVICE, RIFT_S_TRACKER_POSE_LEFT_CAMERA}) {
		xrt_space_relation rel;
		rift_s_tracker_get_tracked_pose(&tracker, pose, 1000000000, &rel);
		// T_world_target * T_target_imu must recover T_world_imu.
		xrt_pose recovered;
		const xrt_pose &offset =
		    pose == RIFT_S_TRACKER_POSE_DEVICE ? tracker.device_from_imu : tracker.left_cam_from_imu;
		math_pose_transform(&rel.pose, &offset, &recovered);
		CHECK(recovered.position.x == Catch::Approx(tracker.pose.position.x).margin(1e-6));
		CHECK(recovered.position.y == Catch::Approx(tracker.pose.position.y).margin(1e-6));
		CHECK(recovered.position.z == Catch::Approx(tracker.pose.position.z).margin(1e-6));
		CHECK(std::abs(recovered.orientation.x - tracker.pose.orientation.x) < 1e-6);
		CHECK(std::abs(recovered.orientation.y - tracker.pose.orientation.y) < 1e-6);
		CHECK(std::abs(recovered.orientation.z - tracker.pose.orientation.z) < 1e-6);
		CHECK(std::abs(recovered.orientation.w - tracker.pose.orientation.w) < 1e-6);
	}
	os_mutex_destroy(&tracker.mutex);
}

TEST_CASE("Rift S IMU clock mapping holds steady through radio bursts and stalls")
{
	rift_s_controller ctrl = {};
	ctrl.gravity_initialized = true;
	ctrl.fusion.rot = XRT_QUAT_IDENTITY;
	std::mt19937 rng(0xc10c);
	std::uniform_real_distribution<double> jitter(0, 2e6);
	const timepoint_ns host_offset = 10000000000LL, min_delay = 3000000;
	time_duration_ns mean_filter = 0, mean_min = INT64_MAX, mean_max = INT64_MIN;
	time_duration_ns mapped_min = INT64_MAX, mapped_max = INT64_MIN;
	unsigned mean_filter_jumps = 0;
	timepoint_ns last_arrival = 0;
	for (int i = 0; i < 20000; i++) {
		timepoint_ns device_ns = 1000000000LL + i * 2000000LL;
		// Reports queue on the radio and leave in bursts every 8 ms; every 1000th burst stalls 30 ms.
		timepoint_ns release = (device_ns / 8000000 + 1) * 8000000;
		if ((device_ns / 8000000) % 1000 == 999)
			release += 30000000;
		timepoint_ns arrival =
		    std::max(last_arrival, release + host_offset + min_delay + (timepoint_ns)jitter(rng));
		last_arrival = arrival;
		REQUIRE(rift_s_controller_record_attitude(&ctrl, device_ns, arrival));
		time_duration_ns old = mean_filter;
		m_clock_offset_a2b(500, device_ns, arrival, &mean_filter);
		if (old != 0 && std::llabs(mean_filter - old) > 2000000)
			mean_filter_jumps++;
		if (i < 1000)
			continue;
		time_duration_ns mapped = ctrl.imu_to_host_ns - host_offset;
		mapped_min = std::min(mapped_min, mapped);
		mapped_max = std::max(mapped_max, mapped);
		mean_min = std::min(mean_min, mean_filter - host_offset);
		mean_max = std::max(mean_max, mean_filter - host_offset);
	}
	printf("clock: min_skew_delay_ms=%.3f..%.3f mean_filter_delay_ms=%.3f..%.3f mean_filter_jumps=%u\n",
	       mapped_min / 1e6, mapped_max / 1e6, mean_min / 1e6, mean_max / 1e6, mean_filter_jumps);
	// The mapped capture time stays within a millisecond of the minimum transport delay and never
	// resets the history. The mean-delay filter it replaces lags a further 3 to 9 ms behind and
	// wanders by several milliseconds with the burst pattern.
	CHECK(ctrl.diagnostics.reject_clock == 0);
	CHECK(ctrl.attitude_history_count == RIFT_S_ATTITUDE_HISTORY_CAPACITY);
	CHECK(mapped_min >= min_delay);
	CHECK(mapped_max - mapped_min < 1000000);
	CHECK(mean_max - mean_min > 4000000);
	CHECK(mean_min > mapped_max);
	// Capture-time lookups succeed just before the newest sample.
	const timepoint_ns newest = 1000000000LL + 19999 * 2000000LL + ctrl.imu_to_host_ns;
	rift_s_attitude_sample sample;
	CHECK(rift_s_controller_get_attitude(&ctrl, newest - 1000000, &sample));

	// A real discontinuity (the device clock restarting 200 ms ahead) discards the history while the
	// estimate moves, then the mapping settles on the new offset and the history refills.
	timepoint_ns device_ns = 1000000000LL + 20000 * 2000000LL + 200000000LL;
	const time_duration_ns settled = last_arrival + 2000000 - device_ns;
	for (int i = 0; i < 400; i++, device_ns += 2000000) {
		REQUIRE(rift_s_controller_record_attitude(&ctrl, device_ns, last_arrival += 2000000));
	}
	CHECK(ctrl.diagnostics.reject_clock > 0);
	CHECK(ctrl.diagnostics.reject_clock < 64);
	// Every one of them is counted under its cause.
	uint64_t by_cause = 0;
	for (uint64_t n : ctrl.diagnostics.reject_clock_by)
		by_cause += n;
	CHECK(by_cause == ctrl.diagnostics.reject_clock);
	CHECK(ctrl.diagnostics.reject_clock_by[RIFT_S_CLOCK_DISCONTINUITY] > 0);
	CHECK(ctrl.diagnostics.reject_clock_by[RIFT_S_CLOCK_BACKWARDS] == 0);
	CHECK(std::llabs(ctrl.imu_to_host_ns - settled) < 1000000);
	CHECK(ctrl.attitude_history_count > 300);
	m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
}

TEST_CASE("Rift S IMU clock re-anchors at once after a sleep that wraps the device counter")
{
	// r18 wake log: the controllers slept 4927 s, longer than one 2^32 us wrap of the device counter,
	// so device time advanced 632 s. The mapping then crept 1/64 per sample towards the new offset
	// and reset the capture history on 662 consecutive samples before tracking could start.
	rift_s_controller ctrl = {};
	ctrl.gravity_initialized = true;
	ctrl.fusion.rot = XRT_QUAT_IDENTITY;
	const time_duration_ns delay = 3000000;
	timepoint_ns device_ns = 1000000000LL, host_ns = 50000000000LL;
	for (int i = 0; i < 500; i++, device_ns += 2000000, host_ns += 2000000)
		REQUIRE(rift_s_controller_record_attitude(&ctrl, device_ns, host_ns + delay));
	REQUIRE(ctrl.diagnostics.reject_clock == 0);

	const time_duration_ns sleep = 4927000000000LL, lost_wrap = 4294967296000LL;
	device_ns += sleep - lost_wrap;
	host_ns += sleep;
	unsigned first_tilt_valid = 0;
	for (unsigned i = 0; i < 400; i++, device_ns += 2000000, host_ns += 2000000) {
		rift_s_controller_record_attitude(&ctrl, device_ns, host_ns + delay);
		rift_s_attitude_sample sample;
		if (first_tilt_valid == 0 && ctrl.attitude_history_count >= 2 &&
		    rift_s_controller_get_attitude(&ctrl, host_ns + delay - 1000000, &sample))
			first_tilt_valid = i + 1;
	}
	INFO("discontinuity resets=" << ctrl.diagnostics.reject_clock_by[RIFT_S_CLOCK_DISCONTINUITY]
	                             << " first usable sample=" << first_tilt_valid);
	CHECK(ctrl.diagnostics.reject_clock_by[RIFT_S_CLOCK_DISCONTINUITY] <= 2);
	CHECK(ctrl.diagnostics.clock_resyncs == 1);
	CHECK(first_tilt_valid > 0);
	CHECK(first_tilt_valid <= 40); // 80 ms of 500 Hz reports
	CHECK(std::llabs(ctrl.imu_to_host_ns - (host_ns - device_ns + delay)) < 1000000);
	m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
}

TEST_CASE("Rift S IMU clock mapping works when the device clock is ahead of the host clock")
{
	// The play-r8 worn log: controller timestamps of 92846 s (a 48-bit clock counting since the
	// controllers woke, the same clock the camera metadata uses) against a host monotonic clock of
	// 33474 s. The skew is then negative. Averaged as an unsigned value it jumped to about
	// INT64_MAX, which showed as clock_discontinuity_ms=9223372036853 and reset the history on every
	// sample.
	const timepoint_ns device_start = 92846078749000LL, host_start = 33474445657118LL;
	m_clock_windowed_skew_tracker *tracker = m_clock_windowed_skew_tracker_alloc(64);
	REQUIRE(tracker != nullptr);
	const time_duration_ns delay = 4000000;
	for (int i = 0; i < 500; i++) {
		timepoint_ns device_ns = device_start + i * 2000000LL;
		m_clock_windowed_skew_tracker_push(tracker, host_start + i * 2000000LL + delay + (i % 5) * 300000,
		                                   device_ns);
		timepoint_ns local;
		REQUIRE(m_clock_windowed_skew_tracker_to_local(tracker, device_ns, &local));
		// The first samples have a window of one to a few; every one must be near the truth.
		CHECK(std::llabs(local - (host_start + i * 2000000LL + delay)) < 2000000);
	}
	m_clock_windowed_skew_tracker_destroy(tracker);

	rift_s_controller ctrl = {};
	ctrl.gravity_initialized = true;
	ctrl.fusion.rot = XRT_QUAT_IDENTITY;
	for (int i = 0; i < 2000; i++) {
		timepoint_ns device_ns = device_start + i * 2000000LL;
		REQUIRE(rift_s_controller_record_attitude(&ctrl, device_ns, host_start + i * 2000000LL + delay));
	}
	CHECK(ctrl.diagnostics.reject_clock == 0);
	CHECK(ctrl.attitude_history_count == RIFT_S_ATTITUDE_HISTORY_CAPACITY);
	CHECK(std::llabs(ctrl.imu_to_host_ns - (host_start - device_start + delay)) < 1000000);
	m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
}

TEST_CASE("Rift S delayed optical yaw corrects capture attitude and preserves later rotation")
{
	rift_s_controller ctrl = {};
	ctrl.gravity_initialized = ctrl.update_yaw_from_optical = true;
	ctrl.device_type = RIFT_S_DEVICE_LEFT_CONTROLLER;
	os_mutex_init(&ctrl.mutex);
	for (int i = 0; i <= 100; i++) {
		record_test_attitude(ctrl, i, i * 0.5f);
	}
	xrt_pose optical = XRT_POSE_IDENTITY;
	const xrt_vec3 axis = {0, 1, 0};
	const timepoint_ns capture_ns = 11101000000LL; // Between samples 50 and 51, yaw 25.25 degrees.
	math_quat_from_angle_vector(DEG_TO_RAD(45.25f), &axis, &optical.orientation);
	optical.position = {0.3f, 1.2f, -0.7f};
	rift_s_controller_push_observed_pose(&ctrl.base, capture_ns, &optical);
	CHECK(ctrl.diagnostics.optical_accepted == 1);
	CHECK_FALSE(rift_s_controller_has_heading_prior(&ctrl, capture_ns));
	CHECK(2 * atan2f(ctrl.fusion.rot.y, ctrl.fusion.rot.w) == Catch::Approx(DEG_TO_RAD(52.f)));
	xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	REQUIRE(rift_s_controller_get_tracked_pose(&ctrl.base, XRT_INPUT_GENERIC_TRACKER_POSE, capture_ns, &rel) ==
	        XRT_SUCCESS);
	CHECK(2 * atan2f(rel.pose.orientation.y, rel.pose.orientation.w) == Catch::Approx(DEG_TO_RAD(27.25f)));
	CHECK((rel.relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT) == 0);
	CHECK((rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0);
	// Earlier queries must not consume a position observed later in time.
	REQUIRE(rift_s_controller_get_tracked_pose(&ctrl.base, XRT_INPUT_GENERIC_TRACKER_POSE, capture_ns - 1000000,
	                                           &rel) == XRT_SUCCESS);
	CHECK((rel.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) == 0);
	// A subsequent small residual establishes a heading prior after correction.
	math_quat_from_angle_vector(DEG_TO_RAD(31.f), &axis, &optical.orientation);
	rift_s_controller_push_observed_pose(&ctrl.base, capture_ns + 1000000, &optical);
	CHECK(rift_s_controller_has_heading_prior(&ctrl, capture_ns + 1000000));
	CHECK_FALSE(rift_s_controller_has_heading_prior(&ctrl, capture_ns));
	if (ctrl.imu_clock != nullptr)
		m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
	os_mutex_destroy(&ctrl.mutex);
}

TEST_CASE("Rift S capture attitude rejects stale, future, gapped and missing history")
{
	rift_s_controller ctrl = {};
	ctrl.gravity_initialized = true;
	os_mutex_init(&ctrl.mutex);
	xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	CHECK(rift_s_controller_get_tracked_pose(&ctrl.base, XRT_INPUT_GENERIC_TRACKER_POSE, 11070000000LL, &rel) ==
	      XRT_ERROR_POSE_NOT_ACTIVE);
	for (int i = 0; i <= RIFT_S_ATTITUDE_HISTORY_CAPACITY + 32; i++) {
		record_test_attitude(ctrl, i, 0);
	}
	CHECK(ctrl.attitude_history_count == RIFT_S_ATTITUDE_HISTORY_CAPACITY);
	xrt_pose optical = XRT_POSE_IDENTITY;
	rift_s_controller_push_observed_pose(&ctrl.base, 11064000000LL, &optical);
	CHECK(ctrl.diagnostics.reject_history == 2);
	CHECK(ctrl.diagnostics.optical_accepted == 0);
	CHECK(ctrl.last_tracked_pose_ts == 0);
	CHECK_FALSE(rift_s_controller_has_heading_prior(&ctrl, 11070000000LL));
	const timepoint_ns latest = 11000000000LL + (RIFT_S_ATTITUDE_HISTORY_CAPACITY + 32) * 2000000LL;
	CHECK(rift_s_controller_get_tracked_pose(&ctrl.base, XRT_INPUT_GENERIC_TRACKER_POSE, latest + 20000001, &rel) ==
	      XRT_ERROR_POSE_NOT_ACTIVE);
	CHECK(rel.relation_flags == 0);
	rift_s_controller_push_observed_pose(&ctrl.base, latest, &optical);
	CHECK(ctrl.diagnostics.optical_accepted == 1);
	rift_s_controller_push_observed_pose(&ctrl.base, latest, &optical);
	CHECK(ctrl.diagnostics.reject_stale == 1);
	record_test_attitude(ctrl, RIFT_S_ATTITUDE_HISTORY_CAPACITY + 50, 0);
	CHECK(rift_s_controller_get_tracked_pose(&ctrl.base, XRT_INPUT_GENERIC_TRACKER_POSE, latest + 1000000, &rel) ==
	      XRT_ERROR_POSE_NOT_ACTIVE);
	optical.position.x = std::numeric_limits<float>::quiet_NaN();
	rift_s_controller_push_observed_pose(&ctrl.base, latest + 36000000, &optical);
	CHECK(ctrl.diagnostics.reject_nonfinite == 1);
	CHECK(ctrl.diagnostics.optical_accepted == 1);
	if (ctrl.imu_clock != nullptr)
		m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
	os_mutex_destroy(&ctrl.mutex);
}

TEST_CASE("Rift S capture prior never marks synthetic fallback as tracked position")
{
	rift_s_controller ctrl = {};
	ctrl.gravity_initialized = true;
	os_mutex_init(&ctrl.mutex);
	for (int i = 0; i < 40; i++) {
		record_test_attitude(ctrl, i, 0);
	}
	ctrl.pose.position = {0.2f, -0.25f, -0.45f};
	xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	REQUIRE(rift_s_controller_get_tracked_pose(&ctrl.base, XRT_INPUT_GENERIC_TRACKER_POSE, 11078000000LL, &rel) ==
	        XRT_SUCCESS);
	CHECK((rel.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) == 0);
	CHECK((rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) == 0);
	CHECK_FALSE(rift_s_controller_has_heading_prior(&ctrl, 11078000000LL));
	if (ctrl.imu_clock != nullptr)
		m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
	os_mutex_destroy(&ctrl.mutex);
}

TEST_CASE("Rift S position-only optical fits never confirm the IMU heading they echo")
{
	rift_s_controller ctrl = {};
	ctrl.gravity_initialized = ctrl.update_yaw_from_optical = true;
	ctrl.device_type = RIFT_S_DEVICE_RIGHT_CONTROLLER;
	os_mutex_init(&ctrl.mutex);
	const xrt_vec3 axis = {0, 1, 0};
	// 5 ms spacing so the bounded history spans the heading-prior age limit.
	for (int i = 0; i <= 1000; i++) {
		math_quat_from_angle_vector(DEG_TO_RAD(10.f), &axis, &ctrl.fusion.rot);
		ctrl.pose.orientation = ctrl.fusion.rot;
		const timepoint_ns device_ns = 1000000000LL + i * 5000000LL;
		REQUIRE(rift_s_controller_record_attitude(&ctrl, device_ns, device_ns + 10000000000LL));
	}
	const timepoint_ns first = 12010000000LL;
	t_constellation_pose_observation obs = {};
	obs.capture_ns = first;
	obs.pose = XRT_POSE_IDENTITY;
	obs.pose.position = {0.1f, 1.1f, -0.4f};
	// The fit returns the prior orientation (10 degrees) without measuring it.
	math_quat_from_angle_vector(DEG_TO_RAD(10.f), &axis, &obs.pose.orientation);
	obs.orientation_observed = false;
	obs.inliers = 3;
	obs.distinct_leds = 2;
	obs.information_rank = 3;
	obs.position_covariance[0] = obs.position_covariance[4] = obs.position_covariance[8] = 1e-6f;
	obs.hypothesis = "heading_translation";
	rift_s_controller_push_pose_observation(&ctrl.base, &obs);
	CHECK(ctrl.diagnostics.optical_accepted == 1);
	CHECK(ctrl.diagnostics.optical_position_only == 1);
	CHECK(ctrl.diagnostics.optical_reliable == 0);
	CHECK(ctrl.last_tracked_pose_ts == first);
	CHECK(ctrl.last_position_covariance[4] == Catch::Approx(1e-6f));
	CHECK_FALSE(rift_s_controller_has_heading_prior(&ctrl, first));

	// Even a wrong orientation in a position-only fit must not rotate the IMU state.
	obs.capture_ns = first + 10000000;
	math_quat_from_angle_vector(DEG_TO_RAD(70.f), &axis, &obs.pose.orientation);
	rift_s_controller_push_pose_observation(&ctrl.base, &obs);
	CHECK(2 * atan2f(ctrl.fusion.rot.y, ctrl.fusion.rot.w) == Catch::Approx(DEG_TO_RAD(10.f)));

	// A fit that measures yaw confirms the heading, and counts as reliable with 3 distinct LEDs.
	obs.capture_ns = first + 20000000;
	math_quat_from_angle_vector(DEG_TO_RAD(11.f), &axis, &obs.pose.orientation);
	obs.orientation_observed = true;
	obs.distinct_leds = 3;
	obs.information_rank = 4;
	rift_s_controller_push_pose_observation(&ctrl.base, &obs);
	CHECK(ctrl.diagnostics.optical_reliable == 1);
	CHECK(rift_s_controller_has_heading_prior(&ctrl, first + 20000000));

	// Position-only updates keep the position tracked but let the heading prior age out.
	obs.orientation_observed = false;
	for (timepoint_ns t = first + 400000000; t <= first + 2400000000LL; t += 400000000) {
		obs.capture_ns = t;
		rift_s_controller_push_pose_observation(&ctrl.base, &obs);
	}
	CHECK(ctrl.last_tracked_pose_ts == first + 2400000000LL);
	CHECK_FALSE(rift_s_controller_has_heading_prior(&ctrl, first + 2400000000LL));
	if (ctrl.imu_clock != nullptr)
		m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
	os_mutex_destroy(&ctrl.mutex);
}

#include "rift_s_model_check.h"
#include <array>
#include <vector>
#include "constellation_rift_s_data.hpp"

namespace {
struct FirmwareModel
{
	std::vector<rift_s_led> leds;
	std::vector<rift_s_lensing_model> lensing = std::vector<rift_s_lensing_model>(2);
	rift_s_controller_imu_calibration calibration{};
	explicit FirmwareModel(bool mirror_x)
	{
		// Device-frame points from the recorded left ring; the right hand is its X mirror.
		for (const auto &l : rift_s_ring_leds) {
			rift_s_led led{};
			led.pos = l.pos;
			led.dir = m_vec3_mul_scalar(l.dir, 1.0f + 0.01f * (l.id % 3)); // firmware normals are not unit
			led.angles = {85, 80, 0};
			if (mirror_x) {
				led.pos.x = -led.pos.x;
				led.dir.x = -led.dir.x;
			}
			leds.push_back(led);
		}
		calibration.num_leds = leds.size();
		calibration.leds = leds.data();
		calibration.num_lensing_models = lensing.size();
		calibration.lensing_models = lensing.data();
		calibration.imu_position = {mirror_x ? 0.005584116f : -0.005584116f, -0.02f, 0.01f};
		calibration.accel.rectification = calibration.gyro.rectification = {{1, 0, 0, 0, 1, 0, 0, 0, 1}};
		calibration.accel_calibration.matrix =
		    calibration.gyro_calibration.matrix = {{1, 0, 0, 0, 1, 0, 0, 0, 1}};
		calibration.accel_calibration.num_values = calibration.gyro_calibration.num_values = 12;
	}
};
} // namespace

TEST_CASE("Rift S model check reports normals, IMU placement and IMU axis handedness")
{
	FirmwareModel left(false);
	rift_s_controller_model_report r;
	rift_s_controller_model_check(&left.calibration, &r);
	CHECK(r.num_leds == 15);
	CHECK(r.num_lensing_models == 2);
	CHECK(r.ids_sequential);
	CHECK(r.normal_length_min == Catch::Approx(1.0f).margin(1e-3));
	CHECK(r.normal_length_max == Catch::Approx(1.02f).margin(1e-3));
	// The recorded ring's normals face away from its centroid.
	CHECK(r.outward_normals >= 13);
	CHECK(r.angle_x_min == 85);
	CHECK(r.angle_y_max == 80);
	CHECK(r.accel_rectification.determinant == Catch::Approx(1));
	CHECK_FALSE(r.accel_rectification.reflection);
	CHECK(r.accel_rectification.rotation_deg == Catch::Approx(0).margin(0.01));
	CHECK(r.accel_gyro_axes_deg == Catch::Approx(0).margin(0.01));
	CHECK(r.accel_calibration.rotation_deg == Catch::Approx(0).margin(0.01));
	CHECK(r.accel_calibration.determinant == Catch::Approx(1));
	CHECK(r.accel_calibration_offset_norm == 0);
	CHECK(r.accel_calibration_values == 12);
	CHECK(r.accel_calibration_vs_rectification_deg == Catch::Approx(0).margin(0.01));

	// A swapped-axis accelerometer rectification is a reflection; a gyro rotated against it is reported.
	left.calibration.accel.rectification = {{0, 1, 0, 1, 0, 0, 0, 0, 1}};
	left.calibration.gyro.rectification = {{0, -1, 0, 1, 0, 0, 0, 0, 1}};
	rift_s_controller_model_check(&left.calibration, &r);
	CHECK(r.accel_rectification.reflection);
	CHECK_FALSE(r.gyro_rectification.reflection);
	CHECK(r.gyro_rectification.rotation_deg == Catch::Approx(90).margin(0.01));
	CHECK(r.accel_gyro_axes_deg > 5);

	// The TrackedObject matrix is compared with the rectification it would replace.
	left.calibration.gyro_calibration.matrix = left.calibration.gyro.rectification;
	left.calibration.gyro_calibration.offset = {0.01f, 0, 0};
	rift_s_controller_model_check(&left.calibration, &r);
	CHECK(r.gyro_calibration.rotation_deg == Catch::Approx(90).margin(0.01));
	CHECK(r.gyro_calibration_vs_rectification_deg == Catch::Approx(0).margin(0.01));
	CHECK(r.gyro_calibration_offset_norm == Catch::Approx(0.01));
	CHECK(r.accel_calibration_vs_rectification_deg == Catch::Approx(180).margin(0.1));
}

TEST_CASE("Rift S gravity check compares the resting accelerometer under both calibrations")
{
	FirmwareModel model(false);
	rift_s_controller_imu_calibration &c = model.calibration;
	// Top-level calibration: a 90 degree axis rotation and an offset along the sample's x axis.
	c.accel.rectification = {{0, -1, 0, 1, 0, 0, 0, 0, 1}};
	c.accel.offset = {0.4f, 0, 0};
	// TrackedObject uses the same sensor-space bias subtraction as the top-level calibration.
	c.accel_calibration.matrix = c.accel.rectification;
	c.accel_calibration.offset = c.accel.offset;
	xrt_vec3 rectified = {0, 9.81f, 0};
	rift_s_controller_gravity_report check;
	rift_s_controller_gravity_check(&c, &rectified, &check);
	REQUIRE(check.valid);
	CHECK(check.rectified_mps2 == Catch::Approx(9.81f));
	CHECK(check.tracked_mps2 == Catch::Approx(9.81f));
	CHECK(check.direction_deg == Catch::Approx(0).margin(0.05));

	// Identity TrackedObject matrix without the offset: the wrong axes and the unremoved bias (here
	// along the sample's gravity axis) are both visible.
	c.accel_calibration.matrix = {{1, 0, 0, 0, 1, 0, 0, 0, 1}};
	c.accel_calibration.offset = {0, 0, 0};
	// The same sensor sample after this factory calibration.
	rectified = {10.21f, 0, 0};
	rift_s_controller_gravity_check(&c, &rectified, &check);
	REQUIRE(check.valid);
	CHECK(check.direction_deg == Catch::Approx(90).margin(0.05));
	CHECK(check.tracked_mps2 == Catch::Approx(9.81 + 0.4).margin(1e-3));

	// A truncated array is not interpreted.
	c.accel_calibration.num_values = 9;
	rift_s_controller_gravity_check(&c, &rectified, &check);
	CHECK_FALSE(check.valid);
}

TEST_CASE("Rift S hand comparison makes mirroring explicit rather than accidental")
{
	FirmwareModel left(false), right(true), unmirrored(false);
	rift_s_controller_hand_comparison cmp;
	rift_s_controller_compare_hands(&left.calibration, &right.calibration, &cmp);
	CHECK(cmp.best == RIFT_S_HANDS_MIRROR_X);
	CHECK(cmp.position_rms_m[RIFT_S_HANDS_MIRROR_X] < 1e-6);
	CHECK(cmp.normal_mean_deg[RIFT_S_HANDS_MIRROR_X] < 0.1);
	CHECK(cmp.imu_difference_m[RIFT_S_HANDS_MIRROR_X] < 1e-6);
	CHECK(cmp.position_rms_m[RIFT_S_HANDS_IDENTICAL] > 0.005);
	// A right model that is a copy of the left (no mirroring) is detected.
	rift_s_controller_compare_hands(&left.calibration, &unmirrored.calibration, &cmp);
	CHECK(cmp.best == RIFT_S_HANDS_IDENTICAL);
	CHECK(cmp.imu_difference_m[RIFT_S_HANDS_MIRROR_X] > 0.01);
}

TEST_CASE("Rift S left and right LED models keep firmware identity, unit normals and the IMU origin")
{
	FirmwareModel left_fw(false), right_fw(true);
	rift_s_controller left = {}, right = {};
	left.base.device_type = XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER;
	right.base.device_type = XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;
	for (auto *c : {&left, &right}) {
		os_mutex_init(&c->mutex);
		c->have_calibration = true;
		c->P_device_imu = XRT_POSE_IDENTITY;
	}
	left.calibration = left_fw.calibration;
	right.calibration = right_fw.calibration;
	for (auto *c : {&left, &right}) {
		c->P_device_imu.position = c->calibration.imu_position;
		math_pose_invert(&c->P_device_imu, &c->P_imu_device);
	}
	t_constellation_led_model lm{}, rm{};
	REQUIRE(rift_s_controller_get_led_model(&left.base, &lm));
	REQUIRE(rift_s_controller_get_led_model(&right.base, &rm));
	CHECK(lm.id == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER);
	CHECK(rm.id == XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER);
	CHECK(lm.id != rm.id);
	REQUIRE(lm.num_leds == rm.num_leds);
	for (unsigned i = 0; i < lm.num_leds; i++) {
		const auto &l = lm.leds[i], &r = rm.leds[i];
		CHECK(l.id == i);
		CHECK(r.id == i);
		CHECK(m_vec3_len(l.dir) == Catch::Approx(1).margin(1e-5));
		CHECK(m_vec3_len(r.dir) == Catch::Approx(1).margin(1e-5));
		// IMU-relative, then OpenCV flip (x, -y, -z): the mirror stays on X for both hands.
		const auto &fw = left_fw.leds[i];
		CHECK(l.pos.x == Catch::Approx(fw.pos.x - left_fw.calibration.imu_position.x));
		CHECK(l.pos.y == Catch::Approx(-(fw.pos.y - left_fw.calibration.imu_position.y)));
		CHECK(l.pos.z == Catch::Approx(-(fw.pos.z - left_fw.calibration.imu_position.z)));
		CHECK(r.pos.x == Catch::Approx(-l.pos.x).margin(1e-6));
		CHECK(r.pos.y == Catch::Approx(l.pos.y).margin(1e-6));
		CHECK(r.pos.z == Catch::Approx(l.pos.z).margin(1e-6));
		CHECK(r.dir.x == Catch::Approx(-l.dir.x).margin(1e-6));
		CHECK(r.dir.y == Catch::Approx(l.dir.y).margin(1e-6));
	}
	t_constellation_led_model_clear(&lm);
	t_constellation_led_model_clear(&rm);
	for (auto *c : {&left, &right})
		os_mutex_destroy(&c->mutex);
}

namespace {
struct PublishedPoseController
{
	rift_s_controller ctrl = {};
	explicit PublishedPoseController(bool right)
	{
		ctrl.base.device_type =
		    right ? XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER : XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER;
		ctrl.gravity_initialized = true;
		ctrl.clock_stable_samples = 32;
		ctrl.P_device_imu = XRT_POSE_IDENTITY;
		ctrl.P_device_imu.position = FirmwareModel(right).calibration.imu_position;
		math_pose_invert(&ctrl.P_device_imu, &ctrl.P_imu_device);
		rift_s_controller_pose_frames(!right, &ctrl.P_device_aim, &ctrl.P_device_grip);
		os_mutex_init(&ctrl.mutex);
	}
	~PublishedPoseController()
	{
		if (ctrl.imu_clock != nullptr)
			m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
		os_mutex_destroy(&ctrl.mutex);
	}
	void
	measured(xrt_quat orientation, xrt_vec3 imu_position, timepoint_ns ts)
	{
		ctrl.pose = {orientation, imu_position};
		ctrl.fusion.rot = orientation;
		ctrl.last_imu_capture_ns = ctrl.last_fix_ns = ts;
		ctrl.last_tracked_pose = ctrl.pose;
		ctrl.last_tracked_pose_ts = ts;
		ctrl.track_vision_updates = ctrl.track_reliable_updates = 10;
		const xrt_vec3 variance = {1e-5f, 1e-5f, 1e-5f};
		rift_s_position_filter_reset(&ctrl.position_filter);
		rift_s_position_filter_fix(&ctrl.position_filter, ts, &imu_position, &variance, 0.15f);
		REQUIRE(rift_s_controller_record_attitude(&ctrl, ts, ts));
	}
	xrt_space_relation
	query(xrt_input_name input, timepoint_ns ts)
	{
		xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
		REQUIRE(rift_s_controller_get_tracked_pose(&ctrl.base, input, ts, &rel) == XRT_SUCCESS);
		return rel;
	}
};

// Independent expectations from the native Oculus driver constants and model components.
xrt_vec3
rotate_x(float deg, xrt_vec3 v)
{
	const float c = cosf(DEG_TO_RAD(deg)), s = sinf(DEG_TO_RAD(deg));
	return {v.x, c * v.y - s * v.z, s * v.y + c * v.z};
}

// Native driver quaternion (x,w)=(0.3371,0.94147), normalized before rigid-pose math.
constexpr float native_model_pitch = 39.40053259f;
xrt_vec3
model_to_device(bool right, xrt_vec3 model)
{
	const xrt_vec3 origin = {right ? 0.007f : -0.007f, 0.036124f, -0.037617f};
	return m_vec3_add(rotate_x(native_model_pitch, model), origin);
}

xrt_vec3
grip_in_device(bool)
{
	// LibOVR LCON grip, independent of the render model and the implementation.
	return {0, -0.03f, 0.04f};
}

void
check_vec(xrt_vec3 actual, xrt_vec3 expected)
{
	CHECK(actual.x == Catch::Approx(expected.x).margin(2e-6));
	CHECK(actual.y == Catch::Approx(expected.y).margin(2e-6));
	CHECK(actual.z == Catch::Approx(expected.z).margin(2e-6));
}
} // namespace

TEST_CASE("Rift S loss hold and reacquire keep the published hand valid and continuous")
{
	const timepoint_ns ms = U_TIME_1MS_IN_NS;
	const xrt_vec3 here = {0.3f, 1.4f, -0.4f}, error = {4, -6, 2};
	for (bool right : {false, true}) {
		PublishedPoseController fixture(right);
		auto &ctrl = fixture.ctrl;
		const timepoint_ns start = os_monotonic_get_ns() - 2000 * ms;
		fixture.measured(XRT_QUAT_IDENTITY, here, start);
		ctrl.track_vision_updates = ctrl.track_reliable_updates = ctrl.position_filter.track_updates = 30;
		// Optical loss while the IMU keeps rotating the wrist. Its acceleration error must
		// never move the IMU origin farther than the existing 10 cm coast bound.
		for (timepoint_ns t = start + 2 * ms; t <= start + 2000 * ms; t += 2 * ms) {
			const xrt_vec3 up = {0, 1, 0};
			math_quat_from_angle_vector(float(t - start) / U_TIME_1S_IN_NS, &up, &ctrl.pose.orientation);
			ctrl.fusion.rot = ctrl.pose.orientation;
			ctrl.last_imu_capture_ns = t;
			REQUIRE(rift_s_controller_record_attitude(&ctrl, t, t));
			ctrl.position_filter.coasting = t - start > 120 * ms;
			rift_s_position_filter_imu(&ctrl.position_filter, t, &error);
			for (auto input : {XRT_INPUT_TOUCH_AIM_POSE, XRT_INPUT_TOUCH_GRIP_POSE}) {
				auto rel = fixture.query(input, t);
				REQUIRE((rel.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0);
				if (t - start > 500 * ms) {
					CHECK_FALSE((rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) !=
					            0);
					CHECK_FALSE(
					    (rel.relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT) != 0);
				}
				const auto &offset =
				    input == XRT_INPUT_TOUCH_AIM_POSE ? ctrl.P_device_aim : ctrl.P_device_grip;
				xrt_pose imu_device_offset, expected;
				math_pose_transform(&ctrl.P_imu_device, &offset, &imu_device_offset);
				const xrt_pose anchor = {ctrl.pose.orientation, here};
				math_pose_transform(&anchor, &imu_device_offset, &expected);
				CHECK(m_vec3_len(m_vec3_sub(rel.pose.position, expected.position)) <= 0.10001f);
				CHECK(rel.pose.orientation.y == Catch::Approx(expected.orientation.y).margin(2e-6));
#ifdef TEST_RIFT_S_STEAMVR_POSE
				const auto model_offset =
				    rift_s_touch_grip_to_model(right ? XRT_HAND_RIGHT : XRT_HAND_LEFT);
				if (input == XRT_INPUT_TOUCH_GRIP_POSE)
					math_pose_transform(&rel.pose, &model_offset, &rel.pose);
				vr::DriverPose_t pose{};
				pose.poseIsValid = true;
				pose.result = vr::TrackingResult_Running_OK;
				apply_pose(&rel, &pose, XRT_SPACE_RELATION_POSITION_VALID_BIT, true);
				REQUIRE(pose.poseIsValid);
				CHECK(pose.result == vr::TrackingResult_Running_OK);
				CHECK(pose.vecPosition[1] > 1.0);
#endif
			}
		}
		REQUIRE(ctrl.position_filter.frozen);
		// The first two fits remain candidates. The third resumes the unchanged optical
		// filter, while only the displayed translation blends out over 100 ms.
		const auto held = fixture.query(XRT_INPUT_TOUCH_GRIP_POSE, os_monotonic_get_ns());
		t_constellation_pose_observation obs{};
		obs.pose = {ctrl.pose.orientation, {-0.2f, 1.5f, -0.5f}};
		obs.position_covariance[0] = obs.position_covariance[4] = obs.position_covariance[8] = 1e-5f;
		for (int i = 0; i < 3; ++i) {
			obs.capture_ns = start + (1894 + 33 * i) * ms;
			rift_s_controller_push_pose_observation(&ctrl.base, &obs);
			CHECK(ctrl.position_filter.frozen == (i < 2));
			const auto rel = fixture.query(XRT_INPUT_TOUCH_GRIP_POSE, os_monotonic_get_ns());
			CHECK(m_vec3_len(m_vec3_sub(rel.pose.position, held.pose.position)) < 0.01f);
		}
		REQUIRE(ctrl.diagnostics.optical_accepted == 1);
		const timepoint_ns recovered = os_monotonic_get_ns();
		auto previous = fixture.query(XRT_INPUT_TOUCH_GRIP_POSE, recovered);
		for (int i = 1; i <= 12; ++i) {
			auto rel = fixture.query(XRT_INPUT_TOUCH_GRIP_POSE, recovered + i * 10 * ms);
			CHECK(m_vec3_len(m_vec3_sub(rel.pose.position, previous.pose.position)) < 0.09f);
			REQUIRE((rel.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0);
			previous = rel;
		}
		// At the end the presentation offset is exactly gone, including future queries.
		xrt_vec3 p, v;
		rift_s_position_filter_predict(&ctrl.position_filter, recovered + 120 * ms, 100 * ms, &p, &v);
		xrt_pose imu_device_offset, expected;
		math_pose_transform(&ctrl.P_imu_device, &ctrl.P_device_grip, &imu_device_offset);
		const xrt_pose target = {ctrl.pose.orientation, p};
		math_pose_transform(&target, &imu_device_offset, &expected);
		check_vec(previous.pose.position, expected.position);
		// Presentation queries never replace the capture-time optical association prior.
		const auto prior = fixture.query(XRT_INPUT_GENERIC_TRACKER_POSE, obs.capture_ns);
		check_vec(prior.pose.position, obs.pose.position);
	}
}

TEST_CASE("Rift S grip stays at its center during rotation for either hand")
{
	const xrt_vec3 center = {0.3f, 1.2f, -0.5f};
	const xrt_vec3 axes[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	for (bool right : {false, true}) {
		PublishedPoseController fixture(right);
		auto &ctrl = fixture.ctrl;
		timepoint_ns ts = 1000000000;
		for (const auto &axis : axes) {
			for (float angle : {0.f, 30.f, 90.f, 170.f}) {
				CAPTURE(right, angle, axis.x, axis.y, axis.z);
				xrt_quat rotation;
				math_quat_from_angle_vector(DEG_TO_RAD(angle), &axis, &rotation);
				// Independently place the measured IMU around the render-model grip center.
				const xrt_vec3 lever = m_vec3_sub(grip_in_device(right), ctrl.P_device_imu.position);
				xrt_vec3 rotated;
				math_quat_rotate_vec3(&rotation, &lever, &rotated);
				fixture.measured(rotation, m_vec3_sub(center, rotated), ts);
				auto grip = fixture.query(XRT_INPUT_TOUCH_GRIP_POSE, ts);
				CHECK((grip.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0);
				check_vec(grip.pose.position, center);
				// Grip +Y and -Z: the native driver root plus the model grip component.
				for (auto [local, hand] :
				     {std::pair{xrt_vec3{0, 1, 0}, rotate_x(native_model_pitch + 20.6f, {0, 1, 0})},
				      {xrt_vec3{0, 0, -1}, rotate_x(native_model_pitch + 20.6f, {0, 0, -1})}}) {
					xrt_vec3 actual, expected;
					math_quat_rotate_vec3(&grip.pose.orientation, &local, &actual);
					math_quat_rotate_vec3(&rotation, &hand, &expected);
					check_vec(actual, expected);
				}
				ts += 2000000;
			}
		}
	}
}

TEST_CASE("Rift S aim and grip mirror between hands and keep the Windows grip-to-aim offset")
{
	PublishedPoseController left(false), right(true);
	const timepoint_ns ts = 1000000000;
	const xrt_vec3 imu_left = {-0.2f, 1.2f, -0.5f}, imu_right = {0.2f, 1.2f, -0.5f};
	left.measured(XRT_QUAT_IDENTITY, imu_left, ts);
	right.measured(XRT_QUAT_IDENTITY, imu_right, ts);
	for (auto input : {XRT_INPUT_TOUCH_AIM_POSE, XRT_INPUT_TOUCH_GRIP_POSE}) {
		auto l = left.query(input, ts), r = right.query(input, ts);
		check_vec(r.pose.position, {-l.pose.position.x, l.pose.position.y, l.pose.position.z});
		CHECK(r.pose.orientation.x == Catch::Approx(l.pose.orientation.x));
		CHECK(r.pose.orientation.y == Catch::Approx(l.pose.orientation.y));
		CHECK(r.pose.orientation.z == Catch::Approx(l.pose.orientation.z));
		CHECK(r.pose.orientation.w == Catch::Approx(l.pose.orientation.w));
	}
	for (auto *fixture : {&left, &right}) {
		auto aim = fixture->query(XRT_INPUT_TOUCH_AIM_POSE, ts);
		auto grip = fixture->query(XRT_INPUT_TOUCH_GRIP_POSE, ts);
		// Aim is forward in the native hand frame, within the rounded model constants.
		xrt_vec3 forward;
		const xrt_vec3 minus_z = {0, 0, -1};
		math_quat_rotate_vec3(&aim.pose.orientation, &minus_z, &forward);
		check_vec(forward, rotate_x(native_model_pitch - 39.4f, minus_z));
		// Grip in the aim frame is the Windows LCON pair: grip (+60 deg X, (0, -0.03, +0.04)) and
		// aim (identity, (0, 0, -0.055)) in the Windows hand frame.
		xrt_pose aim_inv, aim_grip;
		math_pose_invert(&aim.pose, &aim_inv);
		math_pose_transform(&aim_inv, &grip.pose, &aim_grip);
		CHECK(aim_grip.position.x == Catch::Approx(0).margin(1e-4));
		CHECK(aim_grip.position.y == Catch::Approx(-0.03).margin(1e-3));
		CHECK(aim_grip.position.z == Catch::Approx(0.095).margin(1e-3));
		xrt_vec3 grip_minus_z;
		math_quat_rotate_vec3(&aim_grip.orientation, &minus_z, &grip_minus_z);
		check_vec(grip_minus_z, rotate_x(60.0f, minus_z));
	}
}

TEST_CASE("Rift S published frames cannot change capture-time tracking or prediction state")
{
	for (bool right : {false, true}) {
		PublishedPoseController fixture(right);
		auto &ctrl = fixture.ctrl;
		timepoint_ns ts = 1000000000;
		const xrt_vec3 axis = {1, 0, 0};
		for (float angle : {0.f, 45.f, 90.f, 135.f, 179.f}) {
			CAPTURE(right, angle);
			xrt_quat rotation;
			math_quat_from_angle_vector(DEG_TO_RAD(angle), &axis, &rotation);
			ctrl.fusion.last.gyro = {0.5f, -0.3f, 0.2f};
			fixture.measured(rotation, {0.2f, 1.3f, -0.4f}, ts);
			const auto before = fixture.query(XRT_INPUT_GENERIC_TRACKER_POSE, ts);
			const auto fusion = ctrl.fusion;
			const auto filter = ctrl.position_filter;
			const auto optical_pose = ctrl.last_tracked_pose;
			const auto history =
			    ctrl.attitude_history[ctrl.attitude_history_start + ctrl.attitude_history_count - 1];
			for (auto input : {XRT_INPUT_TOUCH_AIM_POSE, XRT_INPUT_TOUCH_GRIP_POSE}) {
				fixture.query(input, ts);
				fixture.query(input, ts + 20000000); // Render-time extrapolation must also be isolated.
			}
			// Even arbitrary publication adjustments cannot alter constellation's prior.
			ctrl.P_device_aim.position = {0.2f, 0.4f, 0.8f};
			ctrl.P_device_grip.orientation = {0, 1, 0, 0};
			fixture.query(XRT_INPUT_TOUCH_AIM_POSE, ts + 20000000);
			fixture.query(XRT_INPUT_TOUCH_GRIP_POSE, ts + 20000000);
			const auto after = fixture.query(XRT_INPUT_GENERIC_TRACKER_POSE, ts);
			CHECK(std::memcmp(&before, &after, sizeof(before)) == 0);
			CHECK(std::memcmp(&fusion, &ctrl.fusion, sizeof(fusion)) == 0);
			CHECK(std::memcmp(&filter, &ctrl.position_filter, sizeof(filter)) == 0);
			CHECK(std::memcmp(&optical_pose, &ctrl.last_tracked_pose, sizeof(optical_pose)) == 0);
			CHECK(std::memcmp(
			          &history,
			          &ctrl.attitude_history[ctrl.attitude_history_start + ctrl.attitude_history_count - 1],
			          sizeof(history)) == 0);
			CHECK(ctrl.last_tracked_pose_ts == ts);
			check_vec(after.pose.position, ctrl.last_tracked_pose.position);
			rift_s_controller_pose_frames(!right, &ctrl.P_device_aim, &ctrl.P_device_grip);
			ts += 2000000;
		}
	}
}

TEST_CASE("Rift S grip lever arm cancels IMU velocity for a twist about its center")
{
	for (bool right : {false, true}) {
		PublishedPoseController fixture(right);
		auto &ctrl = fixture.ctrl;
		const xrt_vec3 axis = {0, 1, 0};
		xrt_quat rotation;
		math_quat_from_angle_vector(DEG_TO_RAD(120.f), &axis, &rotation);
		const xrt_vec3 lever = m_vec3_sub(grip_in_device(right), ctrl.P_device_imu.position);
		xrt_vec3 rotated, omega;
		ctrl.fusion.last.gyro = {0.7f, -0.4f, 0.2f};
		math_quat_rotate_vec3(&rotation, &lever, &rotated);
		math_quat_rotate_vec3(&rotation, &ctrl.fusion.last.gyro, &omega);
		fixture.measured(rotation, m_vec3_sub(xrt_vec3{0.3f, 1.2f, -0.5f}, rotated), 1000000000);
		const xrt_vec3 imu_radius = m_vec3_mul_scalar(rotated, -1);
		xrt_vec3 imu_velocity;
		math_vec3_cross(&omega, &imu_radius, &imu_velocity);
		ctrl.position_filter.state.x[0][1] = imu_velocity.x;
		ctrl.position_filter.state.x[1][1] = imu_velocity.y;
		ctrl.position_filter.state.x[2][1] = imu_velocity.z;
		const auto grip = fixture.query(XRT_INPUT_TOUCH_GRIP_POSE, 1000000000);
		CHECK((grip.relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT) != 0);
		check_vec(grip.linear_velocity, {0, 0, 0});
		check_vec(grip.angular_velocity, omega);
	}
}

namespace {
xrt_quat
quat_from_axis_deg(xrt_vec3 axis, float deg)
{
	math_vec3_normalize(&axis);
	xrt_quat q;
	math_quat_from_angle_vector(DEG_TO_RAD(deg), &axis, &q);
	return q;
}

// Device-frame up of an attitude (IMU to world).
xrt_vec3
device_up(const xrt_quat &attitude)
{
	xrt_quat inverse;
	math_quat_invert(&attitude, &inverse);
	const xrt_vec3 up = {0, 1, 0};
	xrt_vec3 out;
	math_quat_rotate_vec3(&inverse, &up, &out);
	return out;
}

float
tilt_error_deg(const xrt_quat &a, const xrt_quat &b)
{
	return RAD_TO_DEG(m_vec3_angle(device_up(a), device_up(b)));
}
} // namespace

TEST_CASE("Rift S optical tilt correction levels the IMU tilt and leaves heading alone")
{
	const xrt_quat imu = quat_from_axis_deg({0.3f, 1, -0.2f}, 70);
	xrt_quat optical;
	const xrt_quat tilt = quat_from_axis_deg({1, 0, 0.4f}, 9);
	math_quat_rotate(&tilt, &imu, &optical);
	xrt_quat full, half, corrected;
	float angle = rift_s_optical_tilt_correction(&imu, &optical, 1.0f, &full);
	CHECK(RAD_TO_DEG(angle) == Catch::Approx(tilt_error_deg(imu, optical)).margin(1e-3));
	// The correction axis is horizontal: a pure world tilt, no heading change.
	xrt_vec3 axis = {full.x, full.y, full.z};
	CHECK(fabsf(axis.y) < 1e-6f);
	math_quat_rotate(&full, &imu, &corrected);
	CHECK(tilt_error_deg(corrected, optical) < 1e-3f);
	rift_s_optical_tilt_correction(&imu, &optical, 0.5f, &half);
	math_quat_rotate(&half, &imu, &corrected);
	CHECK(tilt_error_deg(corrected, optical) == Catch::Approx(tilt_error_deg(imu, optical) / 2).margin(1e-2));
	// Equal tilt: nothing to do.
	rift_s_optical_tilt_correction(&imu, &imu, 1.0f, &full);
	CHECK(full.w == Catch::Approx(1));
}

TEST_CASE("Rift S attitude bias recovers the right controller's offset from levelled rest windows")
{
	// r14 worn log: the right accelerometer needed -0.058, -1.998, 0.018 m/s^2.
	const xrt_vec3 truth = {-0.058f, -1.998f, 0.018f};
	std::mt19937 rng(7);
	std::normal_distribution<float> noise_deg(0, 2.5f); // single optical fits tilt by 2-3 degrees
	std::uniform_real_distribution<float> any(-1, 1);
	rift_s_attitude_bias b = {};
	// All windows in one orientation: no spread needed once the attitude is known.
	const xrt_quat held = quat_from_axis_deg({0.2f, 0.5f, 1}, 40);
	for (int i = 0; i < RIFT_S_ATTITUDE_BIAS_SAMPLES; i++) {
		const xrt_vec3 g = {0, (float)MATH_GRAVITY_M_S2, 0};
		xrt_quat inverse;
		math_quat_invert(&held, &inverse);
		xrt_vec3 reading;
		math_quat_rotate_vec3(&inverse, &g, &reading);
		reading = m_vec3_add(reading, truth);
		// The levelled attitude carries optical tilt noise.
		xrt_quat noisy, wobble = quat_from_axis_deg({any(rng), 0, any(rng)}, noise_deg(rng));
		math_quat_rotate(&wobble, &held, &noisy);
		rift_s_attitude_bias_add(&b, &reading, &noisy);
		if (i + 1 < RIFT_S_ATTITUDE_BIAS_MIN_SAMPLES)
			CHECK_FALSE(b.valid);
	}
	REQUIRE(b.valid);
	CHECK(m_vec3_len(m_vec3_sub(b.bias, truth)) < 0.25f);
	// Inconsistent windows (e.g. moving) are not accepted.
	rift_s_attitude_bias bad = {};
	for (int i = 0; i < 16; i++) {
		xrt_vec3 reading = {any(rng) * 4, 9.8f + any(rng) * 4, any(rng) * 4};
		rift_s_attitude_bias_add(&bad, &reading, &held);
	}
	CHECK_FALSE(bad.valid);
}

namespace {
struct FusionRig
{
	rift_s_controller ctrl = {};
	xrt_vec3 bias;
	xrt_quat truth = XRT_QUAT_IDENTITY;
	timepoint_ns t = 1000000000;
	int sample = 0;

	explicit FusionRig(xrt_vec3 accel_bias) : bias(accel_bias)
	{
		m_imu_3dof_init(&ctrl.fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
		ctrl.device_type = RIFT_S_DEVICE_RIGHT_CONTROLLER;
		os_mutex_init(&ctrl.mutex);
	}
	~FusionRig()
	{
		if (ctrl.imu_clock != nullptr)
			m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
		m_imu_3dof_close(&ctrl.fusion);
		os_mutex_destroy(&ctrl.mutex);
	}
	// 2 ms IMU samples; @p omega is the true body rate (rad/s, IMU frame). Every 16th sample an
	// optical fit of the true attitude arrives when @p optical.
	void
	run(int samples, xrt_vec3 omega, bool optical)
	{
		for (int i = 0; i < samples; i++, sample++, t += 2000000) {
			xrt_quat step;
			xrt_vec3 dtheta = m_vec3_mul_scalar(omega, 0.002f);
			float a = m_vec3_len(dtheta);
			step = a > 0 ? quat_from_axis_deg(dtheta, RAD_TO_DEG(a)) : xrt_quat XRT_QUAT_IDENTITY;
			math_quat_rotate(&truth, &step, &truth); // body-frame rate
			xrt_quat inverse;
			math_quat_invert(&truth, &inverse);
			const xrt_vec3 g = {0, (float)MATH_GRAVITY_M_S2, 0};
			math_quat_rotate_vec3(&inverse, &g, &ctrl.accel);
			ctrl.accel = m_vec3_add(ctrl.accel, bias);
			ctrl.gyro = omega;
			rift_s_controller_update_fusion(&ctrl, t);
			if (rift_s_controller_record_attitude(&ctrl, t, t))
				ctrl.last_imu_capture_ns = t;
			if (optical && sample % 16 == 0 && ctrl.gravity_initialized) {
				t_constellation_pose_observation obs = {};
				obs.capture_ns = t;
				obs.pose.orientation = truth;
				obs.pose.position = {0.2f, 1.2f, -0.4f};
				obs.orientation_observed = true;
				obs.inliers = obs.distinct_leds = 6;
				obs.information_rank = 4;
				obs.yaw_variance = NAN;
				obs.hypothesis = "test";
				for (float &c : obs.position_covariance)
					c = NAN;
				rift_s_controller_push_pose_observation(&ctrl.base, &obs);
			}
		}
	}
	float
	tilt_error() const
	{
		return tilt_error_deg(ctrl.fusion.rot, truth);
	}
};
} // namespace

TEST_CASE("Rift S offset accelerometer levels to optical tilt, learns its offset, then levels to gravity")
{
	// r14 worn log, right controller: rest readings 7.9..9.4 m/s^2 from a -2 m/s^2 offset. Before
	// fix-r15 the fusion levelled to that reading, up to ~11 degrees off, and applied the stored
	// error quickly while rotating: tilt corrections at the optical solver's 6 degree limit.
	FusionRig rig({-0.058f, -1.998f, 0.018f});
	rig.truth = quat_from_axis_deg({1, 0, 0.3f}, 35);
	const xrt_vec3 still = {0, 0, 0};
	rig.run(200, still, false);
	REQUIRE(rig.ctrl.gravity_initialized);
	CHECK(rig.ctrl.accel_offset);
	CHECK((rig.ctrl.fusion.flags & M_IMU_3DOF_USE_GRAVITY_DUR_20MS) == 0);
	const float startup_error = rig.tilt_error();
	CHECK(startup_error > 5.0f); // the offset reading tilts the startup attitude
	// Turning without optics: gyro only, the startup error neither grows nor jumps.
	rig.run(250, {0.0f, 1.5f, 0.6f}, false);
	rig.run(100, still, false);
	CHECK(rig.tilt_error() < startup_error + 0.5f);
	// Optical fits level the attitude within a few seconds...
	rig.run(1500, still, true);
	CHECK(rig.tilt_error() < 1.0f);
	CHECK(rig.ctrl.optical_tilt_updates >= 30);
	// ...after which rest windows give the offset and gravity levelling returns.
	rig.run(2000, still, true);
	REQUIRE(rig.ctrl.attitude_bias.valid);
	CHECK_FALSE(rig.ctrl.accel_offset);
	CHECK((rig.ctrl.fusion.flags & M_IMU_3DOF_USE_GRAVITY_DUR_20MS) != 0);
	CHECK(m_vec3_len(m_vec3_sub(rig.ctrl.attitude_bias.bias, rig.bias)) < 0.2f);
	// Now rotations and new rest orientations keep the attitude level without optics.
	for (xrt_vec3 omega : {xrt_vec3{2.0f, 0.5f, 0}, xrt_vec3{0, -1.5f, 1.2f}, xrt_vec3{-1.0f, 0, -2.0f}}) {
		rig.run(300, omega, false);
		rig.run(500, still, false);
		CHECK(rig.tilt_error() < 1.5f);
	}
}

TEST_CASE("Rift S accurate accelerometer keeps gravity levelling with optical tilt")
{
	// r14 worn log, left controller: startup 9.88 m/s^2, bias about 0.1 m/s^2.
	FusionRig rig({-0.016f, 0.104f, -0.064f});
	rig.truth = quat_from_axis_deg({0.4f, 0, 1}, 25);
	const xrt_vec3 still = {0, 0, 0};
	rig.run(200, still, false);
	REQUIRE(rig.ctrl.gravity_initialized);
	CHECK_FALSE(rig.ctrl.accel_offset);
	CHECK((rig.ctrl.fusion.flags & M_IMU_3DOF_USE_GRAVITY_DUR_20MS) != 0);
	rig.run(500, still, true);
	CHECK(rig.ctrl.optical_tilt_updates > 0);
	CHECK(rig.tilt_error() < 1.0f);
	// Acceleration during motion is still rejected by the fusion gate.
	rig.ctrl.accel = m_vec3_mul_scalar(rig.ctrl.accel, 2);
	rift_s_controller_update_fusion(&rig.ctrl, rig.t);
	CHECK(rig.ctrl.fusion.grav.is_accel);
}

TEST_CASE("Rift S capture attitude diagnostics distinguish bounds tilt and radio gaps")
{
	rift_s_controller ctrl{};
	rift_s_attitude_sample sample{};
	CHECK_FALSE(rift_s_controller_get_attitude(&ctrl, 1, &sample));
	CHECK(ctrl.diagnostics.last_attitude_reject == RIFT_S_ATTITUDE_EMPTY);
	CHECK_FALSE(rift_s_controller_get_attitude(&ctrl, 0, &sample));
	CHECK(ctrl.diagnostics.last_attitude_reject == RIFT_S_ATTITUDE_INVALID_TIME);
	ctrl.attitude_history_count = 2;
	ctrl.attitude_history[0] = {1000000000, 1000000000, 1000000000, {0, 0, 0, 1}, {0, 0, 0}, true};
	ctrl.attitude_history[1] = {1002000000, 1002000000, 1002000000, {0, 0, 0, 1}, {0, 0, 0}, true};
	CHECK_FALSE(rift_s_controller_get_attitude(&ctrl, 999999999, &sample));
	CHECK(ctrl.diagnostics.last_attitude_reject == RIFT_S_ATTITUDE_TOO_OLD);
	CHECK_FALSE(rift_s_controller_get_attitude(&ctrl, 1022000001, &sample));
	CHECK(ctrl.diagnostics.last_attitude_reject == RIFT_S_ATTITUDE_TOO_NEW);
	CHECK(rift_s_controller_get_attitude(&ctrl, 1001000000, &sample));
	CHECK(sample.capture_ns == 1001000000);
	ctrl.attitude_history[1].tilt_valid = false;
	CHECK_FALSE(rift_s_controller_get_attitude(&ctrl, 1001000000, &sample));
	CHECK(ctrl.diagnostics.last_attitude_reject == RIFT_S_ATTITUDE_TILT_UNREADY);
	CHECK_FALSE(rift_s_controller_get_attitude(&ctrl, 1002000000, &sample));
	CHECK(ctrl.diagnostics.last_attitude_reject == RIFT_S_ATTITUDE_TILT_UNREADY);
	ctrl.attitude_history[1].tilt_valid = true;
	ctrl.attitude_history[1].capture_ns = 1012000000;
	CHECK_FALSE(rift_s_controller_get_attitude(&ctrl, 1001000000, &sample));
	CHECK(ctrl.diagnostics.last_attitude_reject == RIFT_S_ATTITUDE_RADIO_GAP);
	CHECK(ctrl.diagnostics.last_attitude_reject_ns == 1001000000);
	CHECK(ctrl.diagnostics.attitude_rejected[RIFT_S_ATTITUDE_TILT_UNREADY] == 2);
}

TEST_CASE("Rift S moving left hand acquires optical tracking during its first camera exposures")
{
	FusionRig rig({-.016f, .104f, -.064f});
	rig.ctrl.base.device_type = XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER;
	rig.ctrl.update_yaw_from_optical = true;
	rig.truth = quat_from_axis_deg({1, 0, .3f}, 65);
	// Unlike the old stationary gate, bootstrap succeeds above 0.1 rad/s.
	rig.run(160, {.8f, -.6f, .4f}, true);
	REQUIRE(rig.ctrl.gravity_initialized);
	CHECK(rig.ctrl.gravity_moving);
	CHECK(rig.ctrl.diagnostics.optical_accepted >= 2);
	CHECK(rig.tilt_error() < 2);
	xrt_space_relation relation{};
	CHECK(rift_s_controller_get_tracked_pose(&rig.ctrl.base, XRT_INPUT_GENERIC_TRACKER_POSE, rig.t - 33307700,
	                                         &relation) == XRT_SUCCESS);
	CHECK(rig.ctrl.diagnostics.reject_clock == 0);
	CHECK(rig.ctrl.fusion.gyro_bias.value.x == 0); // actual motion must never become gyro bias
}

TEST_CASE("Rift S r13 left clock timing retains capture attitude across late resends")
{
	FusionRig rig({0, 0, 0});
	rig.run(160, {.4f, .2f, 0}, false);
	// r13 left device clock was ahead of host by 1459.856 s. The 24 backwards
	// reports were isolated resends by 1997/1998 us, not clock discontinuities.
	if (rig.ctrl.imu_clock)
		m_clock_windowed_skew_tracker_destroy(rig.ctrl.imu_clock);
	rig.ctrl.imu_clock = nullptr;
	rig.ctrl.attitude_history_count = rig.ctrl.attitude_history_start = 0;
	rig.ctrl.clock_stable_samples = 0;
	rig.ctrl.imu_to_host_ns = 0;
	const timepoint_ns host = 381548007979LL, offset = 1459857273021LL;
	for (int i = 0; i < 1024; i++) {
		timepoint_ns capture = host + i * 1997000LL;
		REQUIRE(rift_s_controller_record_attitude(&rig.ctrl, capture + offset, capture + (i % 3) * 300000));
		if (i % 42 == 0 && i > 0)
			CHECK_FALSE(rift_s_controller_record_attitude(&rig.ctrl, capture + offset - 1997000,
			                                              capture + 2000000));
	}
	CHECK(rig.ctrl.diagnostics.reject_clock_by[RIFT_S_CLOCK_HISTORY_ORDER] == 24);
	CHECK(rig.ctrl.diagnostics.reject_clock_by[RIFT_S_CLOCK_DISCONTINUITY] == 0);
	const timepoint_ns camera_capture = host + 1023 * 1997000LL - 33307700;
	rift_s_attitude_sample at_camera{};
	REQUIRE(rift_s_controller_get_attitude(&rig.ctrl, camera_capture, &at_camera));
	t_constellation_pose_observation obs{};
	obs.capture_ns = camera_capture;
	obs.pose = {at_camera.orientation, {.2f, 1.2f, -.4f}};
	obs.orientation_observed = true;
	obs.distinct_leds = obs.inliers = 6;
	rift_s_controller_push_pose_observation(&rig.ctrl.base, &obs);
	CHECK(rig.ctrl.diagnostics.optical_accepted == 1);
}

TEST_CASE("HMD capture-time IMU attitude predicts with a world-space gyro")
{
	rift_s_tracker tracker = {};
	os_mutex_init(&tracker.mutex);
	os_mutex_init(&tracker.slam_mutex);
	m_imu_3dof_init(&tracker.fusion.i3dof, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
	m_relation_history_create(&tracker.fusion.history);
	rift_s_slam_guard_init(&tracker.slam_guard);
	tracker.ready_for_data = tracker.have_hw2mono = true;
	tracker.last_frame_time = 1000000000;
	tracker.device_from_imu = tracker.left_cam_from_imu = XRT_POSE_IDENTITY;
	const xrt_vec3 axis = {1, 0, 0};
	math_quat_from_angle_vector(DEG_TO_RAD(65), &axis, &tracker.fusion.i3dof.rot);
	const xrt_vec3 accel = {0, 4.145f, -8.89f}, gyro = {0, 0.5f, 0};
	rift_s_tracker_imu_update(&tracker, 1000000000, &accel, &gyro);
	xrt_space_relation recorded, predicted;
	REQUIRE(m_relation_history_get(tracker.fusion.history, 1000000000, &recorded) !=
	        M_RELATION_HISTORY_RESULT_INVALID);
	REQUIRE(m_relation_history_get(tracker.fusion.history, 1005000000, &predicted) !=
	        M_RELATION_HISTORY_RESULT_INVALID);
	xrt_quat expected;
	math_quat_integrate_velocity(&recorded.pose.orientation, &gyro, 0.005, &expected);
	float dot = std::abs(expected.x * predicted.pose.orientation.x + expected.y * predicted.pose.orientation.y +
	                     expected.z * predicted.pose.orientation.z + expected.w * predicted.pose.orientation.w);
	CHECK(dot > 0.999999f);
	m_relation_history_destroy(&tracker.fusion.history);
	m_imu_3dof_close(&tracker.fusion.i3dof);
	os_mutex_destroy(&tracker.slam_mutex);
	os_mutex_destroy(&tracker.mutex);
}

#ifdef TEST_RIFT_S_STEAMVR_POSE
TEST_CASE("Rift S Touch model and world transforms preserve estimated coordinates and confidence")
{
	for (bool position_valid : {false, true}) {
		xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
		rel.pose = {XRT_QUAT_IDENTITY, {0.4f, 1.2f, -0.6f}};
		rel.relation_flags = static_cast<xrt_space_relation_flags>(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
		    (position_valid ? XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT : 0));
		xrt_pose model{XRT_QUAT_IDENTITY, {0.1f, 0.2f, 0.3f}};
		xrt_pose world{{0, float(std::sqrt(0.5)), 0, float(std::sqrt(0.5))}, {1, 2, 3}};
		transform_rift_s_touch_relation(&rel, &model, &world);
		check_vec(rel.pose.position, {0.7f, 3.4f, 2.5f});
		CHECK(((rel.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0) == position_valid);
		CHECK(((rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0) == position_valid);
		vr::DriverPose_t pose{};
		apply_rift_s_touch_pose(&rel, &pose);
		REQUIRE(pose.poseIsValid);
		CHECK(pose.result == (position_valid ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange));
		CHECK(pose.vecPosition[0] == Catch::Approx(0.7f));
		CHECK(pose.vecPosition[1] == Catch::Approx(3.4f));
		CHECK(pose.vecPosition[2] == Catch::Approx(2.5f));
	}
}

TEST_CASE("Rift S Touch OpenVR status follows orientation and position validity independently")
{
	for (bool connected : {false, true}) {
		for (bool orientation_valid : {false, true}) {
			for (bool position_valid : {false, true}) {
				for (bool tracked : {false, true}) {
					xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
					rel.pose = {XRT_QUAT_IDENTITY, {0.3f, 1.4f, -0.4f}};
					rel.linear_velocity = {1, -2, 3};
					rel.angular_velocity = {0.2f, 0.3f, -0.4f};
					rel.relation_flags = static_cast<xrt_space_relation_flags>(
					    (orientation_valid ? XRT_SPACE_RELATION_ORIENTATION_VALID_BIT : 0) |
					    (position_valid ? XRT_SPACE_RELATION_POSITION_VALID_BIT : 0) |
					    (tracked ? XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
					                   XRT_SPACE_RELATION_POSITION_TRACKED_BIT : 0) |
					    XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT |
					    XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
					vr::DriverPose_t pose{};
					pose.deviceIsConnected = connected;
					pose.poseIsValid = true;
					pose.vecVelocity[0] = 99;
					apply_rift_s_touch_pose(&rel, &pose);
					CAPTURE(connected, orientation_valid, position_valid, tracked);
					CHECK(pose.deviceIsConnected == connected);
					CHECK(pose.poseIsValid == orientation_valid);
					CHECK(pose.willDriftInYaw == !position_valid);
					CHECK(pose.result == (!orientation_valid ? vr::TrackingResult_Uninitialized
					                                      : position_valid ? vr::TrackingResult_Running_OK
					                                                       : vr::TrackingResult_Running_OutOfRange));
					CHECK(pose.vecPosition[0] == Catch::Approx(orientation_valid ? 0.3f : 0));
					CHECK(pose.qRotation.w == Catch::Approx(orientation_valid ? 1 : 0));
					CHECK(pose.vecVelocity[0] == (orientation_valid ? 1 : 0));
					CHECK(pose.vecAngularVelocity[1] == Catch::Approx(orientation_valid ? 0.3f : 0));
					// A reused output must clear invalid derivatives without changing pose status.
					rel.relation_flags = static_cast<xrt_space_relation_flags>(
					    rel.relation_flags & ~(XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT |
					                           XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT));
					apply_rift_s_touch_pose(&rel, &pose);
					CHECK(pose.vecVelocity[0] == 0);
					CHECK(pose.vecAngularVelocity[1] == 0);
				}
			}
		}
	}
}

TEST_CASE("Rift S Touch OpenVR adapter rejects a nonfinite estimate")
{
	xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	rel.pose = XRT_POSE_IDENTITY;
	rel.relation_flags = static_cast<xrt_space_relation_flags>(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                        XRT_SPACE_RELATION_POSITION_VALID_BIT);
	vr::DriverPose_t pose{};
	pose.deviceIsConnected = true;
	rel.pose.position.x = std::numeric_limits<float>::quiet_NaN();
	apply_rift_s_touch_pose(&rel, &pose);
	CHECK_FALSE(pose.poseIsValid);
	CHECK(pose.deviceIsConnected);
	CHECK(pose.result == vr::TrackingResult_Running_OutOfRange);
	CHECK(pose.vecPosition[0] == 0);
}

TEST_CASE("SteamVR held pose clears velocity retained from the last tracked pose")
{
	xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	rel.pose = XRT_POSE_IDENTITY;
	rel.pose.position = {0.3f, 1.4f, -0.4f};
	rel.relation_flags = static_cast<xrt_space_relation_flags>(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT |
	    XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
	rel.linear_velocity = {1, -2, 3};
	rel.angular_velocity = {0.2f, 0.3f, -0.4f};
	vr::DriverPose_t pose{};
	pose.poseIsValid = true;
	pose.result = vr::TrackingResult_Running_OK;
	apply_pose(&rel, &pose, XRT_SPACE_RELATION_POSITION_VALID_BIT, true);
	CHECK(pose.vecVelocity[1] == -2);
	CHECK(pose.vecAngularVelocity[1] == Catch::Approx(0.3f));
	// GetPose reuses this DriverPose every frame. Invalid velocity must clear the old
	// value, while current IMU angular velocity continues to be copied.
	rel.relation_flags =
	    static_cast<xrt_space_relation_flags>(rel.relation_flags & ~XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT);
	apply_pose(&rel, &pose, XRT_SPACE_RELATION_POSITION_VALID_BIT, true);
	REQUIRE(pose.poseIsValid);
	CHECK(pose.result == vr::TrackingResult_Running_OK);
	for (double v : pose.vecVelocity)
		CHECK(v == 0);
	CHECK(pose.vecAngularVelocity[1] == Catch::Approx(0.3f));
	// Invalid or overflowing angular velocity must not retain a prior valid sample either.
	rel.angular_velocity.x = std::numeric_limits<float>::infinity();
	apply_pose(&rel, &pose, XRT_SPACE_RELATION_POSITION_VALID_BIT, true);
	for (double v : pose.vecAngularVelocity)
		CHECK(v == 0);
}

TEST_CASE("Rift S Touch without a position fix rests below the head, never at the floor origin")
{
	// r18: before the first fix SteamVR drew the orientation-only controllers at the tracking
	// origin, under the floor. The resting position follows the head heading but not its pitch.
	const float sizes[] = {1.6f, 1.1f};
	for (float height : sizes) {
		xrt_pose head = {XRT_QUAT_IDENTITY, {0.4f, height, -0.2f}};
		auto left = nominal_hand_position(head, XRT_HAND_LEFT);
		auto right = nominal_hand_position(head, XRT_HAND_RIGHT);
		check_vec(left, {0.2f, height - 0.5f, -0.5f});
		check_vec(right, {0.6f, height - 0.5f, -0.5f});
		// Turned 90 degrees left (facing -X) and looking 80 degrees down.
		xrt_quat yaw, pitch;
		const xrt_vec3 up = {0, 1, 0}, side = {1, 0, 0};
		math_quat_from_angle_vector(DEG_TO_RAD(90.f), &up, &yaw);
		math_quat_from_angle_vector(DEG_TO_RAD(-80.f), &side, &pitch);
		math_quat_rotate(&yaw, &pitch, &head.orientation);
		left = nominal_hand_position(head, XRT_HAND_LEFT);
		check_vec(left, {0.1f, height - 0.5f, 0.0f});
		CHECK(left.y > 0.5f);
	}
}

TEST_CASE("SteamVR pose conversion rejects nonfinite pose components")
{
	xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
	relation.pose = XRT_POSE_IDENTITY;
	relation.pose.position.y = 1.6f;
	relation.relation_flags = static_cast<xrt_space_relation_flags>(XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                                                                XRT_SPACE_RELATION_POSITION_VALID_BIT);
	vr::DriverPose_t pose{};
	pose.poseIsValid = true;
	apply_pose(&relation, &pose, XRT_SPACE_RELATION_POSITION_VALID_BIT);
	CHECK(pose.vecPosition[1] == Catch::Approx(1.6f));
	relation.pose.position.x = std::numeric_limits<float>::quiet_NaN();
	apply_pose(&relation, &pose, XRT_SPACE_RELATION_POSITION_VALID_BIT);
	CHECK_FALSE(pose.poseIsValid);
	CHECK(std::isfinite(pose.vecPosition[0]));
	CHECK(pose.result == vr::TrackingResult_Running_OutOfRange);
}
#endif

TEST_CASE("Rift S unconfirmed position candidates cannot rotate or replace the association prior")
{
	rift_s_controller ctrl = {};
	ctrl.gravity_initialized = ctrl.update_yaw_from_optical = true;
	ctrl.device_type = RIFT_S_DEVICE_RIGHT_CONTROLLER;
	ctrl.fusion.rot = ctrl.pose.orientation = XRT_QUAT_IDENTITY;
	os_mutex_init(&ctrl.mutex);
	rift_s_position_filter_reset(&ctrl.position_filter);
	for (int i = 0; i < 100; ++i) {
		timepoint_ns ns = 1000000000LL + i * 2000000LL;
		REQUIRE(rift_s_controller_record_attitude(&ctrl, ns, ns));
	}
	t_constellation_pose_observation obs = {};
	obs.capture_ns = 1100000000LL;
	obs.pose = XRT_POSE_IDENTITY;
	obs.pose.position = {0.1f, 1.1f, -0.4f};
	obs.orientation_observed = true;
	obs.inliers = obs.distinct_leds = 6;
	obs.position_covariance[0] = obs.position_covariance[4] = obs.position_covariance[8] = 1e-6f;
	obs.hypothesis = "test";
	rift_s_controller_push_pose_observation(&ctrl.base, &obs);
	const xrt_pose trusted = ctrl.last_tracked_pose;
	const timepoint_ns trusted_ns = ctrl.last_tracked_pose_ts;
	const auto tilt_updates = ctrl.optical_tilt_updates;
	const xrt_vec3 yaw_axis = {0, 1, 0};
	obs.pose.position.x += 1;
	math_quat_from_angle_vector(DEG_TO_RAD(30.f), &yaw_axis, &obs.pose.orientation);
	for (int i = 1; i <= 2; ++i) {
		obs.capture_ns = trusted_ns + i * 10000000LL;
		rift_s_controller_push_pose_observation(&ctrl.base, &obs);
		CHECK(ctrl.last_tracked_pose_ts == trusted_ns);
		CHECK(ctrl.last_tracked_pose.position.x == trusted.position.x);
		CHECK(ctrl.fusion.rot.y == Catch::Approx(0).margin(1e-6));
		CHECK(ctrl.optical_tilt_updates == tilt_updates);
		CHECK(ctrl.diagnostics.optical_accepted == 1);
	}
	// A continuous fit at the trusted position clears the rejected candidate.
	obs.capture_ns += 10000000LL;
	obs.pose = trusted;
	rift_s_controller_push_pose_observation(&ctrl.base, &obs);
	CHECK(ctrl.position_filter.candidate_count == 0);
	CHECK(ctrl.last_tracked_pose_ts == obs.capture_ns);
	CHECK(ctrl.diagnostics.optical_accepted == 2);
	if (ctrl.imu_clock)
		m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
	os_mutex_destroy(&ctrl.mutex);
}

#ifdef TEST_RIFT_S_STEAMVR_POSE
TEST_CASE("Rift S complete grip to SteamVR model chain preserves metric position in a rotated world")
{
	const xrt_vec3 world_axis = {0, 1, 0};
	xrt_quat world_rotation;
	math_quat_from_angle_vector(DEG_TO_RAD(73.f), &world_axis, &world_rotation);
	const xrt_pose world = {world_rotation, {1.2f, 0.4f, -2.1f}};
	for (bool right : {false, true}) {
		PublishedPoseController fixture(right);
		for (float angle : {0.f, 90.f}) {
			CAPTURE(right, angle);
			const xrt_vec3 axis = {1, 0, 0};
			xrt_quat local_rotation;
			math_quat_from_angle_vector(DEG_TO_RAD(angle), &axis, &local_rotation);
			xrt_pose local = {local_rotation, {right ? 0.3f : -0.3f, 0.8f, -0.6f}}, measured;
			math_pose_transform(&world, &local, &measured);
			const timepoint_ns ts = 1000000000 + (angle > 0 ? 2000000 : 0);
			fixture.measured(measured.orientation, measured.position, ts);
			const auto imu = fixture.query(XRT_INPUT_GENERIC_TRACKER_POSE, ts);
			check_vec(imu.pose.position, measured.position);
			const auto grip = fixture.query(XRT_INPUT_TOUCH_GRIP_POSE, ts);
			const auto grip_model = rift_s_touch_grip_to_model(right ? XRT_HAND_RIGHT : XRT_HAND_LEFT);
			xrt_space_relation model = grip;
			const xrt_pose origin = XRT_POSE_IDENTITY;
			transform_rift_s_touch_relation(&model, &grip_model, &origin);
			// Independent device-from-model lever arm, minus firmware IMU origin.
			const xrt_vec3 model_in_device = model_to_device(right, {0, 0, 0});
			xrt_vec3 lever = m_vec3_sub(model_in_device, fixture.ctrl.P_device_imu.position), rotated;
			math_quat_rotate_vec3(&measured.orientation, &lever, &rotated);
			const auto expected = m_vec3_add(measured.position, rotated);
			check_vec(model.pose.position, expected);
			const xrt_vec3 forward = {0, 0, -1};
			xrt_vec3 expected_forward, actual_forward;
			const auto device_forward = rotate_x(native_model_pitch, forward);
			math_quat_rotate_vec3(&measured.orientation, &device_forward, &expected_forward);
			math_quat_rotate_vec3(&model.pose.orientation, &forward, &actual_forward);
			check_vec(actual_forward, expected_forward);
			vr::DriverPose_t driver{};
			driver.poseIsValid = true;
			apply_rift_s_touch_pose(&model, &driver);
			REQUIRE(driver.poseIsValid);
			CHECK(driver.vecPosition[0] == Catch::Approx(expected.x).margin(2e-6));
			CHECK(driver.vecPosition[1] == Catch::Approx(expected.y).margin(2e-6));
			CHECK(driver.vecPosition[2] == Catch::Approx(expected.z).margin(2e-6));
		}
	}
}
#endif

TEST_CASE("Rift S controller does not integrate missing radio motion")
{
	rift_s_controller ctrl = {};
	m_imu_3dof_init(&ctrl.fusion, 0);
	ctrl.accel = {0, 9.81f, 0};
	ctrl.gyro = {0, 1, 0};
	for (int i = 0; i < 40; ++i) {
		timepoint_ns t = 1000000000LL + i * 2000000LL;
		rift_s_controller_update_fusion(&ctrl, t);
		REQUIRE(rift_s_controller_record_attitude(&ctrl, t, t));
	}
	const xrt_quat before = ctrl.fusion.rot;
	const xrt_vec3 position = {0.3f, 1.1f, -0.5f}, variance = {1e-5f, 1e-5f, 1e-5f};
	rift_s_position_filter_reset(&ctrl.position_filter);
	rift_s_position_filter_fix(&ctrl.position_filter, 1078000000LL, &position, &variance, 0.15f);
	ctrl.optical_heading_valid = true;
	rift_s_controller_update_fusion(&ctrl, 3000000000LL);
	CHECK(ctrl.fusion.rot.y == Catch::Approx(before.y));
	CHECK(ctrl.fusion.rot.w == Catch::Approx(before.w));
	CHECK(ctrl.attitude_history_count == 0);
	CHECK_FALSE(ctrl.optical_heading_valid);
	CHECK(ctrl.position_filter.frozen);
	REQUIRE(rift_s_controller_record_attitude(&ctrl, 3000000000LL, 3000000000LL));
	rift_s_attitude_sample sample{};
	CHECK_FALSE(rift_s_controller_get_attitude(&ctrl, 2000000000LL, &sample));
	rift_s_controller_update_fusion(&ctrl, 3002000000LL);
	CHECK(ctrl.fusion.rot.y != before.y);
	m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
	m_imu_3dof_close(&ctrl.fusion);
}

TEST_CASE("Rift S controller rejects extreme finite IMU and recovers attitude")
{
	rift_s_controller ctrl = {};
	m_imu_3dof_init(&ctrl.fusion, 0);
	ctrl.accel = {0, 9.81f, 0};
	rift_s_controller_update_fusion(&ctrl, 1000000000LL);
	ctrl.gyro = {1e30f, 0, 0};
	rift_s_controller_update_fusion(&ctrl, 1002000000LL);
	CHECK(ctrl.diagnostics.reject_nonfinite == 1);
	CHECK(ctrl.fusion.last.timestamp_ns == 1000000000LL);
	CHECK(std::isfinite(ctrl.fusion.rot.w));
	ctrl.gyro = {0, 0, 0};
	ctrl.fusion.rot.w = NAN; // Recovery also covers a previously poisoned backend state.
	rift_s_controller_update_fusion(&ctrl, 1004000000LL);
	CHECK(std::isfinite(ctrl.fusion.rot.w));
	REQUIRE(rift_s_controller_record_attitude(&ctrl, 1004000000LL, 1004000000LL));
	CHECK(ctrl.attitude_history_count == 1);
	m_clock_windowed_skew_tracker_destroy(ctrl.imu_clock);
	m_imu_3dof_close(&ctrl.fusion);
}

#ifdef TEST_RIFT_S_STEAMVR_POSE
TEST_CASE("SteamVR pose conversion does not publish overflowed rotated velocity")
{
	xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
	relation.pose = XRT_POSE_IDENTITY;
	const xrt_vec3 axis = {0, 0, 1};
	math_quat_from_angle_vector(DEG_TO_RAD(45), &axis, &relation.pose.orientation);
	relation.relation_flags = static_cast<xrt_space_relation_flags>(XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                                                                XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
	const float huge = std::numeric_limits<float>::max();
	relation.angular_velocity = {huge, huge, huge};
	vr::DriverPose_t pose{};
	apply_pose(&relation, &pose);
	for (double value : pose.vecAngularVelocity)
		CHECK(std::isfinite(value));
}
#endif

TEST_CASE("HMD IMU ingestion drops backend input during session replacement")
{
	rift_s_tracker tracker{};
	os_mutex_init(&tracker.mutex);
	os_mutex_init(&tracker.slam_mutex);
	m_imu_3dof_init(&tracker.fusion.i3dof, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
	tracker.ready_for_data = tracker.have_hw2mono = true;
	tracker.last_frame_time = tracker.last_camera_arrival_ns = os_monotonic_get_ns();
	struct Counter
	{
		xrt_imu_sink sink;
		int calls = 0;
	} counter{};
	counter.sink.push_imu = [](xrt_imu_sink *sink, xrt_imu_sample *) {
		++reinterpret_cast<Counter *>(sink)->calls;
	};
	tracker.slam_sinks.imu = &counter.sink;
	const xrt_vec3 accel{0, 9.81f, 0}, gyro{};
	os_mutex_lock(&tracker.slam_mutex);
	rift_s_tracker_imu_update(&tracker, tracker.last_frame_time, &accel, &gyro);
	CHECK(counter.calls == 0);
	CHECK(tracker.fusion.last_imu_local_timestamp_ns == tracker.last_frame_time);
	os_mutex_unlock(&tracker.slam_mutex);
	rift_s_tracker_imu_update(&tracker, tracker.last_frame_time + 4000000, &accel, &gyro);
	CHECK(counter.calls == 1);
	m_imu_3dof_close(&tracker.fusion.i3dof);
	os_mutex_destroy(&tracker.slam_mutex);
	os_mutex_destroy(&tracker.mutex);
}

TEST_CASE("Rift S Touch native frames reproduce Oculus controller constants")
{
	for (bool left : {true, false}) {
		CAPTURE(left);
		xrt_pose model_device, model_grip, device_model, aim, grip;
		rift_s_controller_model_frames(left, &model_device, &model_grip);
		math_pose_invert(&model_device, &device_model);
		// Official SteamVR Oculus driver pose, independently checked against the LCON components.
		const double norm = std::hypot(0.3371, 0.94147);
		CHECK(device_model.orientation.x == Catch::Approx(0.3371 / norm).margin(2e-7));
		CHECK(device_model.orientation.w == Catch::Approx(0.94147 / norm).margin(2e-7));
		check_vec(device_model.position, {left ? -0.007f : 0.007f, 0.036124f, -0.037617f});
		rift_s_controller_pose_frames(left, &aim, &grip);
		// LibOVR LCON table. The rounded driver quaternion/model components differ by 0.000533 degrees.
		check_vec(aim.position, {0, 0, -0.055f});
		check_vec(grip.position, {0, -0.03f, 0.04f});
		CHECK(std::abs(aim.orientation.x) < 5e-6f);
		CHECK(aim.orientation.w == Catch::Approx(1).margin(2e-7));
		CHECK(grip.orientation.x == Catch::Approx(0.5).margin(5e-6));
		CHECK(grip.orientation.w == Catch::Approx(0.866025403784).margin(5e-6));
#ifdef TEST_RIFT_S_STEAMVR_POSE
		PublishedPoseController fixture(!left);
		const xrt_vec3 factory_imu = left ? xrt_vec3{-0.00558411609381f, -0.00711983907968f, 0.0198550093919f}
		                                  : xrt_vec3{0.0055856378749f, -0.00711983907968f, 0.0198451988399f};
		fixture.ctrl.P_device_imu.position = factory_imu;
		math_pose_invert(&fixture.ctrl.P_device_imu, &fixture.ctrl.P_imu_device);
		const xrt_vec3 world_imu = {0.2f, 1.4f, -0.8f};
		fixture.measured(XRT_QUAT_IDENTITY, world_imu, 1000000000);
		auto published = fixture.query(XRT_INPUT_TOUCH_GRIP_POSE, 1000000000);
		const xrt_pose grip_model = rift_s_touch_grip_to_model(left ? XRT_HAND_LEFT : XRT_HAND_RIGHT);
		math_pose_transform(&published.pose, &grip_model, &published.pose);
		check_vec(published.pose.position,
		          m_vec3_add(world_imu, m_vec3_sub(device_model.position, factory_imu)));
		CHECK(published.pose.orientation.x == Catch::Approx(0.3371 / norm).margin(2e-7));
		CHECK(published.pose.orientation.w == Catch::Approx(0.94147 / norm).margin(2e-7));
#endif
	}
}
