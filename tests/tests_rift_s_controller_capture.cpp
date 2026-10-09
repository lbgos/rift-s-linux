// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#include "rift_s_controller.h"
#include <cmath>

enum u_logging_level rift_s_log_level = U_LOGGING_ERROR;

TEST_CASE("Rift S camera attitude bridges one radio scheduling gap with local gyro")
{
	rift_s_controller ctrl{};
	ctrl.attitude_history_count = 2;
	xrt_vec3 axis = {.3f, .8f, -.4f};
	math_vec3_normalize(&axis);
	xrt_quat orientation;
	math_quat_from_angle_vector(.7f, &axis, &orientation);
	math_quat_normalize(&orientation);
	ctrl.attitude_history[0] = {1000000000, 2000000000, 2000000000, orientation, {.8f, -1.2f, .4f}, true};
	ctrl.attitude_history[1] = {1002000000, 2002000000, 2002000000, orientation, {.8f, -1.2f, .4f}, true};
	rift_s_attitude_sample result{};
	REQUIRE(rift_s_controller_get_attitude(&ctrl, 2007000000, &result));
	xrt_quat expected;
	math_quat_integrate_velocity(&orientation, &ctrl.attitude_history[1].gyro, .005f, &expected);
	CHECK(result.capture_ns == 2007000000);
	CHECK(result.device_capture_ns == 1007000000);
	CHECK(result.orientation.x == Catch::Approx(expected.x).margin(1e-7));
	CHECK(result.orientation.y == Catch::Approx(expected.y).margin(1e-7));
	CHECK(result.orientation.z == Catch::Approx(expected.z).margin(1e-7));
	CHECK(result.orientation.w == Catch::Approx(expected.w).margin(1e-7));
	CHECK(rift_s_controller_get_attitude(&ctrl, 2022000000, &result));
	CHECK_FALSE(rift_s_controller_get_attitude(&ctrl, 2022000001, &result));
	CHECK(ctrl.diagnostics.last_attitude_reject == RIFT_S_ATTITUDE_TOO_NEW);
	ctrl.attitude_history[0].capture_ns -= 20000000;
	CHECK_FALSE(rift_s_controller_get_attitude(&ctrl, 2007000000, &result));
	ctrl.attitude_history[0].capture_ns += 20000000;
	ctrl.attitude_history[1].tilt_valid = false;
	CHECK_FALSE(rift_s_controller_get_attitude(&ctrl, 2007000000, &result));
	CHECK_FALSE(rift_s_controller_get_attitude(&ctrl, 1999999999, &result));
}
