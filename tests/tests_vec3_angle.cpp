// Copyright 2022-2024, Collabora, Inc.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Test for m_vec3_angle.
 * @author Moshi Turner <moshiturner@protonmail.com>
 */
#include "xrt/xrt_defines.h"
#include <math/m_vec3.h>


#include "catch_amalgamated.hpp"

TEST_CASE("Vec3Angle")
{
	float sqrt2_2 = sqrtf(2) / 2;
	CHECK(m_vec3_angle({1, 0, 0}, {-1, 0, 0}) == Catch::Approx(M_PI));
	CHECK(m_vec3_angle({1, 0, 0}, {0, 1, 0}) == Catch::Approx(M_PI / 2));
	CHECK(m_vec3_angle({1, 0, 0}, {sqrt2_2, sqrt2_2, 0}) == Catch::Approx(M_PI / 4));
}

TEST_CASE("Vec3Angle remains finite for nearly parallel gravity directions")
{
	for (float x : {.03f, .1f, .7f}) for (float y : {.9f, 1.f, 9.82f}) {
		xrt_vec3 a = {x, y, -.3f};
		xrt_vec3 b = {x + 1e-7f, y, -.3f};
		CHECK(isfinite(m_vec3_angle(a, b)));
		CHECK(m_vec3_angle(a, b) < .001f);
		CHECK(m_vec3_angle(a, m_vec3_mul_scalar(b, -1)) == Catch::Approx(M_PI).margin(.001));
	}
}
