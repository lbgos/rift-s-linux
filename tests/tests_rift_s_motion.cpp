// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#include "rift_s_motion.h"
#include "math/m_api.h"
#include "math/m_vec3.h"
#include <memory>

namespace {
constexpr timepoint_ns MS = U_TIME_1MS_IN_NS;
struct Motion
{
	rift_s_motion *ctx = rift_s_motion_create();
	xrt_pose head = {XRT_QUAT_IDENTITY, {0, 1.6f, 0}};
	rift_s_motion_sample sample = {};
	Motion()
	{
		sample.pose.orientation = XRT_QUAT_IDENTITY;
		sample.timestamp_ns = 1000 * MS;
		for (double &v : sample.position_variance)
			v = 1e-5;
	}
	~Motion()
	{
		rift_s_motion_destroy(&ctx);
	}
	void
	step(unsigned hand, xrt_vec3 p, bool reliable, time_duration_ns dt = 2 * MS)
	{
		sample.timestamp_ns += dt;
		sample.pose.position = p;
		for (double &v : sample.position_variance)
			v = reliable ? 1e-5 : -1;
		rift_s_motion_update(ctx, hand, &sample, &head);
	}
	rift_s_motion_model
	get(unsigned hand, xrt_space_relation &rel, time_duration_ns ahead = 0)
	{
		return rift_s_motion_get(ctx, hand, sample.timestamp_ns + ahead, &rel);
	}
};
void
check_vec(xrt_vec3 a, xrt_vec3 b, double margin = 1e-6)
{
	CHECK(a.x == Catch::Approx(b.x).margin(margin));
	CHECK(a.y == Catch::Approx(b.y).margin(margin));
	CHECK(a.z == Catch::Approx(b.z).margin(margin));
}
} // namespace

TEST_CASE("Behind-back loss follows the saved head-relative position for either hand")
{
	for (unsigned hand : {0u, 1u}) {
		Motion m;
		xrt_space_relation rel{};
		const xrt_vec3 hand_position = {hand ? 0.3f : -0.3f, 1.1f, 0.5f};
		m.step(hand, hand_position, true);
		CHECK(m.get(hand, rel) == RIFT_S_MOTION_NONE);
		m.step(hand, hand_position, false);
		REQUIRE(m.get(hand, rel) == RIFT_S_MOTION_BEHIND_BACK);
		check_vec(rel.pose.position, hand_position);
		const xrt_vec3 up = {0, 1, 0};
		math_quat_from_angle_vector(1.57079632679f, &up, &m.head.orientation);
		m.head.position = {1, 1.7f, 2};
		// The unobserved filter position can be nonsense. It must not replace the reliable support.
		m.step(hand, {100, -100, 100}, false, 800 * MS);
		REQUIRE(m.get(hand, rel) == RIFT_S_MOTION_BEHIND_BACK);
		const xrt_vec3 expected = {1.5f, 1.2f, 2 + (hand ? -0.3f : 0.3f)};
		check_vec(rel.pose.position, expected);
		CHECK_FALSE(rel.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT);
		CHECK(rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT);
		m.step(hand, expected, true);
		CHECK(m.get(hand, rel) != RIFT_S_MOTION_BEHIND_BACK);
	}
}

TEST_CASE("Out-of-view wrist rotation uses an arm pivot and returns through the correction-rate state")
{
	for (unsigned hand : {0u, 1u}) {
		Motion m;
		xrt_space_relation rel{};
		const xrt_vec3 start = {hand ? 0.3f : -0.3f, 1.45f, -0.5f};
		m.step(hand, start, true);
		m.step(hand, start, true);
		m.step(hand, start, false);
		REQUIRE(m.get(hand, rel) == RIFT_S_MOTION_OUT_OF_FOV);
		check_vec(rel.pose.position, start);
		const xrt_vec3 up = {0, 1, 0};
		math_quat_from_angle_vector(1.57079632679f, &up, &m.sample.pose.orientation);
		m.step(hand, {5, -20, -8}, false);
		REQUIRE(m.get(hand, rel) == RIFT_S_MOTION_OUT_OF_FOV);
		// Rotation of a 20 cm forearm pivot changes the IMU position; it is not a frozen translation.
		CHECK(m_vec3_len(m_vec3_sub(rel.pose.position, start)) > 0.25f);
		CHECK(rel.pose.position.y > 1.3f);
		CHECK_FALSE(rel.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT);
		xrt_vec3 estimate = rel.pose.position;
		xrt_vec3 target = m_vec3_add(estimate, {0.03f, 0, 0});
		m.sample.linear_velocity = {0, 0, 0};
		m.step(hand, target, true);
		REQUIRE(m.get(hand, rel) == RIFT_S_MOTION_OUT_OF_FOV);
		CHECK(rel.pose.position.x == Catch::Approx(estimate.x + 0.0005f).margin(2e-6));
		CHECK(rel.linear_velocity.x == Catch::Approx(0.25f).margin(1e-5));
		// The optical return drives the model with a bounded correction rate, with no fixed blend timer.
		for (unsigned i = 0; i < 100 && m.get(hand, rel) == RIFT_S_MOTION_OUT_OF_FOV; ++i)
			m.step(hand, target, true);
		CHECK(m.get(hand, rel) == RIFT_S_MOTION_NONE);
	}
}

TEST_CASE("Motion freshness is a covariance-qualified filter sample, independent of vision cadence")
{
	Motion m;
	xrt_space_relation rel{};
	const xrt_vec3 start = {0.3f, 1.45f, -0.5f};
	for (unsigned i = 0; i < 100; ++i) {
		m.step(1, start, true);
		CHECK(m.get(1, rel) == RIFT_S_MOTION_NONE);
	}
	// The gate is strict; equality with the reliability limit cannot refresh the support.
	for (double &v : m.sample.position_variance)
		v = 0.001;
	m.sample.timestamp_ns += 2 * MS;
	rift_s_motion_update(m.ctx, 1, &m.sample, &m.head);
	CHECK(m.get(1, rel) == RIFT_S_MOTION_OUT_OF_FOV);
	const auto old = rel;
	// Queries do not change the cache or another hand's model.
	for (int i = 0; i < 10; ++i) {
		CHECK(m.get(1, rel, 10 * MS) == RIFT_S_MOTION_OUT_OF_FOV);
		CHECK(m.get(0, rel) == RIFT_S_MOTION_NONE);
	}
	REQUIRE(m.get(1, rel) == RIFT_S_MOTION_OUT_OF_FOV);
	check_vec(rel.pose.position, old.pose.position);
}

TEST_CASE("Near-head loss follows wrist rotation and expires only after spatial convergence")
{
	Motion m;
	xrt_space_relation rel{};
	const xrt_vec3 start = {0.1f, 1.6f, -0.05f};
	m.step(0, start, true);
	m.step(0, start, false);
	REQUIRE(m.get(0, rel) == RIFT_S_MOTION_NEAR_HEAD);
	check_vec(rel.pose.position, start);
	const xrt_vec3 up = {0, 1, 0};
	math_quat_from_angle_vector(0.5f, &up, &m.sample.pose.orientation);
	m.step(0, {10, -30, 4}, false, 600 * MS);
	REQUIRE(m.get(0, rel) == RIFT_S_MOTION_NEAR_HEAD);
	CHECK(m_vec3_len(m_vec3_sub(rel.pose.position, start)) > 0.05f);
	CHECK(rel.pose.position.y > 1.5f);
}

TEST_CASE("At-rest wins priority until measured motion releases its latch")
{
	Motion m;
	xrt_space_relation rel{};
	const xrt_vec3 start = {0.9f, 1.6f, -0.1f};
	for (int i = 0; i < 60; ++i)
		m.step(0, start, true);
	REQUIRE(m.get(0, rel) == RIFT_S_MOTION_AT_REST);
	m.step(0, {40, -20, 30}, false, 700 * MS);
	REQUIRE(m.get(0, rel) == RIFT_S_MOTION_AT_REST);
	check_vec(rel.pose.position, start);
	check_vec(rel.linear_velocity, {0, 0, 0});
	// Lower models were reset by the latch and cannot publish an old estimate on release.
	m.sample.angular_velocity = {1, 0, 0};
	m.step(0, start, false);
	CHECK(m.get(0, rel) == RIFT_S_MOTION_NONE);
	m.step(0, start, true);
	m.step(0, start, false);
	CHECK(m.get(0, rel) == RIFT_S_MOTION_OUT_OF_FOV);
}

TEST_CASE("Correlated close hands attach to the observed hand and preserve the attachment on handover")
{
	for (unsigned hand : {0u, 1u}) {
		Motion m;
		xrt_space_relation rel{};
		const xrt_vec3 p = {0.2f, 1.4f, -0.5f}, q = {0.3f, 1.4f, -0.5f};
		for (int i = 0; i < 60; ++i) {
			const float a = i % 2 ? 0.3f : -0.3f;
			m.sample.linear_acceleration = {a, 2 * a, 3 * a};
			m.step(1 - hand, q, true);
			m.step(hand, p, true);
		}
		for (int i = 0; i < 16; ++i) {
			const float a = i % 2 ? 0.3f : -0.3f;
			m.sample.linear_acceleration = {a, 2 * a, 3 * a};
			m.step(1 - hand, q, true);
			m.step(hand, p, false);
		}
		REQUIRE(m.get(hand, rel) == RIFT_S_MOTION_HAND_OVER_HAND);
		check_vec(rel.pose.position, p);
		m.step(1 - hand, {q.x + 0.1f, q.y + 0.05f, q.z}, true);
		m.step(hand, p, false);
		REQUIRE(m.get(hand, rel) == RIFT_S_MOTION_HAND_OVER_HAND);
		check_vec(rel.pose.position, {p.x + 0.1f, p.y + 0.05f, p.z});
		const auto before = rel.pose.position;
		// Opposite acceleration correlations switch the support, without resetting position.
		for (int i = 0; i < 60; ++i) {
			float a = i % 2 ? 0.3f : -0.3f;
			m.sample.linear_acceleration = {-a, -2 * a, -3 * a};
			m.step(1 - hand, {q.x + 0.1f, q.y + 0.05f, q.z}, true);
			m.sample.linear_acceleration = {a, 2 * a, 3 * a};
			m.step(hand, p, false);
		}
		REQUIRE(m.get(hand, rel) == RIFT_S_MOTION_HAND_OVER_HAND);
		check_vec(rel.pose.position, before);
		rift_s_motion_reset_hand(m.ctx, hand);
		CHECK(m.get(hand, rel) == RIFT_S_MOTION_NONE);
	}
}

TEST_CASE("Model prediction uses body angular rate and queries cannot seed an unobserved hand")
{
	Motion m;
	xrt_space_relation rel{};
	m.sample.pose.orientation = {0, 0, 0, 1};
	m.step(0, {0.1f, 1.6f, -0.1f}, false);
	CHECK(m.get(0, rel) == RIFT_S_MOTION_NONE);
	const xrt_vec3 axis = {1, 0, 0};
	math_quat_from_angle_vector(0.7f, &axis, &m.sample.pose.orientation);
	m.sample.angular_velocity = {0, 1, 0};
	m.step(0, {0.1f, 1.6f, -0.1f}, true);
	m.step(0, {0.1f, 1.6f, -0.1f}, false);
	REQUIRE(m.get(0, rel, 10 * MS) == RIFT_S_MOTION_NEAR_HEAD);
	xrt_quat expected;
	math_quat_integrate_velocity(&m.sample.pose.orientation, &m.sample.angular_velocity, 0.01f, &expected);
	CHECK(rel.pose.orientation.x == Catch::Approx(expected.x).margin(1e-6));
	CHECK(rel.pose.orientation.y == Catch::Approx(expected.y).margin(1e-6));
	CHECK(rel.pose.orientation.z == Catch::Approx(expected.z).margin(1e-6));
	CHECK(rel.pose.orientation.w == Catch::Approx(expected.w).margin(1e-6));
	xrt_vec3 world_rate;
	math_quat_rotate_vec3(&m.sample.pose.orientation, &m.sample.angular_velocity, &world_rate);
	check_vec(rel.angular_velocity, world_rate);
	CHECK(m.get(1, rel) == RIFT_S_MOTION_NONE);
}
