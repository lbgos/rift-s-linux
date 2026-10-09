// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#include "rift_s/rift_s_slam_guard.h"
#include "math/m_api.h"
#include "math/m_vec3.h"
#include <cmath>

static constexpr int64_t MS = 1000 * 1000;
static constexpr int64_t FRAME = 11 * MS; // ~90 Hz pose queries

static xrt_quat
yaw_quat(float rad)
{
	xrt_quat q;
	xrt_vec3 up = {0, 1, 0};
	math_quat_from_angle_vector(rad, &up, &q);
	return q;
}

static float
yaw_of(const xrt_quat &q)
{
	xrt_vec3 fwd = {0, 0, -1};
	xrt_vec3 out;
	math_quat_rotate_vec3(&q, &fwd, &out);
	return atan2f(-out.x, -out.z);
}

static float
dist(xrt_vec3 a, xrt_vec3 b)
{
	return m_vec3_len(m_vec3_sub(a, b));
}

struct Sim
{
	rift_s_slam_guard g;
	int64_t ts = 1000 * MS;
	xrt_quat fusion = {0, 0, 0, 1};
	xrt_vec3 no_pose = {0, 0, 0};
	int resets = 0;

	Sim()
	{
		rift_s_slam_guard_init(&g);
	}

	//! One pose query; returns whether SLAM was published.
	bool
	step(bool valid, xrt_pose slam, xrt_pose *out, rift_s_slam_guard_reason *reason = nullptr)
	{
		ts += FRAME;
		rift_s_slam_guard_reason r;
		bool used = rift_s_slam_guard_update(&g, valid, &slam, ts, &fusion, &no_pose, out, &r);
		if (rift_s_slam_guard_take_reset(&g, ts)) {
			resets++;
		}
		if (reason) {
			*reason = r;
		}
		return used;
	}

	//! Feed a still pose until SLAM is trusted; returns the number of queries it took.
	int
	settle(xrt_pose slam)
	{
		xrt_pose out;
		for (int i = 1; i < 500; i++) {
			if (step(true, slam, &out)) {
				return i;
			}
		}
		return -1;
	}
};

TEST_CASE("No SLAM pose yet: eye height above the floor")
{
	// The Rift S origin already carries 1.6 m, so the tracker frame position is 0.
	xrt_vec3 p = rift_s_slam_guard_no_pose_position(1.6f);
	CHECK(p.y + 1.6f == Catch::Approx(1.6f));
	// An origin without offset gets the full height.
	CHECK(rift_s_slam_guard_no_pose_position(0.0f).y == Catch::Approx(1.6f));

	Sim s;
	s.no_pose = {0, 1.6f, 0};
	xrt_pose out;
	CHECK_FALSE(s.step(false, xrt_pose{}, &out));
	CHECK(out.position.y == Catch::Approx(1.6f));
	CHECK(s.g.state == RIFT_S_SLAM_GUARD_ACQUIRING);
}

TEST_CASE("Startup: SLAM is trusted after a consistent second, aligned onto eye height")
{
	Sim s;
	xrt_pose head = {{0, 0, 0, 1}, {0.1f, 0.02f, -0.1f}};
	int n = s.settle(head);
	CHECK(n >= RIFT_S_SLAM_GUARD_STABLE_SAMPLES);
	CHECK(n * FRAME >= RIFT_S_SLAM_GUARD_STABLE_NS);
	xrt_pose out;
	REQUIRE(s.step(true, head, &out));
	CHECK(dist(out.position, s.no_pose) < 1e-5f);
	CHECK(s.resets == 0);
	CHECK(s.g.recoveries == 0);

	// Startup junk far away is never published.
	Sim junk;
	xrt_pose far = {{0, 0, 0, 1}, {1e7f, 0, 0}};
	for (int i = 0; i < 200; i++) {
		CHECK_FALSE(junk.step(true, far, &out));
	}
}

TEST_CASE("SLAM started on a desk: the head is still at eye height, so the floor stays put")
{
	// SteamVR keeps the Room Setup floor as a fixed height in the driver frame. SLAM that started
	// with the headset on a desk sees the worn headset 0.8 m above its origin; publishing that as is
	// would lift the frame by 0.8 m against the stored floor.
	Sim s;
	s.no_pose = rift_s_slam_guard_no_pose_position(1.6f); // eye height with the 1.6 m origin offset
	xrt_pose head = {yaw_quat(0.7f), {0.2f, 0.8f, -0.3f}};
	REQUIRE(s.settle(head) > 0);
	xrt_pose out;
	REQUIRE(s.step(true, head, &out));
	CHECK(out.position.y + 1.6f == Catch::Approx(1.6f).margin(1e-5));
	// Heading continues from the 3DoF orientation shown while acquiring.
	CHECK(yaw_of(out.orientation) == Catch::Approx(yaw_of(s.fusion)).margin(1e-4));

	// Standing up by 0.3 m is followed as SLAM motion, relative to eye height.
	for (int i = 0; i < 30; i++) {
		head.position.y += 0.01f;
		REQUIRE(s.step(true, head, &out));
	}
	CHECK(out.position.y + 1.6f == Catch::Approx(1.9f).margin(1e-4));
	CHECK(s.g.divergences == 0);
}

TEST_CASE("Walking and quick head motion stay on SLAM")
{
	Sim s;
	xrt_pose head = {{0, 0, 0, 1}, {0, 0, 0}};
	REQUIRE(s.settle(head) > 0);
	xrt_pose out;
	// 4 m/s for 1 s along X, then back.
	for (int i = 0; i < 180; i++) {
		head.position.x += (i < 90 ? 1 : -1) * 4.0f * FRAME / 1e9f;
		REQUIRE(s.step(true, head, &out));
	}
	// Repeated queries at the same time are fine.
	s.ts -= FRAME;
	CHECK(s.step(true, head, &out));
	CHECK(s.g.divergences == 0);
}

TEST_CASE("Idle-headset divergence: hold the last good pose, re-init, realign without a jump")
{
	Sim s;
	xrt_pose head = {{0, 0, 0, 1}, {0.3f, 0.05f, -0.2f}};
	REQUIRE(s.settle(head) > 0);
	xrt_pose out;
	REQUIRE(s.step(true, head, &out));
	const xrt_vec3 good = out.position;

	// The reported runaway: Basalt drifts off at 8 m/s towards (18,-16,19).
	xrt_vec3 target = {18, -16, 19};
	xrt_vec3 dir = m_vec3_normalize(m_vec3_sub(target, head.position));
	xrt_pose runaway = head;
	rift_s_slam_guard_reason reason;
	bool rejected = false;
	for (int i = 0; i < 400; i++) {
		runaway.position = m_vec3_add(runaway.position, m_vec3_mul_scalar(dir, 8.0f * FRAME / 1e9f));
		bool used = s.step(true, runaway, &out, &reason);
		if (!used) {
			if (!rejected) {
				CHECK(reason == RIFT_S_SLAM_GUARD_SPEED);
			}
			rejected = true;
		}
		CHECK(dist(out.position, good) < 0.1f); // never more than one accepted step away
		CHECK(out.position.y > -0.1f);          // never underground
	}
	REQUIRE(rejected);
	CHECK(s.g.state == RIFT_S_SLAM_GUARD_DIVERGED);
	CHECK(s.g.divergences == 1);
	// One immediate re-init, then retries at most every 2 s while still diverged.
	CHECK(s.resets >= 1);
	CHECK(s.resets <= 1 + (int)(400 * FRAME / RIFT_S_SLAM_GUARD_RESET_INTERVAL_NS));

	// Fallback: 3DoF orientation continued in the SLAM heading, position held.
	const xrt_vec3 held = out.position;
	s.fusion = yaw_quat(0.3f);
	s.step(true, runaway, &out);
	CHECK(dist(out.position, held) < 1e-6f);
	CHECK(yaw_of(out.orientation) == Catch::Approx(0.3f).margin(1e-3));

	// SLAM restarts at its new origin with an arbitrary heading; it must be consistent before use.
	xrt_pose fresh = {yaw_quat(-1.2f), {0, 0, 0}};
	int n = s.settle(fresh);
	REQUIRE(n > 0);
	CHECK(n >= RIFT_S_SLAM_GUARD_STABLE_SAMPLES);
	CHECK(s.g.recoveries == 1);
	REQUIRE(s.step(true, fresh, &out));
	CHECK(dist(out.position, held) < 1e-4f);                            // no jump back to the restart origin
	CHECK(yaw_of(out.orientation) == Catch::Approx(0.3f).margin(1e-3)); // no heading jump

	// Moving 0.5 m along the new SLAM X moves 0.5 m in the old frame, rotated by the alignment.
	for (int i = 0; i < 50; i++) {
		fresh.position.x += 0.01f;
		REQUIRE(s.step(true, fresh, &out));
	}
	CHECK(dist(out.position, held) == Catch::Approx(0.5f).margin(1e-3));
	CHECK(out.position.y == Catch::Approx(held.y).margin(1e-4));
}

TEST_CASE("Rejection rules: jump, distance cap and non-finite")
{
	xrt_pose out;
	rift_s_slam_guard_reason reason;
	{
		Sim s;
		xrt_pose head = {{0, 0, 0, 1}, {0, 0, 0}};
		REQUIRE(s.settle(head) > 0);
		head.position.z += 0.6f;
		CHECK_FALSE(s.step(true, head, &out, &reason));
		CHECK(reason == RIFT_S_SLAM_GUARD_JUMP);
		CHECK(s.resets == 1);
	}
	{
		// Slow drift (1 m/s) is not a speed outlier but stops at the 5 m cap.
		Sim s;
		xrt_pose head = {{0, 0, 0, 1}, {0, 0, 0}};
		REQUIRE(s.settle(head) > 0);
		bool used = true;
		while (used) {
			head.position.x += 1.0f * FRAME / 1e9f;
			used = s.step(true, head, &out, &reason);
			REQUIRE(m_vec3_len(out.position) <= RIFT_S_SLAM_GUARD_MAX_DISTANCE_M);
		}
		CHECK(reason == RIFT_S_SLAM_GUARD_TOO_FAR);
	}
	{
		// The reported slow sink: 1 m/s straight down stops 0.3 m under the nominal floor.
		Sim s;
		s.no_pose = rift_s_slam_guard_no_pose_position(1.6f); // floor at y = -1.6 in the tracker frame
		xrt_pose head = {{0, 0, 0, 1}, {0, 0, 0}};
		REQUIRE(s.settle(head) > 0);
		bool used = true;
		while (used) {
			head.position.y -= 1.0f * FRAME / 1e9f;
			used = s.step(true, head, &out, &reason);
		}
		CHECK(reason == RIFT_S_SLAM_GUARD_HEIGHT);
		CHECK(out.position.y >= -1.6f - RIFT_S_SLAM_GUARD_BELOW_FLOOR_M);
		// Crouching to the floor is still fine.
		Sim c;
		c.no_pose = s.no_pose;
		xrt_pose crouch = {{0, 0, 0, 1}, {0, 0, 0}};
		REQUIRE(c.settle(crouch) > 0);
		for (int i = 0; i < 150; i++) {
			crouch.position.y -= 1.0f * FRAME / 1e9f;
			REQUIRE(c.step(true, crouch, &out));
		}
	}
	{
		Sim s;
		xrt_pose head = {{0, 0, 0, 1}, {0, 0, 0}};
		REQUIRE(s.settle(head) > 0);
		head.position.x = NAN;
		CHECK_FALSE(s.step(true, head, &out, &reason));
		CHECK(reason == RIFT_S_SLAM_GUARD_NOT_FINITE);
		CHECK(std::isfinite(out.position.x));
	}
}

TEST_CASE("Lost SLAM pose and user-chosen 3DoF hold without a divergence")
{
	Sim s;
	xrt_pose head = {{0, 0, 0, 1}, {0.2f, 0, 0}};
	REQUIRE(s.settle(head) > 0);
	xrt_pose out;
	head.position.x += 0.01f;
	REQUIRE(s.step(true, head, &out));
	const xrt_vec3 shown = out.position;
	rift_s_slam_guard_reason reason;
	CHECK_FALSE(s.step(false, head, &out, &reason));
	CHECK(reason == RIFT_S_SLAM_GUARD_NO_POSE);
	CHECK(s.g.divergences == 0);
	CHECK(s.resets == 0);
	CHECK(dist(out.position, shown) < 1e-5f);

	Sim u;
	REQUIRE(u.settle(head) > 0);
	rift_s_slam_guard_suspend(&u.g, &u.fusion);
	CHECK(u.g.state == RIFT_S_SLAM_GUARD_DIVERGED);
	CHECK(u.g.divergences == 0);
	CHECK_FALSE(rift_s_slam_guard_take_reset(&u.g, u.ts));
}

TEST_CASE("Feature gate: face-down or covered cameras hold, features back re-init")
{
	rift_s_slam_feature_gate gate = {};
	int64_t ts = 0;
	auto feed = [&](int count, int frames) {
		rift_s_slam_feature_event last = RIFT_S_SLAM_FEATURES_NO_CHANGE;
		for (int i = 0; i < frames; i++) {
			ts += 33 * MS;
			rift_s_slam_feature_event e = rift_s_slam_feature_gate_update(&gate, true, count, ts);
			if (e != RIFT_S_SLAM_FEATURES_NO_CHANGE) {
				last = e;
			}
		}
		return last;
	};
	CHECK(feed(150, 30) == RIFT_S_SLAM_FEATURES_NO_CHANGE);
	// A brief dip (a hand over the cameras for 0.3 s) is ignored.
	CHECK(feed(3, 9) == RIFT_S_SLAM_FEATURES_NO_CHANGE);
	CHECK(feed(150, 5) == RIFT_S_SLAM_FEATURES_NO_CHANGE);
	CHECK_FALSE(gate.lost);
	// Face down: lost after the dwell.
	CHECK(feed(2, 30) == RIFT_S_SLAM_FEATURES_LOST);
	CHECK(gate.lost);
	// Unknown counts and repeated timestamps change nothing.
	CHECK(rift_s_slam_feature_gate_update(&gate, false, 200, ts + 10 * MS * 1000) ==
	      RIFT_S_SLAM_FEATURES_NO_CHANGE);
	CHECK(rift_s_slam_feature_gate_update(&gate, true, 200, ts) == RIFT_S_SLAM_FEATURES_NO_CHANGE);
	// Picked up again.
	CHECK(feed(120, 30) == RIFT_S_SLAM_FEATURES_RETURNED);
	CHECK_FALSE(gate.lost);
}

TEST_CASE("Source guard ignores duplicate and historical timestamps")
{
	Sim s;
	xrt_pose raw = XRT_POSE_IDENTITY, out;
	REQUIRE(s.settle(raw) > 0);
	auto before = s.g;
	xrt_pose camera_query = raw;
	camera_query.position.x += 0.04f;
	rift_s_slam_guard_reason reason;
	CHECK(rift_s_slam_guard_update(&s.g, true, &camera_query, before.good_ns - 1000000,
	                               &s.fusion, &s.no_pose, &out, &reason));
	CHECK(s.g.good_ns == before.good_ns);
	CHECK(s.g.divergences == before.divergences);
	CHECK(dist(out.position, before.good.position) < 1e-6);
}

TEST_CASE("SLAM cannot publish invalid quaternions or a gravity flip")
{
	for (auto q : {xrt_quat{0, 0, 0, 0}, xrt_quat{1, 0, 0, 0}, xrt_quat{0, 0, 0, 2}}) {
		Sim s;
		xrt_pose raw = XRT_POSE_IDENTITY, out;
		REQUIRE(s.settle(raw) > 0);
		raw.orientation = q;
		rift_s_slam_guard_reason reason;
		CHECK_FALSE(s.step(true, raw, &out, &reason));
		CHECK(reason == RIFT_S_SLAM_GUARD_TILT);
		CHECK(out.orientation.w == Catch::Approx(1));
		CHECK(s.g.state == RIFT_S_SLAM_GUARD_DIVERGED);
	}
	// A physical upside-down head has the same IMU tilt and must remain valid.
	xrt_pose inverted = {{1, 0, 0, 0}, {0, 1, 0}};
	CHECK(rift_s_slam_guard_pose_usable(&inverted, &inverted.orientation));
}

TEST_CASE("Saved boundary publishes only its recovered world frame after restart")
{
	Sim s;
	s.no_pose = {0, 1.6f, 0};
	s.g.require_relocalization = true;
	xrt_pose raw = {yaw_quat(-0.8f), {0.4f, 0.2f, -0.3f}}, out;
	rift_s_slam_guard_reason reason;
	for (unsigned i = 0; i < 160; ++i)
		CHECK_FALSE(s.step(true, raw, &out, &reason));
	CHECK(reason == RIFT_S_SLAM_GUARD_RELOCALIZING);
	CHECK_FALSE(s.g.world_valid);
	CHECK_FALSE(s.g.boundary_confirmed);
	CHECK(s.g.resets == 0);

	xrt_pose world = {yaw_quat(0.6f), {-0.7f, 1.3f, 0.3f}};
	rift_s_slam_guard_set_world(&s.g, &world);
	REQUIRE(s.step(true, raw, &out));
	xrt_pose expected;
	math_pose_transform(&world, &raw, &expected);
	CHECK(dist(out.position, expected.position) < 1e-6);
	CHECK(yaw_of(out.orientation) == Catch::Approx(-0.2f).margin(1e-6));
	CHECK_FALSE(s.g.world_transition);
	CHECK(s.g.divergences == 0);

	// The replacement VIO gauge must be matched again, while the saved standing origin stays put.
	s.g.reset_pending = true;
	REQUIRE(rift_s_slam_guard_take_reset(&s.g, s.ts));
	rift_s_slam_guard_suspend(&s.g, &s.fusion);
	raw = {yaw_quat(1.2f), {-0.3f, 0.1f, 0.2f}};
	for (unsigned i = 0; i < 160; ++i)
		CHECK_FALSE(s.step(true, raw, &out, &reason));
	CHECK(reason == RIFT_S_SLAM_GUARD_RELOCALIZING);
	CHECK_FALSE(s.g.world_valid);
	CHECK_FALSE(s.g.boundary_confirmed);
	xrt_pose inverse, recovered;
	math_pose_invert(&raw, &inverse);
	math_pose_transform(&expected, &inverse, &recovered);
	rift_s_slam_guard_set_world(&s.g, &recovered);
	REQUIRE(s.step(true, raw, &out));
	CHECK(dist(out.position, expected.position) < 1e-6);
	CHECK(yaw_of(out.orientation) == Catch::Approx(yaw_of(expected.orientation)).margin(1e-6));
	CHECK_FALSE(s.g.world_transition);
}

TEST_CASE("Restoring saved bounds revokes a room setup's unconfirmed frame")
{
	Sim s;
	s.no_pose = {0, 1.6f, 0};
	xrt_pose raw = XRT_POSE_IDENTITY, out;
	REQUIRE(s.settle(raw) > 0);
	REQUIRE(s.g.world_valid);
	s.g.require_relocalization = true;
	CHECK_FALSE(rift_s_slam_guard_query(&s.g, &raw, s.ts, &s.fusion, &s.no_pose, &out));
	CHECK_FALSE(s.step(true, raw, &out));
	CHECK_FALSE(s.g.world_valid);
	CHECK_FALSE(s.g.world_transition);
	xrt_pose saved{yaw_quat(0.4f), {-0.2f, 1.6f, 0.3f}};
	rift_s_slam_guard_set_world(&s.g, &saved);
	REQUIRE(s.settle(raw) > 0);
	REQUIRE(s.step(true, raw, &out));
	CHECK(dist(out.position, saved.position) < 1e-6);
	CHECK(yaw_of(out.orientation) == Catch::Approx(0.4f).margin(1e-6));
	CHECK_FALSE(s.g.world_transition);
}

TEST_CASE("A bad predicted query cannot reset or displace the world")
{
	Sim s;
	xrt_pose raw = XRT_POSE_IDENTITY, out;
	REQUIRE(s.settle(raw) > 0);
	auto before = s.g;
	raw.position.x = 30;
	CHECK_FALSE(rift_s_slam_guard_query(&s.g, &raw, s.ts + 5 * MS, &s.fusion, &s.no_pose, &out));
	CHECK(s.g.resets == before.resets);
	CHECK(s.g.good_ns == before.good_ns);
	CHECK(dist(s.g.align.position, before.align.position) < 1e-6);
}

TEST_CASE("A backend with no startup poses requests recovery within two seconds")
{
	Sim s;
	xrt_pose out;
	for (int i = 0; i < 600; ++i)
		CHECK_FALSE(s.step(false, xrt_pose{}, &out));
	CHECK(s.resets >= 2);
	CHECK(s.resets <= 3);
	CHECK(dist(out.position, s.no_pose) < 1e-6f);
}

TEST_CASE("Persistent gravity failure retries recovery and retains the world pose")
{
	Sim s;
	xrt_pose head = {yaw_quat(.4f), {.3f, .05f, -.2f}}, out;
	REQUIRE(s.settle(head) > 0);
	const xrt_pose held = s.g.good;
	xrt_vec3 axis{1, 0, 0};
	math_quat_from_angle_vector(3.14159265f, &axis, &head.orientation);
	for (int i = 0; i < 600; ++i) {
		CHECK_FALSE(s.step(true, head, &out));
		CHECK(dist(out.position, held.position) < 1e-6f);
	}
	CHECK(s.resets >= 3);
	CHECK(s.resets <= 4);
	CHECK(s.settle({yaw_quat(-1), {0, 0, 0}}) > 0);
	CHECK(dist(s.g.good.position, held.position) < 1e-5f);
}

TEST_CASE("A fresh backend gets a full recovery interval without moving the world")
{
	Sim s;
	xrt_pose head = {yaw_quat(.4f), {.3f, .05f, -.2f}}, out;
	REQUIRE(s.settle(head) > 0);
	const xrt_pose held = s.g.good;
	rift_s_slam_guard_suspend(&s.g, &s.fusion);
	for (int i = 0; i < 250; ++i)
		CHECK_FALSE(s.step(false, xrt_pose{}, &out));
	const int resets = s.resets;
	rift_s_slam_guard_backend_started(&s.g, s.ts);
	for (int i = 0; i < 180; ++i)
		CHECK_FALSE(s.step(false, xrt_pose{}, &out));
	CHECK(s.resets == resets);
	CHECK(dist(out.position, held.position) < 1e-6f);
	for (int i = 0; i < 10; ++i)
		CHECK_FALSE(s.step(false, xrt_pose{}, &out));
	CHECK(s.resets == resets + 1);
	CHECK(dist(out.position, held.position) < 1e-6f);
}
