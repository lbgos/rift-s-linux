// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#include "internal/constrained_pose.h"
#include "internal/sample.h"
#include "internal/correspondence_search.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <vector>

namespace {
double
distance(xrt_vec3 a, xrt_vec3 b)
{
	return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}
double
angle(xrt_quat a, xrt_quat b)
{
	// Relative rotation conj(a) * b in double; atan2 stays accurate near zero where acos of a float dot does not.
	const double aw = a.w, ax = -a.x, ay = -a.y, az = -a.z, bw = b.w, bx = b.x, by = b.y, bz = b.z;
	const double w = aw * bw - ax * bx - ay * by - az * bz;
	const double x = aw * bx + ax * bw + ay * bz - az * by;
	const double y = aw * by - ax * bz + ay * bw + az * bx;
	const double z = aw * bz + ax * by - ay * bx + az * bw;
	return 2 * std::atan2(std::sqrt(x * x + y * y + z * z), std::abs(w));
}
xrt_quat
rotation(double angle, xrt_vec3 axis)
{
	xrt_quat q;
	math_vec3_normalize(&axis);
	math_quat_from_angle_vector(angle, &axis, &q);
	return q;
}
xrt_quat
conjugate(const xrt_quat &q)
{
	return {-q.x, -q.y, -q.z, q.w};
}
xrt_quat
multiply(const xrt_quat &a, const xrt_quat &b)
{
	xrt_quat q;
	math_quat_rotate(&a, &b, &q);
	return q;
}

struct Scene
{
	std::array<t_constellation_led, 3> leds = {{{11, {-0.047f, -0.033f, -0.017f}, {0, 0, -1}, 3.5f},
	                                            {7, {0.039f, -0.009f, 0.027f}, {0, 0, -1}, 3.5f},
	                                            {23, {0.009f, 0.051f, -0.011f}, {0, 0, -1}, 3.5f}}};
	t_constellation_led_model model{};
	camera_model camera{};
	std::array<blobservation, 2> observations{};
	std::array<constellation_constrained_view, 2> views{};
	xrt_pose truth = XRT_POSE_IDENTITY;
	xrt_pose tilt = XRT_POSE_IDENTITY;
	constellation_constrained_config config;
	Scene()
	{
		model.id = 2;
		model.leds = leds.data();
		model.num_leds = leds.size();
		camera.width = 640;
		camera.height = 480;
		camera.calib.fx = 300;
		camera.calib.fy = 310;
		camera.calib.cx = 320;
		camera.calib.cy = 240;
		camera.calib.model = T_DISTORTION_FISHEYE_KB4;
		camera.calib.fisheye.k1 = 0.02;
		camera.calib.fisheye.k2 = -0.005;
		truth.position = {0.07f, 0.02f, 0.68f};
		truth.orientation = rotation(0.4, {0, 1, 0});
		for (unsigned k = 0; k < 2; k++) {
			views[k].camera_index = k + 3;
			views[k].P_world_cam = XRT_POSE_IDENTITY;
			views[k].calib = &camera;
			views[k].observation = &observations[k];
		}
		views[1].P_world_cam.position = {0.12f, 0.02f, 0.015f};
		views[1].P_world_cam.orientation = rotation(-0.21, {0.1f, 1, 0.03f});
		for (auto &v : views)
			math_pose_invert(&v.P_world_cam, &v.P_cam_world);
		constellation_constrained_default_config(&config);
		config.time_budget_us = 2000000;
		// These fixtures exercise the stock scorer minimum (inliers > sample size) with three
		// LEDs, so they relax Monado's stricter cold-acquisition and tracking policy.
		config.minimum_cold_inliers = 3;
		config.minimum_cold_distinct_leds = 0;
		config.minimum_tracking_inliers = 0;
		config.minimum_tracking_distinct_leds = 0;
	}
	void
	observe(unsigned camera_index, std::initializer_list<unsigned> indices)
	{
		auto &o = observations[camera_index];
		o = {};
		for (unsigned led : indices) {
			xrt_vec3 world, local;
			math_pose_transform_point(&truth, &leds[led].pos, &world);
			math_pose_transform_point(&views[camera_index].P_cam_world, &world, &local);
			blob &b = o.blobs[o.num_blobs++];
			REQUIRE(t_camera_models_project(&camera.calib, local.x, local.y, local.z, &b.x, &b.y));
			b.led_id = LED_MAKE_ID(99, 42); // Temporal labels must not force an assignment.
		}
	}
};
} // namespace

TEST_CASE("A held HMD cannot turn accurate camera pixels into false controller fixes")
{
	for (bool right : {false, true}) {
		Scene s;
		if (right) {
			for (auto &led : s.leds)
				led.pos.x = -led.pos.x;
			s.truth.position.x = -s.truth.position.x;
		}
		s.observe(0, {0, 1});
		s.observe(1, {2});
		constellation_constrained_result live;
		REQUIRE(constellation_constrained_search(&s.model, s.views.data(), 2, &s.tilt, false, false,
		                                          &s.config, &live));
		CHECK(distance(live.P_world_model.position, s.truth.position) < 0.0003);

		// The same pixels fit a wrong world pose perfectly if the camera's held world frame is used.
		xrt_pose delta{rotation(0.5, {0, 1, 0}), {0.6f, 0, -0.2f}}, wrong;
		math_pose_transform(&delta, &s.truth, &wrong);
		for (auto &view : s.views) {
			xrt_pose current = view.P_world_cam;
			math_pose_transform(&delta, &current, &view.P_world_cam);
			math_pose_invert(&view.P_world_cam, &view.P_cam_world);
		}
		constellation_constrained_result stale;
		REQUIRE(constellation_constrained_search(&s.model, s.views.data(), 2, &s.tilt, false, false,
		                                          &s.config, &stale));
		CHECK(distance(stale.P_world_model.position, wrong.position) < 0.0003);
		CHECK(distance(stale.P_world_model.position, s.truth.position) > 0.2);
		CHECK(stale.cost_m < 0.001);

		constellation_tracking_sample sample{};
		xrt_space_relation held = XRT_SPACE_RELATION_ZERO;
		held.pose = delta;
		held.relation_flags = static_cast<xrt_space_relation_flags>(XRT_SPACE_RELATION_POSITION_VALID_BIT |
		                      XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
		CHECK_FALSE(constellation_tracking_sample_set_hmd_pose(&sample, XRT_SUCCESS, &held));
		CHECK_FALSE(sample.have_hmd_pose);
		held.relation_flags = static_cast<xrt_space_relation_flags>(held.relation_flags |
		                      XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
		CHECK_FALSE(constellation_tracking_sample_set_hmd_pose(&sample, XRT_ERROR_POSE_NOT_ACTIVE, &held));
		held.pose = XRT_POSE_IDENTITY;
		CHECK(constellation_tracking_sample_set_hmd_pose(&sample, XRT_SUCCESS, &held));
		CHECK(sample.have_hmd_pose);
		CHECK(distance(sample.P_xrworld_hmd.position, held.pose.position) == 0);
	}
}

TEST_CASE("Constrained two-point geometry recovers 1000 randomized valid poses")
{
	std::mt19937 rng(0x534f4c);
	std::uniform_real_distribution<float> unit(-1, 1);
	unsigned accepted = 0, two_roots = 0;
	for (unsigned attempt = 0; accepted < 1000 && attempt < 10000; attempt++) {
		xrt_vec3 points[2] = {{unit(rng) * 0.06f, unit(rng) * 0.06f, unit(rng) * 0.06f},
		                      {unit(rng) * 0.06f, unit(rng) * 0.06f, unit(rng) * 0.06f}};
		xrt_vec3 gravity = {unit(rng), unit(rng), unit(rng)};
		math_vec3_normalize(&gravity);
		xrt_quat tilt = rotation(unit(rng), {unit(rng), unit(rng), unit(rng)});
		xrt_pose truth = XRT_POSE_IDENTITY;
		truth.orientation = multiply(rotation(unit(rng) * 3.1, gravity), tilt);
		truth.position = {unit(rng) * 0.2f, unit(rng) * 0.2f, 0.8f + unit(rng) * 0.4f};
		xrt_vec3 bearings[2];
		for (unsigned i = 0; i < 2; i++) {
			math_pose_transform_point(&truth, &points[i], &bearings[i]);
			math_vec3_normalize(&bearings[i]);
		}
		xrt_pose heading;
		if (!constellation_constrained_heading_two_point(points, bearings, &truth.orientation, 0.001745329,
		                                                 &heading, nullptr))
			continue;
		REQUIRE(distance(heading.position, truth.position) < 0.0002);
		xrt_pose roots[2];
		unsigned count =
		    constellation_constrained_gravity_two_point(points, bearings, &tilt, &gravity, 0.001745329, roots, nullptr);
		REQUIRE(count >= 1);
		bool recovered = false;
		for (unsigned i = 0; i < count; i++) {
			if (distance(roots[i].position, truth.position) < 0.0003 &&
			    angle(roots[i].orientation, truth.orientation) < 0.001)
				recovered = true;
			for (unsigned p = 0; p < 2; p++) {
				xrt_vec3 projected;
				math_pose_transform_point(&roots[i], &points[p], &projected);
				REQUIRE(projected.z > 0);
				math_vec3_normalize(&projected);
				CHECK(distance(projected, bearings[p]) < 0.00001);
			}
		}
		REQUIRE(recovered);
		two_roots += count == 2;
		accepted++;
	}
	CHECK(accepted == 1000);
	CHECK(two_roots > 100);
}

TEST_CASE("Constrained solvers reject parallel rays negative depth and unresolved yaw")
{
	xrt_vec3 points[2] = {{0, 0, 0}, {0.04f, 0.03f, 0.01f}};
	xrt_vec3 bearings[2] = {{0, 0, 1}, {0, 0, 1}};
	xrt_quat q = {0, 0, 0, 1};
	xrt_vec3 gravity = {0, 1, 0};
	xrt_pose out[2];
	CHECK_FALSE(constellation_constrained_heading_two_point(points, bearings, &q, 0.001745329, out, nullptr));
	CHECK(constellation_constrained_gravity_two_point(points, bearings, &q, &gravity, 0.001745329, out, nullptr) == 0);
	xrt_pose behind = XRT_POSE_IDENTITY;
	behind.position.z = -0.6f;
	for (unsigned i = 0; i < 2; i++) {
		math_pose_transform_point(&behind, &points[i], &bearings[i]);
		math_vec3_normalize(&bearings[i]);
	}
	CHECK_FALSE(constellation_constrained_heading_two_point(points, bearings, &q, 0.001745329, out, nullptr));
	CHECK(constellation_constrained_gravity_two_point(points, bearings, &q, &gravity, 0.001745329, out, nullptr) == 0);
	points[1] = {0, 0.05f, 0};
	bearings[0] = {0, 0, 1};
	bearings[1] = {0, 0.05f, 0.6f};
	math_vec3_normalize(&bearings[1]);
	CHECK(constellation_constrained_gravity_two_point(points, bearings, &q, &gravity, 0.001745329, out, nullptr) == 0);
}

TEST_CASE("Gravity acquisition jointly verifies two plus one calibrated observations")
{
	Scene s;
	s.observe(0, {0, 1});
	s.observe(1, {2});
	constellation_constrained_result result;
	REQUIRE(
	    constellation_constrained_search(&s.model, s.views.data(), 2, &s.tilt, false, false, &s.config, &result));
	CHECK(result.inliers == 3);
	CHECK(result.distinct_leds == 3);
	CHECK(result.information_rank == 4);
	CHECK(result.vision_determines_yaw);
	CHECK(distance(result.P_world_model.position, s.truth.position) < 0.0003);
	CHECK(angle(result.P_world_model.orientation, s.truth.orientation) < 0.001);
	CHECK(result.camera_diagnostics[0].inliers == 2);
	CHECK(result.camera_diagnostics[1].inliers == 1);
	CHECK(result.assignments[0].led_id == 11);
	CHECK(result.assignments[1].led_id == 7);
	CHECK(result.assignments[2].led_id == 23);
	constellation_constrained_result reversed;
	std::reverse(s.views.begin(), s.views.end());
	REQUIRE(
	    constellation_constrained_search(&s.model, s.views.data(), 2, &s.tilt, false, false, &s.config, &reversed));
	CHECK(distance(result.P_world_model.position, reversed.P_world_model.position) < 1e-6);
	CHECK(result.cost_m == Catch::Approx(reversed.cost_m).margin(1e-9));
	CHECK(result.pair_trials == reversed.pair_trials);
}

TEST_CASE("Known capture heading gives translation with three observations")
{
	Scene s;
	// A vertical model baseline observes no heading: the gravity path must report unresolved yaw.
	s.leds[0].pos = {0, -0.02f, 0};
	s.leds[1].pos = {0, 0.03f, 0};
	s.observe(0, {0, 1});
	s.observe(1, {0});
	s.tilt.orientation = s.truth.orientation;
	constellation_constrained_result result;
	CHECK_FALSE(
	    constellation_constrained_search(&s.model, s.views.data(), 2, &s.tilt, false, false, &s.config, &result));
	CHECK(result.unresolved_yaw_pairs > 0);
	REQUIRE(
	    constellation_constrained_search(&s.model, s.views.data(), 2, &s.tilt, true, false, &s.config, &result));
	CHECK(result.origin == CONSTELLATION_CONSTRAINED_HEADING);
	CHECK(result.inliers == 3);
	CHECK(result.distinct_leds == 2);
	CHECK(result.information_rank == 3);
	CHECK_FALSE(result.vision_determines_yaw);
	CHECK(std::isinf(result.covariance[15]));
	CHECK(distance(result.P_world_model.position, s.truth.position) < 0.0001);
}

TEST_CASE("Heading hypotheses claim optical yaw only when the fit observes it")
{
	Scene s;
	s.observe(0, {0, 1});
	s.observe(1, {2});
	s.tilt.orientation = s.truth.orientation;
	constellation_constrained_result result;
	REQUIRE(
	    constellation_constrained_search(&s.model, s.views.data(), 2, &s.tilt, true, false, &s.config, &result));
	CHECK(result.heading_hypotheses > 0);
	CHECK(result.information_rank == 4);
	CHECK(result.vision_determines_yaw);
	CHECK(std::isfinite(result.covariance[15]));
	CHECK(distance(result.P_world_model.position, s.truth.position) < 0.0001);
}

TEST_CASE("Gravity hypotheses recover three observations despite incorrect heading yaw")
{
	Scene s;
	s.observe(0, {0, 1});
	s.observe(1, {2});
	s.tilt.orientation = rotation(-0.8, {0, 1, 0});
	constellation_constrained_result result;
	CHECK_FALSE(constellation_constrained_verify_pose(&s.model, s.views.data(), 2, &s.tilt,
	                                                  CONSTELLATION_CONSTRAINED_HEADING, &s.config, &result));
	REQUIRE(
	    constellation_constrained_search(&s.model, s.views.data(), 2, &s.tilt, true, false, &s.config, &result));
	CHECK(result.origin == CONSTELLATION_CONSTRAINED_GRAVITY);
	CHECK(result.inliers == 3);
	CHECK(result.information_rank == 4);
	CHECK(distance(result.P_world_model.position, s.truth.position) < 0.0003);
	CHECK(angle(result.P_world_model.orientation, s.truth.orientation) < 0.001);
}

TEST_CASE("Prior path counts repeated LED camera observations and requires geometric rank")
{
	Scene s;
	s.observe(0, {0});
	s.observe(1, {0});
	constellation_constrained_result result;
	REQUIRE(
	    constellation_constrained_search(&s.model, s.views.data(), 2, &s.truth, true, true, &s.config, &result));
	CHECK(result.origin == CONSTELLATION_CONSTRAINED_PRIOR);
	CHECK(result.inliers == 2);
	CHECK(result.distinct_leds == 1);
	CHECK(result.information_rank == 3);
	CHECK(result.assignments[0].led_index == result.assignments[1].led_index);
	CHECK(result.assignments[0].camera_index != result.assignments[1].camera_index);
	CHECK_FALSE(
	    constellation_constrained_search(&s.model, s.views.data(), 1, &s.truth, true, true, &s.config, &result));
}

TEST_CASE("Gravity verification counts repeated physical LEDs across calibrated cameras")
{
	Scene s;
	s.observe(0, {0, 1});
	s.observe(1, {0});
	constellation_constrained_result result;
	REQUIRE(constellation_constrained_verify_pose(&s.model, s.views.data(), 2, &s.truth,
	                                              CONSTELLATION_CONSTRAINED_GRAVITY, &s.config, &result));
	CHECK(result.inliers == 3);
	CHECK(result.distinct_leds == 2);
	CHECK(result.information_rank == 4);
	CHECK(result.num_assignments == 3);
}

TEST_CASE("Wrong geometry normal visibility and duplicate LED identities do not pass low-count verification")
{
	Scene s;
	s.observe(0, {0, 1});
	s.observe(1, {2});
	constellation_constrained_result result;
	s.leds[2].pos.y += 0.02f;
	CHECK_FALSE(constellation_constrained_verify_pose(&s.model, s.views.data(), 2, &s.truth,
	                                                  CONSTELLATION_CONSTRAINED_GRAVITY, &s.config, &result));
	s.leds[2].pos.y -= 0.02f;
	for (auto &led : s.leds)
		led.dir.z = 1;
	CHECK_FALSE(
	    constellation_constrained_search(&s.model, s.views.data(), 2, &s.tilt, false, false, &s.config, &result));
	for (auto &led : s.leds)
		led.dir.z = -1;
	s.leds[2].id = s.leds[0].id;
	CHECK_FALSE(
	    constellation_constrained_search(&s.model, s.views.data(), 2, &s.tilt, false, false, &s.config, &result));
	CHECK(result.rejection == CONSTELLATION_CONSTRAINED_INVALID_INPUT);
}

TEST_CASE("Mirrored geometry cannot acquire from the calibrated three-observation fixture")
{
	Scene s;
	s.observe(0, {0, 1});
	s.observe(1, {2});
	for (auto &led : s.leds) {
		led.pos.x = -led.pos.x;
		led.dir.x = -led.dir.x;
	}
	constellation_constrained_result result;
	CHECK_FALSE(
	    constellation_constrained_search(&s.model, s.views.data(), 2, &s.tilt, false, false, &s.config, &result));
}

TEST_CASE("Competing P3P poses and shared blobs retain ambiguity and ownership evidence")
{
	Scene s;
	s.observe(0, {0, 1});
	s.observe(1, {0, 2});
	std::array<constellation_constrained_result, 2> hypotheses;
	REQUIRE(constellation_constrained_verify_pose(&s.model, s.views.data(), 2, &s.truth,
	                                              CONSTELLATION_CONSTRAINED_P3P, &s.config, &hypotheses[0]));
	hypotheses[1] = hypotheses[0];
	hypotheses[1].P_world_model.position.x += 0.1f;
	hypotheses[1].assignments[0].led_index = 1;
	constellation_constrained_result selected;
	CHECK_FALSE(constellation_constrained_select_results(hypotheses.data(), 2, &s.config, &selected));
	CHECK(selected.rejection == CONSTELLATION_CONSTRAINED_AMBIGUOUS);
	CHECK(selected.ambiguity_likelihood_gap == Catch::Approx(0));
	CHECK(constellation_constrained_results_share_blob(&hypotheses[0], &hypotheses[1]));
	for (unsigned i = 0; i < hypotheses[1].num_assignments; i++)
		hypotheses[1].assignments[i].camera_index += 10;
	CHECK_FALSE(constellation_constrained_results_share_blob(&hypotheses[0], &hypotheses[1]));
	hypotheses[1] = hypotheses[0];
	REQUIRE(constellation_constrained_select_results(hypotheses.data(), 2, &s.config, &selected));
}

TEST_CASE("P3P joint verification requires more observations than its three-point sample")
{
	Scene s;
	s.observe(0, {0, 1});
	s.observe(1, {2});
	constellation_constrained_result result;
	CHECK_FALSE(constellation_constrained_verify_pose(&s.model, s.views.data(), 2, &s.truth,
	                                                  CONSTELLATION_CONSTRAINED_P3P, &s.config, &result));
	CHECK(result.rejection == CONSTELLATION_CONSTRAINED_TOO_FEW_INLIERS);
	s.observe(1, {0, 2});
	REQUIRE(constellation_constrained_verify_pose(&s.model, s.views.data(), 2, &s.truth,
	                                              CONSTELLATION_CONSTRAINED_P3P, &s.config, &result));
	CHECK(result.inliers == 4);
}

TEST_CASE("Symmetric anonymous low-count acquisition rejects competing assignments")
{
	Scene s;
	std::array<t_constellation_led, 6> symmetric{};
	for (unsigned i = 0; i < 6; i++) {
		symmetric[i] = s.leds[i % 3];
		symmetric[i].id = i;
		if (i >= 3)
			symmetric[i].pos.x += 0.2f;
	}
	s.model.leds = symmetric.data();
	s.model.num_leds = symmetric.size();
	s.truth.orientation = {0, 0, 0, 1};
	s.truth.position = {0, 0, 0.6f};
	for (unsigned i = 0; i < 3; i++) {
		blob &b = s.observations[0].blobs[i];
		REQUIRE(t_camera_models_project(&s.camera.calib, symmetric[i].pos.x, symmetric[i].pos.y,
		                                symmetric[i].pos.z + 0.6f, &b.x, &b.y));
	}
	s.observations[0].num_blobs = 3;
	constellation_constrained_result result;
	CHECK_FALSE(
	    constellation_constrained_search(&s.model, s.views.data(), 1, &s.tilt, false, false, &s.config, &result));
	CHECK(result.rejection == CONSTELLATION_CONSTRAINED_AMBIGUOUS);
}

TEST_CASE("Coincident different LED identities cannot be silently assigned by a greedy score")
{
	Scene s;
	s.observe(0, {0, 1});
	s.observe(1, {2});
	std::array<t_constellation_led, 4> leds = {s.leds[0], s.leds[1], s.leds[2], s.leds[2]};
	leds[3].id = 99;
	s.model.leds = leds.data();
	s.model.num_leds = leds.size();
	constellation_constrained_result result;
	CHECK_FALSE(constellation_constrained_verify_pose(&s.model, s.views.data(), 2, &s.truth,
	                                                  CONSTELLATION_CONSTRAINED_GRAVITY, &s.config, &result));
	CHECK(result.rejection == CONSTELLATION_CONSTRAINED_AMBIGUOUS);
}

namespace {
#include "constellation_rift_s_data.hpp"

struct Rig
{
	camera_model camera{};
	std::vector<blobservation> observations;
	std::vector<constellation_constrained_view> views;
	explicit Rig(unsigned count)
	{
		camera.width = 640;
		camera.height = 480;
		camera.calib.fx = rift_s_ring_intrinsics[0];
		camera.calib.fy = rift_s_ring_intrinsics[1];
		camera.calib.cx = rift_s_ring_intrinsics[2];
		camera.calib.cy = rift_s_ring_intrinsics[3];
		camera.calib.model = T_DISTORTION_FISHEYE_KB4;
		camera.calib.fisheye = {rift_s_ring_kb4[0], rift_s_ring_kb4[1], rift_s_ring_kb4[2], rift_s_ring_kb4[3]};
		observations.resize(count);
		views.resize(count);
		const float yaw[] = {0, -0.6f, 0.6f, 0.3f, -0.3f};
		const float pitch[] = {0, 0.1f, 0.1f, -0.7f, 0.5f};
		for (unsigned k = 0; k < count; k++) {
			auto &v = views[k];
			v.camera_index = k;
			v.calib = &camera;
			v.observation = &observations[k];
			v.P_world_cam.orientation =
			    multiply(rotation(yaw[k], {0, 1, 0}), rotation(pitch[k], {1, 0, 0}));
			v.P_world_cam.position = {0.04f * k - 0.05f, 0.01f * k, 0};
			math_pose_invert(&v.P_world_cam, &v.P_cam_world);
		}
	}
	void
	clear()
	{
		for (auto &o : observations)
			o = {};
	}
	/* Project an LED into a view; false when it faces away or leaves the frame. */
	bool
	project(unsigned k, const xrt_pose &P_world_model, const t_constellation_led &led, float &x, float &y,
	        double facing_cos) const
	{
		xrt_vec3 world, local, normal_world, normal;
		math_pose_transform_point(&P_world_model, &led.pos, &world);
		math_pose_transform_point(&views[k].P_cam_world, &world, &local);
		math_quat_rotate_vec3(&P_world_model.orientation, &led.dir, &normal_world);
		math_quat_rotate_vec3(&views[k].P_cam_world.orientation, &normal_world, &normal);
		xrt_vec3 to_camera = {-local.x, -local.y, -local.z};
		math_vec3_normalize(&to_camera);
		if (local.z < 0.05f || m_vec3_dot(normal, to_camera) < facing_cos)
			return false;
		if (!t_camera_models_project(&camera.calib, local.x, local.y, local.z, &x, &y))
			return false;
		return x >= 0 && y >= 0 && x < camera.width && y < camera.height;
	}
	void
	add_blob(unsigned k, float x, float y)
	{
		auto &o = observations[k];
		blob &b = o.blobs[o.num_blobs++];
		b = {};
		b.x = x;
		b.y = y;
		b.width = b.height = 3;
		b.led_id = LED_INVALID_ID;
	}
};

xrt_pose
world_tilt_prior(const xrt_quat &truth, double yaw, double tilt_error, std::mt19937 &rng)
{
	std::uniform_real_distribution<float> unit(-1, 1);
	xrt_pose prior = XRT_POSE_IDENTITY;
	prior.position = {42, 42, 42};
	xrt_quat tilt_noise = rotation(tilt_error, {unit(rng), 0, unit(rng)});
	prior.orientation = multiply(tilt_noise, multiply(rotation(yaw, {0, 1, 0}), truth));
	return prior;
}

struct MonteCarlo
{
	unsigned accepted = 0, wrong = 0, rejected = 0, trials = 0;
	std::array<unsigned, 16> reasons{};
	double worst_error_m = 0;
	double worst_identity_error_m = 0;
	double worst_identity_error_sigma = 0;
};

/* Random worn-like views: a partial ring of 3..6 observations spread over two cameras,
 * background blobs, centroid noise and a small IMU tilt error. Cold acquisition, unknown yaw. */
MonteCarlo
run_cold_acquisition(bool mirror_model,
                     unsigned distractors,
                     unsigned trials,
                     uint32_t seed,
                     bool independent_tilt = false,
                     bool owner_contests = false,
                     bool recovered_minimum_only = false)
{
	std::mt19937 rng(seed);
	std::uniform_real_distribution<float> unit(-1, 1), positive(0, 1);
	std::normal_distribution<float> noise(0, 0.3f);
	auto truth_leds = rift_s_ring_leds;
	auto model_leds = rift_s_ring_leds;
	if (mirror_model)
		for (auto &l : model_leds) {
			l.pos.x = -l.pos.x;
			l.dir.x = -l.dir.x;
		}
	t_constellation_led_model model{};
	model.id = 3;
	model.leds = model_leds.data();
	model.num_leds = model_leds.size();
	t_constellation_led_model owner{};
	owner.id = 2;
	owner.leds = truth_leds.data();
	owner.num_leds = truth_leds.size();
	Rig rig(2);
	constellation_constrained_config config;
	constellation_constrained_default_config(&config);
	config.time_budget_us = 200000;
	if (recovered_minimum_only) {
		config.minimum_cold_inliers = 3;
		config.minimum_cold_distinct_leds = 0;
		config.maximum_cold_unexplained_blobs = 1000;
		config.bearing_noise_std_rad = 0;
	}
	MonteCarlo stats;
	while (stats.trials < trials) {
		xrt_pose truth = XRT_POSE_IDENTITY;
		truth.orientation = rotation(3.14159f * positive(rng), {unit(rng), unit(rng), unit(rng)});
		float depth = 0.25f + 0.45f * positive(rng);
		truth.position = {unit(rng) * 0.4f * depth, unit(rng) * 0.3f * depth, depth};
		rig.clear();
		std::vector<std::pair<unsigned, unsigned>> visible;
		for (unsigned k = 0; k < 2; k++)
			for (unsigned l = 0; l < truth_leds.size(); l++) {
				float x, y;
				if (rig.project(k, truth, truth_leds[l], x, y, std::cos(70 * M_PI / 180)))
					visible.push_back({k, l});
			}
		if (visible.size() < 3)
			continue;
		std::shuffle(visible.begin(), visible.end(), rng);
		unsigned keep = std::min<unsigned>(visible.size(), 3 + rng() % 4);
		visible.resize(keep);
		std::array<std::vector<int>, 2> truth_led;
		for (auto [k, l] : visible) {
			float x, y;
			rig.project(k, truth, truth_leds[l], x, y, -1);
			rig.add_blob(k, x + noise(rng), y + noise(rng));
			truth_led[k].push_back(int(l));
		}
		for (unsigned k = 0; k < 2; k++)
			for (unsigned d = 0; d < distractors; d++) {
				rig.add_blob(k, 640 * positive(rng), 480 * positive(rng));
				truth_led[k].push_back(-1);
			}
		stats.trials++;
		xrt_pose prior = world_tilt_prior(truth.orientation, 6.28 * positive(rng), 0.015, rng);
		if (independent_tilt)
			prior.orientation = rotation(3.14159f * positive(rng), {unit(rng), unit(rng), unit(rng)});
		constellation_constrained_result result;
		bool ok = constellation_constrained_search(&model, rig.views.data(), 2, &prior, false, false, &config,
		                                           &result);
		if (owner_contests) {
			// The ring's own controller searches the same exposure with its own tilt.
			xrt_pose owner_prior = world_tilt_prior(truth.orientation, 6.28 * positive(rng), 0.015, rng);
			constellation_constrained_result owner_result;
			bool owner_ok = constellation_constrained_search(&owner, rig.views.data(), 2, &owner_prior, false,
			                                                 false, &config, &owner_result);
			const constellation_constrained_result *claims[2] = {&result, &owner_result};
			bool published[2] = {false, false}, candidates[2] = {ok, owner_ok};
			constellation_constrained_claim blocked[2];
			constellation_constrained_arbitrate(claims, published, candidates, blocked, 2, 2);
			if (ok && !candidates[0])
				result.rejection = CONSTELLATION_CONSTRAINED_SHARED_BLOB;
			ok = candidates[0];
		}
		if (!ok) {
			stats.rejected++;
			stats.reasons[result.rejection]++;
			continue;
		}
		bool identity = true;
		for (unsigned i = 0; i < result.num_assignments; i++) {
			const auto &a = result.assignments[i];
			identity &= truth_led[a.camera_index][a.blob_index] == int(a.led_index);
		}
		double error = distance(result.P_world_model.position, truth.position);
		double sigma = 0;
		for (unsigned i = 0; i < 3; i++)
			sigma += result.covariance[5 * i];
		sigma = std::sqrt(sigma);
		if (!identity) {
			stats.wrong++;
			stats.worst_error_m = std::max(stats.worst_error_m, error);
		} else {
			stats.accepted++;
			stats.worst_identity_error_sigma = std::max(stats.worst_identity_error_sigma, error / sigma);
			stats.worst_identity_error_m = std::max(stats.worst_identity_error_m, error);
		}
	}
	return stats;
}

struct Scenario
{
	float depth_min = 0.25f, depth_max = 0.70f;
	//! IMU tilt error of the prior, radians.
	double tilt_error = 0.015;
	//! Trusted prior: position within prior_error_m of the truth, yaw known within heading_error.
	bool trusted = false;
	float prior_error_m = 0.03f;
	bool heading = false;
	double heading_error = 0.03;
	unsigned min_keep = 3, max_keep = 6;
	float pixel_noise = 0.3f;
};

/* Like run_cold_acquisition for the owner's own model, over a chosen depth range and IMU error. */
MonteCarlo
run_scenario(const Scenario &sc,
             unsigned trials,
             uint32_t seed,
             const constellation_constrained_config *override = nullptr)
{
	std::mt19937 rng(seed);
	std::uniform_real_distribution<float> unit(-1, 1), positive(0, 1);
	std::normal_distribution<float> noise(0, sc.pixel_noise);
	auto leds = rift_s_ring_leds;
	t_constellation_led_model model{};
	model.id = 3;
	model.leds = leds.data();
	model.num_leds = leds.size();
	Rig rig(2);
	constellation_constrained_config config;
	constellation_constrained_default_config(&config);
	config.time_budget_us = 200000;
	if (override)
		config = *override;
	MonteCarlo stats;
	while (stats.trials < trials) {
		xrt_pose truth = XRT_POSE_IDENTITY;
		truth.orientation = rotation(3.14159f * positive(rng), {unit(rng), unit(rng), unit(rng)});
		float depth = sc.depth_min + (sc.depth_max - sc.depth_min) * positive(rng);
		truth.position = {unit(rng) * 0.4f * depth, unit(rng) * 0.3f * depth, depth};
		rig.clear();
		std::vector<std::pair<unsigned, unsigned>> visible;
		for (unsigned k = 0; k < 2; k++)
			for (unsigned l = 0; l < leds.size(); l++) {
				float x, y;
				if (rig.project(k, truth, leds[l], x, y, std::cos(70 * M_PI / 180)))
					visible.push_back({k, l});
			}
		if (visible.size() < sc.min_keep)
			continue;
		std::shuffle(visible.begin(), visible.end(), rng);
		unsigned keep =
		    std::min<unsigned>(visible.size(), sc.min_keep + rng() % (sc.max_keep - sc.min_keep + 1));
		visible.resize(keep);
		std::array<std::vector<int>, 2> truth_led;
		for (auto [k, l] : visible) {
			float x = 0, y = 0;
			rig.project(k, truth, leds[l], x, y, -1);
			rig.add_blob(k, x + noise(rng), y + noise(rng));
			truth_led[k].push_back(int(l));
		}
		stats.trials++;
		double yaw = sc.heading ? sc.heading_error * unit(rng) : 6.28 * positive(rng);
		xrt_pose prior = world_tilt_prior(truth.orientation, yaw, sc.tilt_error, rng);
		if (sc.trusted) {
			xrt_vec3 dir = {unit(rng), unit(rng), unit(rng)};
			math_vec3_normalize(&dir);
			prior.position = {truth.position.x + sc.prior_error_m * dir.x,
			                  truth.position.y + sc.prior_error_m * dir.y,
			                  truth.position.z + sc.prior_error_m * dir.z};
		}
		constellation_constrained_result result;
		if (!constellation_constrained_search(&model, rig.views.data(), 2, &prior, sc.heading, sc.trusted,
		                                      &config, &result)) {
			stats.rejected++;
			stats.reasons[result.rejection]++;
			continue;
		}
		bool identity = true;
		for (unsigned i = 0; i < result.num_assignments; i++) {
			const auto &a = result.assignments[i];
			identity &= truth_led[a.camera_index][a.blob_index] == int(a.led_index);
		}
		double error = distance(result.P_world_model.position, truth.position);
		if (!identity) {
			stats.wrong++;
			stats.worst_error_m = std::max(stats.worst_error_m, error);
		} else {
			stats.accepted++;
			stats.worst_identity_error_m = std::max(stats.worst_identity_error_m, error);
		}
	}
	return stats;
}

void
print(const char *name, const MonteCarlo &s)
{
	std::printf("%s: trials=%u correct_identity=%u wrong_identity=%u rejected=%u worst_wrong_m=%.4f "
	            "worst_correct_m=%.4f worst_correct_sigma=%.2f reasons:",
	            name, s.trials, s.accepted, s.wrong, s.rejected, s.worst_error_m, s.worst_identity_error_m,
	            s.worst_identity_error_sigma);
	for (unsigned r = 0; r < s.reasons.size(); r++)
		if (s.reasons[r])
			std::printf(" %s=%u",
			            constellation_constrained_rejection_name((constellation_constrained_rejection)r),
			            s.reasons[r]);
	std::printf("\n");
}
} // namespace

TEST_CASE("Recorded Rift S frame acquires through the gravity two-point path")
{
	auto leds = rift_s_ring_leds;
	t_constellation_led_model model{};
	model.id = 3;
	model.leds = leds.data();
	model.num_leds = leds.size();
	Rig rig(1);
	for (const auto &b : rift_s_ring_blobs)
		rig.add_blob(0, b[0], b[1]);
	// Camera gravity from the recording; world +Y must map onto it.
	xrt_vec3 up = {0, 1, 0}, gravity = rift_s_ring_gravity;
	math_quat_from_vec_a_to_vec_b(&up, &gravity, &rig.views[0].P_cam_world.orientation);
	rig.views[0].P_cam_world.position = {0, 0, 0};
	math_pose_invert(&rig.views[0].P_cam_world, &rig.views[0].P_world_cam);
	xrt_pose prior_cam = XRT_POSE_IDENTITY;
	math_quat_from_vec_a_to_vec_b(&rift_s_ring_down, &gravity, &prior_cam.orientation);
	xrt_pose prior;
	math_pose_transform(&rig.views[0].P_world_cam, &prior_cam, &prior);
	prior.position = {42, 42, 42};
	constellation_constrained_config config;
	constellation_constrained_default_config(&config);
	config.time_budget_us = 2000000;
	constellation_constrained_result result;
	bool acquired = constellation_constrained_search(&model, rig.views.data(), 1, &prior, false, false, &config, &result);
	INFO("rejection " << constellation_constrained_rejection_name(result.rejection) << " inliers " << result.inliers
	                  << " unexplained " << result.unexplained_blobs << " chi2 " << result.fit_chi_square << "/"
	                  << result.fit_chi_square_limit);
	REQUIRE(acquired);
	CHECK(result.origin == CONSTELLATION_CONSTRAINED_GRAVITY);
	CHECK(result.inliers == 5);
	CHECK(result.distinct_leds == 5);
	CHECK(result.vision_determines_yaw);
	xrt_pose cam_model;
	math_pose_transform(&rig.views[0].P_cam_world, &result.P_world_model, &cam_model);
	// The independent P3P search in tests_constellation finds the same ring at 0.220 m. This fit keeps
	// the measured accelerometer tilt and minimizes object-space ray distance, so allow a few mm.
	CHECK(cam_model.position.z == Catch::Approx(0.220).margin(0.008));
	CHECK(result.cost_m < config.score_gate_m);
	CHECK_FALSE(result.budget_exhausted);

	// Upside-down tilt has no consistent solution.
	xrt_vec3 opposite = {-rift_s_ring_down.x, -rift_s_ring_down.y, -rift_s_ring_down.z};
	math_quat_from_vec_a_to_vec_b(&opposite, &gravity, &prior_cam.orientation);
	math_pose_transform(&rig.views[0].P_world_cam, &prior_cam, &prior);
	CHECK_FALSE(
	    constellation_constrained_search(&model, rig.views.data(), 1, &prior, false, false, &config, &result));

	// The mirrored model must not explain the left ring.
	for (auto &l : leds) {
		l.pos.x = -l.pos.x;
		l.dir.x = -l.dir.x;
	}
	math_quat_from_vec_a_to_vec_b(&rift_s_ring_down, &gravity, &prior_cam.orientation);
	math_pose_transform(&rig.views[0].P_world_cam, &prior_cam, &prior);
	CHECK_FALSE(
	    constellation_constrained_search(&model, rig.views.data(), 1, &prior, false, false, &config, &result));
}

TEST_CASE("Cold low-count acquisition never accepts a wrong pose in randomized partial-ring views")
{
	MonteCarlo clean = run_cold_acquisition(false, 0, 300, 0x5eed1);
	print("clean", clean);
	CHECK(clean.wrong == 0);
	CHECK(clean.accepted > clean.trials / 2);
	CHECK(clean.worst_identity_error_sigma < 4);

	// Background lights: an identity slip is tolerated only when it leaves the pose within 1 cm.
	MonteCarlo cluttered = run_cold_acquisition(false, 4, 300, 0x5eed2);
	print("cluttered", cluttered);
	CHECK(cluttered.wrong <= cluttered.trials / 100);
	CHECK(cluttered.worst_error_m < 0.01);
	CHECK(cluttered.accepted > cluttered.trials / 3);

	// The other hand's (mirrored) model on this ring. Few observations of a self-similar ring cannot
	// always separate handedness geometrically, so most fits must be rejected on their own and the
	// rest by blob ownership once the ring's own controller searches the same exposure.
	MonteCarlo same_tilt = run_cold_acquisition(true, 2, 300, 0x5eed3);
	print("other-hand-same-tilt", same_tilt);
	CHECK(same_tilt.wrong <= same_tilt.trials / 4);
	MonteCarlo other_hand = run_cold_acquisition(true, 2, 300, 0x5eed4, true);
	print("other-hand-independent-tilt", other_hand);
	CHECK(other_hand.wrong <= other_hand.trials / 10);
	MonteCarlo arbitrated = run_cold_acquisition(true, 2, 300, 0x5eed5, true, true);
	print("other-hand-after-arbitration", arbitrated);
	CHECK(arbitrated.wrong == 0);
	MonteCarlo arbitrated_same_tilt = run_cold_acquisition(true, 2, 300, 0x5eed6, false, true);
	print("other-hand-same-tilt-after-arbitration", arbitrated_same_tilt);
	CHECK(arbitrated_same_tilt.wrong == 0);
}

TEST_CASE("Stock minimum count alone admits wrong identities that the cold policy rejects")
{
	// Documents why the cold policy exists: inliers > sample size is necessary, not sufficient.
	constellation_constrained_config minimum;
	constellation_constrained_default_config(&minimum);
	CHECK(minimum.minimum_cold_inliers == 4);
	MonteCarlo strict = run_cold_acquisition(false, 4, 200, 0x5eed7);
	MonteCarlo relaxed = run_cold_acquisition(false, 4, 200, 0x5eed7, false, false, true);
	print("cluttered-recovered-minimum-only", relaxed);
	CHECK(relaxed.wrong > strict.wrong);
	CHECK(strict.wrong <= 2);
}

namespace {
struct RingScene
{
	std::array<t_constellation_led, 15> leds = rift_s_ring_leds;
	t_constellation_led_model model{};
	Rig rig;
	xrt_pose truth = XRT_POSE_IDENTITY;
	std::array<std::vector<int>, CONSTELLATION_CONSTRAINED_MAX_VIEWS> truth_led;
	constellation_constrained_config config;
	explicit RingScene(unsigned cameras) : rig(cameras)
	{
		model.id = 3;
		model.leds = leds.data();
		model.num_leds = leds.size();
		constellation_constrained_default_config(&config);
		config.time_budget_us = 2000000;
	}
	unsigned
	observe(unsigned camera, unsigned max_leds)
	{
		unsigned n = 0;
		for (unsigned l = 0; l < leds.size() && n < max_leds; l++) {
			float x, y;
			if (!rig.project(camera, truth, leds[l], x, y, std::cos(70 * M_PI / 180)))
				continue;
			rig.add_blob(camera, x, y);
			truth_led[camera].push_back(int(l));
			n++;
		}
		return n;
	}
	//! Deterministically pick an orientation where each camera sees at least need[k] LEDs.
	void
	choose_pose(xrt_vec3 position, std::initializer_list<unsigned> need, uint32_t seed)
	{
		std::mt19937 rng(seed);
		std::uniform_real_distribution<float> unit(-1, 1);
		truth.position = position;
		for (unsigned attempt = 0; attempt < 10000; attempt++) {
			truth.orientation = rotation(3.14159f * (unit(rng) + 1) / 2, {unit(rng), unit(rng), unit(rng)});
			unsigned k = 0;
			bool ok = true;
			for (unsigned n : need) {
				unsigned seen = 0;
				for (const auto &led : leds) {
					float x, y;
					seen += rig.project(k, truth, led, x, y, std::cos(70 * M_PI / 180));
				}
				ok &= seen >= n;
				k++;
			}
			if (ok)
				return;
		}
		FAIL("no orientation satisfies the visibility request");
	}
	bool
	identity(const constellation_constrained_result &r) const
	{
		for (unsigned i = 0; i < r.num_assignments; i++)
			if (truth_led[r.assignments[i].camera_index][r.assignments[i].blob_index] !=
			    int(r.assignments[i].led_index))
				return false;
		return r.num_assignments > 0;
	}
};
} // namespace

TEST_CASE("Trusted prior reacquires after hand motion with the tracking minimum count")
{
	RingScene s(2);
	// This fixture checks full reacquisition, including every competing prior pair.
	s.config.max_prior_pair_trials = 10000;
	s.choose_pose({0.03f, 0.06f, 0.42f}, {4}, 1);
	REQUIRE(s.observe(0, 3) == 3);
	xrt_pose prior = s.truth;
	prior.position.x += 0.05f; // Held optical position lags a moving hand.
	prior.position.z -= 0.03f;
	constellation_constrained_result result;
	// Three observations from one camera no longer publish, even with a trusted prior.
	CHECK_FALSE(
	    constellation_constrained_search(&s.model, s.rig.views.data(), 2, &prior, false, true, &s.config, &result));
	CHECK(result.rejection == CONSTELLATION_CONSTRAINED_TOO_FEW_INLIERS);
	CHECK(result.inliers == 3);

	// Four observations of distinct LEDs meet the tracking minimum.
	s.rig.observations[0].num_blobs = 0;
	s.truth_led[0].clear();
	REQUIRE(s.observe(0, 4) == 4);
	REQUIRE(constellation_constrained_search(&s.model, s.rig.views.data(), 2, &prior, false, true, &s.config,
	                                         &result));
	CHECK(s.identity(result));
	CHECK(result.inliers == 4);
	CHECK(distance(result.P_world_model.position, s.truth.position) < 0.005);
	// With heading, the same frame also yields translation hypotheses.
	REQUIRE(constellation_constrained_search(&s.model, s.rig.views.data(), 2, &prior, true, true, &s.config,
	                                         &result));
	CHECK(result.heading_hypotheses > 0);
	CHECK(s.identity(result));
	// A prior far from the ring finds no edges inside the broad association gate.
	prior.position.x += 0.5f;
	CHECK_FALSE(constellation_constrained_search(&s.model, s.rig.views.data(), 2, &prior, true, true, &s.config,
	                                             &result));
	CHECK(result.pair_trials == 0);
}

TEST_CASE("Joint camera result does not depend on camera iteration order")
{
	RingScene s(3);
	s.choose_pose({0.0f, 0.05f, 0.45f}, {2, 2, 2}, 2);
	unsigned total = 0;
	for (unsigned k = 0; k < 3; k++)
		total += s.observe(k, 2);
	REQUIRE(total >= 4);
	std::mt19937 rng(9);
	xrt_pose prior = world_tilt_prior(s.truth.orientation, 2.0, 0, rng);
	constellation_constrained_result reference;
	REQUIRE(constellation_constrained_search(&s.model, s.rig.views.data(), 3, &prior, false, false, &s.config,
	                                         &reference));
	CHECK(s.identity(reference));
	std::array<unsigned, 3> order = {0, 1, 2};
	unsigned permutations = 0;
	do {
		std::vector<constellation_constrained_view> views;
		for (unsigned k : order)
			views.push_back(s.rig.views[k]);
		constellation_constrained_result result;
		REQUIRE(constellation_constrained_search(&s.model, views.data(), 3, &prior, false, false, &s.config,
		                                         &result));
		CHECK(distance(result.P_world_model.position, reference.P_world_model.position) < 1e-6);
		CHECK(angle(result.P_world_model.orientation, reference.P_world_model.orientation) < 1e-5);
		CHECK(result.cost_m == Catch::Approx(reference.cost_m).margin(1e-12));
		CHECK(result.inliers == reference.inliers);
		CHECK(result.pair_trials == reference.pair_trials);
		for (unsigned i = 0; i < result.num_assignments; i++) {
			CHECK(result.assignments[i].camera_index == reference.assignments[i].camera_index);
			CHECK(result.assignments[i].blob_index == reference.assignments[i].blob_index);
			CHECK(result.assignments[i].led_index == reference.assignments[i].led_index);
		}
		permutations++;
	} while (std::next_permutation(order.begin(), order.end()));
	CHECK(permutations == 6);
}

TEST_CASE("Cross-controller arbitration resolves blob ownership before fusion")
{
	constellation_constrained_result left{}, right{}, idle{};
	left.num_assignments = right.num_assignments = 2;
	left.assignments[0] = {0, 4, 1, 1, 0};
	left.assignments[1] = {1, 7, 2, 2, 0};
	right.assignments[0] = {0, 5, 3, 3, 0};
	right.assignments[1] = {1, 7, 4, 4, 0}; // Same blob 7 in camera 1.
	left.inliers = right.inliers = 5;
	left.distinct_leds = right.distinct_leds = 4;
	const constellation_constrained_result *claims[3] = {&left, &right, &idle};
	bool published[3] = {false, false, false}, candidates[3] = {true, true, false};
	constellation_constrained_claim blocked[3];

	// Equal support: neither candidate can own the blob.
	constellation_constrained_arbitrate(claims, published, candidates, blocked, 3, 2);
	CHECK_FALSE(candidates[0]);
	CHECK_FALSE(candidates[1]);
	CHECK(blocked[0] == CONSTELLATION_CLAIM_CANDIDATE);
	CHECK(blocked[1] == CONSTELLATION_CLAIM_CANDIDATE);
	CHECK(blocked[2] == CONSTELLATION_CLAIM_NONE);

	// Clearly more support keeps the blob, independent of device order.
	for (bool reversed : {false, true}) {
		left.inliers = 7;
		left.distinct_leds = 5;
		const constellation_constrained_result *ordered[2] = {reversed ? &right : &left,
		                                                      reversed ? &left : &right};
		bool pub[2] = {false, false}, cand[2] = {true, true};
		constellation_constrained_claim why[2];
		constellation_constrained_arbitrate(ordered, pub, cand, why, 2, 2);
		CHECK(cand[reversed ? 1 : 0]);
		CHECK_FALSE(cand[reversed ? 0 : 1]);
	}
	left.inliers = 5;
	left.distinct_leds = 4;

	// A published fast-path result keeps its blobs; only the new claim loses.
	published[0] = true;
	candidates[0] = false;
	candidates[1] = true;
	constellation_constrained_arbitrate(claims, published, candidates, blocked, 3, 2);
	CHECK(blocked[0] == CONSTELLATION_CLAIM_NONE);
	CHECK(blocked[1] == CONSTELLATION_CLAIM_PUBLISHED);
	CHECK_FALSE(candidates[1]);

	// The published pose keeps the blob this exposure, but a clearly better-supported candidate
	// contests it so its track can end; an equal one does not.
	CHECK_FALSE(constellation_constrained_contests_published(&right, &left, 2));
	right.inliers = 7;
	right.distinct_leds = 5;
	CHECK(constellation_constrained_contests_published(&right, &left, 2));
	CHECK_FALSE(constellation_constrained_contests_published(&right, &idle, 2));
	right.inliers = 5;
	right.distinct_leds = 4;

	// A rejected hypothesis contests only when its fit was consistent and at least as supported.
	published[0] = false;
	for (auto reason : {CONSTELLATION_CONSTRAINED_INCONSISTENT, CONSTELLATION_CONSTRAINED_NO_PAIR,
	                    CONSTELLATION_CONSTRAINED_TOO_FEW_INLIERS, CONSTELLATION_CONSTRAINED_AMBIGUOUS}) {
		for (unsigned inliers : {4u, 5u}) {
			left.rejection = reason;
			left.inliers = inliers;
			candidates[0] = false;
			candidates[1] = true;
			constellation_constrained_arbitrate(claims, published, candidates, blocked, 3, 2);
			bool competitive = reason == CONSTELLATION_CONSTRAINED_TOO_FEW_INLIERS ||
			                   reason == CONSTELLATION_CONSTRAINED_AMBIGUOUS;
			bool contests = competitive && inliers >= right.inliers;
			CHECK(candidates[1] == !contests);
			CHECK((blocked[1] == CONSTELLATION_CLAIM_REJECTED) == contests);
		}
	}

	// No overlap, no conflict; an empty rejected result claims nothing.
	left.inliers = 5;
	right.assignments[1].blob_index = 8;
	candidates[0] = candidates[1] = true;
	constellation_constrained_arbitrate(claims, published, candidates, blocked, 3, 2);
	CHECK(candidates[0]);
	CHECK(candidates[1]);
	left.num_assignments = 0;
	right.assignments[1].blob_index = 7;
	candidates[0] = false;
	candidates[1] = true;
	constellation_constrained_arbitrate(claims, published, candidates, blocked, 3, 2);
	CHECK(candidates[1]);
}

TEST_CASE("Motion continuity rejects identity jumps and rotations the IMU did not see")
{
	constellation_motion_limits limits;
	constellation_constrained_default_motion_limits(&limits);
	constellation_track_point a{}, b{};
	a.P_world_model = XRT_POSE_IDENTITY;
	a.P_world_model.position = {0.1f, -0.2f, 0.5f};
	a.imu_world = rotation(0.3, {0, 1, 0});
	a.P_world_model.orientation = rotation(1.0, {1, 0.2f, 0});
	a.capture_ns = 1000000000;
	b = a;
	b.capture_ns = a.capture_ns + 33000000; // one controller exposure later
	double excess, rotation_error;
	CHECK(constellation_constrained_motion_consistent(&a, &b, &limits, &excess, &rotation_error));
	CHECK(rotation_error < 1e-4);

	// A fast hand: 2.5 m/s for 33 ms is fine, a 30 cm hop is not.
	b.P_world_model.position.x += 0.08f;
	CHECK(constellation_constrained_motion_consistent(&a, &b, &limits, &excess, &rotation_error));
	b.P_world_model.position.x += 0.22f;
	CHECK_FALSE(constellation_constrained_motion_consistent(&a, &b, &limits, &excess, &rotation_error));
	CHECK(excess > 0.1);
	b.P_world_model.position = a.P_world_model.position;

	// Optics and IMU turning together is consistent; optics turning alone is a wrong identity.
	xrt_quat turn = rotation(0.5, {0, 0, 1});
	b.P_world_model.orientation = multiply(a.P_world_model.orientation, turn);
	CHECK_FALSE(constellation_constrained_motion_consistent(&a, &b, &limits, &excess, &rotation_error));
	CHECK(rotation_error == Catch::Approx(0.5).margin(1e-4));
	b.imu_world = multiply(a.imu_world, turn);
	CHECK(constellation_constrained_motion_consistent(&a, &b, &limits, &excess, &rotation_error));

	// The IMU's world yaw lags the optical one (the driver corrects 10% of a large error per
	// pose). A 40 degree wrist pitch then looks like an error of tens of degrees in world coordinates, but
	// the controller turned by the same amount in its own frame.
	xrt_quat optical = rotation(0.4, {0.3f, 1, 0.2f});
	xrt_quat yaw_offset = rotation(70 * M_PI / 180, {0, 1, 0});
	xrt_quat pitch = rotation(40 * M_PI / 180, {1, 0, 0});
	a.P_world_model.orientation = optical;
	a.imu_world = multiply(yaw_offset, optical);
	b.P_world_model.orientation = multiply(optical, pitch);
	b.imu_world = multiply(a.imu_world, pitch);
	CHECK(constellation_constrained_motion_consistent(&a, &b, &limits, &excess, &rotation_error));
	CHECK(rotation_error < 1e-4);
	xrt_quat world_optical = multiply(b.P_world_model.orientation, conjugate(a.P_world_model.orientation));
	xrt_quat world_imu = multiply(b.imu_world, conjugate(a.imu_world));
	CHECK(angle(world_optical, world_imu) > 25 * M_PI / 180);

	// Stale references and time running backwards never vouch for a pose.
	b.capture_ns = a.capture_ns + 400000000;
	CHECK_FALSE(constellation_constrained_motion_consistent(&a, &b, &limits, &excess, &rotation_error));
	b.capture_ns = a.capture_ns - 1;
	CHECK_FALSE(constellation_constrained_motion_consistent(&a, &b, &limits, &excess, &rotation_error));
}

namespace {
constellation_track_point
track_point(xrt_vec3 position, xrt_quat optical, xrt_quat imu, int64_t capture_ns)
{
	constellation_track_point p{};
	p.P_world_model.position = position;
	p.P_world_model.orientation = optical;
	p.imu_world = imu;
	p.capture_ns = capture_ns;
	return p;
}
} // namespace

TEST_CASE("Track publishes only confirmed, continuous results")
{
	constellation_track track;
	constellation_track_init(&track);
	const int64_t frame = 33000000;
	xrt_quat q = rotation(0.7, {0.2f, 1, 0});
	xrt_vec3 p = {0.1f, -0.3f, 0.5f};
	auto at = [&](int k, xrt_vec3 position, xrt_quat optical, bool observed = true) {
		constellation_track_point next = track_point(position, optical, q, k * frame);
		xrt_quat ref_imu = q;
		return constellation_track_update(&track, &next, observed, &ref_imu, nullptr, nullptr);
	};

	// A cold acquisition alone is never published.
	CHECK(at(1, p, q) == CONSTELLATION_TRACK_HOLD_CONFIRMING);
	CHECK(track.state == CONSTELLATION_TRACK_CONFIRMING);
	// A wrong identity cannot confirm it: it replaces the held result instead.
	xrt_vec3 off = {p.x + 0.15f, p.y, p.z};
	CHECK(at(2, off, multiply(rotation(0.5, {0, 0, 1}), q)) == CONSTELLATION_TRACK_HOLD_UNCONFIRMED);
	CHECK(at(3, p, q) == CONSTELLATION_TRACK_HOLD_UNCONFIRMED);
	CHECK(at(4, p, q) == CONSTELLATION_TRACK_PUBLISH);
	CHECK(track.state == CONSTELLATION_TRACK_TRACKING);

	// One wrong result while tracking is held and does not move the reference.
	CHECK(at(5, off, q) == CONSTELLATION_TRACK_HOLD_JUMP);
	CHECK(track.ref.capture_ns == 4 * frame);
	CHECK(at(6, p, q) == CONSTELLATION_TRACK_PUBLISH);
	// An optical flip the gyro did not see is a jump even at the same position.
	CHECK(at(7, p, multiply(rotation(M_PI, {1, 0, 0}), q)) == CONSTELLATION_TRACK_HOLD_JUMP);
	// A position-only fit echoes the IMU, so its (unobserved) orientation is not tested.
	CHECK(at(8, p, rotation(2.0, {0, 1, 0}), false) == CONSTELLATION_TRACK_PUBLISH);

	// Repeated disagreement ends the track; the latest result must then confirm itself.
	xrt_vec3 far = {p.x - 0.4f, p.y, p.z};
	CHECK(at(9, far, q) == CONSTELLATION_TRACK_HOLD_JUMP);
	CHECK(at(10, far, q) == CONSTELLATION_TRACK_HOLD_JUMP);
	CHECK(at(11, far, q) == CONSTELLATION_TRACK_HOLD_RESTART);
	CHECK(track.state == CONSTELLATION_TRACK_CONFIRMING);
	CHECK(at(12, far, q) == CONSTELLATION_TRACK_PUBLISH);

	// Stale and gap handling.
	CHECK(at(12, far, q) == CONSTELLATION_TRACK_HOLD_STALE);
	CHECK_FALSE(constellation_track_expire(&track, 12 * frame + 300000000));
	CHECK(constellation_track_expire(&track, 12 * frame + 300000001));
	CHECK(track.state == CONSTELLATION_TRACK_LOST);
	CHECK(at(30, off, q) == CONSTELLATION_TRACK_HOLD_CONFIRMING);
	CHECK(at(31, off, q) == CONSTELLATION_TRACK_PUBLISH);
	constellation_track_reset(&track);
	CHECK(track.state == CONSTELLATION_TRACK_LOST);
	CHECK(at(32, off, q) == CONSTELLATION_TRACK_HOLD_CONFIRMING);

	// Without the reference attitude nothing vouches for the next result.
	constellation_track_point next = track_point(off, q, q, 33 * frame);
	CHECK(constellation_track_update(&track, &next, true, nullptr, nullptr, nullptr) ==
	      CONSTELLATION_TRACK_HOLD_UNCONFIRMED);
}

TEST_CASE("Track continuity compares IMU attitudes read after a world-yaw correction")
{
	constellation_track track;
	constellation_track_init(&track);
	const int64_t frame = 33000000;
	xrt_quat optical0 = rotation(0.3, {1, 0, 0}), turn = rotation(0.05, {0, 1, 1});
	xrt_quat optical1 = multiply(turn, optical0);
	// The IMU heading is 30 degrees off until the published optical yaw corrects its history.
	xrt_quat yaw_error = rotation(30 * M_PI / 180, {0, 1, 0});
	xrt_vec3 p = {0, 0, 0.6f};
	constellation_track_point first = track_point(p, optical0, multiply(yaw_error, optical0), frame);
	xrt_quat stale_ref = first.imu_world;
	CHECK(constellation_track_update(&track, &first, true, nullptr, nullptr, nullptr) ==
	      CONSTELLATION_TRACK_HOLD_CONFIRMING);
	constellation_track_point second = track_point(p, optical1, multiply(yaw_error, optical1), 2 * frame);
	CHECK(constellation_track_update(&track, &second, true, &stale_ref, nullptr, nullptr) ==
	      CONSTELLATION_TRACK_PUBLISH);
	// Publishing applies the correction to the whole history: both reads now agree with optics.
	xrt_quat optical2 = multiply(turn, optical1);
	constellation_track_point third = track_point(p, optical2, optical2, 3 * frame);
	double excess, rotation_error;
	xrt_quat uncorrected = multiply(yaw_error, optical1);
	constellation_track copy = track;
	CHECK(constellation_track_update(&copy, &third, true, &uncorrected, &excess, &rotation_error) ==
	      CONSTELLATION_TRACK_HOLD_JUMP);
	CHECK(rotation_error == Catch::Approx(30 * M_PI / 180).margin(1e-3));
	CHECK(constellation_track_update(&track, &third, true, &optical1, &excess, &rotation_error) ==
	      CONSTELLATION_TRACK_PUBLISH);
	CHECK(rotation_error < 1e-3);
}

TEST_CASE("Track changes yaw observation source without comparing different world headings")
{
	const int64_t frame = 33000000;
	const xrt_vec3 position = {0, 0, 0.6f};
	const xrt_quat optical = rotation(0.3, {1, 0, 0});
	const xrt_quat imu = multiply(rotation(0.7, {0, 1, 0}), optical);
	constellation_track track;
	constellation_track_init(&track);
	auto update = [&](int k, bool observed, xrt_vec3 p = {0, 0, 0.6f}) {
		auto next = track_point(p, optical, imu, k * frame);
		return constellation_track_update(&track, &next, observed, &imu, nullptr, nullptr);
	};
	CHECK(update(1, true) == CONSTELLATION_TRACK_HOLD_CONFIRMING);
	CHECK(update(2, true) == CONSTELLATION_TRACK_PUBLISH);
	CHECK(update(3, false) == CONSTELLATION_TRACK_PUBLISH);
	CHECK(update(4, true) == CONSTELLATION_TRACK_PUBLISH);
	CHECK(update(5, false) == CONSTELLATION_TRACK_PUBLISH);
	// Changing yaw sources must not hide a positional identity slip.
	CHECK(update(6, true, {0.4f, 0, 0.6f}) == CONSTELLATION_TRACK_HOLD_JUMP);
	CHECK(update(7, true) == CONSTELLATION_TRACK_PUBLISH);
	auto flipped = track_point(position, multiply(rotation(M_PI, {1, 0, 0}), optical), imu, 8 * frame);
	CHECK(constellation_track_update(&track, &flipped, true, &imu, nullptr, nullptr) ==
	      CONSTELLATION_TRACK_HOLD_JUMP);
}

TEST_CASE("Track never publishes randomized wrong identities on a moving hand")
{
	std::mt19937 rng(0x7ac4);
	std::uniform_real_distribution<float> unit(-1, 1), positive(0, 1);
	constellation_track track;
	constellation_track_init(&track);
	const int64_t frame = 16666667;
	xrt_vec3 p = {0, 0, 0.5f}, v = {0, 0, 0};
	xrt_quat q = XRT_QUAT_IDENTITY;
	std::vector<xrt_quat> imu_history;
	unsigned published = 0, published_wrong = 0, correct_results = 0, wrong_results = 0, dropped = 0;
	for (int k = 1; k < 6000; k++) {
		// A hand moving up to about 2 m/s and turning up to about 6 rad/s.
		v = {0.9f * v.x + 0.4f * unit(rng), 0.9f * v.y + 0.4f * unit(rng), 0.9f * v.z + 0.4f * unit(rng)};
		float speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
		if (speed > 2)
			v = {v.x * 2 / speed, v.y * 2 / speed, v.z * 2 / speed};
		p = {p.x + v.x * 0.016667f, p.y + v.y * 0.016667f, p.z + v.z * 0.016667f};
		q = multiply(rotation(0.1 * positive(rng), {unit(rng), unit(rng), unit(rng)}), q);
		imu_history.push_back(q);
		float draw = positive(rng);
		if (draw < 0.3f) {
			dropped++;
			constellation_track_expire(&track, k * frame);
			continue;
		}
		bool wrong = draw < 0.6f;
		xrt_vec3 position = p;
		xrt_quat orientation = q;
		if (wrong) {
			// Identity slips: one LED spacing or more and a rotation about an arbitrary axis.
			float d = 0.04f + 0.2f * positive(rng);
			xrt_vec3 dir = {unit(rng), unit(rng), unit(rng)};
			math_vec3_normalize(&dir);
			position = {p.x + d * dir.x, p.y + d * dir.y, p.z + d * dir.z};
			orientation =
			    multiply(rotation(0.35 + 2.8 * positive(rng), {unit(rng), unit(rng), unit(rng)}), q);
			wrong_results++;
		} else {
			correct_results++;
		}
		constellation_track_expire(&track, k * frame);
		constellation_track_point next = track_point(position, orientation, q, k * frame);
		const xrt_quat *ref_imu = nullptr;
		if (track.state != CONSTELLATION_TRACK_LOST)
			ref_imu = &imu_history[track.ref.capture_ns / frame - 1];
		if (constellation_track_update(&track, &next, true, ref_imu, nullptr, nullptr) ==
		    CONSTELLATION_TRACK_PUBLISH) {
			published++;
			published_wrong += wrong;
		}
	}
	printf("track: correct_results=%u wrong_results=%u dropped=%u published=%u published_wrong=%u\n",
	       correct_results, wrong_results, dropped, published, published_wrong);
	CHECK(published_wrong == 0);
	CHECK(published > correct_results * 3 / 4);
}

TEST_CASE("Known-orientation translation stays bounded near the bearing-separation limit")
{
	std::mt19937 rng(17);
	std::normal_distribution<double> noise(0, 1e-5);
	const xrt_vec3 points[2] = {{0, 0, 0}, {0.004f, 0.001f, 0.0f}};
	xrt_quat q = rotation(0.3, {0.2f, 1, 0.1f});
	for (double depth : {0.3, 0.8, 1.6}) {
		xrt_pose truth = {q, {0.01f, -0.02f, float(depth)}};
		xrt_vec3 bearings[2];
		for (unsigned i = 0; i < 2; i++) {
			math_pose_transform_point(&truth, &points[i], &bearings[i]);
			math_vec3_normalize(&bearings[i]);
		}
		double separation = std::acos(std::clamp<double>(m_vec3_dot(bearings[0], bearings[1]), -1, 1));
		enum constellation_constrained_solver_status status;
		xrt_pose out;
		bool ok = constellation_constrained_heading_two_point(points, bearings, &q, 0.001745329, &out, &status);
		if (separation < 0.001745329) {
			CHECK_FALSE(ok);
			CHECK(status == CONSTELLATION_SOLVER_DEGENERATE_BEARINGS);
			continue;
		}
		REQUIRE(ok);
		CHECK(distance(out.position, truth.position) < 1e-4 * depth);
		// Perturbed bearings must give a finite answer whose error scales with depth/separation.
		for (unsigned trial = 0; trial < 50; trial++) {
			xrt_vec3 noisy[2] = {bearings[0], bearings[1]};
			for (auto &b : noisy) {
				b.x += noise(rng);
				b.y += noise(rng);
				math_vec3_normalize(&b);
			}
			if (!constellation_constrained_heading_two_point(points, noisy, &q, 0.001745329, &out, &status))
				continue;
			CHECK(std::isfinite(out.position.z));
			CHECK(distance(out.position, truth.position) < 10 * 1e-5 * depth / separation + 1e-4);
		}
	}
}

TEST_CASE("Dense clutter exhausts the bounded budget instead of stalling, and adjacency finds the ring first")
{
	RingScene s(2);
	s.choose_pose({-0.02f, 0.04f, 0.4f}, {4, 2}, 3);
	REQUIRE(s.observe(0, 4) + s.observe(1, 2) >= 5);
	std::mt19937 rng(3);
	std::uniform_real_distribution<float> unit(0, 1);
	for (unsigned k = 0; k < 2; k++)
		for (unsigned d = 0; d < 25; d++) {
			s.rig.add_blob(k, 640 * unit(rng), 480 * unit(rng));
			s.truth_led[k].push_back(-1);
		}
	xrt_pose prior = world_tilt_prior(s.truth.orientation, 1.0, 0, rng);
	constellation_constrained_result result;
	s.config.max_pair_trials = 4000;
	bool ok = constellation_constrained_search(&s.model, s.rig.views.data(), 2, &prior, false, false, &s.config,
	                                           &result);
	CHECK(result.budget_exhausted);
	CHECK(result.pair_trials == 4000);
	if (ok)
		CHECK(s.identity(result));
	s.config.time_budget_us = 1;
	CHECK_FALSE(constellation_constrained_search(&s.model, s.rig.views.data(), 2, &prior, false, false, &s.config,
	                                             &result));
	CHECK(result.budget_exhausted);
	CHECK(result.pair_trials <= 1);
}

TEST_CASE("Two controllers in one exposure acquire their own rings and never swap")
{
	std::mt19937 rng(0x7a11);
	std::uniform_real_distribution<float> unit(-1, 1), positive(0, 1);
	std::normal_distribution<float> noise(0, 0.3f);
	auto left_leds = rift_s_ring_leds, right_leds = rift_s_ring_leds;
	for (auto &l : right_leds) {
		l.pos.x = -l.pos.x;
		l.dir.x = -l.dir.x;
	}
	t_constellation_led_model models[2] = {};
	models[0].id = 3;
	models[0].leds = left_leds.data();
	models[0].num_leds = left_leds.size();
	models[1].id = 2;
	models[1].leds = right_leds.data();
	models[1].num_leds = right_leds.size();
	Rig rig(2);
	constellation_constrained_config config;
	constellation_constrained_default_config(&config);
	config.time_budget_us = 200000;
	unsigned trials = 0, acquired[2] = {0, 0}, wrong = 0;
	while (trials < 150) {
		xrt_pose truth[2];
		std::array<std::vector<int>, 2> owner, led_of;
		rig.clear();
		unsigned visible_total[2] = {0, 0};
		for (unsigned hand = 0; hand < 2; hand++) {
			truth[hand] = XRT_POSE_IDENTITY;
			truth[hand].orientation = rotation(3.14159f * positive(rng), {unit(rng), unit(rng), unit(rng)});
			float depth = 0.3f + 0.3f * positive(rng);
			// Hands side by side, sometimes close together.
			truth[hand].position = {(hand ? 1 : -1) * (0.03f + 0.12f * positive(rng)), 0.1f * unit(rng), depth};
			const auto &leds = hand ? right_leds : left_leds;
			for (unsigned k = 0; k < 2; k++)
				for (unsigned l = 0; l < leds.size(); l++) {
					float x, y;
					if (!rig.project(k, truth[hand], leds[l], x, y, std::cos(70 * M_PI / 180)) ||
					    positive(rng) < 0.3f) // partial occlusion by the hand
						continue;
					rig.add_blob(k, x + noise(rng), y + noise(rng));
					owner[k].push_back(int(hand));
					led_of[k].push_back(int(l));
					visible_total[hand]++;
				}
		}
		if (visible_total[0] < 3 && visible_total[1] < 3)
			continue;
		trials++;
		constellation_constrained_result results[2];
		bool candidates[2], published[2] = {false, false};
		constellation_constrained_claim blocked[2];
		for (unsigned hand = 0; hand < 2; hand++) {
			xrt_pose prior = world_tilt_prior(truth[hand].orientation, 6.28 * positive(rng), 0.015, rng);
			candidates[hand] = constellation_constrained_search(&models[hand], rig.views.data(), 2, &prior, false,
			                                                    false, &config, &results[hand]);
		}
		const constellation_constrained_result *claims[2] = {&results[0], &results[1]};
		constellation_constrained_arbitrate(claims, published, candidates, blocked, 2, 2);
		for (unsigned hand = 0; hand < 2; hand++) {
			if (!candidates[hand])
				continue;
			bool own = true;
			for (unsigned i = 0; i < results[hand].num_assignments; i++) {
				const auto &a = results[hand].assignments[i];
				own &= owner[a.camera_index][a.blob_index] == int(hand) &&
				       led_of[a.camera_index][a.blob_index] == int(a.led_index);
			}
			if (own && distance(results[hand].P_world_model.position, truth[hand].position) < 0.03)
				acquired[hand]++;
			else
				wrong++;
		}
	}
	std::printf("two-hands: trials=%u left_acquired=%u right_acquired=%u wrong=%u\n", trials, acquired[0],
	            acquired[1], wrong);
	CHECK(wrong == 0);
	// The mirrored right model must acquire as readily as the left one.
	CHECK(acquired[0] > trials / 4);
	CHECK(acquired[1] > trials / 4);
}

TEST_CASE("Trusted prior search stays near the prior and finds distant rings")
{
	// Arm's length and below the hips: 0.7-1.0 m from the cameras, IMU yaw unknown. The prior
	// only bounds the search; a truncated, yaw-ordered pair list used to miss the correct pair
	// and let a hypothesis half a metre away win.
	Scenario sc;
	sc.depth_min = 0.7f;
	sc.depth_max = 1.0f;
	sc.trusted = true;
	sc.min_keep = 8;
	sc.max_keep = 11;
	MonteCarlo many = run_scenario(sc, 200, 0x1234 + 8);
	print("trusted-far-many", many);
	CHECK(many.wrong == 0);
	CHECK(many.accepted >= 170);
	sc.min_keep = 4;
	sc.max_keep = 7;
	sc.depth_min = 0.3f;
	sc.depth_max = 0.6f;
	MonteCarlo few = run_scenario(sc, 200, 0x1234 + 4);
	print("trusted-near-few", few);
	CHECK(few.wrong == 0);
	CHECK(few.accepted >= 150);
	CHECK(few.worst_identity_error_m < 0.05);
}

TEST_CASE("Fast-path budget does not turn one solution into an ambiguity")
{
	// The fast path has 3 ms. When the pair search used up the budget, refinement stopped too and
	// left candidates with identical LED assignments at slightly different poses, which the
	// ambiguity test then treated as competing identities.
	constellation_constrained_config fast;
	constellation_constrained_default_config(&fast);
	fast.time_budget_us = 3000;
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
	// Instrumentation changes throughput. The native build checks the 3 ms fast path.
	fast.time_budget_us = 200000;
#endif
	Scenario sc;
	sc.depth_min = 0.7f;
	sc.depth_max = 1.0f;
	sc.trusted = true;
	sc.min_keep = 8;
	sc.max_keep = 11;
	MonteCarlo stats = run_scenario(sc, 200, 0x1234 + 8, &fast);
	print("trusted-far-many-3ms", stats);
	CHECK(stats.wrong == 0);
	CHECK(stats.reasons[CONSTELLATION_CONSTRAINED_AMBIGUOUS] <= 20);
	CHECK(stats.accepted >= 160);
}

TEST_CASE("Bounded tilt adjustment absorbs an IMU tilt error of a few degrees")
{
	RingScene s(2);
	s.choose_pose({0.02f, 0.04f, 0.40f}, {5, 4}, 11);
	REQUIRE(s.observe(0, 6) >= 5);
	REQUIRE(s.observe(1, 6) >= 4);
	auto tilted_prior = [&](double degrees) {
		xrt_pose prior = s.truth;
		prior.orientation = multiply(rotation(degrees * M_PI / 180, {1, 0, 1}), s.truth.orientation);
		return prior;
	};
	// Angle between the world gravity directions the two orientations put in the model frame.
	auto tilt_error = [&](const constellation_constrained_result &r) {
		xrt_vec3 gravity = {0, 1, 0}, truth, fit;
		xrt_quat truth_inverse = conjugate(s.truth.orientation),
		         fit_inverse = conjugate(r.P_world_model.orientation);
		math_quat_rotate_vec3(&truth_inverse, &gravity, &truth);
		math_quat_rotate_vec3(&fit_inverse, &gravity, &fit);
		return std::acos(std::clamp(double(m_vec3_dot(truth, fit)), -1.0, 1.0)) * 180 / M_PI;
	};
	constellation_constrained_config fixed = s.config;
	fixed.tilt_prior_std_rad = 0;
	constellation_constrained_result result;

	// 4 degrees off: the fixed-tilt fit keeps the error and absorbs it in its residuals; the
	// adjusted fit moves most of the way to the true tilt. One small ring observes tilt to about
	// 2 degrees, so the 3 degree prior keeps part of the error.
	xrt_pose prior = tilted_prior(4);
	REQUIRE(
	    constellation_constrained_search(&s.model, s.rig.views.data(), 2, &prior, false, false, &fixed, &result));
	CHECK(tilt_error(result) == Catch::Approx(4).margin(0.1));
	CHECK(result.tilt_correction_rad == 0);
	double fixed_chi_square = result.fit_chi_square;
	REQUIRE(constellation_constrained_search(&s.model, s.rig.views.data(), 2, &prior, false, false, &s.config,
	                                         &result));
	CHECK(s.identity(result));
	CHECK(tilt_error(result) < 1.5);
	CHECK(result.tilt_correction_rad * 180 / M_PI > 2.5);
	CHECK(result.fit_chi_square < fixed_chi_square / 4);
	CHECK(distance(result.P_world_model.position, s.truth.position) < 0.003);

	// The adjustment never exceeds its bound, however far off the IMU is.
	prior = tilted_prior(12);
	constellation_constrained_search(&s.model, s.rig.views.data(), 2, &prior, false, false, &s.config, &result);
	CHECK(result.tilt_correction_rad <= s.config.maximum_tilt_correction_rad + 1e-6);

	// Three distinct LEDs keep the prior tilt.
	RingScene few(1);
	few.choose_pose({0.02f, 0.04f, 0.40f}, {3}, 12);
	REQUIRE(few.observe(0, 3) == 3);
	xrt_pose few_prior = few.truth;
	few_prior.orientation = multiply(rotation(2 * M_PI / 180, {1, 0, 1}), few.truth.orientation);
	few_prior.position.x += 0.01f;
	few.config.minimum_tracking_inliers = 3;
	constellation_constrained_search(&few.model, few.rig.views.data(), 1, &few_prior, true, true, &few.config,
	                                 &result);
	CHECK(result.num_assignments == 3);
	CHECK(result.tilt_correction_rad == 0);
}

TEST_CASE("Tilt adjustment recovers acquisitions lost to IMU tilt error without wrong identities")
{
	constellation_constrained_config fixed;
	constellation_constrained_default_config(&fixed);
	fixed.time_budget_us = 200000;
	fixed.tilt_prior_std_rad = 0;
	Scenario sc;
	sc.min_keep = 6;
	sc.max_keep = 11;
	sc.tilt_error = 5 * M_PI / 180;
	MonteCarlo adjusted = run_scenario(sc, 300, 0x7117);
	MonteCarlo kept = run_scenario(sc, 300, 0x7117, &fixed);
	print("tilt-5deg-adjusted", adjusted);
	print("tilt-5deg-fixed", kept);
	CHECK(adjusted.wrong <= kept.wrong);
	CHECK(adjusted.accepted >= kept.accepted + 10);
	CHECK(adjusted.reasons[CONSTELLATION_CONSTRAINED_INCONSISTENT] <
	      kept.reasons[CONSTELLATION_CONSTRAINED_INCONSISTENT] / 2);

	// No tilt error: the extra freedom costs at most a few acquisitions.
	sc.tilt_error = 0;
	MonteCarlo exact = run_scenario(sc, 300, 0x7118);
	MonteCarlo exact_fixed = run_scenario(sc, 300, 0x7118, &fixed);
	print("tilt-0deg-adjusted", exact);
	print("tilt-0deg-fixed", exact_fixed);
	CHECK(exact.wrong == 0);
	CHECK(exact.accepted + 6 >= exact_fixed.accepted);
}

TEST_CASE("Head-relative positions ignore head pitch and roll but follow its heading")
{
	xrt_pose head = XRT_POSE_IDENTITY;
	head.position = {0.3f, 1.6f, -0.2f};
	// A hand 50 cm in front, 60 cm below and 20 cm left of the head.
	auto check = [&](xrt_vec3 expected) {
		xrt_vec3 forward = {0, 0, -1}, world_forward, right = {1, 0, 0}, world_right;
		math_quat_rotate_vec3(&head.orientation, &forward, &world_forward);
		math_quat_rotate_vec3(&head.orientation, &right, &world_right);
		// Build the hand position from the head's horizontal heading.
		world_forward.y = 0;
		world_right.y = 0;
		math_vec3_normalize(&world_forward);
		math_vec3_normalize(&world_right);
		xrt_vec3 hand = head.position;
		hand = m_vec3_add(hand, m_vec3_mul_scalar(world_forward, -expected.z));
		hand = m_vec3_add(hand, m_vec3_mul_scalar(world_right, expected.x));
		hand.y += expected.y;
		xrt_vec3 out;
		constellation_head_relative_position(&head, &hand, &out);
		CHECK(out.x == Catch::Approx(expected.x).margin(1e-4));
		CHECK(out.y == Catch::Approx(expected.y).margin(1e-4));
		CHECK(out.z == Catch::Approx(expected.z).margin(1e-4));
	};
	check({-0.2f, -0.6f, -0.5f});
	// Looking down 60 degrees, or turned 120 degrees and rolled: the same body-relative numbers.
	head.orientation = rotation(-60 * M_PI / 180, {1, 0, 0});
	check({-0.2f, -0.6f, -0.5f});
	head.orientation = multiply(rotation(120 * M_PI / 180, {0, 1, 0}), rotation(25 * M_PI / 180, {0, 0, 1}));
	check({-0.2f, -0.6f, -0.5f});
	xrt_vec3 hand = {0.3f, 1.0f, -0.2f}, out;
	constellation_head_relative_position(&head, &hand, &out);
	CHECK(out.y == Catch::Approx(-0.6).margin(1e-4));
	CHECK(std::hypot(out.x, out.z) < 1e-4);
}

TEST_CASE("Confirmed heading excludes the r12 gravity-hypothesis identity jumps before selection")
{
	Scene s;
	s.observe(0, {0, 1}); s.observe(1, {2});
	constellation_constrained_result result{};
	REQUIRE(constellation_constrained_search(&s.model, s.views.data(), 2, &s.truth,
	                                        true, true, &s.config, &result));
	for (double degrees : {79., 179.}) {
		auto prior = s.truth;
		prior.orientation = multiply(rotation(degrees * M_PI / 180, {0, 1, 0}), prior.orientation);
		CHECK_FALSE(constellation_constrained_search(&s.model, s.views.data(), 2, &prior,
		                                            true, true, &s.config, &result));
		CHECK(result.prior_rotation_rejections > 0);
		// Acquisition after loss still allows every heading, independently of this guard.
		REQUIRE(constellation_constrained_search(&s.model, s.views.data(), 2, &prior,
		                                        false, false, &s.config, &result));
		CHECK(angle(result.P_world_model.orientation, s.truth.orientation) < .001);
	}
}

TEST_CASE("Shared blob logging selects the strongest conflict in any device order")
{
	constellation_constrained_result mine{}, weak{}, strong{};
	for (auto *r : {&mine, &weak, &strong}) {
		r->num_assignments = 1;
		r->assignments[0] = {0, 7, 0, 0, 0};
		r->inliers = 5;
		r->distinct_leds = 4;
	}
	weak.rejection = CONSTELLATION_CONSTRAINED_AMBIGUOUS;
	for (bool reversed : {false, true}) {
		const constellation_constrained_result *claims[] = {&mine, reversed ? &strong : &weak,
		                                                    reversed ? &weak : &strong};
		bool published[] = {false, reversed, !reversed}, candidates[] = {true, false, false};
		constellation_constrained_claim blocked[3];
		unsigned blockers[3];
		constellation_constrained_arbitrate_with_owners(claims, published, candidates, blocked, blockers, 3, 2);
		CHECK_FALSE(candidates[0]);
		CHECK(blocked[0] == CONSTELLATION_CLAIM_PUBLISHED);
		CHECK(claims[blockers[0]] == &strong);
		published[1] = published[2] = false;
		candidates[0] = true;
		candidates[reversed ? 1 : 2] = true;
		constellation_constrained_arbitrate_with_owners(claims, published, candidates, blocked, blockers, 3, 2);
		CHECK(blocked[0] == CONSTELLATION_CLAIM_CANDIDATE);
		CHECK(claims[blockers[0]] == &strong);
	}
}

TEST_CASE("Repeated near-depth candidates cannot erase a recent optical history")
{
	constellation_track track;
	constellation_track_init(&track);
	xrt_quat imu = XRT_QUAT_IDENTITY;
	auto update = [&](int frame, float depth) {
		auto next = track_point({0, 0, depth}, imu, imu, frame * 33000000LL);
		return constellation_track_update(&track, &next, true, &imu, nullptr, nullptr);
	};
	CHECK(update(1, 0.9f) == CONSTELLATION_TRACK_HOLD_CONFIRMING);
	REQUIRE(update(2, 0.9f) == CONSTELLATION_TRACK_PUBLISH);
	CHECK(update(3, 0.2f) == CONSTELLATION_TRACK_HOLD_JUMP);
	CHECK(update(4, 0.2f) == CONSTELLATION_TRACK_HOLD_JUMP);
	CHECK(update(5, 0.2f) == CONSTELLATION_TRACK_HOLD_RESTART);
	CHECK(update(6, 0.2f) == CONSTELLATION_TRACK_HOLD_UNCONFIRMED);
	CHECK(update(7, 0.2f) == CONSTELLATION_TRACK_HOLD_UNCONFIRMED);
	CHECK(track.published.P_world_model.position.z == Catch::Approx(0.9f));
}

TEST_CASE("P3P acquisition forwards three-camera-view roots to joint verification")
{
	Scene s;
	s.observe(0, {0, 1, 2});
	s.observe(1, {0});
	for (auto &o : s.observations)
		for (int i = 0; i < o.num_blobs; ++i) {
			o.blobs[i].led_id = LED_INVALID_ID;
			o.blobs[i].width = o.blobs[i].height = 2;
		}
	s.model.min_acquisition_leds = 5;
	struct Context {
		Scene *scene;
		unsigned checked = 0;
		bool recovered = false;
	} context{&s};
	auto *model = t_constellation_search_model_new(&s.model);
	REQUIRE(model);
	auto *search = correspondence_search_new(&s.camera);
	REQUIRE(search);
	correspondence_search_set_blobs(search, s.observations[0].blobs, 3);
	search->pose_candidate_userdata = &context;
	search->pose_candidate_cb = [](void *userdata, const xrt_pose *pose, const pose_metrics *) {
		auto &ctx = *static_cast<Context *>(userdata);
		++ctx.checked;
		constellation_constrained_result result{};
		if (constellation_constrained_verify_pose(&ctx.scene->model, ctx.scene->views.data(), 2, pose,
		                                          CONSTELLATION_CONSTRAINED_P3P, &ctx.scene->config, &result)) {
			ctx.recovered |= distance(pose->position, ctx.scene->truth.position) < 0.001 &&
			                 angle(pose->orientation, ctx.scene->truth.orientation) < 0.01;
		}
	};
	xrt_pose seed = XRT_POSE_IDENTITY;
	seed.position = {42, 42, 42};
	xrt_vec3 gravity{0, 1, 0}, pos_error{0.001f, 0.001f, 0.001f}, rot_error{0.1f, 0.1f, 0.1f};
	pose_metrics metrics{};
	auto flags = static_cast<correspondence_search_flags>(CS_FLAG_SHALLOW_SEARCH | CS_FLAG_DEEP_SEARCH |
	    CS_FLAG_MATCH_GRAVITY | CS_FLAG_MATCH_ALL_BLOBS | CS_FLAG_JOINT_P3P);
	correspondence_search_find_one_pose(search, model, flags, &seed, &pos_error, &rot_error,
	                                   &gravity, 0.1f, &metrics);
	CHECK(context.checked > 0);
	CHECK(context.recovered);
	SECTION("Three observations alone cannot acquire") {
		s.observe(1, {});
		context.recovered = false;
		seed = XRT_POSE_IDENTITY;
		correspondence_search_find_one_pose(search, model, flags, &seed, &pos_error, &rot_error,
		                                   &gravity, 0.1f, &metrics);
		CHECK_FALSE(context.recovered);
	}
	SECTION("Published blob ownership is excluded before generating P3P roots") {
		context.checked = 0;
		s.views[0].excluded_blobs[1] = true;
		search->excluded_blobs = s.views[0].excluded_blobs;
		seed = XRT_POSE_IDENTITY;
		correspondence_search_find_one_pose(search, model, flags, &seed, &pos_error, &rot_error,
		                                   &gravity, 0.1f, &metrics);
		CHECK(context.checked == 0);
	}
	correspondence_search_free(search);
	t_constellation_search_model_free(model);
}

TEST_CASE("Cold right acquisition uses unclaimed observations after the left publishes")
{
	// Two geometrically identical objects are the hardest ownership case. Raw cold search
	// cannot choose between their rings; arbitration alone never exposes the second solution.
	Scene s;
	const xrt_pose right = s.truth;
	s.config.time_budget_us = 25000;
	xrt_pose left = right;
	left.position.x = -0.18f;
	s.truth = left;
	s.observe(0, {0, 1, 2});
	s.observe(1, {0, 1, 2});
	for (unsigned k = 0; k < 2; k++) {
		for (unsigned l = 0; l < s.leds.size(); l++) {
			xrt_vec3 world, local;
			math_pose_transform_point(&right, &s.leds[l].pos, &world);
			math_pose_transform_point(&s.views[k].P_cam_world, &world, &local);
			auto &o = s.observations[k];
			auto &b = o.blobs[o.num_blobs++];
			REQUIRE(t_camera_models_project(&s.camera.calib, local.x, local.y, local.z, &b.x, &b.y));
			b.led_id = LED_MAKE_ID(99, 42); // Stale labels cannot become ownership.
		}
	}
	constellation_tracking_sample sample{};
	sample.n_devices = 2;
	sample.devices[0].found_device_pose = true;
	REQUIRE(constellation_constrained_verify_pose(&s.model, s.views.data(), 2, &left,
	    CONSTELLATION_CONSTRAINED_P3P, &s.config, &sample.devices[0].joint_result));
	constellation_track track;
	constellation_track_init(&track);
	xrt_pose prior = XRT_POSE_IDENTITY;
	unsigned raw_published = 0, recovered_at = 0;
	for (unsigned frame = 1; frame <= 60; frame++) {
		s.observations[1].num_blobs = frame % 3 ? 6 : 5; // Vary visibility without changing blob indices.
		auto &result = sample.devices[1].joint_result;
		bool candidates[2] = {false, constellation_constrained_search(&s.model, s.views.data(), 2,
		    &prior, false, false, &s.config, &result)};
		const bool published[2] = {true, false};
		const constellation_constrained_result *claims[2] = {&sample.devices[0].joint_result, &result};
		constellation_constrained_claim blocked[2];
		constellation_constrained_arbitrate(claims, published, candidates, blocked, 2, 2);
		raw_published += candidates[1];
		auto remaining = s.views;
		const unsigned excluded = constellation_tracking_sample_exclude_published(&sample, 1,
		    remaining.data(), remaining.size());
		CHECK(excluded == 6);
		if (!constellation_constrained_search(&s.model, remaining.data(), 2, &prior, false, false,
		                                     &s.config, &result))
			continue;
		for (unsigned a = 0; a < result.num_assignments; a++)
			CHECK(result.assignments[a].blob_index >= 3);
		CHECK(distance(result.P_world_model.position, right.position) < 0.001);
		candidates[1] = true;
		constellation_constrained_arbitrate(claims, published, candidates, blocked, 2, 2);
		REQUIRE(candidates[1]);
		constellation_track_point next{result.P_world_model, right.orientation, int64_t(frame) * 33333333};
		auto verdict = constellation_track_update(&track, &next, result.vision_determines_yaw,
		    &right.orientation, nullptr, nullptr);
		if (verdict == CONSTELLATION_TRACK_PUBLISH && !recovered_at)
			recovered_at = frame;
	}
	CHECK(raw_published == 0);
	CHECK(recovered_at > 0);
	CHECK(recovered_at <= 60);
	// Removing the current publication re-enables all blobs, regardless of old LED labels.
	sample.devices[0].found_device_pose = false;
	auto remaining = s.views;
	CHECK(constellation_tracking_sample_exclude_published(&sample, 1, remaining.data(), 2) == 0);
	// The ownership retry does not bypass the optical/IMU continuity gate.
	constellation_track_point jump{right, right.orientation, int64_t(61) * 33333333};
	jump.P_world_model.orientation = rotation(2, {0, 1, 0});
	CHECK(constellation_track_update(&track, &jump, true, &right.orientation, nullptr, nullptr) ==
	      CONSTELLATION_TRACK_HOLD_JUMP);
	std::printf("sequential ownership: raw publications=%u recovered at frame=%u / 60\n", raw_published,
	            recovered_at);
}

TEST_CASE("Cold P3P uses the capture-time tilt without trusting the position")
{
	Scene s;
	s.truth.orientation = multiply(rotation(0.4, {0, 1, 0}), rotation(0.65, {1, 0, 0}));
	s.observe(0, {0, 1, 2});
	s.observe(1, {0});
	for (auto &o : s.observations)
		for (int i = 0; i < o.num_blobs; i++) {
			o.blobs[i].led_id = LED_INVALID_ID;
			o.blobs[i].width = o.blobs[i].height = 2;
		}
	struct Context {
		Scene *scene;
		unsigned recovered = 0;
	} context{&s};
	auto *model = t_constellation_search_model_new(&s.model);
	auto *search = correspondence_search_new(&s.camera);
	REQUIRE(model);
	REQUIRE(search);
	search->pose_candidate_userdata = &context;
	search->pose_candidate_cb = [](void *userdata, const xrt_pose *pose, const pose_metrics *) {
		auto &ctx = *static_cast<Context *>(userdata);
		constellation_constrained_result result{};
		if (constellation_constrained_verify_pose(&ctx.scene->model, ctx.scene->views.data(), 2, pose,
		                                          CONSTELLATION_CONSTRAINED_P3P, &ctx.scene->config, &result) &&
		    distance(result.P_world_model.position, ctx.scene->truth.position) < 0.001 &&
		    angle(result.P_world_model.orientation, ctx.scene->truth.orientation) < 0.01)
			ctx.recovered++;
	};
	correspondence_search_set_blobs(search, s.observations[0].blobs, 3);
	const xrt_vec3 gravity = {0, 1, 0};
	xrt_vec3 camera_gravity = gravity;
	auto flags = static_cast<correspondence_search_flags>(CS_FLAG_SHALLOW_SEARCH | CS_FLAG_DEEP_SEARCH |
	    CS_FLAG_MATCH_GRAVITY | CS_FLAG_MATCH_ALL_BLOBS | CS_FLAG_JOINT_P3P);
	// Only tilt is known. Arbitrary position and yaw must not gate cold acquisition.
	xrt_pose seed = {multiply(rotation(1.7, gravity), s.truth.orientation), {42, -42, 42}};
	pose_metrics metrics{};
	correspondence_search_find_one_pose(search, model, flags, &seed, nullptr, nullptr,
	                                   &camera_gravity, 0.1f, &metrics);
	CHECK(context.recovered > 0);
	SECTION("An incompatible tilt still rejects the same geometry") {
		context.recovered = 0;
		seed = XRT_POSE_IDENTITY;
		correspondence_search_find_one_pose(search, model, flags, &seed, nullptr, nullptr,
		                                   &camera_gravity, 0.1f, &metrics);
		CHECK(context.recovered == 0);
	}
	correspondence_search_free(search);
	t_constellation_search_model_free(model);
}

TEST_CASE("Tracked ring with many LEDs keeps publishing when the prior search is truncated")
{
	/* r18 worn log: a tracked ring puts thousands of pairs on the prior edge list, beyond the
	 * 1000-trial cap, and the fast path rejected 1267 of 1429 tracked frames as uncertain. A
	 * truncated search must still verify the continuing identity, and never a wrong one. */
	Scenario sc;
	sc.trusted = true;
	sc.heading = true;
	sc.min_keep = 8;
	sc.max_keep = 11;
	constellation_constrained_config config;
	constellation_constrained_default_config(&config);
	MonteCarlo capped = run_scenario(sc, 200, 0x5219, &config);
	print("trusted-heading-default-cap", capped);
	CHECK(capped.wrong == 0);
	CHECK(capped.reasons[CONSTELLATION_CONSTRAINED_UNCERTAIN] == 0);
	CHECK(capped.accepted >= 190);

	// Only the first few predicted pairs: the continuing identity is among them.
	config.max_prior_pair_trials = 20;
	MonteCarlo tight = run_scenario(sc, 200, 0x5219 + 1, &config);
	print("trusted-heading-20-pairs", tight);
	CHECK(tight.wrong == 0);
	CHECK(tight.accepted >= 190);

	RingScene s(2);
	s.choose_pose({-0.02f, 0.04f, 0.4f}, {4, 2}, 3);
	REQUIRE(s.observe(0, 4) + s.observe(1, 2) >= 5);
	s.config.max_prior_pair_trials = 1;
	constellation_constrained_result result;
	REQUIRE(constellation_constrained_search(&s.model, s.rig.views.data(), 2, &s.truth, true, true, &s.config,
	                                         &result));
	CHECK(result.budget_exhausted);
	CHECK(s.identity(result));
}
