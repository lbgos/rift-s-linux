// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#include "internal/pose_metrics.h"
#include "internal/ransac_pnp.h"
#include <array>
#include <cmath>
#include <vector>
#include "math/m_vec3.h"
#include "internal/correspondence_search.h"
#include "constellation_rift_s_data.hpp"
#include "tracking/t_led_models.h"

TEST_CASE("Constellation gravity rejects opposite tilt and is symmetric")
{
	const xrt_vec3 gravity = {0, 1, 0};
	xrt_pose prior = XRT_POSE_IDENTITY;
	xrt_pose pose = XRT_POSE_IDENTITY;
	const xrt_vec3 axis = {1, 0, 0};
	math_quat_from_angle_vector(M_PI / 3, &axis, &prior.orientation);
	math_quat_from_angle_vector(-M_PI / 3, &axis, &pose.orientation);
	CHECK(pose_metrics_gravity_error(&pose, &prior, &gravity) == Catch::Approx(2 * M_PI / 3));
	pose.orientation = {0, 0, 0, 1};
	CHECK(pose_metrics_gravity_error(&pose, &prior, &gravity) == Catch::Approx(M_PI / 3));
	CHECK(pose_metrics_gravity_error(&prior, &pose, &gravity) == Catch::Approx(M_PI / 3));
	pose.orientation = {-prior.orientation.x, -prior.orientation.y, -prior.orientation.z, -prior.orientation.w};
	CHECK(pose_metrics_gravity_error(&pose, &prior, &gravity) == Catch::Approx(0).margin(1e-5));
}

TEST_CASE("Constellation gravity permits yaw about camera gravity")
{
	const xrt_vec3 gravity = {0.6f, 0.8f, 0};
	xrt_pose prior = XRT_POSE_IDENTITY;
	xrt_pose pose = XRT_POSE_IDENTITY;
	const xrt_vec3 axis = {1, 0, 0};
	math_quat_from_angle_vector(0.7, &axis, &prior.orientation);
	xrt_quat yaw;
	math_quat_from_angle_vector(1.2, &gravity, &yaw);
	math_quat_rotate(&yaw, &prior.orientation, &pose.orientation);
	CHECK(pose_metrics_gravity_error(&pose, &prior, &gravity) == Catch::Approx(0).margin(1e-5));
}

TEST_CASE("Constellation PnP preserves seed on degenerate correspondences")
{
	std::array<t_constellation_led, 5> leds = {};
	std::array<blob, 5> blobs = {};
	for (size_t i = 0; i < leds.size(); i++) {
		leds[i].id = i;
		blobs[i].led_id = LED_MAKE_ID(2, i);
		blobs[i].x = 320;
		blobs[i].y = 240;
	}
	t_constellation_led_model model = {};
	model.id = 2;
	model.num_leds = leds.size();
	model.leds = leds.data();
	camera_model cam = {};
	cam.calib.fx = cam.calib.fy = 190;
	cam.calib.cx = 320;
	cam.calib.cy = 240;
	cam.calib.model = T_DISTORTION_FISHEYE_KB4;
	xrt_pose pose = XRT_POSE_IDENTITY;
	pose.position = {0.1f, 0.2f, 0.6f};
	int count = 0, inliers = 0;
	CHECK_FALSE(ransac_pnp_pose(&pose, blobs.data(), blobs.size(), &model, &cam, &count, &inliers));
	CHECK(pose.position.x == Catch::Approx(0.1));
	CHECK(pose.position.y == Catch::Approx(0.2));
	CHECK(pose.position.z == Catch::Approx(0.6));
	CHECK(pose.orientation.w == Catch::Approx(1));
}

TEST_CASE("Constellation PnP recovers a known camera pose")
{
	std::array<t_constellation_led, 8> leds = {};
	std::array<blob, 8> blobs = {};
	camera_model cam = {};
	cam.calib.fx = cam.calib.fy = 190;
	cam.calib.cx = 320;
	cam.calib.cy = 240;
	cam.calib.model = T_DISTORTION_FISHEYE_KB4;
	for (size_t i = 0; i < leds.size(); i++) {
		leds[i].id = i;
		leds[i].pos = {i & 1 ? 0.03f : -0.03f, i & 2 ? 0.03f : -0.03f, i & 4 ? 0.02f : -0.02f};
		blobs[i].led_id = LED_MAKE_ID(2, i);
		REQUIRE(t_camera_models_project(&cam.calib, leds[i].pos.x, leds[i].pos.y, leds[i].pos.z + 0.6f,
		                                &blobs[i].x, &blobs[i].y));
	}
	t_constellation_led_model model = {};
	model.id = 2;
	model.num_leds = leds.size();
	model.leds = leds.data();
	xrt_pose pose = XRT_POSE_IDENTITY;
	pose.position.z = 0.5f;
	int count = 0, inliers = 0;
	REQUIRE(ransac_pnp_pose(&pose, blobs.data(), blobs.size(), &model, &cam, &count, &inliers));
	CHECK(std::isfinite(pose.orientation.w));
	CHECK(pose.position.z == Catch::Approx(0.6).margin(1e-4));
	CHECK(count == 8);
	CHECK(inliers == 8);
}

TEST_CASE("Rift S acquires a recorded five-LED ring with an unknown position and yaw")
{
	auto leds = rift_s_ring_leds;
	t_constellation_led_model model = {};
	model.id = XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER;
	model.num_leds = leds.size();
	model.leds = leds.data();
	camera_model cam = {};
	cam.width = 640;
	cam.height = 480;
	cam.calib.fx = rift_s_ring_intrinsics[0];
	cam.calib.fy = rift_s_ring_intrinsics[1];
	cam.calib.cx = rift_s_ring_intrinsics[2];
	cam.calib.cy = rift_s_ring_intrinsics[3];
	cam.calib.model = T_DISTORTION_FISHEYE_KB4;
	cam.calib.fisheye = {rift_s_ring_kb4[0], rift_s_ring_kb4[1], rift_s_ring_kb4[2], rift_s_ring_kb4[3]};
	std::array<blob, 8> blobs = {};
	for (size_t i = 0; i < blobs.size(); i++) {
		blobs[i].x = rift_s_ring_blobs[i][0];
		blobs[i].y = rift_s_ring_blobs[i][1];
		blobs[i].width = rift_s_ring_blobs[i][2];
		blobs[i].height = rift_s_ring_blobs[i][3];
		blobs[i].led_id = LED_INVALID_ID;
	}
	auto *search_model = t_constellation_search_model_new(&model);
	auto *cs = correspondence_search_new(&cam);
	correspondence_search_set_blobs(cs, blobs.data(), blobs.size());
	xrt_vec3 pos_error = {0.1f, 0.1f, 0.1f}, rot_error = {0.5f, 0.5f, 0.5f};
	xrt_vec3 gravity = rift_s_ring_gravity;
	xrt_pose prior = XRT_POSE_IDENTITY;
	prior.position = {42, 42, 42};
	math_quat_from_vec_a_to_vec_b(&rift_s_ring_down, &gravity, &prior.orientation);
	auto flags = static_cast<correspondence_search_flags>(CS_FLAG_SHALLOW_SEARCH | CS_FLAG_HAVE_POSE_PRIOR |
	                                                      CS_FLAG_MATCH_GRAVITY);
	xrt_pose pose = prior;
	pose_metrics score = {};
	CHECK_FALSE(correspondence_search_find_one_pose(cs, search_model, flags, &pose, &pos_error, &rot_error,
	                                                &gravity, M_PI / 6, &score));
	CHECK(pose.position.x == 42);
	model.min_acquisition_leds = 5;
	cs->max_search_ns = 1;
	pose = prior;
	CHECK_FALSE(correspondence_search_find_one_pose(cs, search_model, flags, &pose, &pos_error, &rot_error,
	                                                &gravity, M_PI / 6, &score));
	CHECK(cs->budget_exhausted);
	CHECK(cs->num_trials == 0);
	CHECK(pose.position.x == 42);
	cs->max_search_ns = 0;
	unsigned candidates = 0;
	cs->pose_candidate_userdata = &candidates;
	cs->pose_candidate_cb = [](void *userdata, const xrt_pose *candidate, const pose_metrics *metrics) {
		++*static_cast<unsigned *>(userdata);
		CHECK(std::isfinite(candidate->position.z));
		CHECK(metrics->matched_blobs >= 5);
	};
	pose = prior;
	REQUIRE(correspondence_search_find_one_pose(cs, search_model, flags, &pose, &pos_error, &rot_error, &gravity,
	                                            M_PI / 6, &score));
	CHECK(candidates > 0);
	cs->pose_candidate_cb = nullptr;
	cs->pose_candidate_userdata = nullptr;
	CHECK(score.matched_blobs == 5);
	CHECK(score.visible_leds == 5);
	CHECK(score.unmatched_blobs == 0);
	CHECK(score.reprojection_error / score.matched_blobs < 1.5);
	CHECK_FALSE(POSE_HAS_FLAGS(&score, POSE_MATCH_POSITION));
	CHECK_FALSE(POSE_HAS_FLAGS(&score, POSE_MATCH_STRONG));
	CHECK(pose.position.z == Catch::Approx(0.220).margin(0.003));
	CHECK(pose_metrics_gravity_error(&pose, &prior, &gravity) < M_PI / 6);
	pose = prior;
	xrt_vec3 opposite_down = {-rift_s_ring_down.x, -rift_s_ring_down.y, -rift_s_ring_down.z};
	math_quat_from_vec_a_to_vec_b(&opposite_down, &gravity, &pose.orientation);
	CHECK_FALSE(correspondence_search_find_one_pose(cs, search_model, flags, &pose, &pos_error, &rot_error,
	                                                &gravity, M_PI / 6, &score));
	pose = prior;
	flags = static_cast<correspondence_search_flags>(CS_FLAG_SHALLOW_SEARCH | CS_FLAG_HAVE_POSE_PRIOR);
	CHECK_FALSE(correspondence_search_find_one_pose(cs, search_model, flags, &pose, &pos_error, &rot_error,
	                                                &gravity, M_PI / 6, &score));
	correspondence_search_free(cs);
	t_constellation_search_model_free(search_model);
}

TEST_CASE("Split blobs cannot count as additional LED correspondences")
{
	auto leds = rift_s_ring_leds;
	t_constellation_led_model model = {};
	model.id = XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER;
	model.leds = leds.data();
	model.num_leds = leds.size();
	model.min_acquisition_leds = 5;
	camera_model cam = {};
	cam.width = 640;
	cam.height = 480;
	cam.calib.fx = cam.calib.fy = 190;
	cam.calib.cx = 320;
	cam.calib.cy = 240;
	cam.calib.model = T_DISTORTION_FISHEYE_KB4;
	xrt_pose pose = XRT_POSE_IDENTITY;
	pose.position.z = 0.5;
	pose.orientation = {0.98480775f, 0, 0, 0.17364818f};
	pose_metrics_blob_match_info info = {};
	pose_metrics_match_pose_to_blobs(&pose, nullptr, 0, &model, &cam, &info);
	REQUIRE(info.num_visible_leds >= 5);
	std::array<blob, 6> blobs = {};
	for (size_t i = 0; i < blobs.size(); i++) {
		size_t led = i < 4 ? i : 0;
		blobs[i].x = info.visible_leds[led].pos_px.x;
		blobs[i].y = info.visible_leds[led].pos_px.y;
		blobs[i].width = blobs[i].height = 2;
		blobs[i].led_id = LED_INVALID_ID;
	}
	pose_metrics score = {};
	pose_metrics_evaluate_pose(&score, &pose, blobs.data(), blobs.size(), &model, &cam, nullptr);
	CHECK(score.matched_blobs == 4);
	CHECK(score.unmatched_blobs == 2);
	CHECK_FALSE(pose_metrics_can_acquire_with_gravity(&score, &model));
}

TEST_CASE("Blob extraction keeps first-column peaks and actual scanline coordinates")
{
	std::array<uint8_t, 400> pixels = {};
	pixels[7 * 20 + 5] = 255;
	pixels[8 * 20 + 5] = 100;
	xrt_frame frame = {};
	frame.width = frame.height = 20;
	frame.stride = 20;
	frame.format = XRT_FORMAT_L8;
	frame.data = pixels.data();
	auto *watch = blobwatch_new(96, 128);
	blobservation *observation = nullptr;
	blobwatch_process(watch, &frame, &observation);
	REQUIRE(observation != nullptr);
	REQUIRE(observation->num_blobs == 1);
	const auto &b = observation->blobs[0];
	CHECK(b.brightness == 255);
	CHECK(b.x == 5);
	CHECK(b.y == Catch::Approx(7 + 100.0 / 355));
	CHECK(b.top == 7);
	CHECK(b.height == 2);
	blobwatch_free(watch);
}

TEST_CASE("Constellation search acquires synthetic Rift S rings without a pose prior")
{
	for (bool mirror : {false, true}) {
		auto leds = rift_s_ring_leds;
		if (mirror) {
			for (auto &led : leds) {
				led.pos.x = -led.pos.x;
				led.dir.x = -led.dir.x;
			}
		}
		t_constellation_led_model model = {};
		model.id = mirror ? XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER : XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER;
		model.num_leds = leds.size();
		model.leds = leds.data();
		camera_model cam = {};
		cam.width = 640;
		cam.height = 480;
		cam.calib.fx = rift_s_ring_intrinsics[0];
		cam.calib.fy = rift_s_ring_intrinsics[1];
		cam.calib.cx = rift_s_ring_intrinsics[2];
		cam.calib.cy = rift_s_ring_intrinsics[3];
		cam.calib.model = T_DISTORTION_FISHEYE_KB4;
		cam.calib.fisheye = {rift_s_ring_kb4[0], rift_s_ring_kb4[1], rift_s_ring_kb4[2], rift_s_ring_kb4[3]};
		xrt_pose truth = XRT_POSE_IDENTITY;
		truth.orientation = {0.98480775f, 0, 0, 0.17364818f};
		truth.position.z = 0.5f;
		pose_metrics_blob_match_info info = {};
		pose_metrics_match_pose_to_blobs(&truth, nullptr, 0, &model, &cam, &info);
		REQUIRE(info.num_visible_leds >= 7);
		std::vector<blob> blobs(info.num_visible_leds);
		for (size_t i = 0; i < blobs.size(); i++) {
			blobs[i].x = info.visible_leds[i].pos_px.x;
			blobs[i].y = info.visible_leds[i].pos_px.y;
			blobs[i].width = blobs[i].height = 3;
			blobs[i].led_id = LED_INVALID_ID;
		}
		auto *search_model = t_constellation_search_model_new(&model);
		auto *cs = correspondence_search_new(&cam);
		correspondence_search_set_blobs(cs, blobs.data(), blobs.size());
		xrt_pose recovered = XRT_POSE_IDENTITY;
		pose_metrics score = {};
		REQUIRE(correspondence_search_find_one_pose(cs, search_model, CS_FLAG_SHALLOW_SEARCH, &recovered,
		                                            nullptr, nullptr, nullptr, 0, &score));
		CHECK(score.matched_blobs == info.num_visible_leds);
		CHECK(score.reprojection_error < 1e-5);
		CHECK(recovered.position.z == Catch::Approx(0.5).margin(1e-4));
		correspondence_search_free(cs);
		t_constellation_search_model_free(search_model);
	}
}

TEST_CASE("Rift S recorded accelerometer tilt matches the optical tilt in the LED model frame")
{
	// The recorded down vector comes from the driver's top-level IMU calibration (acc_m/acc_b)
	// alone. If the TrackedObject AccCalibration rotation also belonged between the IMU and the LED
	// model, this unconstrained optical fit would disagree with it by that rotation's tilt.
	auto leds = rift_s_ring_leds;
	t_constellation_led_model model = {};
	model.id = XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER;
	model.num_leds = leds.size();
	model.leds = leds.data();
	model.min_acquisition_leds = 5;
	camera_model cam = {};
	cam.width = 640;
	cam.height = 480;
	cam.calib.fx = rift_s_ring_intrinsics[0];
	cam.calib.fy = rift_s_ring_intrinsics[1];
	cam.calib.cx = rift_s_ring_intrinsics[2];
	cam.calib.cy = rift_s_ring_intrinsics[3];
	cam.calib.model = T_DISTORTION_FISHEYE_KB4;
	cam.calib.fisheye = {rift_s_ring_kb4[0], rift_s_ring_kb4[1], rift_s_ring_kb4[2], rift_s_ring_kb4[3]};
	std::array<blob, 8> blobs = {};
	for (size_t i = 0; i < blobs.size(); i++) {
		blobs[i].x = rift_s_ring_blobs[i][0];
		blobs[i].y = rift_s_ring_blobs[i][1];
		blobs[i].width = rift_s_ring_blobs[i][2];
		blobs[i].height = rift_s_ring_blobs[i][3];
		blobs[i].led_id = LED_INVALID_ID;
	}
	auto *search_model = t_constellation_search_model_new(&model);
	auto *cs = correspondence_search_new(&cam);
	correspondence_search_set_blobs(cs, blobs.data(), blobs.size());
	xrt_vec3 pos_error = {1, 1, 1}, rot_error = {3.2f, 3.2f, 3.2f};
	xrt_vec3 gravity = rift_s_ring_gravity;
	xrt_pose prior = XRT_POSE_IDENTITY;
	math_quat_from_vec_a_to_vec_b(&rift_s_ring_down, &gravity, &prior.orientation);
	// Any tilt is allowed: the optical fit must find the tilt on its own.
	auto flags = static_cast<correspondence_search_flags>(CS_FLAG_SHALLOW_SEARCH | CS_FLAG_DEEP_SEARCH |
	                                                      CS_FLAG_HAVE_POSE_PRIOR | CS_FLAG_MATCH_GRAVITY);
	xrt_pose pose = prior;
	pose_metrics score = {};
	REQUIRE(correspondence_search_find_one_pose(cs, search_model, flags, &pose, &pos_error, &rot_error, &gravity,
	                                            M_PI, &score));
	CHECK(score.matched_blobs == 5);

	pose_metrics_blob_match_info info;
	pose_metrics_match_pose_to_blobs(&pose, blobs.data(), blobs.size(), &model, &cam, &info);
	for (int i = 0; i < info.num_visible_leds; i++) {
		if (info.visible_leds[i].matched_blob) {
			info.visible_leds[i].matched_blob->led_id = LED_MAKE_ID(model.id, info.visible_leds[i].led->id);
		}
	}
	xrt_pose refined = pose;
	int num_leds = 0, inliers = 0;
	REQUIRE(ransac_pnp_pose(&refined, blobs.data(), blobs.size(), &model, &cam, &num_leds, &inliers));
	CHECK(inliers >= 5);

	xrt_vec3 imu_down = rift_s_ring_down;
	math_vec3_normalize(&imu_down);
	for (const xrt_pose *p : {&pose, &refined}) {
		xrt_quat inverse;
		math_quat_invert(&p->orientation, &inverse);
		xrt_vec3 optical_down;
		math_quat_rotate_vec3(&inverse, &gravity, &optical_down);
		math_vec3_normalize(&optical_down);
		double angle = std::acos(std::min(1.0, (double)m_vec3_dot(optical_down, imu_down)));
		CHECK(angle < 3 * M_PI / 180);
	}
	correspondence_search_free(cs);
	t_constellation_search_model_free(search_model);
}

TEST_CASE("Native constellation search rejects noninvertible pixels without a pose change")
{
	camera_model camera{};
	camera.calib.fx = camera.calib.fy = 190;
	camera.calib.cx = 320;
	camera.calib.cy = 240;
	camera.fisheye62_valid = true;
	std::array<blob, 5> blobs{};
	std::array<t_constellation_led, 5> leds{};
	t_constellation_led_model model{};
	model.id = 2;
	model.leds = leds.data();
	model.num_leds = leds.size();
	for (unsigned i = 0; i < blobs.size(); ++i) {
		blobs[i].x = NAN;
		blobs[i].y = 240;
		blobs[i].led_id = LED_MAKE_ID(2, i);
		leds[i].id = i;
	}
	auto search = correspondence_search_new(&camera);
	correspondence_search_set_blobs(search, blobs.data(), blobs.size());
	CHECK(search->num_points == 0);
	correspondence_search_free(search);
	xrt_pose pose = XRT_POSE_IDENTITY;
	pose.position = {.1f, .2f, .6f};
	int count = 0, inliers = 99;
	CHECK_FALSE(ransac_pnp_pose(&pose, blobs.data(), blobs.size(), &model, &camera, &count, &inliers));
	CHECK(inliers == 0);
	CHECK(pose.position.x == Catch::Approx(.1));
	CHECK(pose.position.y == Catch::Approx(.2));
	CHECK(pose.position.z == Catch::Approx(.6));
	CHECK(pose.orientation.w == 1);
}

TEST_CASE("Blob extraction merges a top-left peak without a previous scanline")
{
	std::array<uint8_t, 400> pixels = {};
	pixels[0] = 255;
	pixels[20] = 100;
	xrt_frame frame = {};
	frame.width = frame.height = 20;
	frame.stride = 20;
	frame.format = XRT_FORMAT_L8;
	frame.data = pixels.data();
	auto *watch = blobwatch_new(96, 128);
	blobservation *observation = nullptr;
	blobwatch_process(watch, &frame, &observation);
	REQUIRE(observation != nullptr);
	REQUIRE(observation->num_blobs == 1);
	CHECK(observation->blobs[0].brightness == 255);
	CHECK(observation->blobs[0].x == 0);
	CHECK(observation->blobs[0].y == Catch::Approx(100.0 / 355));
	CHECK(observation->blobs[0].top == 0);
	CHECK(observation->blobs[0].height == 2);
	blobwatch_free(watch);
}

TEST_CASE("LED model cleanup releases both arrays and supports repeated cleanup")
{
	t_constellation_led_model model = {};
	t_constellation_led_model_init(2, nullptr, &model, 3, 4);
	REQUIRE(model.leds != nullptr);
	REQUIRE(model.bounding_points != nullptr);
	t_constellation_led_model_clear(&model);
	CHECK(model.leds == nullptr);
	CHECK(model.bounding_points == nullptr);
	CHECK(model.num_leds == 0);
	CHECK(model.num_bounding_points == 0);
	t_constellation_led_model_clear(&model);
	t_constellation_led_model_init(2, nullptr, &model, 0, 0);
	t_constellation_led_model_clear(&model);
}
