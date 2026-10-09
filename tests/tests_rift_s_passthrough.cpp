// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#include "rift_s_passthrough.h"
#include "os/os_time.h"
#include "util/u_frame.h"
#include <cmath>

static rift_s_camera_calibration_block
calibration()
{
	rift_s_camera_calibration_block block{};
	for (auto &c : block.cameras) {
		c.device_from_camera.v[0] = c.device_from_camera.v[15] = 1;
		c.device_from_camera.v[5] = c.device_from_camera.v[10] = -1;
		c.projection = {320, 240, 200, 200};
	}
	return block;
}

TEST_CASE("Passthrough projects calibrated fisheye rays and sensor roll")
{
	auto block = calibration();
	auto &c = block.cameras[RIFT_S_CAMERA_FRONT_LEFT];
	xrt_vec2 pixel;
	REQUIRE(rift_s_passthrough_project(&c, 0, 0, &pixel));
	CHECK(pixel.x == Catch::Approx(320));
	CHECK(pixel.y == Catch::Approx(240));
	c.distortion.k[0] = 0.1f;
	c.distortion.p1 = 0.01f;
	REQUIRE(rift_s_passthrough_project(&c, 1, 0, &pixel));
	double t = atan(1.0);
	CHECK(pixel.x == Catch::Approx(320 + 200 * t * (1 + 0.1 * t * t)));
	CHECK(pixel.y == Catch::Approx(240 + 200 * 0.01 * t * t));
	c.distortion = {};
	// Native camera has a 90 degree roll in firmware device coordinates.
	c.device_from_camera.v[0] = c.device_from_camera.v[5] = 0;
	c.device_from_camera.v[1] = -1;
	c.device_from_camera.v[4] = -1;
	REQUIRE(rift_s_passthrough_project(&c, 1, 0, &pixel));
	CHECK(pixel.x == Catch::Approx(320));
	CHECK(pixel.y == Catch::Approx(240 - 200 * t));
	c.device_from_camera.v[10] = 1;
	CHECK_FALSE(rift_s_passthrough_project(&c, 0, 0, &pixel));
}

TEST_CASE("Passthrough keeps synchronized SLAM frames only while requested")
{
	auto block = calibration();
	auto *camera = rift_s_passthrough_create(&block);
	REQUIRE(camera);
	xrt_frame *left = nullptr, *right = nullptr, *out = nullptr;
	u_frame_create_one_off(XRT_FORMAT_L8, 640, 480, &left);
	u_frame_create_one_off(XRT_FORMAT_L8, 640, 480, &right);
	std::fill_n(left->data, left->size, 70);
	std::fill_n(right->data, right->size, 180);
	left->timestamp = right->timestamp = os_monotonic_get_ns();
	rift_s_passthrough_push(camera, left, right);
	CHECK_FALSE(camera->get_frame(camera, &out));
	camera->set_active(camera, true);
	rift_s_passthrough_push(camera, left, right);
	REQUIRE(camera->get_frame(camera, &out));
	CHECK(out->width == 640);
	CHECK(out->height == 320);
	CHECK(out->format == XRT_FORMAT_R8G8B8);
	CHECK(out->timestamp == left->timestamp);
	CHECK(out->data[160 * out->stride + 160 * 3] == 70);
	CHECK(out->data[160 * out->stride + 480 * 3] == 180);
	auto *held = out;
	xrt_frame *cached = nullptr;
	REQUIRE(camera->get_frame(camera, &cached));
	CHECK(cached == held);
	xrt_frame_reference(&cached, nullptr);
	left->timestamp = right->timestamp = os_monotonic_get_ns() - 300000000;
	rift_s_passthrough_push(camera, left, right);
	CHECK_FALSE(camera->get_frame(camera, &cached));
	camera->set_active(camera, false);
	CHECK_FALSE(camera->get_frame(camera, &cached));
	camera->set_active(camera, true);
	CHECK_FALSE(camera->get_frame(camera, &cached));
	// A consumer reference stays alive after stopping/destroying the provider.
	rift_s_passthrough_destroy(camera);
	CHECK(held->data[160 * held->stride + 160 * 3] == 70);
	xrt_frame_reference(&out, nullptr);
	xrt_frame_reference(&left, nullptr);
	xrt_frame_reference(&right, nullptr);
}

TEST_CASE("Passthrough rectifies the front camera quarter-turn and downward pitch")
{
	auto block = calibration();
	auto &c = block.cameras[RIFT_S_CAMERA_FRONT_LEFT];
	const float angle = 0.35f;
	c.device_from_camera = {};
	c.device_from_camera.v[1] = std::cos(angle);
	c.device_from_camera.v[2] = -std::sin(angle);
	c.device_from_camera.v[4] = 1;
	c.device_from_camera.v[9] = -std::sin(angle);
	c.device_from_camera.v[10] = -std::cos(angle);
	c.device_from_camera.v[15] = 1;
	xrt_vec2 center;
	REQUIRE(rift_s_passthrough_project(&c, 0, 0, &center));
	CHECK(center.x == Catch::Approx(320 + 200 * angle));
	CHECK(center.y == Catch::Approx(240));
	xrt_vec2 right;
	REQUIRE(rift_s_passthrough_project(&c, 0.1f, 0, &right));
	CHECK(right.y > center.y);
}

TEST_CASE("Passthrough rejects an incomplete sensor pair without replacing the last complete capture")
{
	auto block = calibration();
	auto *camera = rift_s_passthrough_create(&block);
	REQUIRE(camera);
	camera->set_active(camera, true);
	xrt_frame *left = nullptr, *right = nullptr, *out = nullptr;
	u_frame_create_one_off(XRT_FORMAT_L8, 640, 480, &left);
	u_frame_create_one_off(XRT_FORMAT_L8, 640, 480, &right);
	std::fill_n(left->data, left->size, 70);
	std::fill_n(right->data, right->size, 180);
	left->timestamp = right->timestamp = os_monotonic_get_ns();
	left->source_sequence = right->source_sequence = 10;
	rift_s_passthrough_push(camera, left, right);
	REQUIRE(camera->get_frame(camera, &out));
	const auto complete_timestamp = out->timestamp;
	xrt_frame_reference(&out, nullptr);

	xrt_frame *bad_left = nullptr, *bad_right = nullptr;
	u_frame_create_one_off(XRT_FORMAT_L8, 640, 480, &bad_left);
	u_frame_create_one_off(XRT_FORMAT_L8, 640, 480, &bad_right);
	std::fill_n(bad_left->data, bad_left->size, 75);
	std::fill_n(bad_right->data, bad_right->size, 185);
	bad_left->timestamp = bad_right->timestamp = complete_timestamp + 1;
	bad_left->source_sequence = bad_right->source_sequence = 11;
	uint8_t *right_storage = bad_right->data;
	SECTION("missing left payload") { std::fill_n(bad_left->data, bad_left->size, 0); }
	SECTION("missing right payload") { std::fill_n(bad_right->data, bad_right->size, 0); }
	SECTION("truncated final sensor row") { --bad_right->size; }
	SECTION("row stride shorter than the image") { bad_left->stride = 639; }
	SECTION("different camera exposure timestamp") { ++bad_right->timestamp; }
	SECTION("different source frame sequence") { ++bad_right->source_sequence; }
	SECTION("empty camera") { bad_right->height = 0; }
	SECTION("missing storage") { bad_right->data = nullptr; }
	rift_s_passthrough_push(camera, bad_left, bad_right);
	REQUIRE(camera->get_frame(camera, &out));
	CHECK(out->timestamp == complete_timestamp);
	CHECK(out->data[160 * out->stride + 160 * 3] == 70);
	CHECK(out->data[160 * out->stride + 480 * 3] == 180);
	xrt_frame_reference(&out, nullptr);
	// Once the retained raw capture expires, the cached conversion cannot renew it.
	left->timestamp = right->timestamp = os_monotonic_get_ns() - 300000000;
	CHECK_FALSE(camera->get_frame(camera, &out));
	rift_s_passthrough_destroy(camera);
	bad_right->data = right_storage;
	xrt_frame_reference(&left, nullptr);
	xrt_frame_reference(&right, nullptr);
	xrt_frame_reference(&bad_left, nullptr);
	xrt_frame_reference(&bad_right, nullptr);
}

TEST_CASE("Passthrough accepts a dark stereo scene and recovers both sensors together")
{
	auto block = calibration();
	auto *camera = rift_s_passthrough_create(&block);
	REQUIRE(camera);
	camera->set_active(camera, true);
	xrt_frame *left = nullptr, *right = nullptr, *out = nullptr;
	u_frame_create_one_off(XRT_FORMAT_L8, 640, 480, &left);
	u_frame_create_one_off(XRT_FORMAT_L8, 640, 480, &right);
	std::fill_n(left->data, left->size, 0);
	std::fill_n(right->data, right->size, 0);
	left->timestamp = right->timestamp = os_monotonic_get_ns();
	rift_s_passthrough_push(camera, left, right);
	REQUIRE(camera->get_frame(camera, &out));
	CHECK(out->data[160 * out->stride + 160 * 3] == 0);
	CHECK(out->data[160 * out->stride + 480 * 3] == 0);
	xrt_frame_reference(&out, nullptr);
	std::fill_n(left->data, left->size, 80);
	std::fill_n(right->data, right->size, 190);
	left->timestamp = right->timestamp = os_monotonic_get_ns();
	rift_s_passthrough_push(camera, left, right);
	REQUIRE(camera->get_frame(camera, &out));
	CHECK(out->data[160 * out->stride + 160 * 3] == 80);
	CHECK(out->data[160 * out->stride + 480 * 3] == 190);
	xrt_frame_reference(&out, nullptr);
	rift_s_passthrough_destroy(camera);
	xrt_frame_reference(&left, nullptr);
	xrt_frame_reference(&right, nullptr);
}

TEST_CASE("Passthrough rectification and export share the 110 degree pinhole")
{
	auto block = calibration();
	auto *camera = rift_s_passthrough_create(&block);
	REQUIRE(camera);
	CHECK(camera->focal_length.x == Catch::Approx(112.033206).margin(1e-4));
	CHECK(camera->focal_length.y == camera->focal_length.x);
	CHECK(2 * atan(160.0 / camera->focal_length.x) * 180 / M_PI == Catch::Approx(110));
	for (const auto &pose : camera->head_from_camera) {
		CHECK(pose.orientation.w == 1);
		CHECK(pose.orientation.x == 0);
	}
	// A tilted/rolled physical sensor is sampled along the exported head-aligned ray.
	xrt_vec2 pixel;
	REQUIRE(rift_s_passthrough_project(&block.cameras[RIFT_S_CAMERA_FRONT_LEFT],
	                                  (319 - camera->center.x) / camera->focal_length.x, 0, &pixel));
	CHECK(pixel.x == Catch::Approx(320 + 200 * atan(159.5 / camera->focal_length.x)).margin(1e-4));
	rift_s_passthrough_destroy(camera);
}
