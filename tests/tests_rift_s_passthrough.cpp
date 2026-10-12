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

TEST_CASE("Passthrough precomputed resampling matches the scalar reference")
{
	// Representative fisheye calibration: tangential terms plus the
	// rolled/pitched mounting of the front sensors, differing per eye.
	auto block = calibration();
	const float angle = 0.35f;
	for (int eye = 0; eye < 2; ++eye) {
		auto &c = block.cameras[CAM_IDX_TO_ID[eye]];
		c.distortion.k[0] = eye == 0 ? 0.1f : 0.15f;
		c.distortion.p1 = 0.01f;
		c.device_from_camera = {};
		c.device_from_camera.v[1] = std::cos(angle);
		c.device_from_camera.v[2] = -std::sin(angle);
		c.device_from_camera.v[4] = 1;
		c.device_from_camera.v[9] = -std::sin(angle);
		c.device_from_camera.v[10] = -std::cos(angle);
		c.device_from_camera.v[15] = 1;
	}
	for (uint32_t pad : {0u, 32u}) {
		auto *camera = rift_s_passthrough_create(&block);
		REQUIRE(camera);
		camera->set_active(camera, true);
		xrt_frame *src[2] = {nullptr, nullptr};
		const uint64_t ts = os_monotonic_get_ns();
		for (int eye = 0; eye < 2; ++eye) {
			// Widen the allocation, then narrow the image: same pixels,
			// larger row stride, padding left as sentinel.
			u_frame_create_one_off(XRT_FORMAT_L8, 640 + pad, 480, &src[eye]);
			src[eye]->width = 640;
			REQUIRE(src[eye]->stride == 640 + pad);
			for (uint32_t y = 0; y < src[eye]->height; ++y) {
				uint8_t *row = src[eye]->data + y * src[eye]->stride;
				for (uint32_t x = 0; x < src[eye]->width; ++x)
					row[x] = (uint8_t)((x * 3 + y * 5 + eye * 11 + 1) & 0xff);
				std::fill_n(row + src[eye]->width, pad, 0xa5);
			}
			src[eye]->timestamp = ts;
			src[eye]->source_sequence = 42;
		}
		rift_s_passthrough_push(camera, src[0], src[1]);
		xrt_frame *out = nullptr;
		REQUIRE(camera->get_frame(camera, &out));
		REQUIRE(out->width == 640);
		REQUIRE(out->height == 320);
		REQUIRE(out->format == XRT_FORMAT_R8G8B8);
		CHECK(out->stereo_format == XRT_STEREO_FORMAT_SBS);
		CHECK(out->timestamp == src[0]->timestamp);
		// Frames stay immutable while the scalar reference reads them.
		size_t mismatches = 0, zeros = 0, nonzeros = 0;
		uint32_t first_x = 0, first_y = 0;
		int first_eye = 0;
		uint8_t first_got = 0, first_want = 0;
		for (int eye = 0; eye < 2; ++eye) {
			const auto &c = block.cameras[CAM_IDX_TO_ID[eye]];
			for (uint32_t y = 0; y < 320; ++y) {
				for (uint32_t x = 0; x < 320; ++x) {
					float rx = (x - camera->center.x) / camera->focal_length.x;
					float ry = (y - camera->center.y) / camera->focal_length.y;
					xrt_vec2 uv{};
					uint8_t want = 0;
					// Original per-pixel scalar path, verbatim.
					if (rift_s_passthrough_project(&c, rx, ry, &uv) && uv.x >= 0 &&
					    uv.y >= 0 && uv.x < src[eye]->width - 1 &&
					    uv.y < src[eye]->height - 1) {
						uint32_t ix = (uint32_t)uv.x, iy = (uint32_t)uv.y;
						float dx = uv.x - ix, dy = uv.y - iy;
						const uint8_t *a =
						    src[eye]->data + iy * src[eye]->stride + ix;
						want = (uint8_t)((a[0] * (1 - dx) + a[1] * dx) * (1 - dy) +
						                 (a[src[eye]->stride] * (1 - dx) +
						                  a[src[eye]->stride + 1] * dx) *
						                     dy);
					}
					want == 0 ? ++zeros : ++nonzeros;
					const uint8_t *dst = out->data + (size_t)y * out->stride +
					                     (size_t)(eye * 320 + x) * 3;
					if (dst[0] != want || dst[1] != want || dst[2] != want) {
						if (mismatches == 0) {
							first_eye = eye;
							first_x = x;
							first_y = y;
							first_got = dst[0];
							first_want = want;
						}
						++mismatches;
					}
				}
			}
		}
		INFO("pad=" << pad << " zeros=" << zeros << " nonzeros=" << nonzeros << " first mismatch eye="
		            << first_eye << " x=" << first_x << " y=" << first_y << " got=" << (int)first_got
		            << " want=" << (int)first_want);
		CHECK(mismatches == 0);
		// Both in-bounds interpolation and out-of-bounds edges are covered.
		CHECK(zeros > 0);
		CHECK(nonzeros > 0);
		xrt_frame_reference(&out, nullptr);
		rift_s_passthrough_destroy(camera);
		xrt_frame_reference(&src[0], nullptr);
		xrt_frame_reference(&src[1], nullptr);
	}
}
