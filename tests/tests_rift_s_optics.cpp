// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0

#include "catch_amalgamated.hpp"
#include "../src/xrt/state_trackers/steamvr_drv/ovrd_display.hpp"
extern "C" {
#include "rift_s/rift_s_optics.h"
}

#include <array>
#include <cmath>

namespace {

constexpr float default_ipd_m = 0.0635f;
xrt_uv_triplet
sample(const rift_s_optics &optics, float u, float v)
{
	xrt_uv_triplet out{};
	rift_s_optics_compute(&optics, u, v, &out);
	return out;
}

rift_s_optics
make_optics(uint32_t eye, float ipd_m = default_ipd_m)
{
	rift_s_optics optics{};
	REQUIRE(rift_s_optics_init(&optics, eye, ipd_m, 1, 1));
	return optics;
}

// Vertices of the Oculus runtime's left-eye distortion mesh (IPD 63.5 mm) inside the FOV:
// tangent-space position and where each channel lands in left-eye view pixels.
struct Vertex
{
	double tan_x, tan_y;
	std::array<double, 2> r, g, b;
};

const Vertex oculus_vertices[] = {
    {-0.95006, -0.98320, {111.19, 205.81}, {114.94, 209.70}, {116.69, 211.50}},
    {-0.95004, -0.27761, {13.75, 546.34}, {19.12, 547.91}, {21.65, 548.65}},
    {-0.95007, 0.42799, {28.26, 981.18}, {33.36, 978.88}, {35.73, 977.81}},
    {-0.95000, 1.13350, {136.12, 1283.08}, {139.43, 1279.13}, {140.89, 1277.39}},
    {-0.45762, -0.98331, {334.43, 132.08}, {336.78, 137.12}, {337.87, 139.47}},
    {-0.45764, -0.27765, {262.28, 510.22}, {265.27, 512.04}, {266.88, 513.02}},
    {-0.45756, 0.42798, {273.28, 1033.12}, {276.31, 1030.28}, {277.87, 1028.83}},
    {-0.45747, 1.13332, {351.87, 1354.64}, {353.93, 1349.53}, {354.89, 1347.15}},
    {0.03492, -0.98312, {629.95, 103.28}, {629.75, 108.86}, {629.66, 111.49}},
    {0.03492, -0.27760, {636.22, 495.96}, {636.03, 497.53}, {635.91, 498.49}},
    {0.03492, 0.42797, {635.28, 1053.85}, {635.06, 1051.16}, {634.94, 1049.66}},
    {0.03492, 1.13354, {628.43, 1381.96}, {628.26, 1376.39}, {628.18, 1373.79}},
    {0.52741, -0.98320, {918.76, 140.76}, {916.10, 145.72}, {914.88, 147.99}},
    {0.52739, -0.27760, {998.28, 514.59}, {994.78, 516.43}, {992.96, 517.39}},
    {0.52738, 0.42795, {986.10, 1026.78}, {982.62, 1023.96}, {980.86, 1022.53}},
    {0.52736, 1.13347, {899.41, 1346.23}, {897.12, 1341.33}, {896.12, 1339.16}},
    {1.01986, -0.98316, {1129.41, 217.39}, {1125.55, 221.11}, {1123.80, 222.80}},
    {1.01998, -0.27763, {1225.99, 551.80}, {1220.62, 553.26}, {1218.11, 553.94}},
    {1.02003, 0.42803, {1211.68, 973.31}, {1206.53, 971.14}, {1204.12, 970.13}},
    {1.01982, 1.13348, {1104.16, 1271.41}, {1100.83, 1267.71}, {1099.33, 1266.04}},
};

} // namespace

TEST_CASE("compute reproduces the Oculus runtime mesh")
{
	const auto optics = make_optics(0);
	const double width = optics.tan_left + optics.tan_right;
	const double height = optics.tan_up + optics.tan_down;
	for (const Vertex &vertex : oculus_vertices) {
		const double expected_x = (vertex.tan_x + optics.tan_left) / width;
		const double expected_y = (vertex.tan_y + optics.tan_up) / height;
		for (const auto *px : {&vertex.r, &vertex.g, &vertex.b}) {
			const bool is_r = px == &vertex.r, is_g = px == &vertex.g;
			auto pick = [&](const xrt_uv_triplet &uv) { return is_r ? uv.r : is_g ? uv.g : uv.b; };
			const float u = (float)((*px)[0] / 1280.0), v = (float)((*px)[1] / 1440.0);
			const xrt_vec2 got = pick(sample(optics, u, v));
			// One pixel outward along the radius, in tangent units: the local px -> tan scale.
			const double dx = (*px)[0] - optics.lens_center_px.x, dy = (*px)[1] - optics.lens_center_px.y;
			const double r = std::hypot(dx, dy);
			const xrt_vec2 step = pick(sample(optics, (float)(((*px)[0] + dx / r) / 1280.0),
			                                  (float)(((*px)[1] + dy / r) / 1440.0)));
			const double one_px =
			    std::hypot((step.x - got.x) * width, (step.y - got.y) * height);
			const double error = std::hypot((got.x - expected_x) * width, (got.y - expected_y) * height);
			CHECK(error < one_px);
		}
	}
}

TEST_CASE("lens centre maps to tangent zero")
{
	for (uint32_t eye = 0; eye < 2; eye++) {
		for (float ipd_m : {0.0535f, default_ipd_m, 0.068f, 0.0735f}) {
			const auto optics = make_optics(eye, ipd_m);
			const auto uv = sample(optics, optics.lens_center_px.x / 1280.0f, optics.lens_center_px.y / 1440.0f);
			const float x = optics.tan_left / (optics.tan_left + optics.tan_right);
			const float y = optics.tan_up / (optics.tan_up + optics.tan_down);
			for (const xrt_vec2 &channel : {uv.r, uv.g, uv.b}) {
				CHECK(channel.x == Catch::Approx(x).margin(1e-6));
				CHECK(channel.y == Catch::Approx(y).margin(1e-6));
			}
		}
	}
}

TEST_CASE("right eye mirrors the left eye")
{
	const auto left = make_optics(0, 0.068f);
	const auto right = make_optics(1, 0.068f);
	CHECK(right.lens_center_px.x == Catch::Approx(1280.0f - left.lens_center_px.x));
	for (const Vertex &vertex : oculus_vertices) {
		const auto l = sample(left, (float)(vertex.g[0] / 1280.0), (float)(vertex.g[1] / 1440.0));
		const auto r = sample(right, (float)(1.0 - vertex.g[0] / 1280.0), (float)(vertex.g[1] / 1440.0));
		CHECK(r.g.x == Catch::Approx(1.0f - l.g.x).margin(1e-5));
		CHECK(r.g.y == Catch::Approx(l.g.y).margin(1e-5));
		CHECK(r.r.x == Catch::Approx(1.0f - l.r.x).margin(1e-5));
		CHECK(r.b.x == Catch::Approx(1.0f - l.b.x).margin(1e-5));
	}
}


TEST_CASE("SteamVR render size follows the upright FOV and centre pixel density")
{
	struct TestHmd
	{
		xrt_device device{};
		xrt_hmd_parts parts{};
		std::array<rift_s_optics, 2> optics;
	} hmd;
	hmd.device.hmd = &hmd.parts;
	hmd.parts.view_count = 2;
	hmd.parts.screens[0].w_pixels = 1440;
	hmd.parts.screens[0].h_pixels = 2560;
	hmd.device.compute_distortion = [](xrt_device *device, uint32_t eye, float u, float v,
	                                   xrt_uv_triplet *result) {
		auto *hmd = reinterpret_cast<TestHmd *>(device);
		rift_s_optics_compute(&hmd->optics[eye], u, v, result);
		return XRT_SUCCESS;
	};
	for (uint32_t eye = 0; eye < 2; eye++) {
		hmd.optics[eye] = make_optics(eye, 0.068f);
		hmd.parts.distortion.fov[eye] = hmd.optics[eye].fov;
		auto &view = hmd.parts.views[eye];
		view.display.w_pixels = 1280;
		view.display.h_pixels = 1440;
		view.viewport.w_pixels = 1440;
		view.viewport.h_pixels = 1280;
		view.viewport.y_pixels = eye * 1280;
		view.rot.vecs[0] = {0, 1};
		view.rot.vecs[1] = {-1, 0};
	}
	uint32_t width = 0, height = 0;
	ovrd_get_recommended_render_target_size(&hmd.device, 1.0f, &width, &height);
	CHECK(width == Catch::Approx(1648).margin(2));
	CHECK(height == Catch::Approx(1771).margin(2));
	const double aspect = (hmd.optics[0].tan_left + hmd.optics[0].tan_right) /
	                      (hmd.optics[0].tan_up + hmd.optics[0].tan_down);
	CHECK((double)width / height == Catch::Approx(aspect).margin(0.001));
	const auto base_width = width, base_height = height;
	ovrd_get_recommended_render_target_size(&hmd.device, 1.4f, &width, &height);
	CHECK(width == Catch::Approx(base_width * 1.4).margin(2));
	CHECK(height == Catch::Approx(base_height * 1.4).margin(2));

	// A wider projection needs more source pixels while preserving the same optical density.
	for (uint32_t eye = 0; eye < 2; eye++) {
		REQUIRE(rift_s_optics_init(&hmd.optics[eye], eye, 0.068f, 1.0f, 1.25f));
		hmd.parts.distortion.fov[eye] = hmd.optics[eye].fov;
	}
	ovrd_get_recommended_render_target_size(&hmd.device, 1.0f, &width, &height);
	CHECK(width == Catch::Approx(base_width).margin(2));
	CHECK(height == Catch::Approx(base_height * 1.25).margin(2));

	// Drivers without a compute model fall back to upright per-eye dimensions.
	hmd.device.compute_distortion = nullptr;
	ovrd_get_recommended_render_target_size(&hmd.device, 1.0f, &width, &height);
	CHECK(width == 1280);
	CHECK(height == 1440);
}

TEST_CASE("OpenVR projection and distortion agree on the zero ray")
{
	for (uint32_t eye = 0; eye < 2; eye++) {
		const auto optics = make_optics(eye);
		const auto uv = sample(optics, optics.lens_center_px.x / 1280, optics.lens_center_px.y / 1440);
		float left, right, top, bottom;
		ovrd_get_projection_raw(optics.fov, &left, &right, &top, &bottom);
		// SteamVR: M11=2/(B-T), M12=(B+T)/(B-T), texture V=(1-NDC_y)/2.
		const xrt_vec2 zero_uv{-left / (right - left), bottom / (bottom - top)};
		for (const xrt_vec2 &channel : {uv.r, uv.g, uv.b}) {
			CHECK(zero_uv.x == Catch::Approx(channel.x).margin(1e-6));
			CHECK(zero_uv.y == Catch::Approx(channel.y).margin(1e-6));
		}
	}
}
