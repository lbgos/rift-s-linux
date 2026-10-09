// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#include "internal/constrained_pose.h"
#include "internal/sample.h"
#include <array>
#include <algorithm>
#include <cmath>
#include "constellation_rift_s_data.hpp"

namespace {
struct FactoryCamera
{
	std::array<float, 4> intrinsics;
	std::array<float, 6> radial;
	std::array<float, 2> tangential;
	xrt_matrix_4x4 device_from_camera;
};
// Factory camera calibration read from a Rift S headset.
const std::array<FactoryCamera, 5> factory = {{
    {{190.499924f, 190.499924f, 312.359253f, 233.292801f},
     {0.274657339f, 0.072651282f, -0.284994245f, 0.224762872f, -0.0842394754f, 0.0130622024f},
     {-0.000742965436f, -0.000313968281f},
     {{-0.000666916079f, 0.936471105f, -0.350744158f, 0.0f, 0.999993384f, -0.000631041592f, -0.00358627178f, 0.0f,
       -0.00357977417f, -0.350744218f, -0.936464429f, 0.0f, -0.055822663f, -0.0286783446f, -0.0698858649f, 1.0f}}},
    {{190.177917f, 190.177917f, 315.850128f, 237.907181f},
     {0.279467911f, 0.0535824969f, -0.255054951f, 0.202277347f, -0.0756442547f, 0.0117395986f},
     {4.58915711e-05f, -0.000414691196f},
     {{-0.00550019974f, 0.936006129f, -0.351940721f, 0.0f, 0.999975204f, 0.00669397181f, 0.00217518141f, 0.0f,
       0.00439186441f, -0.351920038f, -0.936019778f, 0.0f, 0.0548296794f, -0.0278966054f, -0.0701900721f, 1.0f}}},
    {{190.294937f, 190.294937f, 318.57016f, 233.739182f},
     {0.268673301f, 0.0967906564f, -0.325616896f, 0.257986337f, -0.0972961113f, 0.0150451222f},
     {0.000216994711f, 0.000161503413f},
     {{-0.369307697f, 0.877442598f, -0.306114942f, 0.0f, -0.0468660891f, -0.346567303f, -0.936853588f, 0.0f,
       -0.928124666f, -0.33164084f, 0.169112265f, 0.0f, -0.0773649067f, -0.0278907306f, -0.0490732379f, 1.0f}}},
    {{190.489212f, 190.489212f, 319.239746f, 239.368988f},
     {0.276072472f, 0.0793345198f, -0.300511897f, 0.236862913f, -0.0880616605f, 0.0134672392f},
     {0.000866621442f, -9.05699198e-05f},
     {{0.372085989f, 0.87367934f, -0.313426882f, 0.0f, -0.0441717803f, 0.353956819f, 0.934218109f, 0.0f, 0.927146614f,
       -0.333764851f, 0.170294344f, 0.0f, 0.0775006264f, -0.027320113f, -0.0488821045f, 1.0f}}},
    {{189.886932f, 189.886932f, 315.196442f, 236.638504f},
     {0.275883079f, 0.0488088988f, -0.227129221f, 0.164120957f, -0.0545484312f, 0.00755312294f},
     {0.00021202871f, -0.000509813079f},
     {{-0.99999702f, -0.000720949087f, -0.00232819817f, 0.0f, -0.00232536392f, -0.00390743697f, 0.999989688f, 0.0f,
       -0.000730038912f, 0.999992132f, 0.00390574872f, 0.0f, -0.000778796442f, 0.0459865034f, -0.0662369207f, 1.0f}}},
}};
camera_model
calibrated_camera(unsigned index)
{
	const auto &f = factory[index];
	camera_model c{};
	c.width = 640;
	c.height = 480;
	c.calib.fx = f.intrinsics[0];
	c.calib.fy = f.intrinsics[1];
	c.calib.cx = f.intrinsics[2];
	c.calib.cy = f.intrinsics[3];
	c.fisheye62_valid = true;
	std::copy(f.radial.begin(), f.radial.end(), c.fisheye62_radial);
	c.fisheye62_p1 = f.tangential[0];
	c.fisheye62_p2 = f.tangential[1];
	return c;
}
// Project LED fixtures forward without calling the production inverse.
xrt_vec2
factory_project(const FactoryCamera &c, xrt_vec3 p)
{
	double r = std::hypot(p.x, p.y), theta = std::atan2(r, p.z);
	double a = r > 1e-12 ? theta * p.x / r : 0, b = r > 1e-12 ? theta * p.y / r : 0;
	double polynomial = 1;
	for (unsigned i = 0; i < 6; ++i)
		polynomial += c.radial[i] * std::pow(theta, 2 * i + 2);
	double p1 = c.tangential[0], p2 = c.tangential[1];
	double x = a * polynomial, y = b * polynomial;
	double norm2 = x * x + y * y, dot = p1 * x + p2 * y;
	return {float(c.intrinsics[0] * (x + 2 * x * dot + p1 * norm2) + c.intrinsics[2]),
	        float(c.intrinsics[1] * (y + 2 * y * dot + p2 * norm2) + c.intrinsics[3])};
}
double
error(xrt_vec3 a, xrt_vec3 b)
{
	return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}
} // namespace

TEST_CASE("Rift S Fisheye62 matches calibrated forward pixel references")
{
	// Fixed forward references cover all five calibrations and both tangential axes.
	const std::array<xrt_vec2, 3> angular = {{{0.8f, 0.6f}, {-1.1f, 0.4f}, {0.2f, -0.7f}}};
	const std::array<std::array<xrt_vec2, 3>, 5> pixels = {{
	    {{{497.100364f, 371.917144f}, {49.367862f, 328.687290f}, {355.628758f, 81.468439f}}},
	    {{{500.828640f, 376.514696f}, {53.946196f, 332.982240f}, {359.179827f, 86.220055f}}},
	    {{{503.808216f, 372.667374f}, {56.865364f, 329.002682f}, {361.892456f, 82.231382f}}},
	    {{{505.258047f, 378.673729f}, {57.725275f, 334.557148f}, {362.776597f, 87.375536f}}},
	    {{{499.717526f, 374.841928f}, {54.394805f, 331.299469f}, {358.441628f, 85.310572f}}},
	}};
	for (unsigned k = 0; k < factory.size(); ++k) {
		auto c = calibrated_camera(k);
		for (unsigned i = 0; i < angular.size(); ++i) {
			CAPTURE(k, i);
			const double theta = std::hypot(angular[i].x, angular[i].y);
			const double scale = std::sin(theta) / theta;
			const xrt_vec3 ray = {float(angular[i].x * scale), float(angular[i].y * scale),
			                      float(std::cos(theta))};
			xrt_vec2 pixel;
			REQUIRE(camera_model_project(&c, ray.x, ray.y, ray.z, &pixel.x, &pixel.y));
			CHECK(pixel.x == Catch::Approx(pixels[k][i].x).epsilon(0).margin(0.0001));
			CHECK(pixel.y == Catch::Approx(pixels[k][i].y).epsilon(0).margin(0.0001));
			xrt_vec3 recovered;
			REQUIRE(camera_model_unproject(&c, pixels[k][i].x, pixels[k][i].y, &recovered.x, &recovered.y,
			                               &recovered.z));
			CHECK(error(recovered, ray) < 1e-6);
		}
	}
}

TEST_CASE("Rift S factory Fisheye62 pixel bearings round trip across all five sensors")
{
	for (unsigned k = 0; k < factory.size(); ++k) {
		auto c = calibrated_camera(k);
		for (unsigned y = 0; y < 480; y += 16)
			for (unsigned x = 0; x < 640; x += 16) {
				xrt_vec3 ray;
				REQUIRE(camera_model_unproject(&c, x, y, &ray.x, &ray.y, &ray.z));
				auto pixel = factory_project(factory[k], ray);
				REQUIRE(std::hypot(pixel.x - x, pixel.y - y) < 0.0002);
			}
	}
}

TEST_CASE("Rift S known LED geometry recovers offset and unit lateral gain through native calibration")
{
	auto leds = rift_s_ring_leds;
	t_constellation_led_model model{};
	model.leds = leds.data();
	model.num_leds = leds.size();
	constellation_constrained_config config;
	constellation_constrained_default_config(&config);
	config.time_budget_us = 2000000;
	// Isolate metric calibration: geometry and LED identities are known, no acquisition policy.
	config.minimum_tracking_inliers = 3;
	config.minimum_tracking_distinct_leds = 3;
	config.minimum_tilt_distinct_leds = 256;
	config.ambiguity_chi_square = 0.00001;
	double max_error = 0, min_gain = 2, max_gain = 0;
	unsigned recovered = 0;
	for (unsigned k = 0; k < factory.size(); ++k) {
		auto c = calibrated_camera(k);
		xrt_pose world_camera;
		math_pose_from_isometry(&factory[k].device_from_camera, &world_camera);
		// Native camera rays transformed by the actual factory extrinsic into device/world coordinates.
		constellation_constrained_view view{};
		view.calib = &c;
		view.P_world_cam = world_camera;
		math_pose_invert(&world_camera, &view.P_cam_world);
		blobservation observation{};
		view.observation = &observation;
		xrt_vec3 previous_truth{}, previous_recovered{};
		for (int step = -6; step <= 6; ++step) {
			xrt_pose camera_pose = XRT_POSE_IDENTITY;
			camera_pose.position = {step * .11f, -.04f, .45f};
			const xrt_vec3 axis = {1, 0, 0};
			math_quat_from_angle_vector(3.14159265f, &axis, &camera_pose.orientation);
			xrt_pose truth;
			math_pose_transform(&world_camera, &camera_pose, &truth);
			observation = {};
			for (const auto &led : leds) {
				xrt_vec3 local, normal;
				math_pose_transform_point(&camera_pose, &led.pos, &local);
				math_quat_rotate_vec3(&camera_pose.orientation, &led.dir, &normal);
				xrt_vec3 to_camera = {-local.x, -local.y, -local.z};
				math_vec3_normalize(&to_camera);
				if (normal.x * to_camera.x + normal.y * to_camera.y + normal.z * to_camera.z <
				    std::cos(LED_ANGLE * M_PI / 180))
					continue;
				auto pixel = factory_project(factory[k], local);
				if (pixel.x < 0 || pixel.x >= 640 || pixel.y < 0 || pixel.y >= 480)
					continue;
				auto &b = observation.blobs[observation.num_blobs++];
				b.x = pixel.x;
				b.y = pixel.y;
			}
			if (observation.num_blobs < 4)
				continue;
			// Translation recovery from an imperfect retained pose exercises the actual multi-LED fitter.
			xrt_pose prior = truth;
			prior.position.x += .001f;
			prior.position.y -= .001f;
			constellation_constrained_result result{};
			REQUIRE(constellation_constrained_verify_pose(
			    &model, &view, 1, &prior, CONSTELLATION_CONSTRAINED_PRIOR, &config, &result));
			max_error = std::max(max_error, error(truth.position, result.P_world_model.position));
			if (step > -6) {
				double truth_motion = error(truth.position, previous_truth);
				if (truth_motion > .01) {
					double gain =
					    error(result.P_world_model.position, previous_recovered) / truth_motion;
					min_gain = std::min(min_gain, gain);
					max_gain = std::max(max_gain, gain);
				}
			}
			previous_truth = truth.position;
			previous_recovered = result.P_world_model.position;
			++recovered;
		}
	}
	INFO("recovered=" << recovered << " worst position m=" << max_error << " gain=" << min_gain << ".."
	                  << max_gain);
	CHECK(recovered >= 45);
	CHECK(max_error < 0.00002);
	CHECK(min_gain > .9999);
	CHECK(max_gain < 1.0001);
}

TEST_CASE("Rift S ring recovers depth with unit gain from 0.3 to 0.8 m through cold and tracked search")
{
	/* The r18 report: controllers drawn farther than the hands. Project the recorded ring through
	 * each factory calibration and extrinsic straight ahead of that camera, then recover it with the
	 * search the tracker runs: cold (gravity only, no position) and tracked (trusted prior 2 cm off,
	 * known heading, 3 ms fast-path budget). Depth must come back unbiased and with unit gain. */
	auto leds = rift_s_ring_leds;
	t_constellation_led_model model{};
	model.id = 3;
	model.leds = leds.data();
	model.num_leds = leds.size();
	double worst = 0, min_gain = 2, max_gain = 0;
	unsigned cold = 0, tracked = 0;
	for (unsigned k = 0; k < factory.size(); ++k) {
		auto c = calibrated_camera(k);
		xrt_pose world_camera;
		math_pose_from_isometry(&factory[k].device_from_camera, &world_camera);
		constellation_constrained_view view{};
		view.calib = &c;
		view.P_world_cam = world_camera;
		math_pose_invert(&world_camera, &view.P_cam_world);
		blobservation observation{};
		view.observation = &observation;
		for (bool trusted : {false, true}) {
			double previous_depth = 0, previous_recovered = 0;
			for (float depth = 0.3f; depth <= 0.801f; depth += 0.1f) {
				CAPTURE(k, trusted, depth);
				xrt_pose camera_pose = XRT_POSE_IDENTITY;
				camera_pose.position = {0.01f, -0.04f, depth};
				const xrt_vec3 axis = {1, 0, 0};
				math_quat_from_angle_vector(3.14159265f, &axis, &camera_pose.orientation);
				xrt_pose truth;
				math_pose_transform(&world_camera, &camera_pose, &truth);
				observation = {};
				for (const auto &led : leds) {
					xrt_vec3 local, normal;
					math_pose_transform_point(&camera_pose, &led.pos, &local);
					math_quat_rotate_vec3(&camera_pose.orientation, &led.dir, &normal);
					xrt_vec3 to_camera = {-local.x, -local.y, -local.z};
					math_vec3_normalize(&to_camera);
					if (normal.x * to_camera.x + normal.y * to_camera.y + normal.z * to_camera.z <
					    std::cos(LED_ANGLE * M_PI / 180))
						continue;
					auto pixel = factory_project(factory[k], local);
					auto &b = observation.blobs[observation.num_blobs++];
					b = {};
					b.x = pixel.x;
					b.y = pixel.y;
					b.width = b.height = 3;
					b.led_id = LED_INVALID_ID;
				}
				REQUIRE(observation.num_blobs >= 5);
				constellation_constrained_config config;
				constellation_constrained_default_config(&config);
				xrt_pose prior = truth;
				if (trusted) {
					config.time_budget_us = 3000;
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
					config.time_budget_us = 200000;
#endif
					prior.position.x += 0.02f;
				} else {
					prior.position = {42, 42, 42};
				}
				constellation_constrained_result result{};
				REQUIRE(constellation_constrained_search(&model, &view, 1, &prior, trusted, trusted, &config,
				                                         &result));
				(trusted ? tracked : cold)++;
				worst = std::max(worst, error(truth.position, result.P_world_model.position));
				// Depth along this camera's axis, recovered and true.
				xrt_vec3 in_camera;
				math_pose_transform_point(&view.P_cam_world, &result.P_world_model.position, &in_camera);
				if (previous_depth > 0) {
					double gain = (in_camera.z - previous_recovered) / (depth - previous_depth);
					min_gain = std::min(min_gain, gain);
					max_gain = std::max(max_gain, gain);
				}
				previous_depth = depth;
				previous_recovered = in_camera.z;
			}
		}
	}
	INFO("cold=" << cold << " tracked=" << tracked << " worst m=" << worst << " depth gain=" << min_gain << ".."
	             << max_gain);
	CHECK(cold == 30);
	CHECK(tracked == 30);
	CHECK(worst < 0.0005);
	CHECK(min_gain > 0.999);
	CHECK(max_gain < 1.001);
}

TEST_CASE("Rift S wide-angle KB4 bearing converges at the sensor diagonal")
{
	t_camera_model_params c{};
	c.model = T_DISTORTION_FISHEYE_KB4;
	c.fx = c.fy = rift_s_ring_intrinsics[0];
	c.cx = rift_s_ring_intrinsics[2];
	c.cy = rift_s_ring_intrinsics[3];
	c.fisheye = {rift_s_ring_kb4[0], rift_s_ring_kb4[1], rift_s_ring_kb4[2], rift_s_ring_kb4[3]};
	const double theta = 1.55, bearing = .65;
	xrt_vec3 truth = {float(std::sin(theta) * std::cos(bearing)), float(std::sin(theta) * std::sin(bearing)),
	                  float(std::cos(theta))};
	float u, v;
	xrt_vec3 recovered;
	REQUIRE(t_camera_models_project(&c, truth.x, truth.y, truth.z, &u, &v));
	REQUIRE(t_camera_models_unproject(&c, u, v, &recovered.x, &recovered.y, &recovered.z));
	CHECK(error(truth, recovered) < .000001);
}

TEST_CASE("Native camera failures leave explicit invalid coordinates")
{
	auto camera = calibrated_camera(0);
	float x = 7, y = 8, z = 9;
	CHECK_FALSE(camera_model_unproject(&camera, NAN, 240, &x, &y, &z));
	CHECK(std::isnan(x));
	CHECK(std::isnan(y));
	CHECK(std::isnan(z));
	CHECK_FALSE(camera_model_undistort(&camera, NAN, 240, &x, &y));
	CHECK(std::isnan(x));
	CHECK(std::isnan(y));
	CHECK_FALSE(camera_model_project(&camera, 0, 0, -1, &x, &y));
	CHECK(std::isnan(x));
	CHECK(std::isnan(y));
	camera.fisheye62_valid = false;
	camera.calib.model = T_DISTORTION_FISHEYE_KB4;
	CHECK_FALSE(camera_model_unproject(&camera, NAN, 240, &x, &y, &z));
	CHECK(std::isnan(x));
	CHECK(std::isnan(y));
	CHECK(std::isnan(z));
}
