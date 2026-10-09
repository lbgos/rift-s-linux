// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#include "rift_s_world_anchor.h"
#include "rift_s_slam_guard.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <unistd.h>
#include <cstdlib>
#include "math/m_api.h"

namespace {
rift_s_world_anchor_config
calibration(bool native = true)
{
	rift_s_world_anchor_config c{};
	const float k[2][6] = {
	    {0.274657339f, 0.072651282f, -0.284994245f, 0.224762872f, -0.0842394754f, 0.0130622024f},
	    {0.279467911f, 0.0535824969f, -0.255054951f, 0.202277347f, -0.0756442547f, 0.0117395986f}};
	c.camera[0].projection = {312.359253f, 233.292801f, 190.499924f, 190.499924f};
	c.camera[1].projection = {315.850128f, 237.907181f, 190.177917f, 190.177917f};
	for (unsigned e = 0; e < 2; ++e) {
		c.camera[e].roi.extent = {640, 480};
		std::copy_n(k[e], 6, c.camera[e].distortion.k);
	}
	c.camera[0].distortion.p1 = -0.000742965436f;
	c.camera[0].distortion.p2 = -0.000313968281f;
	c.camera[1].distortion.p1 = 4.58915711e-05f;
	c.camera[1].distortion.p2 = -0.000414691196f;
	c.imu_from_camera[0] = {{0.695778f, 0.695791f, -0.127312f, 0.124737f}, {-0.055528f, -0.009581f, -0.001194f}};
	c.imu_from_camera[1] = {{0.693510f, 0.697892f, -0.125286f, 0.127646f}, {0.055125f, -0.008799f, -0.001498f}};
	for (unsigned e = 0; e < 2; ++e) {
		auto &p = c.imu_from_camera[e];
		if (!native)
			p.orientation = {0, 0, 0, 1};
		double n = std::sqrt(p.orientation.x * p.orientation.x + p.orientation.y * p.orientation.y +
		                     p.orientation.z * p.orientation.z + p.orientation.w * p.orientation.w);
		p.orientation.x /= n;
		p.orientation.y /= n;
		p.orientation.z /= n;
		p.orientation.w /= n;
	}
	return c;
}
cv::Matx33d
rotation(xrt_quat q)
{
	double x = q.x, y = q.y, z = q.z, w = q.w;
	return {1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),
	        2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
	        2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)};
}
cv::Vec3d
translation(xrt_pose p)
{
	return {p.position.x, p.position.y, p.position.z};
}
std::array<std::vector<rift_s_world_anchor_observation>, 2>
observations(const rift_s_world_anchor_config &c, xrt_pose head)
{
	std::array<std::vector<rift_s_world_anchor_observation>, 2> out;
	for (int y = 50; y <= 430; y += 45)
		for (int x = 55; x <= 590; x += 55) {
			xrt_vec2 pixel{float(x), float(y)};
			xrt_vec3 ray;
			REQUIRE(rift_s_world_anchor_unproject(&c.camera[0], &pixel, &ray));
			double depth = 1.2 + 0.15 * ((x / 55 + 2 * y / 45) % 9);
			cv::Vec3d in_imu =
			    rotation(c.imu_from_camera[0].orientation) * (cv::Vec3d{ray.x, ray.y, ray.z} * depth) +
			    translation(c.imu_from_camera[0]);
			cv::Vec3d world = rotation(head.orientation) * in_imu + translation(head);
			for (unsigned e = 0; e < 2; ++e) {
				cv::Vec3d native = rotation(c.imu_from_camera[e].orientation).t() *
				                   (in_imu - translation(c.imu_from_camera[e]));
				xrt_vec3 p{float(native[0]), float(native[1]), float(native[2])};
				xrt_vec2 projected;
				REQUIRE(rift_s_world_anchor_project(&c.camera[e], &p, &projected));
				out[e].push_back({{float(world[0]), float(world[1]), float(world[2])}, projected});
			}
		}
	return out;
}
bool
solve(const rift_s_world_anchor_config &c,
      const std::array<std::vector<rift_s_world_anchor_observation>, 2> &o,
      rift_s_world_anchor_result &r)
{
	const rift_s_world_anchor_observation *ptr[2] = {o[0].data(), o[1].data()};
	uint32_t n[2] = {uint32_t(o[0].size()), uint32_t(o[1].size())};
	return rift_s_world_anchor_solve(&c, ptr, n, &r);
}
struct Images
{
	cv::Mat images[RIFT_S_WORLD_MAX_CAMERAS];
	xrt_frame frames[RIFT_S_WORLD_MAX_CAMERAS]{};
	Images(const rift_s_world_anchor_config &c, float head_x = 0, float yaw = 0)
	{
		cv::Mat small(160, 200, CV_8UC1);
		cv::RNG random(19);
		random.fill(small, cv::RNG::UNIFORM, 0, 256);
		cv::Mat texture;
		cv::resize(small, texture, {2000, 1600}, 0, 0, cv::INTER_CUBIC);
		for (unsigned e = 0; e < (c.camera_count ? c.camera_count : 2); ++e) {
			cv::Mat map(480, 640, CV_32FC2);
			for (int y = 0; y < 480; ++y)
				for (int x = 0; x < 640; ++x) {
					xrt_vec2 pixel{float(x), float(y)};
					xrt_vec3 ray{};
					if (!rift_s_world_anchor_unproject(&c.camera[e], &pixel, &ray)) {
						map.at<cv::Vec2f>(y, x) = {-1, -1};
						continue;
					}
					auto r = rotation({0, float(std::sin(yaw / 2)), 0, float(std::cos(yaw / 2))});
					auto d = r * rotation(c.imu_from_camera[e].orientation) *
					         cv::Vec3d{ray.x, ray.y, ray.z};
					auto origin = r * translation(c.imu_from_camera[e]) + cv::Vec3d{head_x, 0, 0};
					double scale = (2.3 - origin[2]) / d[2];
					map.at<cv::Vec2f>(y, x) =
					    scale > 0 ? cv::Vec2f{float(1000 + 100 * (d[0] * scale + origin[0])),
					                          float(800 + 100 * (d[1] * scale + origin[1]))}
					              : cv::Vec2f{-1, -1};
				}
			cv::remap(texture, images[e], map, cv::Mat(), cv::INTER_LINEAR);
			frames[e].format = XRT_FORMAT_L8;
			frames[e].width = 640;
			frames[e].height = 480;
			frames[e].stride = images[e].step;
			frames[e].size = images[e].total();
			frames[e].data = images[e].data;
			frames[e].timestamp = 1000000000;
		}
	}
};
using Anchor = std::unique_ptr<rift_s_world_anchor, decltype(&rift_s_world_anchor_destroy)>;
} // namespace

TEST_CASE("Room recovery matches the native Fisheye62 pixel references")
{
	auto c = calibration();
	const xrt_vec2 angular[] = {{0.8f, 0.6f}, {-1.1f, 0.4f}, {0.2f, -0.7f}};
	const xrt_vec2 pixels[2][3] = {
	    {{497.100364f, 371.917144f}, {49.367862f, 328.687290f}, {355.628758f, 81.468439f}},
	    {{500.828640f, 376.514696f}, {53.946196f, 332.982240f}, {359.179827f, 86.220055f}},
	};
	for (unsigned camera = 0; camera < 2; ++camera) {
		for (unsigned i = 0; i < 3; ++i) {
			CAPTURE(camera, i);
			double theta = std::hypot(angular[i].x, angular[i].y), scale = std::sin(theta) / theta;
			xrt_vec3 ray{float(angular[i].x * scale), float(angular[i].y * scale), float(std::cos(theta))};
			xrt_vec2 pixel;
			REQUIRE(rift_s_world_anchor_project(&c.camera[camera], &ray, &pixel));
			CHECK(pixel.x == Catch::Approx(pixels[camera][i].x).epsilon(0).margin(0.0001));
			CHECK(pixel.y == Catch::Approx(pixels[camera][i].y).epsilon(0).margin(0.0001));
			xrt_vec3 recovered;
			REQUIRE(rift_s_world_anchor_unproject(&c.camera[camera], &pixels[camera][i], &recovered));
			CHECK(std::hypot(std::hypot(recovered.x - ray.x, recovered.y - ray.y), recovered.z - ray.z) < 1e-6);
		}
	}
}

TEST_CASE("Rift S world anchor inverts all six radial and both tangential camera terms")
{
	auto c = calibration();
	double worst = 0;
	for (unsigned e = 0; e < 2; ++e)
		for (int y = 10; y < 480; y += 17)
			for (int x = 10; x < 640; x += 23) {
				xrt_vec2 pixel{float(x), float(y)}, roundtrip;
				xrt_vec3 ray;
				INFO("pixel " << x << "," << y);
				REQUIRE(rift_s_world_anchor_unproject(&c.camera[e], &pixel, &ray));
				REQUIRE(rift_s_world_anchor_project(&c.camera[e], &ray, &roundtrip));
				worst =
				    std::max(worst, double(std::hypot(roundtrip.x - pixel.x, roundtrip.y - pixel.y)));
			}
	CHECK(worst < 0.0001);
}
TEST_CASE("Rift S stereo anchors recover IMU pose without position bias or lateral gain")
{
	auto c = calibration();
	for (float x : {-2.f, 0.f, 2.f}) {
		xrt_pose expected{{0, 0.24740396f, 0, 0.96891242f}, {x, 1.3f, -0.45f}};
		auto obs = observations(c, expected);
		rift_s_world_anchor_result r{};
		REQUIRE(solve(c, obs, r));
		CHECK(r.relocalized);
		CHECK(r.inliers[0] >= 80);
		CHECK(r.inliers[1] >= 80);
		CHECK(cv::norm(translation(expected) - translation(r.world_from_imu)) < 0.00005);
		CHECK(r.max_reprojection_px < 0.001);
		CHECK(cv::norm(cv::Mat(rotation(expected.orientation) - rotation(r.world_from_imu.orientation))) <
		      0.0001);
	}
}
TEST_CASE("Rift S stereo anchors reject camera disagreement and concentrated false matches")
{
	auto c = calibration();
	xrt_pose expected{{0, 0, 0, 1}, {0, 1.3f, 0}};
	auto obs = observations(c, expected);
	rift_s_world_anchor_result r{};
	for (auto &o : obs[1])
		o.world_point.x += 0.12f;
	CHECK_FALSE(solve(c, obs, r));
	obs = observations(c, expected);
	for (auto &eye : obs)
		for (auto &o : eye)
			o.pixel = {320 + 0.02f * (o.pixel.x - 320), 240 + 0.02f * (o.pixel.y - 240)};
	CHECK_FALSE(solve(c, obs, r));
	obs = observations(c, expected);
	for (auto &eye : obs)
		for (size_t i = 0; i < eye.size() * 3 / 4; ++i)
			eye[i].world_point.x += float(i % 7) * 0.2f;
	CHECK_FALSE(solve(c, obs, r));
}
TEST_CASE("Rift S visual keyframes persist and invalid maps fail closed")
{
	auto c = calibration(false);
	Images images(c);
	char directory[] = "/tmp/rift-s-anchor-XXXXXX";
	REQUIRE(mkdtemp(directory));
	std::string path = std::string(directory) + "/map";
	Anchor anchor(rift_s_world_anchor_create(&c, path.c_str()), rift_s_world_anchor_destroy);
	REQUIRE(anchor);
	CHECK_FALSE(rift_s_world_anchor_has_map(anchor.get()));
	rift_s_world_anchor_result r{};
	xrt_pose trusted{{0, 0, 0, 1}, {0, 0, 0}};
	REQUIRE(rift_s_world_anchor_process(anchor.get(), &images.frames[0], &images.frames[1], &trusted, &r));
	CHECK(r.recorded);
	CHECK(r.keyframes == 1);
	REQUIRE(rift_s_world_anchor_save(anchor.get()));
	anchor.reset(rift_s_world_anchor_create(&c, path.c_str()));
	REQUIRE(rift_s_world_anchor_has_map(anchor.get()));
	REQUIRE(rift_s_world_anchor_process(anchor.get(), &images.frames[0], &images.frames[1], nullptr, &r));
	CHECK(r.relocalized);
	CHECK(cv::norm(translation(r.world_from_imu)) < 0.03);
	rift_s_world_anchor_clear(anchor.get());
	CHECK_FALSE(rift_s_world_anchor_has_map(anchor.get()));
	REQUIRE(rift_s_world_anchor_reload(anchor.get()));
	images.frames[1].timestamp += 1;
	CHECK_FALSE(rift_s_world_anchor_process(anchor.get(), &images.frames[0], &images.frames[1], nullptr, &r));
	auto changed = c;
	changed.camera[0].projection.fx += 0.01f;
	Anchor mismatch(rift_s_world_anchor_create(&changed, path.c_str()), rift_s_world_anchor_destroy);
	REQUIRE(mismatch);
	CHECK_FALSE(rift_s_world_anchor_has_map(mismatch.get()));
	{
		std::fstream file(path, std::ios::in | std::ios::out | std::ios::binary);
		file.seekg(80);
		char original;
		REQUIRE(file.get(original));
		file.seekp(80);
		file.put(char(original ^ 0x7f));
	}
	CHECK_FALSE(rift_s_world_anchor_reload(anchor.get()));
	CHECK_FALSE(rift_s_world_anchor_has_map(anchor.get()));
	CHECK_FALSE(rift_s_world_anchor_save(anchor.get()));
	rift_s_world_anchor_clear(anchor.get());
	REQUIRE(rift_s_world_anchor_save(anchor.get()));
	Anchor empty(rift_s_world_anchor_create(&c, path.c_str()), rift_s_world_anchor_destroy);
	REQUIRE(empty);
	CHECK_FALSE(rift_s_world_anchor_has_map(empty.get()));
	unlink(path.c_str());
	rmdir(directory);
}

TEST_CASE("A changed boundary cannot reuse an older physical world's map")
{
	auto c = calibration(false);
	char directory[] = "/tmp/rift-s-boundary-XXXXXX";
	REQUIRE(mkdtemp(directory));
	std::string path = std::string(directory) + "/map", boundary = std::string(directory) + "/boundary";
	const char *original = R"({"jsonid":"chaperone_info","universes":[{"universeID":"2",
	    "standing":{"yaw":0.3,"translation":[0.5,0.1,-0.2]},"play_area":[2,3],
	    "collision_bounds":[[[0,0,0],[0,2,0],[1,2,0],[1,0,0]]],"time":"before"}]})";
	std::ofstream(boundary) << original;
	c.require_boundary = true;
	std::snprintf(c.boundary_path, sizeof(c.boundary_path), "%s", boundary.c_str());
	Images images(c);
	Anchor anchor(rift_s_world_anchor_create(&c, path.c_str()), rift_s_world_anchor_destroy);
	REQUIRE(anchor);
	rift_s_world_anchor_result result{};
	CHECK(rift_s_world_anchor_has_boundary(anchor.get()));
	xrt_pose trusted{{0, 0, 0, 1}, {0, 0, 0}};
	REQUIRE(rift_s_world_anchor_process(anchor.get(), &images.frames[0], &images.frames[1], &trusted, &result));
	REQUIRE(rift_s_world_anchor_save(anchor.get()));
	REQUIRE(rift_s_world_anchor_reload(anchor.get()));
	SECTION("SteamVR metadata and JSON formatting do not change the room frame") {
		std::ofstream(boundary) << R"({ "universes": [{"time":"after", "play_area":[2.0,3.0],
		    "standing":{"translation":[0.5,0.1,-0.2],"yaw":0.3},"universeID":"2",
		    "collision_bounds":[[[0.0,0,0],[0,2.0,0],[1.0,2,0],[1,0,0]]]}],
		    "jsonid":"chaperone_info" })";
		REQUIRE(rift_s_world_anchor_reload(anchor.get()));
		CHECK(rift_s_world_anchor_has_map(anchor.get()));
		REQUIRE(rift_s_world_anchor_save(anchor.get()));
	}
	SECTION("A changed standing yaw cannot silently rebind a live map") {
		std::string changed = original;
		changed.replace(changed.find("0.3"), 3, "0.4");
		std::ofstream(boundary) << changed;
		CHECK_FALSE(rift_s_world_anchor_save(anchor.get()));
		CHECK_FALSE(rift_s_world_anchor_reload(anchor.get()));
	}
	SECTION("A changed play area cannot reuse the saved map") {
		std::string changed = original;
		changed.replace(changed.find("[2,3]"), 5, "[2,4]");
		std::ofstream(boundary) << changed;
		CHECK_FALSE(rift_s_world_anchor_save(anchor.get()));
		CHECK_FALSE(rift_s_world_anchor_reload(anchor.get()));
	}
	SECTION("Changed collision bounds cannot reuse the saved map") {
		std::string changed = original;
		changed.replace(changed.find("[1,2,0]"), 7, "[1.5,2,0]");
		std::ofstream(boundary) << changed;
		CHECK_FALSE(rift_s_world_anchor_save(anchor.get()));
		CHECK_FALSE(rift_s_world_anchor_reload(anchor.get()));
	}
	// Accept wrote the new boundary but saving its new map failed. The old map remains on disk.
	std::string changed = original;
	changed.replace(changed.find("[0.5,0.1,-0.2]"), 14, "[0.6,0.1,-0.2]");
	std::ofstream(boundary) << changed;
	CHECK_FALSE(rift_s_world_anchor_reload(anchor.get()));
	CHECK_FALSE(rift_s_world_anchor_has_map(anchor.get()));
	CHECK(rift_s_world_anchor_has_boundary(anchor.get()));
	unlink(boundary.c_str());
	CHECK_FALSE(rift_s_world_anchor_has_boundary(anchor.get()));
	CHECK_FALSE(rift_s_world_anchor_save(anchor.get()));
	unlink(path.c_str());
	rmdir(directory);
}

TEST_CASE("Legacy maps migrate only while their exact saved boundary still matches")
{
	for (uint32_t version : {2u, 3u}) {
		CAPTURE(version);
		auto c = calibration(false);
		char directory[] = "/tmp/rift-s-legacy-XXXXXX";
		REQUIRE(mkdtemp(directory));
		std::string path = std::string(directory) + "/map", boundary = std::string(directory) + "/boundary";
		const std::string original = R"({"jsonid":"chaperone_info","universes":[{"universeID":"2",
		    "standing":{"yaw":0.3,"translation":[0.5,0.1,-0.2]},"play_area":[2,3],
		    "collision_bounds":[[[0,0,0],[0,2,0],[1,2,0],[1,0,0]]],"time":"before"}]})";
		std::ofstream(boundary) << original;
		c.require_boundary = true;
		std::snprintf(c.boundary_path, sizeof(c.boundary_path), "%s", boundary.c_str());
		Images images(c);
		Anchor anchor(rift_s_world_anchor_create(&c, path.c_str()), rift_s_world_anchor_destroy);
		xrt_pose trusted = XRT_POSE_IDENTITY;
		rift_s_world_anchor_result result{};
		REQUIRE(rift_s_world_anchor_process(anchor.get(), &images.frames[0], &images.frames[1], &trusted, &result));
		REQUIRE(rift_s_world_anchor_save(anchor.get()));
		std::ifstream input(path, std::ios::binary);
		std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(input)), {});
		auto read32 = [&](size_t offset) {
			REQUIRE(offset + 4 <= bytes.size());
			return uint32_t(bytes[offset]) | uint32_t(bytes[offset + 1]) << 8 |
			       uint32_t(bytes[offset + 2]) << 16 | uint32_t(bytes[offset + 3]) << 24;
		};
		if (version == 2) {
			std::vector<uint8_t> legacy(bytes.begin(), bytes.begin() + 28);
			size_t offset = 28;
			uint32_t count = read32(24);
			for (unsigned frame = 0; frame < count; ++frame) {
				uint32_t points = read32(offset + 28);
				legacy.insert(legacy.end(), bytes.begin() + offset, bytes.begin() + offset + 32);
				offset += 32;
				for (unsigned point = 0; point < points; ++point, offset += 80)
					legacy.insert(legacy.end(), bytes.begin() + offset, bytes.begin() + offset + 76);
			}
			REQUIRE(offset + 8 == bytes.size());
			legacy.resize(legacy.size() + 8);
			bytes = std::move(legacy);
		}
		auto fnv = [](const uint8_t *data, size_t size) {
			uint64_t h = 14695981039346656037ULL;
			for (size_t i = 0; i < size; ++i)
				h = (h ^ data[i]) * 1099511628211ULL;
			return h;
		};
		auto put64 = [&](size_t offset, uint64_t value) {
			for (unsigned i = 0; i < 8; ++i)
				bytes[offset + i] = uint8_t(value >> (8 * i));
		};
		bytes[4] = uint8_t(version);
		put64(16, fnv(reinterpret_cast<const uint8_t *>(original.data()), original.size()));
		put64(bytes.size() - 8, fnv(bytes.data(), bytes.size() - 8));
		std::ofstream file(path, std::ios::binary | std::ios::trunc);
		file.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
		file.close();
		REQUIRE(rift_s_world_anchor_reload(anchor.get()));
		CHECK(rift_s_world_anchor_has_map(anchor.get()));
		std::string rewritten = original;
		rewritten.replace(rewritten.find("before"), 6, "after");
		std::ofstream(boundary) << rewritten;
		CHECK_FALSE(rift_s_world_anchor_reload(anchor.get()));
		CHECK_FALSE(rift_s_world_anchor_save(anchor.get()));
		std::ifstream retained(path, std::ios::binary);
		std::vector<uint8_t> unchanged((std::istreambuf_iterator<char>(retained)), {});
		CHECK(unchanged == bytes);
		std::ofstream(boundary) << original;
		REQUIRE(rift_s_world_anchor_reload(anchor.get()));
		REQUIRE(rift_s_world_anchor_save(anchor.get()));
		std::ifstream upgraded(path, std::ios::binary);
		upgraded.seekg(4);
		CHECK(upgraded.get() == 4);
		std::ofstream(boundary) << rewritten;
		REQUIRE(rift_s_world_anchor_reload(anchor.get()));
		CHECK(rift_s_world_anchor_has_map(anchor.get()));
		unlink(path.c_str());
		unlink(boundary.c_str());
		rmdir(directory);
	}
}

TEST_CASE("Private installed calibration loads its saved room map", "[.world-map-check]")
{
	const char *config_path = std::getenv("RIFT_S_WORLD_CHECK_CONFIG");
	const char *map_path = std::getenv("RIFT_S_WORLD_CHECK_MAP");
	REQUIRE(config_path);
	REQUIRE(map_path);
	rift_s_world_anchor_config config{};
	std::ifstream input(config_path, std::ios::binary);
	REQUIRE(input.read(reinterpret_cast<char *>(&config), sizeof(config)));
	Anchor anchor(rift_s_world_anchor_create(&config, map_path), rift_s_world_anchor_destroy);
	REQUIRE(anchor);
	CHECK(rift_s_world_anchor_has_boundary(anchor.get()));
	CHECK(rift_s_world_anchor_has_map(anchor.get()));
}

// Real camera replay stays opt-in. Recordings contain private room images and never enter the repo.
TEST_CASE("Cross-session camera map replay", "[.camera-replay]")
{
	const char *session_a = std::getenv("RIFT_S_WORLD_REPLAY_A"), *session_b = std::getenv("RIFT_S_WORLD_REPLAY_B");
	REQUIRE(session_a);
	REQUIRE(session_b);
	rift_s_world_anchor_config c{};
	std::ifstream config(std::string(session_a) + "/calibration.bin", std::ios::binary);
	config.seekg(0, std::ios::end);
	const auto bytes = config.tellg();
	config.seekg(0);
	struct LegacyConfig
	{
		rift_s_camera_calibration camera[2];
		xrt_pose imu_from_camera[2];
		bool require_boundary;
		char boundary_path[1024];
	};
	if (bytes == std::streampos(sizeof(LegacyConfig))) {
		LegacyConfig old{};
		REQUIRE(config.read(reinterpret_cast<char *>(&old), sizeof(old)));
		std::copy_n(old.camera, 2, c.camera);
		std::copy_n(old.imu_from_camera, 2, c.imu_from_camera);
		c.camera_count = 2;
	} else {
		REQUIRE(bytes == std::streampos(sizeof(c)));
		REQUIRE(config.read(reinterpret_cast<char *>(&c), sizeof(c)));
	}
	const unsigned camera_count = c.camera_count ? c.camera_count : 2;
	const char *boundary = std::getenv("RIFT_S_WORLD_REPLAY_BOUNDARY");
	c.require_boundary = boundary != nullptr;
	c.boundary_path[0] = 0;
	if (boundary) {
		REQUIRE(std::strlen(boundary) < sizeof(c.boundary_path));
		std::snprintf(c.boundary_path, sizeof(c.boundary_path), "%s", boundary);
	}
	const char *override_path = std::getenv("RIFT_S_WORLD_REPLAY_MAP");
	std::string path = override_path ? override_path : std::string(session_a) + "/replay-map.bin";
	bool load_only = std::getenv("RIFT_S_WORLD_REPLAY_LOAD_MAP") != nullptr;
	// A caller-selected path is never overwritten unless explicitly requested.
	if (!load_only) {
		REQUIRE((!override_path || access(path.c_str(), F_OK) != 0 ||
		         std::getenv("RIFT_S_WORLD_REPLAY_RESET_MAP")));
		unlink(path.c_str());
	}
	Anchor map(rift_s_world_anchor_create(&c, path.c_str()), rift_s_world_anchor_destroy);
	REQUIRE(map);
	auto read_frame = [](const std::string &dir, unsigned index, unsigned eye, int64_t ts,
	                     std::vector<uint8_t> &storage, xrt_frame &frame) {
		char name[64];
		std::snprintf(name, sizeof(name), "/%04u-%u.pgm", index, eye);
		std::ifstream file(dir + name, std::ios::binary);
		std::string magic;
		unsigned width, height, maximum;
		REQUIRE(file >> magic >> width >> height >> maximum);
		REQUIRE(magic == "P5");
		REQUIRE(maximum == 255);
		REQUIRE(width <= 2048);
		REQUIRE(height <= 2048);
		file.get();
		storage.resize(width * height);
		REQUIRE(file.read(reinterpret_cast<char *>(storage.data()), storage.size()));
		frame = {};
		frame.width = width;
		frame.height = height;
		frame.stride = width;
		frame.size = storage.size();
		frame.data = storage.data();
		frame.format = XRT_FORMAT_L8;
		frame.timestamp = ts;
	};
	unsigned keyframes = 0, matched = 0, attempted = 0, confirmations = 0;
	int64_t start_ns = 0, confirmed_ns = 0, previous_ns = 0;
	xrt_pose previous = XRT_POSE_IDENTITY, confirmed = XRT_POSE_IDENTITY, confirmed_head = XRT_POSE_IDENTITY;
	double max_error = 0, max_vote_translation = 0, max_vote_angle = 0;
	for (const auto &[dir, record] :
	     {std::pair{std::string(session_a), true}, std::pair{std::string(session_b), false}}) {
		if (record && load_only) {
			REQUIRE(rift_s_world_anchor_has_map(map.get()));
			continue;
		}
		std::ifstream poses(dir + "/poses.csv");
		REQUIRE(poses.good());
		std::string line;
		if (!record) {
			if (!load_only)
				REQUIRE(rift_s_world_anchor_save(map.get()));
			if (std::getenv("RIFT_S_WORLD_REPLAY_RECORD_ONLY")) {
				REQUIRE(keyframes > 0);
				std::printf("Recorded map keyframes=%u\n", keyframes);
				return;
			}
			map.reset();
			map = Anchor(rift_s_world_anchor_create(&c, path.c_str()), rift_s_world_anchor_destroy);
			REQUIRE(rift_s_world_anchor_has_map(map.get()));
		}
		while (std::getline(poses, line)) {
			unsigned index;
			long long ts;
			int valid;
			xrt_pose raw;
			REQUIRE(std::sscanf(line.c_str(), "%u,%lld,%d,%f,%f,%f,%f,%f,%f,%f", &index, &ts, &valid,
			                    &raw.orientation.x, &raw.orientation.y, &raw.orientation.z,
			                    &raw.orientation.w, &raw.position.x, &raw.position.y,
			                    &raw.position.z) == 10);
			if (!record && index >= 100 && std::getenv("RIFT_S_WORLD_REPLAY_SHORT"))
				break;
			if (!record && !start_ns)
				start_ns = ts;
			if (record && !valid)
				continue;
			std::vector<uint8_t> storage[RIFT_S_WORLD_MAX_CAMERAS];
			xrt_frame frames[RIFT_S_WORLD_MAX_CAMERAS];
			const xrt_frame *ptr[RIFT_S_WORLD_MAX_CAMERAS]{};
			for (unsigned eye = 0; eye < camera_count; ++eye) {
				read_frame(dir, index, eye, ts, storage[eye], frames[eye]);
				ptr[eye] = &frames[eye];
			}
			rift_s_world_anchor_result result{};
			bool processed = rift_s_world_anchor_process_rig_gravity(
			    map.get(), ptr, camera_count, record ? &raw : nullptr,
			    valid ? &raw.orientation : nullptr, &result);
			if (record) {
				keyframes = std::max(keyframes, result.keyframes);
				if (index % 12 == 0)
					std::printf(
					    "Record index=%u features=%u/%u/%u/%u/%u points_by_camera=%u/%u/%u/%u/%u "
					    "stereo=%u points=%u keyframes=%u\n",
					    index, result.features[0], result.features[1], result.features[2],
					    result.features[3], result.features[4], result.camera_points[0],
					    result.camera_points[1], result.camera_points[2], result.camera_points[3],
					    result.camera_points[4], result.stereo_matches, result.triangulated,
					    result.keyframes);
				continue;
			}
			attempted++;
			if (index % 12 == 0)
				std::printf(
				    "Attempt index=%u features=%u/%u/%u/%u/%u matches=%u/%u/%u/%u/%u "
				    "inliers=%u/%u/%u/%u/%u support=%u error_px=%.3f "
				    "matched=%d rejection=%s\n",
				    index, result.features[0], result.features[1], result.features[2],
				    result.features[3], result.features[4], result.matches[0], result.matches[1],
				    result.matches[2], result.matches[3], result.matches[4], result.inliers[0],
				    result.inliers[1], result.inliers[2], result.inliers[3], result.inliers[4],
				    result.supporting_cameras, result.max_reprojection_px, result.relocalized,
				    result.rejection ? result.rejection : "none");
			if (!processed || !result.relocalized || !valid) {
				if (!valid || ts - previous_ns > 3000000000LL)
					confirmations = 0;
				continue;
			}
			matched++;
			xrt_pose inverse, candidate;
			math_pose_invert(&raw, &inverse);
			math_pose_transform(&result.world_from_imu, &inverse, &candidate);
			xrt_vec3 up{0, 1, 0}, world_up;
			math_quat_rotate_vec3(&candidate.orientation, &up, &world_up);
			if (world_up.y <= 0.9961947f) {
				confirmations = 0;
				continue;
			}
			rift_s_slam_guard_yaw_of(&candidate.orientation, &candidate.orientation);
			xrt_vec3 rotated;
			math_quat_rotate_vec3(&candidate.orientation, &raw.position, &rotated);
			candidate.position = {result.world_from_imu.position.x - rotated.x,
			                      result.world_from_imu.position.y - rotated.y,
			                      result.world_from_imu.position.z - rotated.z};
			double distance = cv::norm(translation(candidate) - translation(previous));
			cv::Vec3d rv;
			cv::Rodrigues(rotation(candidate.orientation).t() * rotation(previous.orientation), rv);
			double angle = cv::norm(rv);
			bool agrees = confirmations && ts > previous_ns && ts - previous_ns <= 3000000000LL &&
			              distance < 0.04 && angle < 3 * M_PI / 180;
			if (agrees) {
				max_vote_translation = std::max(max_vote_translation, distance);
				max_vote_angle = std::max(max_vote_angle, angle);
			}
			confirmations = agrees ? confirmations + 1 : 1;
			previous = candidate;
			previous_ns = ts;
			max_error = std::max(max_error, double(result.max_reprojection_px));
			if (confirmations >= 3 && !confirmed_ns) {
				confirmed_ns = ts;
				confirmed = candidate;
				confirmed_head = result.world_from_imu;
			}
			std::printf(
			    "Replay time_s=%.3f inliers=%u/%u error_px=%.3f votes=%u transform=%.4f,%.4f,%.4f\n",
			    (ts - start_ns) * 1e-9, result.inliers[0], result.inliers[1], result.max_reprojection_px,
			    confirmations, candidate.position.x, candidate.position.y, candidate.position.z);
		}
	}
	std::printf(
	    "Replay summary keyframes=%u attempts=%u matched=%u confirmed_s=%.3f max_error_px=%.3f"
	    " vote_translation_m=%.6f vote_angle_deg=%.6f transform=%.4f,%.4f,%.4f"
	    " head_position=%.6f,%.6f,%.6f head_orientation=%.6f,%.6f,%.6f,%.6f\n",
	    keyframes, attempted, matched, confirmed_ns ? (confirmed_ns - start_ns) * 1e-9 : -1.0, max_error,
	    max_vote_translation, max_vote_angle * 180 / M_PI, confirmed.position.x, confirmed.position.y,
	    confirmed.position.z, confirmed_head.position.x, confirmed_head.position.y, confirmed_head.position.z,
	    confirmed_head.orientation.x, confirmed_head.orientation.y, confirmed_head.orientation.z,
	    confirmed_head.orientation.w);
	if (!load_only)
		REQUIRE(keyframes > 0);
	REQUIRE(confirmed_ns > 0);
}

TEST_CASE("Native fisheye recovery tolerates subpixel peripheral feature noise")
{
	auto c = calibration();
	xrt_pose expected{{0, 0.24740396f, 0, 0.96891242f}, {0.2f, 0.1f, -0.45f}};
	auto noisy = observations(c, expected);
	for (unsigned eye = 0; eye < 2; ++eye) {
		for (unsigned i = 0; i < noisy[eye].size(); ++i) {
			// Corner localization noise is measured in native pixels, not the nonlinear
			// perspective plane used internally by OpenCV's PnP implementation.
			noisy[eye][i].pixel.x += float(0.6 * std::sin(i * 1.7));
			noisy[eye][i].pixel.y += float(0.6 * std::cos(i * 2.3));
		}
	}
	rift_s_world_anchor_result result{};
	REQUIRE(solve(c, noisy, result));
	CHECK(cv::norm(translation(result.world_from_imu) - translation(expected)) < 0.025);
	CHECK(result.inliers[0] >= 60);
	CHECK(result.inliers[1] >= 60);
}

TEST_CASE("Native reprojection drops a few wrong correspondences without losing the room")
{
	auto c = calibration();
	xrt_pose expected{{0, 0.24740396f, 0, 0.96891242f}, {0.2f, 0.1f, -0.45f}};
	auto obs = observations(c, expected);
	for (auto &eye : obs)
		for (unsigned i = 0; i < eye.size(); i += 13)
			eye[i].pixel.x += 4;
	rift_s_world_anchor_result result{};
	REQUIRE(solve(c, obs, result));
	CHECK(result.max_reprojection_px <= 2.5f);
	CHECK(cv::norm(translation(result.world_from_imu) - translation(expected)) < 0.025);
	CHECK(result.inliers[0] >= 60);
	CHECK(result.inliers[1] >= 60);
}

TEST_CASE("Stereo map rejects unrelated camera images")
{
	auto c = calibration(false);
	Images images(c);
	cv::RNG random(251);
	random.fill(images.images[1], cv::RNG::UNIFORM, 0, 256);
	Anchor anchor(rift_s_world_anchor_create(&c, nullptr), rift_s_world_anchor_destroy);
	xrt_pose trusted = XRT_POSE_IDENTITY;
	rift_s_world_anchor_result result{};
	CHECK_FALSE(rift_s_world_anchor_process(anchor.get(), &images.frames[0], &images.frames[1], &trusted, &result));
	CHECK_FALSE(rift_s_world_anchor_has_map(anchor.get()));
}

TEST_CASE("Five camera recovery accepts side support and rejects conflicting room clusters")
{
	auto c = calibration(false);
	c.camera_count = 5;
	for (unsigned e = 2; e < 5; ++e) {
		c.camera[e] = c.camera[0];
		c.imu_from_camera[e] = XRT_POSE_IDENTITY;
	}
	c.imu_from_camera[2] = {{0, -0.70710678f, 0, 0.70710678f}, {-0.07f, 0, 0}};
	c.imu_from_camera[3] = {{0, 0.70710678f, 0, 0.70710678f}, {0.07f, 0, 0}};
	c.imu_from_camera[4] = {{-0.70710678f, 0, 0, 0.70710678f}, {0, -0.04f, 0}};
	xrt_pose expected{{0, 0.24740396f, 0, 0.96891242f}, {1.2f, 1.3f, -0.45f}};
	std::vector<rift_s_world_anchor_observation> o[5];
	const rift_s_world_anchor_observation *ptr[5]{};
	uint32_t n[5]{};
	for (unsigned e = 0; e < 5; ++e) {
		for (int y = 50; y <= 430; y += 45)
			for (int x = 55; x <= 590; x += 55) {
				xrt_vec2 pixel{float(x), float(y)};
				xrt_vec3 ray;
				REQUIRE(rift_s_world_anchor_unproject(&c.camera[e], &pixel, &ray));
				double depth = 1.2 + 0.15 * ((x / 55 + 2 * y / 45) % 9);
				auto imu = rotation(c.imu_from_camera[e].orientation) *
				               (cv::Vec3d{ray.x, ray.y, ray.z} * depth) +
				           translation(c.imu_from_camera[e]);
				auto world = rotation(expected.orientation) * imu + translation(expected);
				o[e].push_back({{float(world[0]), float(world[1]), float(world[2])}, pixel});
			}
		ptr[e] = o[e].data();
		n[e] = o[e].size();
	}
	rift_s_world_anchor_result r{};
	SECTION("Side cameras recover with front and top cameras unavailable")
	{
		n[0] = n[1] = n[4] = 0;
		REQUIRE(rift_s_world_anchor_solve_rig(&c, ptr, n, 5, &r));
		CHECK(r.supporting_cameras == 2);
		CHECK(cv::norm(translation(expected) - translation(r.world_from_imu)) < 0.00005);
	}
	SECTION("One unrelated confident camera cannot overturn four agreeing cameras")
	{
		for (auto &obs : o[4])
			obs.world_point.x += 0.4f;
		REQUIRE(rift_s_world_anchor_solve_rig(&c, ptr, n, 5, &r));
		CHECK(r.supporting_cameras == 4);
		CHECK(cv::norm(translation(expected) - translation(r.world_from_imu)) < 0.00005);
	}
	SECTION("Two conflicting confident clusters cannot confirm a boundary")
	{
		for (unsigned e : {2u, 3u})
			for (auto &obs : o[e])
				obs.world_point.x += 0.4f;
		CHECK_FALSE(rift_s_world_anchor_solve_rig(&c, ptr, n, 5, &r));
		CHECK(std::string(r.rejection) == "camera_disagreement");
	}
}

TEST_CASE("Temporal room mapping supplies landmarks without camera baseline")
{
	auto c = calibration(false);
	c.camera_count = 5;
	for (unsigned e = 0; e < 5; ++e) {
		c.camera[e] = c.camera[0];
		c.imu_from_camera[e] = XRT_POSE_IDENTITY;
	}
	Images initial(c), moved(c, 0.12f, 0.15f);
	Anchor anchor(rift_s_world_anchor_create(&c, "/tmp/rift-s-temporal-unsaved.bin"), rift_s_world_anchor_destroy);
	REQUIRE(anchor);
	const xrt_frame *ptr[5];
	for (unsigned e = 0; e < 5; ++e)
		ptr[e] = &initial.frames[e];
	xrt_pose trusted = XRT_POSE_IDENTITY;
	rift_s_world_anchor_result r{};
	CHECK_FALSE(rift_s_world_anchor_process_rig(anchor.get(), ptr, 5, &trusted, &r));
	trusted.position.x = 0.12f;
	trusted.orientation = {0, float(std::sin(0.075)), 0, float(std::cos(0.075))};
	for (unsigned e = 0; e < 5; ++e) {
		moved.frames[e].timestamp += 300000000LL;
		ptr[e] = &moved.frames[e];
	}
	REQUIRE(rift_s_world_anchor_process_rig(anchor.get(), ptr, 5, &trusted, &r));
	CHECK(r.recorded);
	for (unsigned e = 0; e < 5; ++e)
		CHECK(r.camera_points[e] >= 40);
	REQUIRE(rift_s_world_anchor_process_rig(anchor.get(), ptr, 5, nullptr, &r));
	CHECK(r.supporting_cameras == 5);
	CHECK(cv::norm(translation(trusted) - translation(r.world_from_imu)) < 0.03);
}

TEST_CASE("Room recovery solves yaw and translation with exposure gravity")
{
	auto c = calibration();
	xrt_quat tilt{0.14943813f, 0, 0, 0.98877108f};
	xrt_quat yaw{0, 0.24740396f, 0, 0.96891242f};
	xrt_pose expected{tilt, {0.4f, -0.2f, 0.6f}};
	math_quat_rotate(&yaw, &tilt, &expected.orientation);
	auto o = observations(c, expected);
	for (unsigned eye = 0; eye < 2; ++eye)
		for (size_t i = 0; i < o[eye].size(); ++i) {
			if (i % 11 == 0)
				o[eye][i].pixel.x += 60;
			else {
				o[eye][i].pixel.x += float(0.25 * std::sin(double(i)));
				o[eye][i].pixel.y += float(0.25 * std::cos(double(i)));
			}
		}
	const rift_s_world_anchor_observation *obs[2] = {o[0].data(), o[1].data()};
	uint32_t counts[2] = {uint32_t(o[0].size()), uint32_t(o[1].size())};
	rift_s_world_anchor_result result{};
	REQUIRE(rift_s_world_anchor_solve_rig_gravity(&c, obs, counts, 2, &tilt, &result));
	CHECK(cv::norm(translation(result.world_from_imu) - translation(expected)) < 0.01);
	xrt_quat inverse, alignment;
	math_quat_invert(&tilt, &inverse);
	math_quat_rotate(&result.world_from_imu.orientation, &inverse, &alignment);
	CHECK(std::abs(alignment.x) < 1e-5);
	CHECK(std::abs(alignment.z) < 1e-5);
	CHECK(std::abs(std::abs(alignment.y) - yaw.y) < 0.002);
	SECTION("Invalid gravity cannot produce a localized pose") {
		tilt.x = std::numeric_limits<float>::quiet_NaN();
		CHECK_FALSE(rift_s_world_anchor_solve_rig_gravity(&c, obs, counts, 2, &tilt, &result));
		CHECK_FALSE(result.relocalized);
	}
}

TEST_CASE("Automatic room updates reject VIO drift in an overlapping view")
{
	auto c = calibration(false);
	Images images(c);
	Anchor map(rift_s_world_anchor_create(&c, nullptr), rift_s_world_anchor_destroy);
	REQUIRE(map);
	const xrt_frame *frames[2] = {&images.frames[0], &images.frames[1]};
	xrt_pose trusted = XRT_POSE_IDENTITY;
	rift_s_world_anchor_result result{};
	REQUIRE(rift_s_world_anchor_process_rig(map.get(), frames, 2, &trusted, &result));
	// The camera stayed still; a plausible but drifting VIO pose must not poison its map.
	trusted.position.x = 0.3f;
	for (unsigned eye = 0; eye < 2; ++eye)
		images.frames[eye].timestamp += 3000000000LL;
	CHECK_FALSE(rift_s_world_anchor_process_rig_gravity(map.get(), frames, 2, &trusted,
	                                                   &trusted.orientation, &result));
	CHECK(std::string(result.rejection) == "map_update_disagreement");
	CHECK(result.keyframes == 1);
	REQUIRE(rift_s_world_anchor_process_rig_gravity(map.get(), frames, 2, nullptr,
	                                               &trusted.orientation, &result));
	CHECK(cv::norm(translation(result.world_from_imu)) < 0.03);
}
