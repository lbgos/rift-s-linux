// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "rift_s_world_anchor.h"
#include "util/u_json.h"
#include "util/u_logging.h"
#include <cstdlib>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/flann.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <string>
#include <vector>
#include <memory>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {
constexpr uint32_t MAX_KEYFRAMES = 64, MAX_POINTS = 1000, MIN_POINTS = 40;
constexpr double MAX_ERROR_PX = 2.5;
struct Transform
{
	cv::Matx33d r = cv::Matx33d::eye();
	cv::Vec3d t = {0, 0, 0};
	cv::Vec3d
	apply(const cv::Vec3d &p) const
	{
		return r * p + t;
	}
	Transform
	inverse() const
	{
		return {r.t(), -(r.t() * t)};
	}
	Transform
	operator*(const Transform &b) const
	{
		return {r * b.r, r * b.t + t};
	}
};
bool
finite_pose(const xrt_pose &p)
{
	const auto &q = p.orientation;
	double n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
	return std::isfinite(n) && std::abs(n - 1) < 0.01 && std::isfinite(p.position.x) &&
	       std::isfinite(p.position.y) && std::isfinite(p.position.z);
}
Transform
transform(const xrt_pose &p)
{
	const auto &q = p.orientation;
	double x = q.x, y = q.y, z = q.z, w = q.w;
	return {{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 2 * (x * y + z * w),
	         1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 2 * (x * z - y * w), 2 * (y * z + x * w),
	         1 - 2 * (x * x + y * y)},
	        {p.position.x, p.position.y, p.position.z}};
}
xrt_pose
pose(const Transform &t)
{
	cv::Vec3d rv;
	cv::Rodrigues(t.r, rv);
	double angle = cv::norm(rv), scale = angle > 1e-12 ? std::sin(angle / 2) / angle : 0.5;
	return {{float(rv[0] * scale), float(rv[1] * scale), float(rv[2] * scale), float(std::cos(angle / 2))},
	        {float(t.t[0]), float(t.t[1]), float(t.t[2])}};
}
double
angle_between(const Transform &a, const Transform &b)
{
	cv::Vec3d rv;
	cv::Rodrigues(a.r.t() * b.r, rv);
	return cv::norm(rv);
}
unsigned
camera_count(const rift_s_world_anchor_config &c)
{
	return c.camera_count ? c.camera_count : 2;
}
bool
calibration_valid(const rift_s_world_anchor_config &c)
{
	if (camera_count(c) < 2 || camera_count(c) > RIFT_S_WORLD_MAX_CAMERAS)
		return false;
	uint64_t pixels = 0;
	for (unsigned eye = 0; eye < camera_count(c); ++eye) {
		const auto &cam = c.camera[eye];
		if (cam.roi.extent.w < 64 || cam.roi.extent.w > 2048 || cam.roi.extent.h < 64 ||
		    cam.roi.extent.h > 2048 || !finite_pose(c.imu_from_camera[eye]) ||
		    !std::isfinite(cam.projection.fx) || !std::isfinite(cam.projection.fy) || cam.projection.fx < 10 ||
		    cam.projection.fy < 10 || !std::isfinite(cam.projection.cx) || !std::isfinite(cam.projection.cy) ||
		    !std::isfinite(cam.distortion.p1) || !std::isfinite(cam.distortion.p2))
			return false;
		for (float k : cam.distortion.k)
			if (!std::isfinite(k))
				return false;
		pixels += uint64_t(cam.roi.extent.w) * uint64_t(cam.roi.extent.h);
	}
	// Bound retained ray/warp caches to the native five-camera crop budget.
	if (pixels > uint64_t(RIFT_S_WORLD_MAX_CAMERAS) * 640 * 480)
		return false;
	double baseline = cv::norm(transform(c.imu_from_camera[0]).t - transform(c.imu_from_camera[1]).t);
	return camera_count(c) > 2 || (baseline > 0.02 && baseline < 0.5);
}
cv::Vec2d
distort(const rift_s_camera_calibration &c, cv::Vec2d angular)
{
	double t2 = angular.dot(angular), radial = c.distortion.k[5];
	for (int i = 4; i >= 0; --i)
		radial = radial * t2 + c.distortion.k[i];
	angular *= radial * t2 + 1;
	const cv::Vec2d tangent{c.distortion.p1, c.distortion.p2};
	return angular * (1 + 2 * angular.dot(tangent)) + tangent * angular.dot(angular);
}
bool
project(const rift_s_camera_calibration &c, cv::Vec3d p, cv::Point2d &out)
{
	if (p[2] <= 0.01 || !std::isfinite(cv::norm(p)))
		return false;
	double r = std::hypot(p[0], p[1]), scale = r > 1e-12 ? std::atan2(r, p[2]) / r : 1 / p[2];
	auto pixel = distort(c, {p[0] * scale, p[1] * scale});
	out = {c.projection.fx * pixel[0] + c.projection.cx, c.projection.fy * pixel[1] + c.projection.cy};
	return std::isfinite(out.x) && std::isfinite(out.y);
}
bool
unproject(const rift_s_camera_calibration &c, cv::Point2d pixel, cv::Vec3d &ray)
{
	if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y) || c.projection.fx <= 0 || c.projection.fy <= 0)
		return false;
	cv::Vec2d target{(pixel.x - c.projection.cx) / c.projection.fx, (pixel.y - c.projection.cy) / c.projection.fy};
	// Invert the radial+tangential polynomial on the angular plane, then tan(theta).
	cv::Vec2d uv = target;
	if (cv::norm(uv) > 1.2)
		uv *= 1.2 / cv::norm(uv);
	for (int i = 0; i < 30; ++i) {
		cv::Vec2d residual = distort(c, uv) - target;
		if (cv::norm(residual) < 1e-10)
			break;
		constexpr double eps = 1e-6;
		cv::Vec2d dx = (distort(c, uv + cv::Vec2d{eps, 0}) - distort(c, uv - cv::Vec2d{eps, 0})) / (2 * eps);
		cv::Vec2d dy = (distort(c, uv + cv::Vec2d{0, eps}) - distort(c, uv - cv::Vec2d{0, eps})) / (2 * eps);
		cv::Matx22d j{dx[0], dy[0], dx[1], dy[1]};
		if (std::abs(cv::determinant(j)) < 1e-8)
			return false;
		cv::Vec2d step = j.inv() * residual;
		if (cv::norm(step) > 0.25)
			step *= 0.25 / cv::norm(step);
		uv -= step;
		if (!std::isfinite(cv::norm(uv)))
			return false;
	}
	double theta = cv::norm(uv);
	if (theta > 1.55 || cv::norm(distort(c, uv) - target) > 1e-6)
		return false;
	double s = theta > 1e-12 ? std::sin(theta) / theta : 1;
	ray = {uv[0] * s, uv[1] * s, std::cos(theta)};
	cv::Point2d check;
	return project(c, ray, check) && cv::norm(check - pixel) < 0.001;
}
bool
spread(const std::vector<cv::Point2d> &pixels, const rift_s_camera_calibration &c)
{
	unsigned bins = 0;
	double minx = 1e9, miny = 1e9, maxx = -1e9, maxy = -1e9;
	for (auto p : pixels) {
		int x = std::clamp(int(4 * p.x / c.roi.extent.w), 0, 3),
		    y = std::clamp(int(3 * p.y / c.roi.extent.h), 0, 2);
		bins |= 1u << (4 * y + x);
		minx = std::min(minx, p.x);
		maxx = std::max(maxx, p.x);
		miny = std::min(miny, p.y);
		maxy = std::max(maxy, p.y);
	}
	unsigned occupied = 0;
	for (; bins; bins >>= 1)
		occupied += bins & 1;
	return occupied >= 4 && maxx - minx > 0.25 * c.roi.extent.w && maxy - miny > 0.2 * c.roi.extent.h;
}

// PnP supplies the robust seed. All subsequent fitting is on the four parameters
// allowed by gravity, using the calibrated fisheye residual rather than perspective pixels.
bool
refine_gravity(const rift_s_world_anchor_config &c,
               unsigned camera,
               const xrt_quat &attitude,
               const std::vector<cv::Point3d> &world,
               const std::vector<cv::Point2d> &pixels,
               Transform &cam_from_world)
{
	Transform raw = transform({attitude, {0, 0, 0}});
	Transform imu_from_camera = transform(c.imu_from_camera[camera]);
	Transform seed = cam_from_world.inverse() * imu_from_camera.inverse();
	cv::Matx33d delta = seed.r * raw.r.t();
	double yaw = std::atan2(delta(0, 2) - delta(2, 0), delta(0, 0) + delta(2, 2));
	cv::Vec4d parameters{yaw, seed.t[0], seed.t[1], seed.t[2]};
	auto camera_pose = [&](const cv::Vec4d &p) {
		double s = std::sin(p[0]), co = std::cos(p[0]);
		Transform imu{{co, 0, s, 0, 1, 0, -s, 0, co}, {p[1], p[2], p[3]}};
		imu.r = imu.r * raw.r;
		return (imu * imu_from_camera).inverse();
	};
	for (unsigned iteration = 0; iteration < 15; ++iteration) {
		cv::Matx44d normal = cv::Matx44d::zeros();
		cv::Vec4d gradient{0, 0, 0, 0};
		Transform current = camera_pose(parameters);
		unsigned used = 0;
		for (size_t i = 0; i < world.size(); ++i) {
			cv::Vec3d point{world[i].x, world[i].y, world[i].z};
			cv::Point2d projected;
			if (!project(c.camera[camera], current.apply(point), projected))
				continue;
			cv::Vec2d residual{projected.x - pixels[i].x, projected.y - pixels[i].y};
			cv::Matx<double, 2, 4> jacobian;
			bool finite = true;
			for (unsigned axis = 0; axis < 4; ++axis) {
				cv::Vec4d perturbed = parameters;
				perturbed[axis] += 1e-5;
				cv::Point2d next;
				if (!project(c.camera[camera], camera_pose(perturbed).apply(point), next)) {
					finite = false;
					break;
				}
				jacobian(0, axis) = (next.x - projected.x) / 1e-5;
				jacobian(1, axis) = (next.y - projected.y) / 1e-5;
			}
			if (!finite)
				continue;
			double weight = std::min(1.0, MAX_ERROR_PX / std::max(cv::norm(residual), 1e-9));
			normal += weight * (jacobian.t() * jacobian);
			gradient += weight * (jacobian.t() * residual);
			++used;
		}
		if (used < 30)
			return false;
		cv::Vec4d step;
		if (!cv::solve(normal, gradient, step, cv::DECOMP_CHOLESKY) || !std::isfinite(cv::norm(step)))
			return false;
		double scale = std::max({1.0, std::abs(step[0]) / 0.05,
		                         cv::norm(cv::Vec3d{step[1], step[2], step[3]}) / 0.2});
		parameters -= step / scale;
		if (cv::norm(step) < 1e-7)
			break;
	}
	cam_from_world = camera_pose(parameters);
	return std::isfinite(cv::norm(cam_from_world.t));
}
struct Point
{
	cv::Vec3d world;
	std::array<uint8_t, 32> descriptors[2];
	uint8_t cameras[2] = {0, 1};
};
struct Keyframe
{
	Transform world_from_imu;
	std::vector<Point> points;
	int64_t last_refresh_ns = 0;
};
struct Features
{
	std::vector<cv::KeyPoint> keys;
	cv::Mat descriptors;
};
struct StereoWarp
{
	cv::Mat x, y;
	bool initialized = false;
};
void
build_warp(StereoWarp &warp, const rift_s_world_anchor_config &c, cv::Mat &rays)
{
	warp.initialized = true;
	Transform cameras[2] = {transform(c.imu_from_camera[0]), transform(c.imu_from_camera[1])};
	if (angle_between(cameras[0], cameras[1]) < 3 * M_PI / 180)
		return;
	cv::Matx33d target_from_source = cameras[1].r.t() * cameras[0].r;
	if (rays.empty()) {
		rays.create(c.camera[0].roi.extent.h, c.camera[0].roi.extent.w, CV_64FC3);
		for (int y = 0; y < rays.rows; ++y)
			for (int x = 0; x < rays.cols; ++x) {
				cv::Vec3d ray{0, 0, 0};
				unproject(c.camera[0], {double(x), double(y)}, ray);
				rays.at<cv::Vec3d>(y, x) = ray;
			}
	}
	warp.x.create(c.camera[0].roi.extent.h, c.camera[0].roi.extent.w, CV_32F);
	warp.y.create(warp.x.size(), CV_32F);
	for (int y = 0; y < warp.x.rows; ++y)
		for (int x = 0; x < warp.x.cols; ++x) {
			cv::Point2d pixel;
			bool valid = project(c.camera[1], target_from_source * rays.at<cv::Vec3d>(y, x), pixel);
			warp.x.at<float>(y, x) = valid ? float(pixel.x) : -1;
			warp.y.at<float>(y, x) = valid ? float(pixel.y) : -1;
		}
}
// Match patches at subpixel precision before triangulating. Independent ORB detections
// discard most stereo correspondences in a room and leave noisy integer-pixel depths.
// Backward tracking, descriptor agreement and calibrated reprojection validate each pair.
std::vector<cv::DMatch>
stereo_matches(Features (&features)[2],
               const cv::Mat (&images)[2],
               const rift_s_world_anchor_config &config,
               cv::ORB &orb,
               const StereoWarp *warp = nullptr)
{
	Transform camera[2] = {transform(config.imu_from_camera[0]), transform(config.imu_from_camera[1])};
	cv::Mat target;
	bool rectified = warp && !warp->x.empty();
	if (rectified)
		cv::remap(images[1], target, warp->x, warp->y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
	else
		target = images[1];
	std::vector<cv::Point2f> left, right, backward;
	for (const auto &key : features[0].keys) {
		left.push_back(key.pt);
		cv::Vec3d ray;
		cv::Point2d guess = key.pt;
		if (unproject(config.camera[0], key.pt, ray)) {
			cv::Vec3d p = camera[1].inverse().apply(camera[0].apply(ray * 2.0));
			if (rectified)
				p = camera[0].r.t() * camera[1].r * p;
			project(config.camera[rectified ? 0 : 1], p, guess);
		}
		right.push_back(guess);
	}
	if (left.empty())
		return {};
	std::vector<uint8_t> forward_ok, backward_ok;
	std::vector<float> forward_error, backward_error;
	cv::calcOpticalFlowPyrLK(images[0], target, left, right, forward_ok, forward_error, {21, 21}, 4,
	                         {cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01},
	                         cv::OPTFLOW_USE_INITIAL_FLOW);
	backward = left;
	cv::calcOpticalFlowPyrLK(target, images[0], right, backward, backward_ok, backward_error, {21, 21}, 4,
	                         {cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01},
	                         cv::OPTFLOW_USE_INITIAL_FLOW);
	std::vector<cv::Mat> pyramid(orb.getNLevels());
	pyramid[0] = images[1];
	for (int level = 1; level < orb.getNLevels(); ++level) {
		double scale = std::pow(orb.getScaleFactor(), level);
		cv::resize(images[1], pyramid[level],
		           {cvRound(images[1].cols / scale), cvRound(images[1].rows / scale)}, 0, 0,
		           cv::INTER_LINEAR_EXACT);
	}
	Features tracked;
	for (size_t i = 0; i < left.size(); ++i) {
		if (!forward_ok[i] || !backward_ok[i] || forward_error[i] > 20 ||
		    cv::norm(backward[i] - left[i]) > 0.75)
			continue;
		cv::KeyPoint key = features[0].keys[i];
		key.pt = right[i];
		if (rectified) {
			cv::Vec3d ray;
			cv::Point2d native;
			if (!unproject(config.camera[0], right[i], ray) ||
			    !project(config.camera[1], camera[1].r.t() * camera[0].r * ray, native))
				continue;
			key.pt = native;
		}
		// ORB compute preserves a supplied orientation. Recompute it in the right
		// patch rather than copy the left camera's intensity-centroid angle.
		double scale = std::pow(orb.getScaleFactor(), key.octave);
		const auto &level = pyramid[key.octave];
		int x = cvRound(key.pt.x / scale), y = cvRound(key.pt.y / scale);
		if (x < 16 || y < 16 || x + 16 >= level.cols || y + 16 >= level.rows)
			continue;
		double mx = 0, my = 0;
		for (int dy = -15; dy <= 15; ++dy) {
			int radius = cvRound(std::sqrt(225 - dy * dy));
			for (int dx = -radius; dx <= radius; ++dx) {
				double intensity = level.at<uint8_t>(y + dy, x + dx);
				mx += dx * intensity;
				my += dy * intensity;
			}
		}
		key.angle = float(std::atan2(my, mx) * 180 / M_PI);
		if (key.angle < 0)
			key.angle += 360;
		key.class_id = int(i);
		tracked.keys.push_back(key);
	}
	orb.compute(images[1], tracked.keys, tracked.descriptors);
	std::vector<cv::DMatch> out;
	for (size_t i = 0; i < tracked.keys.size(); ++i) {
		int source = tracked.keys[i].class_id;
		int distance = 0;
		for (unsigned word = 0; word < 4; ++word) {
			uint64_t a, b;
			std::memcpy(&a, features[0].descriptors.ptr(source) + word * 8, 8);
			std::memcpy(&b, tracked.descriptors.ptr(int(i)) + word * 8, 8);
			distance += __builtin_popcountll(a ^ b);
		}
		if (distance < 72)
			out.emplace_back(source, int(i), float(distance));
	}
	features[1] = std::move(tracked);
	return out;
}

// Explicit scalar encoding avoids padding-dependent camera hashes and caps map allocation before reads.
struct Bytes
{
	std::vector<uint8_t> data;
	size_t offset = 0;
	bool ok = true;
	void
	put32(uint32_t v)
	{
		for (int i = 0; i < 4; ++i)
			data.push_back(uint8_t(v >> (8 * i)));
	}
	uint32_t
	get32()
	{
		if (offset + 4 > data.size()) {
			ok = false;
			return 0;
		}
		uint32_t v = 0;
		for (int i = 0; i < 4; ++i)
			v |= uint32_t(data[offset++]) << (8 * i);
		return v;
	}
	void
	put64(uint64_t v)
	{
		put32(uint32_t(v));
		put32(uint32_t(v >> 32));
	}
	uint64_t
	get64()
	{
		uint64_t a = get32(), b = get32();
		return a | (b << 32);
	}
	void
	putfloat(float f)
	{
		uint32_t v;
		std::memcpy(&v, &f, 4);
		put32(v);
	}
	float
	getfloat()
	{
		uint32_t v = get32();
		float f;
		std::memcpy(&f, &v, 4);
		return f;
	}
};
uint64_t
hash(const uint8_t *data, size_t size)
{
	uint64_t h = 14695981039346656037ULL;
	for (size_t i = 0; i < size; ++i) {
		h ^= data[i];
		h *= 1099511628211ULL;
	}
	return h;
}
bool
boundary_number(const cJSON *value, Bytes &out)
{
	if (!cJSON_IsNumber(value) || !std::isfinite(value->valuedouble) || std::abs(value->valuedouble) > 10000)
		return false;
	float v = float(value->valuedouble);
	out.putfloat(v == 0 ? 0 : v);
	return true;
}
bool
boundary_vector(const cJSON *value, unsigned count, Bytes &out)
{
	if (!cJSON_IsArray(value) || cJSON_GetArraySize(value) != int(count))
		return false;
	for (unsigned i = 0; i < count; ++i)
		if (!boundary_number(cJSON_GetArrayItem(value, int(i)), out))
			return false;
	return true;
}
bool
boundary_hash(const rift_s_world_anchor_config &c, uint64_t &out, uint64_t *legacy = nullptr)
{
	out = 0;
	if (!c.require_boundary)
		return true;
	if (!c.boundary_path[0])
		return false;
	FILE *file = std::fopen(c.boundary_path, "rb");
	if (!file)
		return false;
	struct stat st{};
	bool ok = fstat(fileno(file), &st) == 0 && S_ISREG(st.st_mode) && st.st_size > 0 && st.st_size <= 1024 * 1024;
	std::vector<uint8_t> bytes(ok ? size_t(st.st_size) : 0);
	if (ok)
		ok = std::fread(bytes.data(), 1, bytes.size(), file) == bytes.size();
	std::fclose(file);
	if (!ok)
		return false;
	if (legacy)
		*legacy = hash(bytes.data(), bytes.size());
	bytes.push_back(0);
	std::unique_ptr<cJSON, decltype(&cJSON_Delete)> json(cJSON_Parse(reinterpret_cast<char *>(bytes.data())),
	                                                   cJSON_Delete);
	const auto *id = cJSON_GetObjectItemCaseSensitive(json.get(), "jsonid");
	if (!cJSON_IsString(id) || std::strcmp(id->valuestring, "chaperone_info") != 0)
		return false;
	const auto *universes = cJSON_GetObjectItemCaseSensitive(json.get(), "universes");
	const cJSON *room = nullptr;
	// The SteamVR driver publishes universe 2. Unrelated universes and serialization metadata
	// cannot change the binding between this world's landmarks and its standing frame.
	for (int i = 0; i < cJSON_GetArraySize(universes); ++i) {
		const auto *u = cJSON_GetArrayItem(universes, i);
		const auto *universe = cJSON_GetObjectItemCaseSensitive(u, "universeID");
		if (cJSON_IsString(universe) && std::strcmp(universe->valuestring, "2") == 0) {
			if (room)
				return false;
			room = u;
		}
	}
	if (!room)
		return false;
	Bytes semantic;
	semantic.put32(1); // Semantic boundary format.
	semantic.put64(2);
	const auto *standing = cJSON_GetObjectItemCaseSensitive(room, "standing");
	if (!boundary_vector(cJSON_GetObjectItemCaseSensitive(standing, "translation"), 3, semantic) ||
	    !boundary_number(cJSON_GetObjectItemCaseSensitive(standing, "yaw"), semantic) ||
	    !boundary_vector(cJSON_GetObjectItemCaseSensitive(room, "play_area"), 2, semantic))
		return false;
	const auto *bounds = cJSON_GetObjectItemCaseSensitive(room, "collision_bounds");
	int count = cJSON_GetArraySize(bounds);
	if (!cJSON_IsArray(bounds) || count <= 0 || count > 2048)
		return false;
	semantic.put32(uint32_t(count));
	for (int i = 0; i < count; ++i) {
		const auto *quad = cJSON_GetArrayItem(bounds, i);
		if (!cJSON_IsArray(quad) || cJSON_GetArraySize(quad) != 4)
			return false;
		for (int j = 0; j < 4; ++j)
			if (!boundary_vector(cJSON_GetArrayItem(quad, j), 3, semantic))
				return false;
	}
	out = hash(semantic.data.data(), semantic.data.size());
	return true;
}
void
resolve_boundary_path(rift_s_world_anchor_config &c)
{
	if (!c.require_boundary || c.boundary_path[0])
		return;
	const char *home = std::getenv("HOME"), *config = std::getenv("XDG_CONFIG_HOME");
	if (!home)
		return;
	std::string registry =
	    (config ? std::string(config) : std::string(home) + "/.config") + "/openvr/openvrpaths.vrpath";
	FILE *file = std::fopen(registry.c_str(), "rb");
	if (!file)
		return;
	char bytes[8192]{};
	size_t count = std::fread(bytes, 1, sizeof(bytes) - 1, file);
	bool complete = std::feof(file);
	std::fclose(file);
	if (!complete || !count)
		return;
	cJSON *json = cJSON_Parse(bytes);
	const cJSON *entry = cJSON_GetArrayItem(cJSON_GetObjectItemCaseSensitive(json, "config"), 0);
	if (cJSON_IsString(entry) && entry->valuestring && entry->valuestring[0]) {
		std::string path = std::string(entry->valuestring) + "/chaperone_info.vrchap";
		if (path.size() < sizeof(c.boundary_path))
			std::memcpy(c.boundary_path, path.c_str(), path.size() + 1);
	}
	cJSON_Delete(json);
}
uint64_t
calibration_hash(const rift_s_world_anchor_config &c, unsigned count = 0)
{
	Bytes b;
	for (unsigned e = 0; e < (count ? count : camera_count(c)); ++e) {
		const auto &cam = c.camera[e];
		b.put32(cam.roi.extent.w);
		b.put32(cam.roi.extent.h);
		b.putfloat(cam.projection.fx);
		b.putfloat(cam.projection.fy);
		b.putfloat(cam.projection.cx);
		b.putfloat(cam.projection.cy);
		for (float k : cam.distortion.k)
			b.putfloat(k);
		b.putfloat(cam.distortion.p1);
		b.putfloat(cam.distortion.p2);
		const auto &p = c.imu_from_camera[e];
		b.putfloat(p.orientation.x);
		b.putfloat(p.orientation.y);
		b.putfloat(p.orientation.z);
		b.putfloat(p.orientation.w);
		b.putfloat(p.position.x);
		b.putfloat(p.position.y);
		b.putfloat(p.position.z);
	}
	return hash(b.data.data(), b.data.size());
}
} // namespace

struct rift_s_world_anchor
{
	rift_s_world_anchor_config config;
	std::string path;
	uint64_t bound_boundary = 0;
	bool boundary_bound = false;
	bool storage_rejected = false;
	std::vector<Keyframe> frames;
	bool index_dirty = true;
	std::vector<cv::Vec3d> indexed_world;
	cv::Mat indexed_descriptors[2];
	cv::Ptr<cv::DescriptorMatcher> index[2];
	StereoWarp warps[RIFT_S_WORLD_MAX_CAMERAS][RIFT_S_WORLD_MAX_CAMERAS];
	cv::Mat camera_rays[RIFT_S_WORLD_MAX_CAMERAS];
	// Trusted capture reference for temporal depth in room directions without rig overlap.
	bool reference_valid = false;
	Transform reference_pose;
	int64_t reference_ns = 0;
	cv::Mat reference_images[RIFT_S_WORLD_MAX_CAMERAS];
	Features reference_features[RIFT_S_WORLD_MAX_CAMERAS];
	std::string record_dir;
	unsigned recorded_pairs = 0;
	cv::Ptr<cv::ORB> orb = cv::ORB::create(1400, 1.2f, 8, 19, 0, 2, cv::ORB::HARRIS_SCORE, 31, 12);
};

bool
rift_s_world_anchor_project(const rift_s_camera_calibration *c, const xrt_vec3 *p, xrt_vec2 *out)
{
	if (!c || !p || !out)
		return false;
	cv::Point2d q;
	if (!project(*c, {p->x, p->y, p->z}, q))
		return false;
	*out = {float(q.x), float(q.y)};
	return true;
}
bool
rift_s_world_anchor_unproject(const rift_s_camera_calibration *c, const xrt_vec2 *p, xrt_vec3 *out)
{
	if (!c || !p || !out)
		return false;
	cv::Vec3d q;
	if (!unproject(*c, {p->x, p->y}, q))
		return false;
	*out = {float(q[0]), float(q[1]), float(q[2])};
	return true;
}
bool
rift_s_world_anchor_solve(const rift_s_world_anchor_config *config,
                          const rift_s_world_anchor_observation *obs[2],
                          const uint32_t counts[2],
                          rift_s_world_anchor_result *result)
{
	return rift_s_world_anchor_solve_rig(config, obs, counts, 2, result);
}
bool
rift_s_world_anchor_solve_rig(const rift_s_world_anchor_config *config,
                              const rift_s_world_anchor_observation *const obs[],
                              const uint32_t counts[],
                              unsigned count,
                              rift_s_world_anchor_result *result)
{
	return rift_s_world_anchor_solve_rig_gravity(config, obs, counts, count, nullptr, result);
}
bool
rift_s_world_anchor_solve_rig_gravity(const rift_s_world_anchor_config *config,
                                      const rift_s_world_anchor_observation *const obs[],
                                      const uint32_t counts[],
                                      unsigned count,
                                      const xrt_quat *attitude,
                                      rift_s_world_anchor_result *result)
{
	if (!config || !obs || !counts || !result)
		return false;
	*result = {};
	if (!calibration_valid(*config) || count < 2 || count > camera_count(*config))
		return false;
	if (attitude && !finite_pose({*attitude, {0, 0, 0}}))
		return false;
	try {
		Transform estimates[RIFT_S_WORLD_MAX_CAMERAS];
		bool usable[RIFT_S_WORLD_MAX_CAMERAS]{};
		result->rejection = "matches";
		result->matches[0] = counts[0];
		result->matches[1] = counts[1];
		for (unsigned eye = 0; eye < count; ++eye) {
			result->matches[eye] = counts[eye];
			if (!obs[eye] || counts[eye] < MIN_POINTS)
				continue;
			if (counts[eye] > 4096)
				return false;
			usable[eye] = [&]() -> bool {
				const auto &c = config->camera[eye];
				std::vector<cv::Point3d> world;
				std::vector<cv::Point2d> normalized, pixels;
				for (uint32_t i = 0; i < counts[eye]; ++i) {
					const auto &o = obs[eye][i];
					cv::Vec3d ray;
					if (!unproject(c, {o.pixel.x, o.pixel.y}, ray) ||
					    !std::isfinite(o.world_point.x) || !std::isfinite(o.world_point.y) ||
					    !std::isfinite(o.world_point.z))
						return false;
					world.emplace_back(o.world_point.x, o.world_point.y, o.world_point.z);
					normalized.emplace_back(ray[0] / ray[2], ray[1] / ray[2]);
					pixels.emplace_back(o.pixel.x, o.pixel.y);
				}
				cv::Vec3d rv, tv;
				cv::Mat inliers;
				double threshold = MAX_ERROR_PX / std::max(c.projection.fx, c.projection.fy);
				result->rejection = "pnp";
				bool solved =
				    cv::solvePnPRansac(world, normalized, cv::Matx33d::eye(), cv::noArray(), rv, tv,
				                       false, 150, float(threshold), 0.999, inliers, cv::SOLVEPNP_EPNP);
				result->inliers[eye] = uint32_t(inliers.rows);
				result->rejection = "inliers";
				if (!solved || inliers.rows < 30 || inliers.rows < 0.55 * counts[eye])
					return false;
				std::vector<cv::Point3d> iw;
				std::vector<cv::Point2d> in, ip;
				for (int j = 0; j < inliers.rows; ++j) {
					int i = inliers.at<int>(j);
					iw.push_back(world[i]);
					in.push_back(normalized[i]);
					ip.push_back(pixels[i]);
				}
				Transform cam_from_world;
				// RANSAC runs in the perspective plane, whose scale varies across a fisheye
				// image. Rescore in native pixels and refit the remaining support rather
				// than reject a whole room match for one perspective-plane inlier.
				for (unsigned pass = 0; pass < 4; ++pass) {
					cv::Rodrigues(rv, cam_from_world.r);
					cam_from_world.t = tv;
					if (attitude) {
						if (!refine_gravity(*config, eye, *attitude, iw, ip, cam_from_world))
							return false;
						cv::Rodrigues(cam_from_world.r, rv);
						tv = cam_from_world.t;
					} else {
						cv::solvePnPRefineLM(iw, in, cv::Matx33d::eye(), cv::noArray(), rv, tv);
						cv::Rodrigues(rv, cam_from_world.r);
						cam_from_world.t = tv;
					}
					if (!std::isfinite(cv::norm(tv)) || !std::isfinite(cv::norm(rv)))
						return false;
					std::vector<cv::Point3d> kept_world;
					std::vector<cv::Point2d> kept_normalized, kept_pixels;
					float max_error = 0;
					for (size_t i = 0; i < iw.size(); ++i) {
						cv::Point2d projected;
						if (!project(c, cam_from_world.apply({iw[i].x, iw[i].y, iw[i].z}),
						             projected))
							continue;
						double error = cv::norm(projected - ip[i]);
						if (error > MAX_ERROR_PX)
							continue;
						kept_world.push_back(iw[i]);
						kept_normalized.push_back(in[i]);
						kept_pixels.push_back(ip[i]);
						max_error = std::max(max_error, float(error));
					}
					result->inliers[eye] = uint32_t(kept_world.size());
					result->rejection = "native_inliers";
					if (kept_world.size() < 30 || kept_world.size() < 0.55 * counts[eye])
						return false;
					result->rejection = "coverage";
					if (!spread(kept_pixels, c))
						return false;
					if (kept_world.size() == iw.size()) {
						result->max_reprojection_px =
						    std::max(result->max_reprojection_px, max_error);
						break;
					}
					result->rejection = "refinement";
					if (pass == 3)
						return false;
					iw = std::move(kept_world);
					in = std::move(kept_normalized);
					ip = std::move(kept_pixels);
				}
				estimates[eye] =
				    cam_from_world.inverse() * transform(config->imu_from_camera[eye]).inverse();
				return true;
			}();
		}
		unsigned best = count, support = 0;
		for (unsigned i = 0; i < count; ++i) {
			if (!usable[i])
				continue;
			unsigned agreeing = 0;
			for (unsigned j = 0; j < count; ++j)
				if (usable[j] && cv::norm(estimates[i].t - estimates[j].t) <= 0.06 &&
				    angle_between(estimates[i], estimates[j]) <= 3 * M_PI / 180)
					++agreeing;
			if (agreeing > support ||
			    (agreeing == support && best < count && result->inliers[i] > result->inliers[best])) {
				best = i;
				support = agreeing;
			}
		}
		if (support < 2) {
			if (support)
				result->rejection = "camera_disagreement";
			return false;
		}
		result->rejection = "camera_disagreement";
		// Two independent confident clusters would leave the room identity ambiguous.
		for (unsigned i = 0; i < count; ++i)
			for (unsigned j = i + 1; j < count; ++j) {
				if (!usable[i] || !usable[j])
					continue;
				if (cv::norm(estimates[i].t - estimates[j].t) <= 0.06 &&
				    angle_between(estimates[i], estimates[j]) <= 3 * M_PI / 180 &&
				    (cv::norm(estimates[i].t - estimates[best].t) > 0.12 ||
				     angle_between(estimates[i], estimates[best]) > 6 * M_PI / 180))
					return false;
			}
		result->supporting_cameras = support;
		result->world_from_imu = pose(estimates[best]);
		result->rejection = nullptr;
		result->relocalized = finite_pose(result->world_from_imu);
		return result->relocalized;
	} catch (const cv::Exception &) {
		return false;
	}
}

namespace {
void
put_pose(Bytes &b, const Transform &t)
{
	xrt_pose p = pose(t);
	b.putfloat(p.orientation.x);
	b.putfloat(p.orientation.y);
	b.putfloat(p.orientation.z);
	b.putfloat(p.orientation.w);
	b.putfloat(p.position.x);
	b.putfloat(p.position.y);
	b.putfloat(p.position.z);
}
xrt_pose
get_pose(Bytes &b)
{
	xrt_pose p;
	p.orientation.x = b.getfloat();
	p.orientation.y = b.getfloat();
	p.orientation.z = b.getfloat();
	p.orientation.w = b.getfloat();
	p.position.x = b.getfloat();
	p.position.y = b.getfloat();
	p.position.z = b.getfloat();
	return p;
}
bool
load(rift_s_world_anchor &a)
{
	FILE *file = std::fopen(a.path.c_str(), "rb");
	if (!file) {
		U_LOG_I("World map absent errno=%d", errno);
		return false;
	}
	struct stat st{};
	if (fstat(fileno(file), &st) != 0 || !S_ISREG(st.st_mode) || st.st_size < 28 || st.st_size > 8 * 1024 * 1024) {
		std::fclose(file);
		return false;
	}
	Bytes b;
	b.data.resize(size_t(st.st_size));
	bool ok = std::fread(b.data.data(), 1, b.data.size(), file) == b.data.size();
	std::fclose(file);
	if (!ok)
		return false;
	uint32_t magic = b.get32(), version = b.get32();
	if (magic != 0x41575352 || (version != 2 && version != 3 && version != 4)) {
		U_LOG_I("World map rejected: format/version");
		return false;
	}
	if (b.get64() != calibration_hash(a.config, version == 2 ? 2 : 0)) {
		U_LOG_I("World map rejected: calibration mismatch");
		return false;
	}
	uint64_t current_boundary = 0, legacy_boundary = 0;
	uint64_t saved_boundary = b.get64();
	if (!boundary_hash(a.config, current_boundary, &legacy_boundary) ||
	    saved_boundary != (version < 4 ? legacy_boundary : current_boundary)) {
		U_LOG_I("World map rejected: committed boundary mismatch");
		return false;
	}
	uint32_t frames = b.get32();
	if (!frames || frames > MAX_KEYFRAMES)
		return false;
	std::vector<Keyframe> loaded;
	for (uint32_t i = 0; i < frames; ++i) {
		xrt_pose p = get_pose(b);
		if (!b.ok || !finite_pose(p))
			return false;
		Keyframe k{transform(p), {}};
		uint32_t count = b.get32();
		if (count < MIN_POINTS || count > MAX_POINTS)
			return false;
		for (uint32_t j = 0; j < count; ++j) {
			Point point;
			for (int d = 0; d < 3; ++d)
				point.world[d] = b.getfloat();
			if (!std::isfinite(cv::norm(point.world)) || cv::norm(point.world - k.world_from_imu.t) > 10 ||
			    b.offset + 64 > b.data.size())
				return false;
			for (auto &descriptor : point.descriptors)
				for (auto &v : descriptor)
					v = b.data[b.offset++];
			if (version >= 3) {
				uint32_t pair = b.get32();
				point.cameras[0] = pair & 0xff;
				point.cameras[1] = (pair >> 8) & 0xff;
				if (point.cameras[0] >= camera_count(a.config) ||
				    point.cameras[1] >= camera_count(a.config))
					return false;
			}
			k.points.push_back(point);
		}
		loaded.push_back(std::move(k));
	}
	size_t end = b.offset;
	uint64_t checksum = b.get64();
	if (!b.ok || b.offset != b.data.size() || checksum != hash(b.data.data(), end))
		return false;
	a.frames = std::move(loaded);
	a.bound_boundary = current_boundary;
	a.boundary_bound = true;
	a.index_dirty = true;
	U_LOG_I("World map loaded keyframes=%zu bytes=%zu", a.frames.size(), b.data.size());
	return true;
}
} // namespace
rift_s_world_anchor *
rift_s_world_anchor_create(const rift_s_world_anchor_config *config, const char *path)
{
	if (!config || !calibration_valid(*config))
		return nullptr;
	try {
		auto a = std::make_unique<rift_s_world_anchor>();
		a->config = *config;
		resolve_boundary_path(a->config);
		a->path = path ? path : "";
		const char *record = std::getenv("RIFT_S_WORLD_RECORD_DIR"),
		           *bench = std::getenv("MONADO_STEAMVR_BENCH");
		if (record && record[0] && bench && (std::strcmp(bench, "1") == 0 || std::strcmp(bench, "true") == 0)) {
			if (mkdir(record, 0700) == 0 || errno == EEXIST) {
				a->record_dir = record;
				FILE *file = std::fopen((a->record_dir + "/calibration.bin").c_str(), "wb");
				if (file) {
					std::fwrite(&a->config, sizeof(a->config), 1, file);
					std::fclose(file);
				}
			}
		}
		if (!a->path.empty())
			a->storage_rejected = !load(*a) && access(a->path.c_str(), F_OK) == 0;
		return a.release();
	} catch (const std::exception &) {
		return nullptr;
	}
}
void
rift_s_world_anchor_record_frames(rift_s_world_anchor *a,
                                  const xrt_frame *left,
                                  const xrt_frame *right,
                                  const xrt_pose *raw)
{
	const xrt_frame *frames[2] = {left, right};
	rift_s_world_anchor_record_rig_frames(a, frames, 2, raw);
}
void
rift_s_world_anchor_record_rig_frames(rift_s_world_anchor *a,
                                      const xrt_frame *const frames[],
                                      unsigned count,
                                      const xrt_pose *raw)
{
	if (!a || a->record_dir.empty() || a->recorded_pairs >= 600 || !frames || count < 2 ||
	    count > RIFT_S_WORLD_MAX_CAMERAS || !frames[0])
		return;
	for (unsigned eye = 0; eye < count; ++eye) {
		const auto *f = frames[eye];
		if (!f || f->timestamp != frames[0]->timestamp || !f->data || f->format != XRT_FORMAT_L8 ||
		    f->stride < f->width || !f->width || !f->height || f->size < (f->height - 1) * f->stride + f->width)
			return;
	}
	unsigned index = a->recorded_pairs++;
	for (unsigned eye = 0; eye < count; ++eye) {
		char name[64];
		std::snprintf(name, sizeof(name), "/%04u-%u.pgm", index, eye);
		FILE *file = std::fopen((a->record_dir + name).c_str(), "wb");
		if (!file)
			return;
		const auto *f = frames[eye];
		std::fprintf(file, "P5\n%u %u\n255\n", f->width, f->height);
		for (uint32_t row = 0; row < f->height; ++row)
			std::fwrite(f->data + row * f->stride, 1, f->width, file);
		std::fclose(file);
	}
	FILE *file = std::fopen((a->record_dir + "/poses.csv").c_str(), "a");
	if (!file)
		return;
	xrt_pose p = raw ? *raw : xrt_pose{{0, 0, 0, 1}, {0, 0, 0}};
	std::fprintf(file, "%u,%lld,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n", index, (long long)frames[0]->timestamp,
	             raw != nullptr, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w, p.position.x,
	             p.position.y, p.position.z);
	std::fclose(file);
}

void
rift_s_world_anchor_destroy(rift_s_world_anchor *a)
{
	delete a;
}
bool
rift_s_world_anchor_has_map(const rift_s_world_anchor *a)
{
	return a && !a->frames.empty();
}
bool
rift_s_world_anchor_has_boundary(const rift_s_world_anchor *a)
{
	uint64_t binding;
	return a && a->config.require_boundary && boundary_hash(a->config, binding);
}
void
rift_s_world_anchor_clear(rift_s_world_anchor *a)
{
	if (!a)
		return;
	a->frames.clear();
	a->boundary_bound = false;
	a->storage_rejected = false;
	a->reference_valid = false;
	a->index_dirty = true;
}
bool
rift_s_world_anchor_reload(rift_s_world_anchor *a)
{
	if (!a)
		return false;
	a->frames.clear();
	a->boundary_bound = false;
	a->reference_valid = false;
	a->index_dirty = true;
	bool loaded = !a->path.empty() && load(*a);
	a->storage_rejected = !loaded && !a->path.empty() && access(a->path.c_str(), F_OK) == 0;
	return loaded;
}
bool
rift_s_world_anchor_save(rift_s_world_anchor *a)
{
	if (!a || a->path.empty())
		return false;
	if (a->storage_rejected) {
		U_LOG_W("World map save rejected: existing map was not loaded; explicit room setup is required");
		return false;
	}
	auto parent_slash = a->path.find_last_of('/');
	if (parent_slash != std::string::npos) {
		auto parent = a->path.substr(0, parent_slash);
		if (mkdir(parent.c_str(), 0700) != 0 && errno != EEXIST)
			return false;
	}
	uint64_t committed_boundary;
	if (!boundary_hash(a->config, committed_boundary))
		return false;
	if (a->boundary_bound && a->bound_boundary != committed_boundary) {
		U_LOG_W("World map save rejected: standing frame or boundary changed without room setup");
		return false;
	}
	Bytes b;
	b.put32(0x41575352);
	b.put32(4);
	b.put64(calibration_hash(a->config));
	b.put64(committed_boundary);
	b.put32(uint32_t(a->frames.size()));
	for (const auto &k : a->frames) {
		put_pose(b, k.world_from_imu);
		b.put32(uint32_t(k.points.size()));
		for (const auto &p : k.points) {
			for (double v : p.world.val)
				b.putfloat(float(v));
			for (const auto &d : p.descriptors)
				b.data.insert(b.data.end(), d.begin(), d.end());
			b.put32(p.cameras[0] | (uint32_t(p.cameras[1]) << 8));
		}
	}
	b.put64(hash(b.data.data(), b.data.size()));
	std::string temporary = a->path + ".tmp-XXXXXX";
	std::vector<char> name(temporary.begin(), temporary.end());
	name.push_back(0);
	int fd = mkstemp(name.data());
	if (fd < 0)
		return false;
	bool ok = fchmod(fd, 0600) == 0;
	size_t offset = 0;
	while (ok && offset < b.data.size()) {
		ssize_t n = write(fd, b.data.data() + offset, b.data.size() - offset);
		if (n <= 0)
			ok = false;
		else
			offset += size_t(n);
	}
	if (ok && fsync(fd) != 0)
		ok = false;
	if (close(fd) != 0)
		ok = false;
	uint64_t latest_boundary;
	if (ok && (!boundary_hash(a->config, latest_boundary) || latest_boundary != committed_boundary))
		ok = false;
	if (ok && rename(name.data(), a->path.c_str()) != 0)
		ok = false;
	if (!ok)
		unlink(name.data());
	if (ok) {
		auto slash = a->path.find_last_of('/');
		std::string dir = slash == std::string::npos ? "." : a->path.substr(0, slash);
		int d = open(dir.c_str(), O_RDONLY | O_DIRECTORY);
		if (d < 0)
			ok = false;
		else {
			if (fsync(d) != 0)
				ok = false;
			close(d);
		}
	}
	if (ok) {
		a->bound_boundary = committed_boundary;
		a->boundary_bound = true;
	}
	return ok;
}
namespace {
void
build_index(rift_s_world_anchor &a)
{
	if (!a.index_dirty)
		return;
	a.indexed_world.clear();
	for (const auto &k : a.frames)
		for (const auto &point : k.points)
			a.indexed_world.push_back(point.world);
	for (unsigned bank = 0; bank < 2; ++bank) {
		a.indexed_descriptors[bank].create(int(a.indexed_world.size()), 32, CV_8U);
		unsigned row = 0;
		for (const auto &k : a.frames)
			for (const auto &point : k.points)
				std::memcpy(a.indexed_descriptors[bank].ptr(int(row++)), point.descriptors[bank].data(),
				            32);
		if (a.indexed_world.size() > 2000)
			a.index[bank] =
			    cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
		else
			a.index[bank] = cv::makePtr<cv::BFMatcher>(cv::NORM_HAMMING);
		a.index[bank]->add(std::vector<cv::Mat>{a.indexed_descriptors[bank]});
		a.index[bank]->train();
	}
	a.index_dirty = false;
}
std::vector<rift_s_world_anchor_observation>
room_matches(rift_s_world_anchor &a, const Features &f)
{
	std::vector<cv::DMatch> candidates;
	if (f.descriptors.rows < 2 || a.indexed_world.size() < 4)
		return {};
	for (unsigned bank = 0; bank < 2; ++bank) {
		std::vector<std::vector<cv::DMatch>> nearest;
		a.index[bank]->knnMatch(f.descriptors, nearest, 4);
		for (const auto &list : nearest) {
			if (list.empty() || list[0].distance >= 64)
				continue;
			const auto &best = list[0];
			float second = 256;
			// Repeated observations of one physical landmark are not competing identities.
			for (size_t i = 1; i < list.size(); ++i)
				if (cv::norm(a.indexed_world[list[i].trainIdx] - a.indexed_world[best.trainIdx]) >
				    0.03) {
					second = list[i].distance;
					break;
				}
			if (best.distance < 0.75f * second)
				candidates.push_back(best);
		}
	}
	std::sort(candidates.begin(), candidates.end(),
	          [](const auto &a, const auto &b) { return a.distance < b.distance; });
	std::vector<bool> used_feature(f.keys.size()), used_point(a.indexed_world.size());
	std::vector<rift_s_world_anchor_observation> out;
	for (const auto &m : candidates) {
		if (used_feature[m.queryIdx] || used_point[m.trainIdx])
			continue;
		used_feature[m.queryIdx] = used_point[m.trainIdx] = true;
		const auto &w = a.indexed_world[m.trainIdx];
		const auto &p = f.keys[m.queryIdx].pt;
		out.push_back({{float(w[0]), float(w[1]), float(w[2])}, {p.x, p.y}});
	}
	return out;
}
} // namespace
bool
rift_s_world_anchor_process(rift_s_world_anchor *a,
                            const xrt_frame *left,
                            const xrt_frame *right,
                            const xrt_pose *trusted,
                            rift_s_world_anchor_result *result)
{
	const xrt_frame *frames[2] = {left, right};
	return rift_s_world_anchor_process_rig(a, frames, 2, trusted, result);
}
bool
rift_s_world_anchor_process_rig(rift_s_world_anchor *a,
                                const xrt_frame *const frames[],
                                unsigned count,
                                const xrt_pose *trusted,
                                rift_s_world_anchor_result *result)
{
	return rift_s_world_anchor_process_rig_gravity(a, frames, count, trusted, nullptr, result);
}
bool
rift_s_world_anchor_process_rig_gravity(rift_s_world_anchor *a,
                                        const xrt_frame *const frames[],
                                        unsigned count,
                                        const xrt_pose *trusted,
                                        const xrt_quat *attitude,
                                        rift_s_world_anchor_result *result)
{
	if (!a || !frames || !result || count < 2 || count > camera_count(a->config))
		return false;
	*result = {};
	result->keyframes = uint32_t(a->frames.size());
	if (!frames[0] || frames[0]->timestamp <= 0)
		return false;
	for (unsigned e = 0; e < count; ++e) {
		const auto *f = frames[e];
		const auto &c = a->config.camera[e];
		if (!f || f->timestamp != frames[0]->timestamp || f->format != XRT_FORMAT_L8 || !f->data ||
		    f->width != uint32_t(c.roi.extent.w) || f->height != uint32_t(c.roi.extent.h) ||
		    f->stride < f->width || f->size < (f->height - 1) * f->stride + f->width) {
			result->rejection = "frame";
			return false;
		}
	}
	try {
		size_t replace = a->frames.size();
		if (trusted) {
			if (!finite_pose(*trusted))
				return false;
			Transform world_from_imu = transform(*trusted);
			for (size_t i = 0; i < a->frames.size(); ++i) {
				auto &k = a->frames[i];
				if (cv::norm(world_from_imu.t - k.world_from_imu.t) < 0.2 &&
				    angle_between(world_from_imu, k.world_from_imu) < 15 * M_PI / 180) {
					if (k.points.size() >= 3 * MAX_POINTS / 4 ||
					    (k.last_refresh_ns > 0 && frames[0]->timestamp >= k.last_refresh_ns &&
					     frames[0]->timestamp - k.last_refresh_ns < 2000000000LL))
						return false;
					k.last_refresh_ns = frames[0]->timestamp;
					replace = i;
					break;
				}
			}
			// Evict the most redundant viewpoint rather than erase a unique room direction.
			if (replace == a->frames.size() && a->frames.size() >= MAX_KEYFRAMES) {
				double redundancy = 1e9;
				for (size_t i = 0; i < a->frames.size(); ++i) {
					double nearest = 1e9;
					for (size_t j = 0; j < a->frames.size(); ++j)
						if (i != j)
							nearest = std::min(
							    nearest, cv::norm(a->frames[i].world_from_imu.t -
							                      a->frames[j].world_from_imu.t) +
							                 angle_between(a->frames[i].world_from_imu,
									               a->frames[j].world_from_imu));
					if (nearest < redundancy) {
						redundancy = nearest;
						replace = i;
					}
				}
			}
		}
		Features features[RIFT_S_WORLD_MAX_CAMERAS];
		cv::Mat images[RIFT_S_WORLD_MAX_CAMERAS];
		for (unsigned e = 0; e < count; ++e) {
			images[e] = cv::Mat(int(frames[e]->height), int(frames[e]->width), CV_8UC1, frames[e]->data,
			                    frames[e]->stride);
			a->orb->detectAndCompute(images[e], cv::noArray(), features[e].keys, features[e].descriptors);
			result->features[e] = uint32_t(features[e].descriptors.rows);
		}
		if (trusted) {
			Transform world_from_imu = transform(*trusted);
			if (attitude && !a->frames.empty()) {
				build_index(*a);
				std::vector<rift_s_world_anchor_observation> matched[RIFT_S_WORLD_MAX_CAMERAS];
				const rift_s_world_anchor_observation *obs[RIFT_S_WORLD_MAX_CAMERAS]{};
				uint32_t counts[RIFT_S_WORLD_MAX_CAMERAS]{};
				unsigned overlap = 0;
				for (unsigned e = 0; e < count; ++e) {
					matched[e] = room_matches(*a, features[e]);
					obs[e] = matched[e].data();
					counts[e] = uint32_t(matched[e].size());
					overlap += counts[e] >= MIN_POINTS;
				}
				// A novel room direction can extend the map. Overlapping views must agree
				// with its existing geometry before automatic updates can replace landmarks.
				if (overlap >= 2) {
					rift_s_world_anchor_result checked{};
					if (!rift_s_world_anchor_solve_rig_gravity(&a->config, obs, counts, count,
					                                           attitude, &checked) ||
					    cv::norm(transform(checked.world_from_imu).t - world_from_imu.t) > 0.08 ||
					    angle_between(transform(checked.world_from_imu), world_from_imu) > 5 * M_PI / 180) {
						result->rejection = "map_update_disagreement";
						return false;
					}
				}
			}
			Keyframe keyframe{world_from_imu, {}, frames[0]->timestamp};
			unsigned bins[RIFT_S_WORLD_MAX_CAMERAS][12]{}, source_points[RIFT_S_WORLD_MAX_CAMERAS]{};
			std::vector<bool> used[RIFT_S_WORLD_MAX_CAMERAS];
			std::vector<cv::Point2d> coverage[RIFT_S_WORLD_MAX_CAMERAS];
			for (unsigned e = 0; e < count; ++e)
				used[e].resize(features[e].keys.size());
			// Front stereo, side/front overlaps, and the top camera supply room directions.
			std::vector<std::array<unsigned, 2>> pairs = {{0, 1}, {2, 0}, {3, 1}, {4, 0}, {4, 1},
			                                              {2, 4}, {3, 4}, {0, 2}, {1, 3}, {2, 3}};
			double movement = a->reference_valid ? cv::norm(world_from_imu.t - a->reference_pose.t) : 0;
			bool temporal = a->reference_valid && frames[0]->timestamp > a->reference_ns &&
			                frames[0]->timestamp - a->reference_ns <= 2000000000LL && movement >= 0.05 &&
			                movement <= 0.65 &&
			                angle_between(world_from_imu, a->reference_pose) < 45 * M_PI / 180;
			if (temporal)
				for (unsigned e = 0; e < count; ++e)
					pairs.push_back({e, e});
			for (const auto &pair : pairs) {
				unsigned first = pair[0], second = pair[1];
				bool previous_capture = first == second;
				if (first >= count || second >= count ||
				    features[first].descriptors.rows < int(MIN_POINTS) ||
				    features[second].descriptors.rows < int(MIN_POINTS))
					continue;
				rift_s_world_anchor_config stereo_config = a->config;
				stereo_config.camera_count = 2;
				stereo_config.camera[0] = a->config.camera[first];
				stereo_config.camera[1] = a->config.camera[second];
				stereo_config.imu_from_camera[0] = a->config.imu_from_camera[first];
				stereo_config.imu_from_camera[1] = a->config.imu_from_camera[second];
				if (previous_capture)
					stereo_config.imu_from_camera[1] =
					    pose(world_from_imu.inverse() * a->reference_pose *
					         transform(a->config.imu_from_camera[second]));
				Transform cameras[2] = {transform(stereo_config.imu_from_camera[0]),
				                        transform(stereo_config.imu_from_camera[1])};
				if (cv::norm(cameras[1].t - cameras[0].t) < 0.025)
					continue;
				StereoWarp temporal_warp;
				auto &warp = previous_capture ? temporal_warp : a->warps[first][second];
				if (!warp.initialized)
					build_warp(warp, stereo_config, a->camera_rays[first]);
				Features pair_features[2] = {features[first], previous_capture
				                                                  ? a->reference_features[second]
				                                                  : features[second]};
				cv::Mat pair_images[2] = {images[first], previous_capture ? a->reference_images[second]
				                                                          : images[second]};
				auto stereo = stereo_matches(pair_features, pair_images, stereo_config, *a->orb, &warp);
				result->stereo_matches += uint32_t(stereo.size());
				for (const auto &m : stereo) {
					cv::Point2d pixels[2] = {pair_features[0].keys[m.queryIdx].pt,
					                         pair_features[1].keys[m.trainIdx].pt};
					int bin = std::clamp(int(4 * pixels[0].x / frames[first]->width), 0, 3) +
					          4 * std::clamp(int(3 * pixels[0].y / frames[first]->height), 0, 2);
					if (used[first][m.queryIdx] || bins[first][bin] >= MAX_POINTS / 24 ||
					    source_points[first] >= MAX_POINTS / (count == 2 ? 1 : count))
						continue;
					cv::Vec3d rays[2];
					if (!unproject(stereo_config.camera[0], pixels[0], rays[0]) ||
					    !unproject(stereo_config.camera[1], pixels[1], rays[1]))
						continue;
					cv::Vec3d d0 = cameras[0].r * rays[0], d1 = cameras[1].r * rays[1],
					          baseline = cameras[1].t - cameras[0].t;
					double dot = d0.dot(d1), denom = 1 - dot * dot;
					if (denom < 0.000225)
						continue;
					double depth0 = (d0.dot(baseline) - dot * d1.dot(baseline)) / denom;
					double depth1 = (dot * d0.dot(baseline) - d1.dot(baseline)) / denom;
					if (depth0 < 0.25 || depth0 > 8 || depth1 < 0.25 || depth1 > 8)
						continue;
					cv::Vec3d point =
					    (cameras[0].t + depth0 * d0 + cameras[1].t + depth1 * d1) * 0.5;
					bool good = true;
					for (unsigned eye = 0; eye < 2; ++eye) {
						cv::Point2d projected;
						if (!project(stereo_config.camera[eye],
						             cameras[eye].inverse().apply(point), projected) ||
						    cv::norm(projected - pixels[eye]) > 1.0)
							good = false;
					}
					if (!good)
						continue;
					Point p;
					p.world = world_from_imu.apply(point);
					p.cameras[0] = first;
					p.cameras[1] = second;
					std::memcpy(p.descriptors[0].data(),
					            pair_features[0].descriptors.ptr(m.queryIdx), 32);
					std::memcpy(p.descriptors[1].data(),
					            pair_features[1].descriptors.ptr(m.trainIdx), 32);
					keyframe.points.push_back(p);
					used[first][m.queryIdx] = true;
					++source_points[first];
					coverage[first].push_back(pixels[0]);
					if (!previous_capture)
						coverage[second].push_back(pixels[1]);
					++bins[first][bin];
					++result->camera_points[first];
					if (!previous_capture)
						++result->camera_points[second];
					if (keyframe.points.size() >= MAX_POINTS)
						break;
				}
				if (keyframe.points.size() >= MAX_POINTS)
					break;
			}
			// Keep enough baseline for temporal depth, but never bridge a stale pose interval.
			if (!a->reference_valid || movement >= 0.08 ||
			    frames[0]->timestamp - a->reference_ns > 2000000000LL ||
			    angle_between(world_from_imu, a->reference_pose) > 20 * M_PI / 180) {
				a->reference_valid = true;
				a->reference_pose = world_from_imu;
				a->reference_ns = frames[0]->timestamp;
				for (unsigned e = 0; e < count; ++e) {
					a->reference_images[e] = images[e].clone();
					a->reference_features[e] = features[e];
				}
			}
			result->triangulated = uint32_t(keyframe.points.size());
			unsigned supported = 0;
			for (unsigned e = 0; e < count; ++e)
				supported +=
				    result->camera_points[e] >= MIN_POINTS && spread(coverage[e], a->config.camera[e]);
			if (keyframe.points.size() < MIN_POINTS || supported < 2) {
				result->rejection = "triangulation";
				return false;
			}
			if (replace < a->frames.size()) {
				if (cv::norm(a->frames[replace].world_from_imu.t - world_from_imu.t) < 0.2 &&
				    angle_between(a->frames[replace].world_from_imu, world_from_imu) <
				        15 * M_PI / 180 &&
				    a->frames[replace].points.size() < MAX_POINTS &&
				    keyframe.points.size() < a->frames[replace].points.size() + 25)
					return false;
				a->frames[replace] = std::move(keyframe);
			} else
				a->frames.push_back(std::move(keyframe));
			a->index_dirty = true;
			result->recorded = true;
			result->keyframes = uint32_t(a->frames.size());
			return true;
		}
		if (a->frames.empty())
			return false;
		build_index(*a);
		std::vector<rift_s_world_anchor_observation> observations[RIFT_S_WORLD_MAX_CAMERAS];
		const rift_s_world_anchor_observation *obs[RIFT_S_WORLD_MAX_CAMERAS]{};
		uint32_t counts[RIFT_S_WORLD_MAX_CAMERAS]{};
		for (unsigned e = 0; e < count; ++e) {
			observations[e] = room_matches(*a, features[e]);
			obs[e] = observations[e].data();
			counts[e] = observations[e].size();
		}
		rift_s_world_anchor_result solved{};
		bool ok = rift_s_world_anchor_solve_rig_gravity(&a->config, obs, counts, count, attitude, &solved);
		for (unsigned e = 0; e < count; ++e)
			solved.features[e] = result->features[e];
		solved.keyframes = result->keyframes;
		*result = solved;
		return ok;
	} catch (const cv::Exception &) {
		result->rejection = "opencv";
		return false;
	}
}
