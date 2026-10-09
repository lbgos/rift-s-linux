// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "constrained_pose.h"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace {
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;
using Q = Eigen::Quaterniond;
using M4 = Eigen::Matrix4d;
using M6 = Eigen::Matrix<double, 6, 6>;
using V6 = Eigen::Matrix<double, 6, 1>;
using Clock = std::chrono::steady_clock;
constexpr double pi = 3.14159265358979323846;

V3
vec(const xrt_vec3 &v)
{
	return V3(v.x, v.y, v.z);
}
Q
quat(const xrt_quat &q)
{
	return Q(q.w, q.x, q.y, q.z).normalized();
}
xrt_vec3
xvec(const V3 &v)
{
	return {float(v.x()), float(v.y()), float(v.z())};
}
xrt_pose
pose(const Q &q, const V3 &t)
{
	return {{float(q.x()), float(q.y()), float(q.z()), float(q.w())}, xvec(t)};
}
struct Pose
{
	Q q;
	V3 t;
};
Pose
from_pose(const xrt_pose &p)
{
	return {quat(p.orientation), vec(p.position)};
}
Pose
compose(const Pose &a, const Pose &b)
{
	return {a.q * b.q, a.q * b.t + a.t};
}
bool
valid_pose(const xrt_pose &p)
{
	const Q q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
	return q.coeffs().allFinite() && std::abs(q.norm() - 1.0) < 0.01 && vec(p.position).allFinite();
}
enum constellation_constrained_solver_status
valid_pair(const xrt_vec3 bearings[2], double min_angle, V3 &a, V3 &b)
{
	a = vec(bearings[0]);
	b = vec(bearings[1]);
	if (!a.allFinite() || !b.allFinite() || a.norm() < 1e-9 || b.norm() < 1e-9 || !std::isfinite(min_angle))
		return CONSTELLATION_SOLVER_INVALID_INPUT;
	a.normalize();
	b.normalize();
	if (a.dot(b) < std::cos(min_angle) && a.dot(b) > -std::cos(min_angle))
		return CONSTELLATION_SOLVER_OK;
	return CONSTELLATION_SOLVER_DEGENERATE_BEARINGS;
}
/* Least-squares translation from perpendicular ray projectors; rejects rank loss and LEDs behind the camera. */
bool
translation(const V3 p[2], const V3 u[2], const Q &q, V3 &out)
{
	M3 m = M3::Zero();
	V3 b = V3::Zero();
	for (unsigned i = 0; i < 2; i++) {
		M3 a = M3::Identity() - u[i] * u[i].transpose();
		m += a;
		b -= a * (q * p[i]);
	}
	Eigen::ColPivHouseholderQR<M3> solve(m);
	solve.setThreshold(1e-8);
	if (solve.rank() != 3)
		return false;
	out = solve.solve(b);
	return out.allFinite() && (q * p[0] + out).dot(u[0]) > 0 && (q * p[1] + out).dot(u[1]) > 0 &&
	       (q * p[0] + out).z() > 0 && (q * p[1] + out).z() > 0;
}
void
set_status(enum constellation_constrained_solver_status *out, enum constellation_constrained_solver_status value)
{
	if (out)
		*out = value;
}

struct Ray
{
	V3 u;
	unsigned blob_index;
};
struct View
{
	const constellation_constrained_view *input;
	Pose world_cam, cam_world;
	std::vector<Ray> rays;
};
struct Match
{
	unsigned view, ray, led;
	double squared;
};
struct Candidate
{
	Pose p;
	constellation_constrained_origin origin;
	unsigned sample_size;
	unsigned candidates = 0, visible = 0, distinct = 0;
	double sum = 0, cost = 0;
	std::vector<Match> matches;
	M4 covariance = M4::Zero();
	unsigned rank = 0;
	//! Whether yaw about gravity is a free parameter of the optical fit.
	bool yaw_free = true;
	//! World gravity in the model frame of the hypothesis as generated: the reference tilt.
	V3 model_gravity;
	double association_gap = std::numeric_limits<double>::infinity();
	std::array<unsigned, CONSTELLATION_CONSTRAINED_MAX_VIEWS> camera_visible{}, camera_candidates{},
	    camera_inliers{};
	Candidate(Pose pose, constellation_constrained_origin source, unsigned samples)
	    : p(std::move(pose)), origin(source), sample_size(samples),
	      yaw_free(source == CONSTELLATION_CONSTRAINED_GRAVITY || source == CONSTELLATION_CONSTRAINED_P3P),
	      model_gravity(p.q.conjugate() * V3::UnitY())
	{}
};
struct Context
{
	t_constellation_led_model *model;
	const constellation_constrained_config &cfg;
	std::vector<View> views;
	constellation_constrained_result &stats;
	Clock::time_point deadline;
	//! No trusted position prior: the cold acquisition policy applies.
	bool cold = false;
	//! Trusted prior position that bounds every hypothesis (see maximum_prior_distance_m).
	bool gated = false;
	V3 prior_position = V3::Zero();
	bool heading_gated = false;
	Q prior_rotation = Q::Identity();
};

bool
outside_prior_gate(Context &ctx, const Candidate &c)
{
	if (ctx.gated && (c.p.t - ctx.prior_position).norm() > ctx.cfg.maximum_prior_distance_m) {
		ctx.stats.prior_distance_rejections++;
		return true;
	}
	if (ctx.heading_gated && c.p.q.angularDistance(ctx.prior_rotation) > ctx.cfg.maximum_prior_rotation_rad) {
		ctx.stats.prior_rotation_rejections++;
		return true;
	}
	return false;
}

bool
prepare(Context &ctx, const constellation_constrained_view *views, unsigned count)
{
	if (!ctx.model || !ctx.model->leds || ctx.model->num_leds < 2 || !views || count == 0 ||
	    count > CONSTELLATION_CONSTRAINED_MAX_VIEWS || !(ctx.cfg.score_gate_m > 0) ||
	    !(ctx.cfg.association_gate_m >= ctx.cfg.score_gate_m) || !(ctx.cfg.ray_noise_std_m > 0) ||
	    !(ctx.cfg.minimum_bearing_separation_rad > 0) || ctx.cfg.max_pair_trials == 0 ||
	    !(ctx.cfg.minimum_bearing_separation_rad < pi / 2) ||
	    !(ctx.cfg.maximum_depth_m > ctx.cfg.minimum_depth_m) || ctx.cfg.minimum_depth_m < 0 ||
	    !(ctx.cfg.maximum_position_std_m > 0) || !(ctx.cfg.maximum_yaw_std_rad > 0) ||
	    !(ctx.cfg.ambiguity_chi_square > 0) || ctx.cfg.time_budget_us == 0)
		return false;
	for (double value :
	     {ctx.cfg.score_gate_m, ctx.cfg.association_gate_m, ctx.cfg.ray_noise_std_m,
	      ctx.cfg.minimum_bearing_separation_rad, ctx.cfg.minimum_depth_m, ctx.cfg.maximum_depth_m,
	      ctx.cfg.maximum_position_std_m, ctx.cfg.maximum_yaw_std_rad, ctx.cfg.ambiguity_chi_square}) {
		if (!std::isfinite(value))
			return false;
	}
	std::array<bool, 256> identities{};
	for (unsigned l = 0; l < ctx.model->num_leds; l++) {
		const auto &led = ctx.model->leds[l];
		if (!vec(led.pos).allFinite() || !vec(led.dir).allFinite() || vec(led.dir).norm() < 1e-9 ||
		    identities[led.id])
			return false;
		identities[led.id] = true;
	}
	for (unsigned k = 0; k < count; k++) {
		const auto &v = views[k];
		if (!v.calib || !v.observation || !valid_pose(v.P_world_cam) || !valid_pose(v.P_cam_world) ||
		    v.observation->num_blobs < 0 || v.observation->num_blobs > MAX_BLOBS_PER_FRAME)
			return false;
		for (const auto &previous : ctx.views)
			if (previous.input->camera_index == v.camera_index)
				return false;
		View prepared{&v, from_pose(v.P_world_cam), from_pose(v.P_cam_world), {}};
		Pose identity = compose(prepared.cam_world, prepared.world_cam);
		if (identity.t.norm() > 1e-4 || identity.q.angularDistance(Q::Identity()) > 1e-4)
			return false;
		for (int b = 0; b < v.observation->num_blobs; b++) {
			if (v.excluded_blobs[b])
				continue;
			const blob &blob = v.observation->blobs[b];
			float x, y, z;
			if (!std::isfinite(blob.x) || !std::isfinite(blob.y) ||
			    !camera_model_unproject(v.calib, blob.x, blob.y, &x, &y, &z))
				continue;
			V3 u(x, y, z);
			if (!u.allFinite() || u.norm() < 1e-9 || u.z() <= 0)
				continue;
			prepared.rays.push_back({u.normalized(), unsigned(b)});
		}
		ctx.views.push_back(std::move(prepared));
	}
	/* Stable physical camera order; input permutation must not affect truncation or tie breaks. */
	std::sort(ctx.views.begin(), ctx.views.end(),
	          [](const View &a, const View &b) { return a.input->camera_index < b.input->camera_index; });
	ctx.stats.num_camera_diagnostics = ctx.views.size();
	for (unsigned k = 0; k < ctx.views.size(); k++) {
		auto &d = ctx.stats.camera_diagnostics[k];
		d.camera_index = ctx.views[k].input->camera_index;
		d.detected_blobs = ctx.views[k].input->observation->num_blobs;
		d.eligible_blobs = ctx.views[k].rays.size();
	}
	return true;
}

bool
visible(Context &ctx, const Pose &p_cam, unsigned led, V3 &x)
{
	const auto &l = ctx.model->leds[led];
	x = p_cam.q * vec(l.pos) + p_cam.t;
	if (!x.allFinite() || !(x.z() > ctx.cfg.minimum_depth_m) || x.z() > ctx.cfg.maximum_depth_m) {
		ctx.stats.depth_rejections++;
		return false;
	}
	V3 normal = p_cam.q * vec(l.dir).normalized();
	if (normal.dot(-x.normalized()) < std::cos(LED_ANGLE * pi / 180)) {
		ctx.stats.invisible_rejections++;
		return false;
	}
	if (ctx.model->check_led_visibility) {
		V3 origin = p_cam.q.conjugate() * -p_cam.t;
		if (!ctx.model->check_led_visibility(ctx.model, led, xvec(origin)))
			return false;
	}
	return true;
}

bool
score(Context &ctx, Candidate &candidate)
{
	candidate.matches.clear();
	candidate.visible = candidate.candidates = candidate.distinct = 0;
	candidate.sum = 0;
	candidate.camera_visible.fill(0);
	candidate.camera_candidates.fill(0);
	candidate.camera_inliers.fill(0);
	candidate.association_gap = std::numeric_limits<double>::infinity();
	const double broad2 = ctx.cfg.association_gate_m * ctx.cfg.association_gate_m;
	const double gate2 = ctx.cfg.score_gate_m * ctx.cfg.score_gate_m;
	std::array<bool, 256> distinct{};
	for (unsigned k = 0; k < ctx.views.size(); k++) {
		const auto &v = ctx.views[k];
		Pose pc = compose(v.cam_world, candidate.p);
		std::vector<Match> edges;
		for (unsigned l = 0; l < ctx.model->num_leds; l++) {
			V3 x;
			if (!visible(ctx, pc, l, x))
				continue;
			/* Projection is only an in-frame visibility check, never a pixel residual gate. */
			float px, py;
			if (!camera_model_project(v.input->calib, x.x(), x.y(), x.z(), &px, &py) || px < 0 ||
			    py < 0 || px >= v.input->calib->width || py >= v.input->calib->height)
				continue;
			candidate.visible++;
			candidate.camera_visible[k]++;
			for (unsigned b = 0; b < v.rays.size(); b++) {
				double depth = x.dot(v.rays[b].u);
				if (!(depth > 0))
					continue;
				double e = std::max(0.0, x.squaredNorm() - depth * depth);
				if (e <= broad2)
					edges.push_back({k, b, l, e});
			}
		}
		std::sort(edges.begin(), edges.end(), [](const Match &a, const Match &b) {
			if (a.squared != b.squared)
				return a.squared < b.squared;
			if (a.led != b.led)
				return a.led < b.led;
			return a.ray < b.ray;
		});
		std::array<bool, MAX_BLOBS_PER_FRAME> used_blobs{};
		std::array<bool, 256> used_leds{};
		for (const auto &e : edges) {
			if (used_blobs[e.ray] || used_leds[e.led])
				continue;
			used_blobs[e.ray] = used_leds[e.led] = true;
			candidate.candidates++;
			candidate.camera_candidates[k]++;
			candidate.sum += std::min(e.squared, gate2);
			if (e.squared <= gate2) {
				candidate.matches.push_back(e);
				candidate.camera_inliers[k]++;
				if (!distinct[e.led]) {
					distinct[e.led] = true;
					candidate.distinct++;
				}
			}
		}
		/* Competing LED assignments at a fixed pose are also hypotheses. Test unassigned LEDs and
		 * pair swaps without inventing permanent identities from temporal blob labels. */
		for (const auto &m : candidate.matches) {
			if (m.view != k)
				continue;
			for (const auto &e : edges) {
				if (e.ray != m.ray || e.led == m.led || e.squared > gate2)
					continue;
				double difference = e.squared - m.squared;
				if (used_leds[e.led]) {
					bool swapped = false;
					for (const auto &other : candidate.matches) {
						if (other.view != k || other.led != e.led)
							continue;
						for (const auto &back : edges)
							if (back.ray == other.ray && back.led == m.led &&
							    back.squared <= gate2) {
								difference += back.squared - other.squared;
								swapped = true;
								break;
							}
					}
					if (!swapped)
						continue;
				}
				candidate.association_gap =
				    std::min(candidate.association_gap,
				             difference / (ctx.cfg.ray_noise_std_m * ctx.cfg.ray_noise_std_m));
			}
		}
	}
	std::sort(candidate.matches.begin(), candidate.matches.end(), [](const Match &a, const Match &b) {
		if (a.view != b.view)
			return a.view < b.view;
		if (a.ray != b.ray)
			return a.ray < b.ray;
		return a.led < b.led;
	});
	if (candidate.matches.size() <= candidate.sample_size || candidate.candidates <= candidate.sample_size) {
		ctx.stats.insufficient_hypotheses++;
		return false;
	}
	candidate.cost = std::sqrt(candidate.sum / (candidate.candidates - candidate.sample_size));
	return candidate.cost <= ctx.cfg.score_gate_m;
}

unsigned
parameters(const Candidate &c)
{
	/* Heading and retained-pose hypotheses leave yaw to the IMU prior unless the optical fit
	 * independently constrains it (see refine). */
	return c.yaw_free ? 4 : 3;
}

//! Where the hypothesis' reference gravity now points in the world; +Y when the tilt is unchanged.
V3
tilted_gravity(const Candidate &c)
{
	return (c.p.q * c.model_gravity).normalized();
}

//! Angle between the refined tilt and the tilt the hypothesis was generated with.
double
tilt_change(const Candidate &c)
{
	return std::acos(std::clamp(tilted_gravity(c).y(), -1.0, 1.0));
}

/* Too few distinct LEDs hardly observe tilt, and freeing it there mostly lets the other hand's
 * mirrored ring fit a small subset of this one. */
bool
tilt_free(const Context &ctx, const Candidate &c)
{
	return ctx.cfg.tilt_prior_std_rad > 0 && ctx.cfg.maximum_tilt_correction_rad > 0 &&
	       c.distinct >= ctx.cfg.minimum_tilt_distinct_leds;
}

/* Parameters: translation, yaw about world gravity, and tilt about world X and Z. Tilt starts at the
 * IMU's (or P3P's) value and is held there by a prior of tilt_prior_std_rad, expressed as a
 * pseudo-observation with the ray noise so it weighs against the object-space residuals. */
void
normal_equations(Context &ctx, const Candidate &c, M6 &information, V6 &gradient, double &sse)
{
	information.setZero();
	gradient.setZero();
	sse = 0;
	for (const auto &m : c.matches) {
		const auto &v = ctx.views[m.view];
		const V3 &u = v.rays[m.ray].u;
		V3 rp = c.p.q * vec(ctx.model->leds[m.led].pos);
		V3 x = v.cam_world.q * (rp + c.p.t) + v.cam_world.t;
		M3 a = M3::Identity() - u * u.transpose();
		Eigen::Matrix<double, 3, 6> j;
		j.leftCols<3>() = v.cam_world.q.toRotationMatrix();
		j.col(3) = v.cam_world.q * V3::UnitY().cross(rp);
		j.col(4) = v.cam_world.q * V3::UnitX().cross(rp);
		j.col(5) = v.cam_world.q * V3::UnitZ().cross(rp);
		if (parameters(c) == 3)
			j.col(3).setZero();
		j = a * j;
		V3 residual = a * x;
		information += j.transpose() * j;
		gradient += j.transpose() * residual;
		sse += residual.squaredNorm();
	}
	if (!tilt_free(ctx, c)) {
		information(4, 4) = information(5, 5) = 1;
		information.block<4, 2>(0, 4).setZero();
		information.block<2, 4>(4, 0).setZero();
		gradient.tail<2>().setZero();
		return;
	}
	/* Rotating by small angles (ax, az) about world X and Z moves the reference gravity g by
	 * (-az * g.y, ., ax * g.y): the prior residual is g's horizontal part. */
	double w = std::pow(ctx.cfg.ray_noise_std_m / ctx.cfg.tilt_prior_std_rad, 2);
	V3 g = tilted_gravity(c);
	information(4, 4) += w;
	information(5, 5) += w;
	gradient(4) += w * g.z();
	gradient(5) -= w * g.x();
}

//! Rotate a refined pose back onto the tilt bound when an update leaves it.
void
bound_tilt(const Context &ctx, Candidate &c)
{
	double angle = tilt_change(c);
	double excess = angle - ctx.cfg.maximum_tilt_correction_rad;
	if (!(excess > 0))
		return;
	V3 axis = tilted_gravity(c).cross(V3::UnitY());
	if (axis.norm() < 1e-12)
		return;
	c.p.q = (Q(Eigen::AngleAxisd(excess, axis.normalized())) * c.p.q).normalized();
}

/* Gauss-Newton over translation, yaw about world gravity when free, and the bounded tilt. The
 * covariance uses the proposed ray noise floor, or the fit residual when larger. */
bool
refine_with(Context &ctx, Candidate &c, bool yaw)
{
	c.yaw_free = yaw;
	/* Not bounded by the search deadline: at most 24 retained candidates of 6 iterations each.
	 * Stopping here would leave candidates with the same identities at slightly different
	 * unrefined poses, which then look like competitors. */
	for (unsigned iteration = 0; iteration < 6; iteration++) {
		M6 information;
		V6 gradient;
		double sse;
		normal_equations(ctx, c, information, gradient, sse);
		if (parameters(c) == 3)
			information(3, 3) = 1;
		Eigen::ColPivHouseholderQR<M6> solve(information);
		solve.setThreshold(1e-8);
		if (solve.rank() != 6)
			return false;
		V6 change = -solve.solve(gradient);
		if (!change.allFinite() || change.head<3>().norm() > 0.1 || std::abs(change(3)) > 0.4 ||
		    change.tail<2>().norm() > 0.4)
			break;
		Candidate updated = c;
		updated.p.t += change.head<3>();
		updated.p.q =
		    (Q(Eigen::AngleAxisd(change(4), V3::UnitX())) * Q(Eigen::AngleAxisd(change(5), V3::UnitZ())) *
		     Q(Eigen::AngleAxisd(change(3), V3::UnitY())) * c.p.q)
		        .normalized();
		bound_tilt(ctx, updated);
		if (!score(ctx, updated) || updated.matches.size() < c.matches.size() || updated.cost > c.cost + 1e-9)
			break;
		c = std::move(updated);
		if (change.norm() < 1e-7)
			break;
	}
	M6 info;
	V6 gradient;
	double sse;
	normal_equations(ctx, c, info, gradient, sse);
	unsigned dims = parameters(c);
	if (dims == 3)
		info(3, 3) = 1;
	Eigen::ColPivHouseholderQR<M6> solve(info);
	solve.setThreshold(1e-8);
	c.rank = unsigned(solve.rank()) - 2 - (dims == 3 ? 1 : 0);
	if (c.rank != dims)
		return false;
	double variance = std::max(ctx.cfg.ray_noise_std_m * ctx.cfg.ray_noise_std_m,
	                           sse / std::max(1.0, double(2 * c.matches.size()) - dims));
	// Tilt is marginalized: its remaining freedom widens the translation and yaw covariance.
	M6 covariance = variance * solve.solve(M6::Identity());
	c.covariance = covariance.topLeftCorner<4, 4>();
	c.covariance = ((c.covariance + c.covariance.transpose()) * 0.5).eval();
	if (dims == 3)
		c.covariance(3, 3) = std::numeric_limits<double>::infinity();
	return c.covariance.topLeftCorner<3, 3>().allFinite();
}

bool
refine(Context &ctx, Candidate &c)
{
	if (c.origin == CONSTELLATION_CONSTRAINED_GRAVITY || c.origin == CONSTELLATION_CONSTRAINED_P3P)
		return refine_with(ctx, c, true);
	/* A known-heading hypothesis may still let the optical fit measure yaw. Only claim it with
	 * redundant observations and a bounded yaw variance; otherwise keep yaw from the prior. */
	if (c.matches.size() >= 3) {
		Candidate trial = c;
		if (refine_with(ctx, trial, true) && std::sqrt(trial.covariance(3, 3)) <= ctx.cfg.maximum_yaw_std_rad) {
			c = std::move(trial);
			return true;
		}
	}
	return refine_with(ctx, c, false);
}

bool
better(const Candidate &a, const Candidate &b)
{
	if (a.matches.size() != b.matches.size())
		return a.matches.size() > b.matches.size();
	if (a.cost != b.cost)
		return a.cost < b.cost;
	return a.sample_size < b.sample_size;
}
bool
same_assignments(const Candidate &a, const Candidate &b)
{
	if (a.matches.size() != b.matches.size())
		return false;
	for (unsigned i = 0; i < a.matches.size(); i++) {
		const auto &x = a.matches[i], &y = b.matches[i];
		if (x.view != y.view || x.ray != y.ray || x.led != y.led)
			return false;
	}
	return true;
}
bool
same(const Candidate &a, const Candidate &b)
{
	return same_assignments(a, b) && (a.p.t - b.p.t).norm() < 1e-4 && a.p.q.angularDistance(b.p.q) < 1e-3;
}
/* The same identities at nearby poses are one solution reached from different seeds, not an
 * identity ambiguity. Poorly conditioned fits (distant rings, few LEDs) converge to slightly
 * different poses; the covariance and the uncertainty gate already account for that spread. */
bool
same_solution(const Context &ctx, const Candidate &a, const Candidate &b)
{
	return same_assignments(a, b) && (a.p.t - b.p.t).norm() <= ctx.cfg.maximum_position_std_m &&
	       a.p.q.angularDistance(b.p.q) <= ctx.cfg.maximum_yaw_std_rad;
}
void
retain(std::vector<Candidate> &candidates, Candidate c)
{
	for (auto &old : candidates)
		if (same(c, old)) {
			if (better(c, old))
				old = std::move(c);
			return;
		}
	candidates.push_back(std::move(c));
	std::sort(candidates.begin(), candidates.end(), better);
	if (candidates.size() > 24)
		candidates.erase(candidates.begin() + 24, candidates.end());
}

/* Unmatched rays that pass within the conflict gate of an LED this pose predicts visible: the
 * hypothesis says an LED is there yet assigns that blob to nothing. Rays within two LED radii of
 * an accepted LED are fragments of that blob, not independent evidence. */
unsigned
unexplained(Context &ctx, const Candidate &c)
{
	const double conflict = ctx.cfg.conflict_gate_m;
	unsigned count = 0;
	for (unsigned k = 0; k < ctx.views.size(); k++) {
		const auto &v = ctx.views[k];
		Pose pc = compose(v.cam_world, c.p);
		std::vector<V3> predicted, accepted;
		std::vector<double> fragment;
		for (unsigned l = 0; l < ctx.model->num_leds; l++) {
			V3 x;
			if (!visible(ctx, pc, l, x))
				continue;
			predicted.push_back(x);
		}
		std::vector<bool> matched(v.rays.size(), false);
		for (const auto &m : c.matches) {
			if (m.view != k)
				continue;
			matched[m.ray] = true;
			const auto &led = ctx.model->leds[m.led];
			accepted.push_back(pc.q * vec(led.pos) + pc.t);
			fragment.push_back(2 * led.radius_mm * 1e-3);
		}
		auto near = [](const V3 &x, const V3 &u, double gate) {
			double d = x.dot(u);
			return d > 0 && x.squaredNorm() - d * d <= gate * gate;
		};
		for (unsigned b = 0; b < v.rays.size(); b++) {
			if (matched[b])
				continue;
			const V3 &u = v.rays[b].u;
			bool is_fragment = false;
			for (unsigned i = 0; i < accepted.size() && !is_fragment; i++)
				is_fragment = near(accepted[i], u, fragment[i]);
			if (is_fragment)
				continue;
			for (const auto &x : predicted)
				if (near(x, u, conflict)) {
					count++;
					break;
				}
		}
	}
	return count;
}

/* Angular residual chi-square of the accepted observations and its Wilson-Hilferty quantile. */
void
fit_test(Context &ctx, const Candidate &c, double &chi_square, double &limit)
{
	chi_square = 0;
	for (const auto &m : c.matches) {
		const auto &v = ctx.views[m.view];
		V3 x = v.cam_world.q * (c.p.q * vec(ctx.model->leds[m.led].pos) + c.p.t) + v.cam_world.t;
		double depth = std::max(x.dot(v.rays[m.ray].u), 1e-6);
		double angle = std::sqrt(m.squared) / depth / ctx.cfg.bearing_noise_std_rad;
		chi_square += angle * angle;
	}
	// The tilt prior's pseudo-observations pay for the two tilt parameters.
	if (tilt_free(ctx, c))
		chi_square += std::pow(tilt_change(c) / ctx.cfg.tilt_prior_std_rad, 2);
	double dof = std::max(1.0, 2.0 * c.matches.size() - parameters(c));
	double h = 2 / (9 * dof);
	limit = dof * std::pow(1 - h + ctx.cfg.cold_fit_quantile_z * std::sqrt(h), 3);
}

bool
finish(Context &ctx, std::vector<Candidate> &candidates)
{
	if (candidates.empty()) {
		ctx.stats.rejection = ctx.stats.hypotheses ? CONSTELLATION_CONSTRAINED_TOO_FEW_INLIERS
		                                           : CONSTELLATION_CONSTRAINED_NO_PAIR;
		return false;
	}
	std::vector<Candidate> fitted;
	bool refined = false;
	for (auto &c : candidates) {
		if (!refine(ctx, c))
			continue;
		refined = true;
		if (!outside_prior_gate(ctx, c))
			retain(fitted, std::move(c));
	}
	if (fitted.empty()) {
		// Refined fits that all drifted away from a trusted prior are not near it at all.
		ctx.stats.rejection =
		    refined ? CONSTELLATION_CONSTRAINED_NO_PAIR : CONSTELLATION_CONSTRAINED_RANK_DEFICIENT;
		return false;
	}
	Candidate &best = fitted.front();
	ctx.stats.unexplained_blobs = unexplained(ctx, best);
	bool fit_rejected = false;
	if (ctx.cfg.bearing_noise_std_rad > 0) {
		fit_test(ctx, best, ctx.stats.fit_chi_square, ctx.stats.fit_chi_square_limit);
		fit_rejected = ctx.stats.fit_chi_square > ctx.stats.fit_chi_square_limit;
	}
	unsigned minimum_inliers = ctx.cold ? ctx.cfg.minimum_cold_inliers : ctx.cfg.minimum_tracking_inliers;
	unsigned minimum_distinct =
	    ctx.cold ? ctx.cfg.minimum_cold_distinct_leds : ctx.cfg.minimum_tracking_distinct_leds;
	bool cold_count = best.matches.size() < minimum_inliers || best.distinct < minimum_distinct;
	bool cold_rejected = fit_rejected || ctx.stats.unexplained_blobs > ctx.cfg.maximum_cold_unexplained_blobs;
	ctx.stats.P_world_model = pose(best.p.q, best.p.t);
	ctx.stats.origin = best.origin;
	ctx.stats.cost_m = best.cost;
	ctx.stats.inliers = best.matches.size();
	ctx.stats.distinct_leds = best.distinct;
	ctx.stats.candidate_observations = best.candidates;
	ctx.stats.projected_visible_leds = best.visible;
	ctx.stats.information_rank = best.rank;
	ctx.stats.vision_determines_yaw = parameters(best) == 4;
	ctx.stats.tilt_correction_rad = tilt_change(best);
	for (unsigned r = 0; r < 4; r++)
		for (unsigned c = 0; c < 4; c++)
			ctx.stats.covariance[4 * r + c] = best.covariance(r, c);
	ctx.stats.num_assignments = best.matches.size();
	for (unsigned k = 0; k < ctx.views.size(); k++) {
		auto &d = ctx.stats.camera_diagnostics[k];
		d.projected_visible_leds = best.camera_visible[k];
		d.candidate_observations = best.camera_candidates[k];
		d.inliers = best.camera_inliers[k];
	}
	for (unsigned i = 0; i < best.matches.size(); i++) {
		const auto &m = best.matches[i];
		const auto &v = ctx.views[m.view];
		ctx.stats.assignments[i] = {v.input->camera_index, v.rays[m.ray].blob_index, m.led,
		                            ctx.model->leds[m.led].id, std::sqrt(m.squared)};
	}
	/* Consistency first: a count rejection then means the fit itself was sound, which
	 * arbitration relies on when weighing rejected hypotheses. */
	if (cold_rejected) {
		ctx.stats.rejection = CONSTELLATION_CONSTRAINED_INCONSISTENT;
		return false;
	}
	if (cold_count) {
		ctx.stats.rejection = CONSTELLATION_CONSTRAINED_TOO_FEW_INLIERS;
		return false;
	}
	Eigen::SelfAdjointEigenSolver<M3> position(best.covariance.topLeftCorner<3, 3>());
	if (std::sqrt(position.eigenvalues().maxCoeff()) > ctx.cfg.maximum_position_std_m ||
	    (parameters(best) == 4 && std::sqrt(best.covariance(3, 3)) > ctx.cfg.maximum_yaw_std_rad)) {
		ctx.stats.rejection = CONSTELLATION_CONSTRAINED_UNCERTAIN;
		return false;
	}
	ctx.stats.second_best_cost_m = std::numeric_limits<double>::infinity();
	ctx.stats.ambiguity_likelihood_gap = best.association_gap;
	if (std::isfinite(best.association_gap)) {
		ctx.stats.second_best_cost_m = std::sqrt(
		    std::max(0.0, best.sum + best.association_gap * ctx.cfg.ray_noise_std_m * ctx.cfg.ray_noise_std_m) /
		    (best.candidates - best.sample_size));
		if (best.association_gap <= ctx.cfg.ambiguity_chi_square) {
			ctx.stats.rejection = CONSTELLATION_CONSTRAINED_AMBIGUOUS;
			return false;
		}
	}
	for (unsigned i = 1; i < fitted.size(); i++) {
		const auto &other = fitted[i];
		if (same_solution(ctx, best, other))
			continue;
		/* A competitor must explain all the same observations. Extra observations are evidence, not missing-LED
		 * penalties. */
		bool covers = true;
		for (const auto &b : best.matches) {
			bool found = false;
			for (const auto &o : other.matches)
				if (b.view == o.view && b.ray == o.ray) {
					found = true;
					break;
				}
			if (!found) {
				covers = false;
				break;
			}
		}
		if (!covers)
			continue;
		double best_sse = 0, other_sse = 0;
		for (const auto &b : best.matches) {
			best_sse += b.squared;
			for (const auto &o : other.matches)
				if (b.view == o.view && b.ray == o.ray) {
					other_sse += o.squared;
					break;
				}
		}
		double gap = (other_sse - best_sse) / (ctx.cfg.ray_noise_std_m * ctx.cfg.ray_noise_std_m);
		if (gap < ctx.stats.ambiguity_likelihood_gap) {
			ctx.stats.ambiguity_likelihood_gap = gap;
			ctx.stats.second_best_cost_m = other.cost;
		}
		if (gap <= ctx.cfg.ambiguity_chi_square) {
			ctx.stats.rejection = CONSTELLATION_CONSTRAINED_AMBIGUOUS;
			return false;
		}
	}
	ctx.stats.rejection = CONSTELLATION_CONSTRAINED_ACCEPTED;
	return true;
}

/* One minimal sample: two LED/blob correspondences in the same calibrated view. */
struct PairSample
{
	unsigned view;
	unsigned la, lb, ba, bb;
};

/* Generate gravity-constrained and, if available, heading-constrained hypotheses from one pair,
 * require the generating pair to survive visibility and the tight gate, then score jointly. */
void
try_pair(Context &ctx, const PairSample &s, const Pose &prior, bool heading, std::vector<Candidate> &candidates)
{
	const auto &v = ctx.views[s.view];
	const auto &cfg = ctx.cfg;
	xrt_vec3 p[2] = {ctx.model->leds[s.la].pos, ctx.model->leds[s.lb].pos};
	xrt_vec3 u[2] = {xvec(v.rays[s.ba].u), xvec(v.rays[s.bb].u)};
	Q qc = v.cam_world.q * prior.q;
	xrt_quat tilt = pose(qc, V3::Zero()).orientation;
	struct Root
	{
		xrt_pose p;
		constellation_constrained_origin origin;
	};
	std::array<Root, 3> roots;
	unsigned count = 0;

	xrt_pose gravity_roots[2];
	xrt_vec3 g = xvec(v.cam_world.q * V3::UnitY());
	enum constellation_constrained_solver_status status;
	unsigned n = constellation_constrained_gravity_two_point(p, u, &tilt, &g, cfg.minimum_bearing_separation_rad,
	                                                         gravity_roots, &status);
	switch (status) {
	case CONSTELLATION_SOLVER_OK: break;
	case CONSTELLATION_SOLVER_UNRESOLVED_YAW: ctx.stats.unresolved_yaw_pairs++; break;
	case CONSTELLATION_SOLVER_NO_ROOT: ctx.stats.no_root_pairs++; break;
	case CONSTELLATION_SOLVER_NONPOSITIVE_DEPTH: ctx.stats.depth_pairs++; break;
	default: ctx.stats.degenerate_pairs++; break;
	}
	for (unsigned i = 0; i < n; i++)
		roots[count++] = {gravity_roots[i], CONSTELLATION_CONSTRAINED_GRAVITY};
	ctx.stats.gravity_hypotheses += n;
	if (heading) {
		xrt_pose heading_root;
		if (constellation_constrained_heading_two_point(p, u, &tilt, cfg.minimum_bearing_separation_rad,
		                                                &heading_root, nullptr)) {
			roots[count++] = {heading_root, CONSTELLATION_CONSTRAINED_HEADING};
			ctx.stats.heading_hypotheses++;
		}
	}
	for (unsigned i = 0; i < count; i++) {
		ctx.stats.hypotheses++;
		Pose cam_model = from_pose(roots[i].p);
		bool seeded = true;
		for (const auto &pair : {std::pair<unsigned, unsigned>(s.la, s.ba), {s.lb, s.bb}}) {
			V3 x;
			if (!visible(ctx, cam_model, pair.first, x)) {
				seeded = false;
				break;
			}
			double d = x.dot(v.rays[pair.second].u);
			if (x.squaredNorm() - d * d > cfg.score_gate_m * cfg.score_gate_m) {
				seeded = false;
				break;
			}
		}
		if (!seeded) {
			ctx.stats.seed_rejections++;
			continue;
		}
		Candidate c{compose(v.world_cam, cam_model), roots[i].origin, 2};
		if (!outside_prior_gate(ctx, c) && score(ctx, c))
			retain(candidates, std::move(c));
	}
}

/* Pairs of candidate edges near a trusted prediction, tightest edges first. Temporal blob labels
 * are never consulted; edges come from the broad physical association gate. */
std::vector<PairSample>
prior_pairs(Context &ctx, const Pose &prior)
{
	struct Edge
	{
		unsigned led, ray;
		double squared;
	};
	struct Scored
	{
		PairSample s;
		double key;
	};
	std::vector<Scored> scored;
	const double broad2 = ctx.cfg.association_gate_m * ctx.cfg.association_gate_m;
	for (unsigned k = 0; k < ctx.views.size(); k++) {
		const auto &v = ctx.views[k];
		Pose pc = compose(v.cam_world, prior);
		std::vector<Edge> edges;
		for (unsigned l = 0; l < ctx.model->num_leds; l++) {
			V3 x = pc.q * vec(ctx.model->leds[l].pos) + pc.t;
			if (!(x.z() > 0))
				continue;
			for (unsigned b = 0; b < v.rays.size(); b++) {
				double d = x.dot(v.rays[b].u);
				double e = std::max(0.0, x.squaredNorm() - d * d);
				if (d > 0 && e <= broad2)
					edges.push_back({l, b, e});
			}
		}
		for (unsigned i = 0; i < edges.size(); i++)
			for (unsigned j = i + 1; j < edges.size(); j++) {
				const auto &a = edges[i], &b = edges[j];
				if (a.led == b.led || a.ray == b.ray)
					continue;
				scored.push_back({{k, a.led, b.led, a.ray, b.ray}, std::sqrt(a.squared) + std::sqrt(b.squared)});
			}
	}
	std::stable_sort(scored.begin(), scored.end(), [](const Scored &a, const Scored &b) { return a.key < b.key; });
	std::vector<PairSample> out;
	out.reserve(scored.size());
	for (const auto &s : scored)
		out.push_back(s.s);
	return out;
}

} // namespace

void
constellation_constrained_default_config(constellation_constrained_config *c)
{
	/* Score/association gates and the angular separation follow the stock Windows defaults.
	 * Everything else is Monado policy chosen with the synthetic
	 * partial-ring tests in tests_constellation_constrained: object-space ray noise 1 mm,
	 * position std 3 cm, yaw std 20 degrees, ambiguity gap chi-square 9, at least 4 observations
	 * of 3 distinct LEDs (cold and tracking), at most one unmatched blob within 8 mm (twice the
	 * score gate) of a predicted visible LED, bearing noise 0.0026 rad (about 0.5 px at the Rift S
	 * focal length of ~190 px) tested at the 99.9% chi-square quantile, a tilt adjustment of
	 * 3 degrees std bounded at 6 degrees on fits of at least 4 distinct LEDs, and the trial/time
	 * caps. */
	*c = {};
	c->score_gate_m = 0.004;
	c->association_gate_m = 0.15;
	c->maximum_prior_distance_m = 0.15;
	c->maximum_prior_rotation_rad = 30 * pi / 180;
	c->tilt_prior_std_rad = 3 * pi / 180;
	c->maximum_tilt_correction_rad = 6 * pi / 180;
	c->minimum_tilt_distinct_leds = 4;
	c->minimum_bearing_separation_rad = 0.1 * pi / 180;
	c->minimum_depth_m = 0.0;
	c->maximum_depth_m = 1.75;
	c->ray_noise_std_m = 0.001;
	c->maximum_position_std_m = 0.03;
	c->maximum_yaw_std_rad = 0.35;
	c->ambiguity_chi_square = 9.0;
	c->minimum_cold_inliers = 4;
	c->minimum_cold_distinct_leds = 3;
	c->minimum_tracking_inliers = 4;
	c->minimum_tracking_distinct_leds = 3;
	c->maximum_cold_unexplained_blobs = 1;
	c->conflict_gate_m = 0.008;
	c->bearing_noise_std_rad = 0.0026;
	c->cold_fit_quantile_z = 3.09;
	c->max_pair_trials = 20000;
	c->max_prior_pair_trials = 1000;
	c->time_budget_us = 25000;
	c->seed = 0;
}

const char *
constellation_constrained_origin_name(enum constellation_constrained_origin origin)
{
	switch (origin) {
	case CONSTELLATION_CONSTRAINED_GRAVITY: return "gravity_p2p";
	case CONSTELLATION_CONSTRAINED_HEADING: return "heading_translation";
	case CONSTELLATION_CONSTRAINED_PRIOR: return "retained_prior";
	case CONSTELLATION_CONSTRAINED_P3P: return "p3p";
	}
	return "unknown";
}

const char *
constellation_constrained_rejection_name(enum constellation_constrained_rejection rejection)
{
	switch (rejection) {
	case CONSTELLATION_CONSTRAINED_ACCEPTED: return "none";
	case CONSTELLATION_CONSTRAINED_INVALID_INPUT: return "invalid_input";
	case CONSTELLATION_CONSTRAINED_NO_PAIR: return "no_pair";
	case CONSTELLATION_CONSTRAINED_TOO_FEW_INLIERS: return "verification_count";
	case CONSTELLATION_CONSTRAINED_RANK_DEFICIENT: return "rank";
	case CONSTELLATION_CONSTRAINED_UNCERTAIN: return "uncertainty";
	case CONSTELLATION_CONSTRAINED_AMBIGUOUS: return "ambiguity";
	case CONSTELLATION_CONSTRAINED_SHARED_BLOB: return "shared_blob";
	case CONSTELLATION_CONSTRAINED_INCONSISTENT: return "inconsistent";
	}
	return "unknown";
}

const char *
constellation_constrained_solver_status_name(enum constellation_constrained_solver_status status)
{
	switch (status) {
	case CONSTELLATION_SOLVER_OK: return "ok";
	case CONSTELLATION_SOLVER_INVALID_INPUT: return "invalid_input";
	case CONSTELLATION_SOLVER_DEGENERATE_BEARINGS: return "degenerate_bearings";
	case CONSTELLATION_SOLVER_DEGENERATE_MODEL: return "degenerate_model";
	case CONSTELLATION_SOLVER_UNRESOLVED_YAW: return "unresolved_yaw";
	case CONSTELLATION_SOLVER_NO_ROOT: return "no_root";
	case CONSTELLATION_SOLVER_NONPOSITIVE_DEPTH: return "nonpositive_depth";
	}
	return "unknown";
}

bool
constellation_constrained_heading_two_point(const xrt_vec3 points[2],
                                            const xrt_vec3 bearings[2],
                                            const xrt_quat *orientation,
                                            double min_angle,
                                            xrt_pose *out,
                                            enum constellation_constrained_solver_status *status)
{
	set_status(status, CONSTELLATION_SOLVER_INVALID_INPUT);
	V3 a, b;
	if (!orientation || !out || !points || !bearings)
		return false;
	enum constellation_constrained_solver_status pair = valid_pair(bearings, min_angle, a, b);
	if (pair != CONSTELLATION_SOLVER_OK) {
		set_status(status, pair);
		return false;
	}
	Q q = quat(*orientation);
	V3 p[2] = {vec(points[0]), vec(points[1])};
	if (!q.coeffs().allFinite() || !p[0].allFinite() || !p[1].allFinite())
		return false;
	if ((p[1] - p[0]).norm() < 1e-6) {
		set_status(status, CONSTELLATION_SOLVER_DEGENERATE_MODEL);
		return false;
	}
	V3 t;
	const V3 u[2] = {a, b};
	if (!translation(p, u, q, t)) {
		set_status(status, CONSTELLATION_SOLVER_NONPOSITIVE_DEPTH);
		return false;
	}
	*out = pose(q, t);
	set_status(status, CONSTELLATION_SOLVER_OK);
	return true;
}

unsigned
constellation_constrained_gravity_two_point(const xrt_vec3 points[2],
                                            const xrt_vec3 bearings[2],
                                            const xrt_quat *tilt,
                                            const xrt_vec3 *gravity,
                                            double min_angle,
                                            xrt_pose out[2],
                                            enum constellation_constrained_solver_status *status)
{
	set_status(status, CONSTELLATION_SOLVER_INVALID_INPUT);
	V3 a, b;
	if (!tilt || !gravity || !out || !points || !bearings)
		return 0;
	enum constellation_constrained_solver_status pair = valid_pair(bearings, min_angle, a, b);
	if (pair != CONSTELLATION_SOLVER_OK) {
		set_status(status, pair);
		return 0;
	}
	V3 g = vec(*gravity);
	if (!g.allFinite() || g.norm() < 1e-9)
		return 0;
	g.normalize();
	Q q0 = quat(*tilt);
	V3 p[2] = {vec(points[0]), vec(points[1])};
	V3 d = q0 * (p[1] - p[0]);
	if (!d.allFinite() || !q0.coeffs().allFinite())
		return 0;
	if (d.norm() < 1e-6) {
		set_status(status, CONSTELLATION_SOLVER_DEGENERATE_MODEL);
		return 0;
	}
	/* R(yaw)*d must lie in the plane of the two bearings: A cos + B sin + C = 0. */
	V3 n = a.cross(b).normalized(), parallel = g * d.dot(g), perpendicular = d - parallel;
	double A = n.dot(perpendicular), B = n.dot(g.cross(perpendicular)), C = n.dot(parallel);
	double amplitude = std::hypot(A, B);
	/* Relative to the model baseline: below this the pair does not observe heading. */
	const double eps = 1e-6 * d.norm();
	if (amplitude < eps) {
		/* Never turn an arbitrary heading into tracked orientation. */
		set_status(status, std::abs(C) < eps ? CONSTELLATION_SOLVER_UNRESOLVED_YAW : CONSTELLATION_SOLVER_NO_ROOT);
		return 0;
	}
	if (std::abs(C) > amplitude * (1 + 1e-12)) {
		set_status(status, CONSTELLATION_SOLVER_NO_ROOT);
		return 0;
	}
	double phase = std::atan2(B, A), offset = std::acos(std::clamp(-C / amplitude, -1.0, 1.0));
	const V3 u[2] = {a, b};
	unsigned count = 0;
	for (double yaw : {phase + offset, phase - offset}) {
		Q q = Q(Eigen::AngleAxisd(yaw, g)) * q0;
		V3 t;
		if (!translation(p, u, q, t))
			continue;
		if (count && q.angularDistance(quat(out[0].orientation)) < 1e-7)
			continue;
		out[count++] = pose(q, t);
	}
	set_status(status, count ? CONSTELLATION_SOLVER_OK : CONSTELLATION_SOLVER_NONPOSITIVE_DEPTH);
	return count;
}

bool
constellation_constrained_search(t_constellation_led_model *model,
                                 const constellation_constrained_view *views,
                                 unsigned count,
                                 const xrt_pose *prior,
                                 bool heading,
                                 bool trusted,
                                 const constellation_constrained_config *cfg,
                                 constellation_constrained_result *out)
{
	if (!out)
		return false;
	*out = {};
	out->rejection = CONSTELLATION_CONSTRAINED_INVALID_INPUT;
	if (!cfg || !prior || !valid_pose(*prior))
		return false;
	Context ctx{model, *cfg, {}, *out, Clock::now() + std::chrono::microseconds(cfg->time_budget_us)};
	if (!prepare(ctx, views, count))
		return false;
	Pose prior_pose = from_pose(*prior);
	ctx.cold = !trusted;
	ctx.gated = trusted && cfg->maximum_prior_distance_m > 0;
	ctx.prior_position = prior_pose.t;
	ctx.heading_gated = trusted && heading && cfg->maximum_prior_rotation_rad > 0;
	ctx.prior_rotation = prior_pose.q;
	std::vector<Candidate> candidates;
	if (trusted && heading) {
		/* The retained prior is a one-point hypothesis; it still needs independent observations. */
		Candidate prior_candidate{prior_pose, CONSTELLATION_CONSTRAINED_PRIOR, 1};
		out->hypotheses++;
		if (score(ctx, prior_candidate))
			retain(candidates, prior_candidate);
	}

	bool complete = true, prior_truncated = false;
	if (trusted && heading) {
		/* A reliable heading makes the prior's predicted LED positions meaningful, so the
		 * tightest predicted edges go first. With a ring in view the list easily exceeds the trial
		 * cap and the fast-path budget (thousands of pairs); requiring it exhausted rejected almost
		 * every tracked frame (r18). Stop at the cap and judge the candidates found: the
		 * continuing identity is tried first, and identity jumps still meet the ambiguity test,
		 * the prior gates and the track's IMU rotation check. */
		std::vector<PairSample> pairs = prior_pairs(ctx, prior_pose);
		for (const auto &s : pairs) {
			if (out->pair_trials >= cfg->max_prior_pair_trials || Clock::now() >= ctx.deadline) {
				prior_truncated = true;
				break;
			}
			out->pair_trials++;
			try_pair(ctx, s, prior_pose, heading, candidates);
		}
	}
	if (Clock::now() < ctx.deadline) {
		/* Ordered model pairs that a single camera can see together, nearest first. This
		 * ordering does not depend on the prior's yaw. With a trusted prior only blobs whose
		 * rays pass near the prior position take part, and the prior gate bounds the result. */
		complete = true;
		struct LedPair
		{
			unsigned a, b;
			double distance;
		};
		std::vector<LedPair> led_pairs;
		double model_radius = 0;
		const double covisible = std::cos(2 * LED_ANGLE * pi / 180);
		for (unsigned a = 0; a < model->num_leds; a++) {
			model_radius = std::max(model_radius, vec(model->leds[a].pos).norm());
			for (unsigned b = 0; b < model->num_leds; b++) {
				if (a == b)
					continue;
				const auto &la = model->leds[a], &lb = model->leds[b];
				if (vec(la.dir).normalized().dot(vec(lb.dir).normalized()) < covisible)
					continue;
				led_pairs.push_back({a, b, (vec(la.pos) - vec(lb.pos)).norm()});
			}
		}
		std::stable_sort(led_pairs.begin(), led_pairs.end(),
		                 [](const LedPair &x, const LedPair &y) { return x.distance < y.distance; });
		/* Unordered blob pairs per view, nearest first: image neighbours usually are model neighbours. */
		struct BlobPair
		{
			unsigned a, b;
			double angle;
		};
		const double near2 = std::pow(cfg->maximum_prior_distance_m + model_radius, 2);
		std::vector<std::vector<BlobPair>> blob_pairs(ctx.views.size());
		size_t longest = 0;
		for (unsigned k = 0; k < ctx.views.size(); k++) {
			const auto &v = ctx.views[k];
			const auto &rays = v.rays;
			std::vector<bool> near(rays.size(), true);
			if (ctx.gated) {
				V3 x = v.cam_world.q * prior_pose.t + v.cam_world.t;
				for (unsigned a = 0; a < rays.size(); a++) {
					double d = x.dot(rays[a].u);
					near[a] = d > 0 && x.squaredNorm() - d * d <= near2;
				}
			}
			for (unsigned a = 0; a < rays.size(); a++)
				for (unsigned b = a + 1; b < rays.size(); b++)
					if (near[a] && near[b])
						blob_pairs[k].push_back(
						    {a, b, std::acos(std::clamp(rays[a].u.dot(rays[b].u), -1.0, 1.0))});
			std::stable_sort(blob_pairs[k].begin(), blob_pairs[k].end(),
			                 [](const BlobPair &x, const BlobPair &y) { return x.angle < y.angle; });
			longest = std::max(longest, blob_pairs[k].size());
		}
		/* Visit (blob pair rank + model pair rank) diagonals, interleaving cameras, so the trial cap
		 * and time budget are spent on adjacent assignments first in every view. */
		const unsigned cap = out->pair_trials + cfg->max_pair_trials;
		if (!led_pairs.empty() && longest) {
			const size_t diagonals = longest + led_pairs.size() - 1;
			for (size_t sum = 0; sum < diagonals && complete; sum++) {
				for (unsigned k = 0; k < ctx.views.size() && complete; k++) {
					const auto &bp = blob_pairs[k];
					size_t first = sum >= led_pairs.size() ? sum - led_pairs.size() + 1 : 0;
					for (size_t i = first; i <= sum && i < bp.size(); i++) {
						if (out->pair_trials >= cap || Clock::now() >= ctx.deadline) {
							complete = false;
							break;
						}
						const auto &lp = led_pairs[sum - i];
						out->pair_trials++;
						try_pair(ctx, {k, lp.a, lp.b, bp[i].a, bp[i].b}, prior_pose, heading,
						         candidates);
					}
				}
			}
		}
	} else {
		complete = false;
	}
	out->budget_exhausted = !complete || prior_truncated;
	return finish(ctx, candidates);
}

bool
constellation_constrained_verify_pose(t_constellation_led_model *model,
                                      const constellation_constrained_view *views,
                                      unsigned count,
                                      const xrt_pose *p,
                                      constellation_constrained_origin origin,
                                      const constellation_constrained_config *cfg,
                                      constellation_constrained_result *out)
{
	if (!out)
		return false;
	*out = {};
	out->rejection = CONSTELLATION_CONSTRAINED_INVALID_INPUT;
	if (!cfg || !p || !valid_pose(*p))
		return false;
	Context ctx{model, *cfg, {}, *out, Clock::now() + std::chrono::microseconds(cfg->time_budget_us)};
	if (!prepare(ctx, views, count))
		return false;
	/* A hypothesis built from k correspondences explains them by construction: require more. */
	unsigned sample_size = origin == CONSTELLATION_CONSTRAINED_PRIOR ? 1u
	                       : origin == CONSTELLATION_CONSTRAINED_P3P ? 3u
	                                                                 : 2u;
	Candidate c{from_pose(*p), origin, sample_size};
	// P3P is the cold-acquisition fallback, so the same cold verification policy applies.
	ctx.cold = origin == CONSTELLATION_CONSTRAINED_P3P;
	out->hypotheses = 1;
	std::vector<Candidate> candidates;
	if (score(ctx, c))
		candidates.push_back(std::move(c));
	return finish(ctx, candidates);
}

bool
constellation_constrained_results_share_blob(const constellation_constrained_result *a,
                                             const constellation_constrained_result *b)
{
	if (!a || !b)
		return false;
	for (unsigned i = 0; i < a->num_assignments; i++)
		for (unsigned j = 0; j < b->num_assignments; j++) {
			const auto &x = a->assignments[i], &y = b->assignments[j];
			if (x.camera_index == y.camera_index && x.blob_index == y.blob_index)
				return true;
		}
	return false;
}

static bool
clearly_stronger(const constellation_constrained_result &mine,
                 const constellation_constrained_result &other,
                 unsigned margin)
{
	return mine.inliers >= other.inliers + margin && mine.distinct_leds >= other.distinct_leds;
}

static bool
competitive_rejection(const constellation_constrained_result &r)
{
	return r.num_assignments > 0 && (r.rejection == CONSTELLATION_CONSTRAINED_TOO_FEW_INLIERS ||
	                                 r.rejection == CONSTELLATION_CONSTRAINED_AMBIGUOUS ||
	                                 r.rejection == CONSTELLATION_CONSTRAINED_UNCERTAIN);
}

void
constellation_constrained_arbitrate(const constellation_constrained_result *const *results,
                                    const bool *published,
                                    bool *candidates,
                                    enum constellation_constrained_claim *blocked,
                                    unsigned count,
                                    unsigned margin)
{
	constellation_constrained_arbitrate_with_owners(results, published, candidates, blocked, nullptr, count, margin);
}
void
constellation_constrained_arbitrate_with_owners(const constellation_constrained_result *const *results,
                                                const bool *published,
                                                bool *candidates,
                                                enum constellation_constrained_claim *blocked,
                                                unsigned *blockers,
                                                unsigned count,
                                                unsigned margin)
{
	if (!results || !published || !candidates || !blocked)
		return;
	for (unsigned i = 0; i < count; i++) {
		blocked[i] = CONSTELLATION_CLAIM_NONE;
		if (blockers)
			blockers[i] = count;
	}
	/* Decide every candidate against the original claims before clearing any of them, so the
	 * outcome does not depend on device order. */
	for (unsigned a = 0; a < count; a++) {
		if (!candidates[a] || !results[a])
			continue;
		const auto &mine = *results[a];
		for (unsigned b = 0; b < count; b++) {
			if (b == a || !results[b] ||
			    !constellation_constrained_results_share_blob(results[a], results[b]))
				continue;
			const auto &other = *results[b];
			if (published[b]) {
				blocked[a] = CONSTELLATION_CLAIM_PUBLISHED;
				if (blockers)
					blockers[a] = b;
				break; // No stronger reason exists.
			} else if (candidates[b]) {
				if (!clearly_stronger(mine, other, margin)) {
					blocked[a] = CONSTELLATION_CLAIM_CANDIDATE;
					if (blockers)
						blockers[a] = b;
				}
			} else if (blocked[a] == CONSTELLATION_CLAIM_NONE && competitive_rejection(other) &&
			           other.inliers >= mine.inliers) {
				blocked[a] = CONSTELLATION_CLAIM_REJECTED;
				if (blockers)
					blockers[a] = b;
			}
		}
	}
	for (unsigned i = 0; i < count; i++)
		if (blocked[i] != CONSTELLATION_CLAIM_NONE)
			candidates[i] = false;
}

void
constellation_constrained_default_motion_limits(constellation_motion_limits *limits)
{
	// Proposed values: fast hand motion stays below about 3 m/s; 2 cm covers fit noise.
	*limits = {3.0, 0.02, 10 * pi / 180, 300000000};
}

bool
constellation_constrained_motion_consistent(const constellation_track_point *prev,
                                            const constellation_track_point *next,
                                            const constellation_motion_limits *limits,
                                            double *position_excess_m,
                                            double *rotation_error_rad)
{
	double excess = std::numeric_limits<double>::infinity(), rotation = pi;
	bool ok = false;
	if (prev && next && limits && valid_pose(prev->P_world_model) && valid_pose(next->P_world_model) &&
	    next->capture_ns > prev->capture_ns && next->capture_ns - prev->capture_ns <= limits->max_gap_ns) {
		double dt = (next->capture_ns - prev->capture_ns) * 1e-9;
		double moved = (vec(next->P_world_model.position) - vec(prev->P_world_model.position)).norm();
		excess = moved - (limits->position_slack_m + limits->max_speed_mps * dt);
		// Body-frame deltas: a constant offset between the optical and IMU worlds cancels.
		Q optical = quat(prev->P_world_model.orientation).conjugate() * quat(next->P_world_model.orientation);
		Q imu = quat(prev->imu_world).conjugate() * quat(next->imu_world);
		rotation = optical.angularDistance(imu);
		ok = excess <= 0 && rotation <= limits->max_rotation_error_rad;
	}
	if (position_excess_m)
		*position_excess_m = excess;
	if (rotation_error_rad)
		*rotation_error_rad = rotation;
	return ok;
}

void
constellation_track_init(constellation_track *track)
{
	*track = {};
	track->state = CONSTELLATION_TRACK_LOST;
	track->max_jumps = 3;
	constellation_constrained_default_motion_limits(&track->limits);
}

void
constellation_track_reset(constellation_track *track)
{
	track->state = CONSTELLATION_TRACK_LOST;
	track->consecutive_jumps = 0;
	track->have_published = false;
}

bool
constellation_track_expire(constellation_track *track, int64_t capture_ns)
{
	if (track->state == CONSTELLATION_TRACK_LOST || capture_ns - track->ref.capture_ns <= track->limits.max_gap_ns)
		return false;
	track->state = CONSTELLATION_TRACK_LOST;
	track->consecutive_jumps = 0;
	track->have_published = false;
	return true;
}

enum constellation_track_verdict
constellation_track_update(constellation_track *track,
                           const constellation_track_point *next_in,
                           bool orientation_observed,
                           const xrt_quat *ref_imu_world,
                           double *position_excess_m,
                           double *rotation_error_rad)
{
	double excess = std::numeric_limits<double>::infinity(), rotation = pi;
	constellation_track_point next = *next_in;
	if (!orientation_observed)
		next.P_world_model.orientation = next.imu_world;
	enum constellation_track_verdict verdict = CONSTELLATION_TRACK_PUBLISH;
	if (track->state != CONSTELLATION_TRACK_LOST && next.capture_ns <= track->ref.capture_ns) {
		verdict = CONSTELLATION_TRACK_HOLD_STALE;
	} else if (track->state == CONSTELLATION_TRACK_LOST) {
		track->state = CONSTELLATION_TRACK_CONFIRMING;
		verdict = CONSTELLATION_TRACK_HOLD_CONFIRMING;
	} else {
		bool consistent = false;
		if (ref_imu_world) {
			constellation_track_point ref = track->ref;
			ref.imu_world = *ref_imu_world;
			// An optical/IMU heading offset only cancels when both endpoints observe yaw.
			// A source change has no measured optical rotation to compare; position still gates it.
			if (!track->ref_orientation_observed || !orientation_observed) {
				ref.P_world_model.orientation = ref.imu_world;
				next.P_world_model.orientation = next.imu_world;
			}
			consistent = constellation_constrained_motion_consistent(&ref, &next, &track->limits, &excess,
			                                                         &rotation);
			if (orientation_observed)
				next.P_world_model.orientation = next_in->P_world_model.orientation;
		}
		if (track->state == CONSTELLATION_TRACK_CONFIRMING) {
			if (track->have_published && next.capture_ns > track->published.capture_ns &&
			    next.capture_ns - track->published.capture_ns <= track->limits.max_gap_ns) {
				double dt = (next.capture_ns - track->published.capture_ns) * 1e-9;
				double distance =
				    (vec(next.P_world_model.position) - vec(track->published.P_world_model.position))
				        .norm();
				consistent = consistent && distance <= track->limits.position_slack_m +
				                                           track->limits.max_speed_mps * dt;
			}
			if (consistent)
				track->state = CONSTELLATION_TRACK_TRACKING;
			else
				verdict = CONSTELLATION_TRACK_HOLD_UNCONFIRMED;
		} else if (consistent) {
			track->consecutive_jumps = 0;
		} else if (++track->consecutive_jumps >= track->max_jumps) {
			// Either the track or the new evidence is wrong; publish neither until confirmed.
			track->state = CONSTELLATION_TRACK_CONFIRMING;
			track->consecutive_jumps = 0;
			verdict = CONSTELLATION_TRACK_HOLD_RESTART;
		} else {
			verdict = CONSTELLATION_TRACK_HOLD_JUMP;
		}
	}
	if (verdict != CONSTELLATION_TRACK_HOLD_STALE && verdict != CONSTELLATION_TRACK_HOLD_JUMP) {
		track->ref = next;
		track->ref_orientation_observed = orientation_observed;
	}
	if (verdict == CONSTELLATION_TRACK_PUBLISH) {
		track->published = next;
		track->have_published = true;
	}
	if (position_excess_m)
		*position_excess_m = excess;
	if (rotation_error_rad)
		*rotation_error_rad = rotation;
	return verdict;
}

const char *
constellation_track_state_name(enum constellation_track_state state)
{
	switch (state) {
	case CONSTELLATION_TRACK_LOST: return "lost";
	case CONSTELLATION_TRACK_CONFIRMING: return "confirming";
	case CONSTELLATION_TRACK_TRACKING: return "tracking";
	default: return "unknown";
	}
}

const char *
constellation_track_verdict_name(enum constellation_track_verdict verdict)
{
	switch (verdict) {
	case CONSTELLATION_TRACK_PUBLISH: return "publish";
	case CONSTELLATION_TRACK_HOLD_CONFIRMING: return "confirming";
	case CONSTELLATION_TRACK_HOLD_UNCONFIRMED: return "unconfirmed";
	case CONSTELLATION_TRACK_HOLD_JUMP: return "jump";
	case CONSTELLATION_TRACK_HOLD_RESTART: return "jump_restart";
	case CONSTELLATION_TRACK_HOLD_STALE: return "stale_exposure";
	}
	return "unknown";
}

bool
constellation_constrained_contests_published(const constellation_constrained_result *candidate,
                                             const constellation_constrained_result *published,
                                             unsigned margin)
{
	return candidate && published && constellation_constrained_results_share_blob(candidate, published) &&
	       clearly_stronger(*candidate, *published, margin);
}

const char *
constellation_constrained_claim_name(enum constellation_constrained_claim claim)
{
	switch (claim) {
	case CONSTELLATION_CLAIM_NONE: return "none";
	case CONSTELLATION_CLAIM_PUBLISHED: return "published";
	case CONSTELLATION_CLAIM_CANDIDATE: return "candidate";
	case CONSTELLATION_CLAIM_REJECTED: return "rejected";
	}
	return "unknown";
}

bool
constellation_constrained_select_results(const constellation_constrained_result *results,
                                         unsigned count,
                                         const constellation_constrained_config *cfg,
                                         constellation_constrained_result *out)
{
	if (!out)
		return false;
	const constellation_constrained_result *best = nullptr;
	for (unsigned i = 0; results && i < count; i++) {
		const auto &r = results[i];
		if (r.rejection != CONSTELLATION_CONSTRAINED_ACCEPTED)
			continue;
		if (!best || r.inliers > best->inliers || (r.inliers == best->inliers && r.cost_m < best->cost_m))
			best = &r;
	}
	if (!best || !cfg || !(cfg->ray_noise_std_m > 0)) {
		*out = {};
		out->rejection = CONSTELLATION_CONSTRAINED_TOO_FEW_INLIERS;
		return false;
	}
	/* Copy first so out may alias one of the candidate records. */
	const constellation_constrained_result selected = *best;
	bool ambiguous = false;
	double smallest_gap = selected.ambiguity_likelihood_gap;
	double second_cost = selected.second_best_cost_m;
	for (unsigned i = 0; i < count; i++) {
		const auto &other = results[i];
		if (&other == best || other.rejection != CONSTELLATION_CONSTRAINED_ACCEPTED)
			continue;
		bool covers = true, same_assignments = selected.num_assignments == other.num_assignments;
		double best_sse = 0, other_sse = 0;
		for (unsigned b = 0; b < selected.num_assignments; b++) {
			const auto &x = selected.assignments[b];
			bool found = false;
			for (unsigned j = 0; j < other.num_assignments; j++) {
				const auto &y = other.assignments[j];
				if (x.camera_index == y.camera_index && x.blob_index == y.blob_index) {
					found = true;
					same_assignments &= x.led_index == y.led_index;
					best_sse += x.residual_m * x.residual_m;
					other_sse += y.residual_m * y.residual_m;
					break;
				}
			}
			if (!found) {
				covers = false;
				break;
			}
		}
		if (!covers)
			continue;
		if (same_assignments &&
		    (vec(selected.P_world_model.position) - vec(other.P_world_model.position)).norm() < 1e-4 &&
		    quat(selected.P_world_model.orientation).angularDistance(quat(other.P_world_model.orientation)) <
		        1e-3)
			continue;
		double gap = (other_sse - best_sse) / (cfg->ray_noise_std_m * cfg->ray_noise_std_m);
		if (gap < smallest_gap) {
			smallest_gap = gap;
			second_cost = other.cost_m;
		}
		if (gap <= cfg->ambiguity_chi_square)
			ambiguous = true;
	}
	*out = selected;
	out->ambiguity_likelihood_gap = smallest_gap;
	out->second_best_cost_m = second_cost;
	if (ambiguous)
		out->rejection = CONSTELLATION_CONSTRAINED_AMBIGUOUS;
	return !ambiguous;
}
