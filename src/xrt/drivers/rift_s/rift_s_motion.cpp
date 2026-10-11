// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "rift_s_motion.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <mutex>
#include <new>

namespace {
struct Vec
{
	double x = 0, y = 0, z = 0;
	Vec
	operator+(Vec b) const
	{
		return {x + b.x, y + b.y, z + b.z};
	}
	Vec
	operator-(Vec b) const
	{
		return {x - b.x, y - b.y, z - b.z};
	}
	Vec
	operator*(double s) const
	{
		return {x * s, y * s, z * s};
	}
	double
	length() const
	{
		return std::sqrt(x * x + y * y + z * z);
	}
	bool
	inside(double t) const
	{
		return -t < x && x < t && -t < y && y < t && -t < z && z < t;
	}
};
static Vec
cross(Vec a, Vec b)
{
	return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
static double
dot(Vec a, Vec b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
static Vec
components(Vec a, Vec b)
{
	return {a.x * b.x, a.y * b.y, a.z * b.z};
}

struct Quat
{
	double x = 0, y = 0, z = 0, w = 1;
	Quat
	normalized() const
	{
		double n = std::sqrt(x * x + y * y + z * z + w * w);
		return {x / n, y / n, z / n, w / n};
	}
	Quat
	inverse() const
	{
		return Quat{-x, -y, -z, w}.normalized();
	}
	Quat
	operator*(Quat b) const
	{
		return Quat{w * b.x + x * b.w + y * b.z - z * b.y, w * b.y - x * b.z + y * b.w + z * b.x,
		            w * b.z + x * b.y - y * b.x + z * b.w, w * b.w - x * b.x - y * b.y - z * b.z}
		    .normalized();
	}
	Vec
	rotate(Vec v) const
	{
		Vec q{x, y, z}, t = cross(q, v) * 2;
		return v + t * w + cross(q, t);
	}
};
static Quat
exp_rotation(Vec v)
{
	double angle = v.length();
	double scale = angle > 1e-12 ? std::sin(angle / 2) / angle : 0.5;
	return Quat{v.x * scale, v.y * scale, v.z * scale, std::cos(angle / 2)}.normalized();
}
static Vec
log_rotation(Quat q)
{
	q = q.normalized();
	double n = Vec{q.x, q.y, q.z}.length();
	if (n * n < 1e-20)
		return Vec{q.x, q.y, q.z} * (2 / q.w);
	double angle = std::abs(q.w) < 1e-10 ? (q.w > 0 ? M_PI : -M_PI) : 2 * std::atan(n / q.w);
	return Vec{q.x, q.y, q.z} * (angle / n);
}
struct Pose
{
	Quat q;
	Vec p;
	Vec
	apply(Vec v) const
	{
		return p + q.rotate(v);
	}
	Pose
	inverse() const
	{
		Quat qi = q.inverse();
		return {qi, qi.rotate(p * -1)};
	}
	Pose
	operator*(Pose b) const
	{
		return {q * b.q, apply(b.p)};
	}
};
static Pose
pose_from(const xrt_pose &p)
{
	return {{p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w},
	        {p.position.x, p.position.y, p.position.z}};
}
static xrt_pose
pose_to(Pose p)
{
	return {{float(p.q.x), float(p.q.y), float(p.q.z), float(p.q.w)}, {float(p.p.x), float(p.p.y), float(p.p.z)}};
}
static Vec
vec_from(xrt_vec3 v)
{
	return {v.x, v.y, v.z};
}
static xrt_vec3
vec_to(Vec v)
{
	return {float(v.x), float(v.y), float(v.z)};
}

struct Sample
{
	double t = -1;
	Pose pose;
	Vec v, a, w;
	std::array<double, 3> variance = {-1, -1, -1};
};
static Sample
sample_from(const rift_s_motion_sample &s)
{
	return {double(s.timestamp_ns) / U_TIME_1S_IN_NS,
	        pose_from(s.pose),
	        vec_from(s.linear_velocity),
	        vec_from(s.linear_acceleration),
	        vec_from(s.angular_velocity),
	        {s.position_variance[0], s.position_variance[1], s.position_variance[2]}};
}
static Pose
predict(const Sample &s, double t)
{
	// Model prediction uses constant rates; acceleration is not a quadratic position term.
	double dt = t - s.t;
	return {s.pose.q * exp_rotation(s.w * dt), s.pose.p + s.v * dt};
}
struct Record
{
	Sample sample, behavior;
	double reliable_t = -1;
	Vec reliable_position, relative_position;
	Quat current_orientation;
	int model = 0;
	void
	store(Sample s, const Pose &head)
	{
		sample = s;
		current_orientation = s.pose.q;
		if (std::all_of(s.variance.begin(), s.variance.end(), [](double v) { return 0 < v && v < 0.001; })) {
			reliable_t = s.t;
			reliable_position = s.pose.p;
			relative_position = head.inverse().apply(s.pose.p);
		}
	}
	bool
	fresh(double epsilon) const
	{
		return sample.t <= reliable_t + epsilon;
	}
	Pose
	source() const
	{
		return behavior.t == -1 ? Pose{current_orientation, reliable_position} : predict(behavior, sample.t);
	}
};

struct PairSample
{
	double t;
	Vec a, b;
};
struct PairWindow
{
	std::deque<PairSample> items;
	bool ready = false;
	size_t allocated = 0;
	void
	push(PairSample s, double seconds, size_t capacity)
	{
		if (allocated == 0)
			allocated = capacity;
		if (items.size() == allocated) {
			if (items.front().t + seconds < items.back().t + 0.0005) {
				items.pop_front();
				ready = ready && items.size() >= 2;
			} else
				allocated += capacity;
		}
		items.push_back(s);
		while (!items.empty() && !(s.t < items.front().t + seconds + 0.0005)) {
			items.pop_front();
			ready = items.size() >= 2;
		}
	}
	std::array<Vec, 5>
	stats() const
	{
		Vec sa, sb, saa, sbb, sab;
		for (auto &s : items) {
			sa = sa + s.a;
			sb = sb + s.b;
			saa = saa + components(s.a, s.a);
			sbb = sbb + components(s.b, s.b);
			sab = sab + components(s.a, s.b);
		}
		double n = double(items.size());
		if (n < 2)
			return {};
		return {sa * (1 / n), sb * (1 / n), (saa - components(sa, sa) * (1 / n)) * (1 / (n - 1)),
		        (sbb - components(sb, sb) * (1 / n)) * (1 / (n - 1)),
		        (sab - components(sa, sb) * (1 / n)) * (1 / (n - 1))};
	}
	void
	reset()
	{
		items.clear();
		ready = false;
	}
};

struct Rest
{
	int state = 0;
	PairWindow history;
	Vec mean_a, mean_b;
	Pose latch;
	bool
	update(const Record &r, Sample &out)
	{
		history.push({r.sample.t, r.sample.w, r.sample.a}, 0.1, 300);
		double factor = state == 3 ? 3 : 1;
		auto s = history.stats();
		bool variance_ok = history.ready && s[2].inside(factor * 0.000076) && s[3].inside(factor * 0.005);
		if (variance_ok) {
			mean_a = s[0];
			mean_b = s[1];
		}
		bool mean_ok =
		    (r.sample.w - mean_a).inside(factor * 0.005) && (r.sample.a - mean_b).inside(factor * 0.065);
		bool fresh = r.fresh(0.01), far = r.relative_position.length() > 0.8;
		int next = state;
		switch (state) {
		case 0:
			if (variance_ok)
				next = 1;
			break;
		case 1:
			if (fresh && far)
				next = 2;
			break;
		case 2:
			if (mean_ok)
				next = 3;
			else if (fresh && !far)
				next = 1;
			break;
		case 3:
			if (!mean_ok)
				next = 0;
			break;
		}
		if (state != 3 && next == 3)
			latch = r.sample.pose;
		state = next;
		if (state != 3)
			return false;
		out = {};
		out.t = r.sample.t;
		out.pose = latch;
		return true;
	}
	void
	reset()
	{
		state = 0;
		history.reset();
		mean_a = {};
		mean_b = {};
	}
};

struct RotationModel
{
	int state = 0;
	Sample output;
	Pose correction;
	Vec anchor;
	double cap = 0.25;
	std::deque<double> ratios;
	bool
	update(const Record &r, const Pose &head, const Pose &torso, bool torso_valid, bool near, Sample &out)
	{
		bool fresh = r.fresh(0.0003);
		double radius = r.relative_position.length(), height = r.reliable_position.y - head.p.y;
		bool in_range = near ? radius < 0.25 : 0.25 < radius && radius < 1.1 && height > -0.6;
		Vec delta = r.reliable_position - output.pose.p;
		double distance = delta.length(), previous_t = output.t;
		// Compute the return rate from the previous cache before replacing that cache.
		Vec return_rate = r.sample.v;
		if (distance > 0)
			return_rate = return_rate + delta * (std::min(std::max(distance * 50, 0.25), cap) / distance);
		int next = state;
		switch (state) {
		case 0:
			if (!torso_valid)
				break;
			next = 1;
			[[fallthrough]];
		case 1:
			if (in_range && (near || fresh))
				next = fresh ? 2 : 3;
			break;
		case 2:
			if (!fresh)
				next = 3;
			else if (!in_range)
				next = 0;
			break;
		case 3:
			if (fresh)
				next = 4;
			break;
		case 4:
			if (distance <= 0.001)
				next = 0;
			break;
		}
		if (next == 3 && state != 3) {
			double seed = 0.2;
			if (!near) {
				if (height < -0.2)
					seed = 0.5;
				else if (!ratios.empty()) {
					seed = 0;
					for (double v : ratios)
						seed += v;
					seed /= ratios.size();
				}
			}
			correction = {exp_rotation({-0.087, 0, 0}), {0, 0.025, near ? 0.2 : -seed}};
			Pose source = r.source();
			anchor = torso.inverse().apply(source.p + source.q.rotate(correction.inverse().p));
			cap = 0.25;
		}
		state = next;
		if (!near && state == 2) {
			double w = r.sample.w.length();
			double ratio = w > 0 ? std::clamp(r.sample.v.length() / w, 0.2, 0.4) : 0.2;
			if (ratios.size() == 500)
				ratios.pop_front();
			ratios.push_back(ratio);
		}
		if (state == 3) {
			output = r.sample;
			output.variance = {-1, -1, -1};
			output.pose.p = torso.p + (r.sample.pose.q * correction.inverse().q).rotate(correction.p) +
			                torso.q.rotate(anchor);
			cap = std::max(cap, 0.2 * r.sample.a.length());
		} else if (state == 4) {
			Vec previous = output.pose.p;
			output = r.sample;
			output.variance = {-1, -1, -1};
			output.v = return_rate;
			output.pose.p = previous + return_rate * (r.sample.t - previous_t);
		}
		if (state != 3 && state != 4)
			return false;
		out = output;
		return true;
	}
	void
	reset()
	{
		state = 0;
	}
};

struct Behind
{
	int state = 0;
	bool
	update(const Record &r, const Pose &head, Sample &out)
	{
		bool fresh = r.fresh(0.0003);
		Vec look = head.q.rotate({0, 0, 1}), delta = r.sample.pose.p - head.p;
		look.y = delta.y = 0;
		if (state == 0 && !fresh) {
			bool behind = dot(look, delta) > 0 && r.reliable_position.y - head.p.y < -0.3 &&
			              (r.reliable_position - head.p).length() < 1.2;
			state = behind ? 1 : 2;
		} else if (state != 0 && fresh)
			state = 0;
		if (state != 1)
			return false;
		out = r.sample;
		out.pose.p = head.apply(r.relative_position);
		return true;
	}
	void
	reset()
	{
		state = 0;
	}
};

struct HandOverHand
{
	int state = 0;
	bool latch = false;
	PairWindow history;
	Sample output;
	Vec other_support, torso_support;
	bool
	update(const Record &p, const Record &q, const Pose &head, const Pose &torso, Sample &out)
	{
		history.push({p.sample.t, p.sample.a, q.sample.a}, 0.1, 100);
		auto s = history.stats();
		bool all_above = false, all_below = false;
		if (history.items.size() >= 2 && s[2].x > 0.004 && s[2].y > 0.004 && s[2].z > 0.004 &&
		    s[3].x >= 0.004 && s[3].y >= 0.004 && s[3].z >= 0.004) {
			Vec corr{s[4].x / std::sqrt(s[2].x * s[3].x), s[4].y / std::sqrt(s[2].y * s[3].y),
			         s[4].z / std::sqrt(s[2].z * s[3].z)};
			all_above = corr.x > 0.25 && corr.y > 0.25 && corr.z > 0.25;
			all_below = corr.x < 0.25 && corr.y < 0.25 && corr.z < 0.25;
		}
		bool lost = !p.fresh(0.05), other_fresh = q.fresh(0.05);
		bool close = (q.sample.pose.inverse() * p.sample.pose).p.length() < 0.2;
		Vec look = head.q.rotate({0, 0, 1});
		Vec dp = p.sample.pose.p - head.p, dq = q.sample.pose.p - head.p;
		dp.y = dq.y = 0;
		bool both_gate = dot(look, dp) > 0 && dot(look, dq) > 0;
		bool mismatch = p.model != 0 && p.model != 3, other_model = q.model != 0;
		if (!other_fresh)
			latch = false;
		int next = state;
		switch (state) {
		case 0:
			if (!other_fresh)
				break;
			next = 1;
			[[fallthrough]];
		case 1:
			latch = true;
			if (!other_fresh)
				next = 0;
			else if (!lost && close && !both_gate)
				next = 2;
			break;
		case 2:
			if (!other_fresh)
				next = 0;
			else if (lost)
				next = all_above ? 3 : 4;
			else if (!close)
				next = 1;
			break;
		case 3:
			if (mismatch || other_model)
				next = 0;
			else if (!lost || (other_fresh && all_below))
				next = 4;
			break;
		case 4:
			if (mismatch || other_model || !lost || !other_fresh)
				next = 0;
			else if (latch && all_above)
				next = 3;
			break;
		}
		if (state != next && (next == 3 || next == 4)) {
			Pose basis = next == 3 ? q.sample.pose : torso;
			Vec support = basis.inverse().apply(state == 3 || state == 4 ? output.pose.p : p.sample.pose.p);
			if (next == 3)
				other_support = support;
			else
				torso_support = support;
		}
		state = next;
		if (state != 3 && state != 4)
			return false;
		output = p.sample;
		output.pose.p = state == 3 ? q.sample.pose.apply(other_support) : torso.apply(torso_support);
		out = output;
		return true;
	}
	void
	reset()
	{
		state = 0;
		history.reset();
	}
};

static bool
project_yaw(Quat q, Quat &yaw)
{
	Vec x = q.rotate({1, 0, 0});
	x.y = 0;
	if (x.length() < 0.1)
		return false;
	yaw = exp_rotation({0, std::atan2(-x.z, x.x), 0});
	return true;
}

struct Torso
{
	bool valid = false;
	Pose output;
	Quat previous_controller, previous_head;
	PairWindow history;
	void
	update(const Sample &s, const Pose &head)
	{
		Vec position = head.apply({0, -0.075, 0.0805});
		Quat controller_yaw, head_yaw;
		if (!project_yaw(head.q, head_yaw) || !project_yaw(s.pose.q, controller_yaw))
			return;
		history.push({s.t,
		              {log_rotation(controller_yaw).y, log_rotation(head_yaw).y,
		               log_rotation(head_yaw.inverse() * controller_yaw).y},
		              {}},
		             0.1, 100);
		if (!history.ready) {
			if (!valid) {
				output = {head_yaw, position};
				valid = true;
			}
			return;
		}
		Vec variance = history.stats()[2];
		Quat result = output.q;
		if (variance.z >= 0.001) {
			if (std::abs(log_rotation(head_yaw.inverse() * result).y) >= 0.523 && variance.x >= 0.0027) {
				double weight = std::abs(variance.y) / (std::abs(variance.x) + std::abs(variance.y));
				Quat target = controller_yaw *
				              exp_rotation(log_rotation(controller_yaw.inverse() * head_yaw) * weight);
				Vec residual = log_rotation(result.inverse() * target);
				result = result * exp_rotation({0, (residual.y > 0 ? 1 : -1) * 0.018 * 0.523, 0});
			}
		} else {
			result = result * (std::abs(variance.x) <= std::abs(variance.y)
			                       ? previous_controller.inverse() * controller_yaw
			                       : previous_head.inverse() * head_yaw);
		}
		// The output is itself projected, just like the source yaw filters.
		Quat projected;
		if (project_yaw(result, projected)) {
			output = {projected, position};
			valid = true;
		}
		previous_controller = controller_yaw;
		previous_head = head_yaw;
	}
};

struct Hand
{
	Record record;
	Torso torso;
	Rest rest;
	RotationModel near, rotation;
	HandOverHand together;
	Behind behind;
	void
	update(const Record &other, const Pose &head, const Pose &body, bool body_valid)
	{
		if (record.reliable_t == -1)
			return;
		Sample estimate;
		int model = 0;
		if (rest.update(record, estimate))
			model = 1;
		else if (near.update(record, head, body, body_valid, true, estimate))
			model = 2;
		else if (other.reliable_t != -1 && together.update(record, other, head, body, estimate))
			model = 3;
		else if (behind.update(record, head, estimate))
			model = 4;
		else if (rotation.update(record, head, body, body_valid, false, estimate))
			model = 5;
		// Lower priorities must acquire again after another model has owned the output.
		if (model != 0) {
			if (model < 2)
				near.reset();
			if (model < 3)
				together.reset();
			if (model < 4)
				behind.reset();
			if (model < 5)
				rotation.reset();
		}
		record.model = model;
		record.behavior = model ? estimate : Sample{};
	}
};

static bool
finite(Vec v)
{
	return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
static bool
finite(Pose p)
{
	double n = p.q.x * p.q.x + p.q.y * p.q.y + p.q.z * p.q.z + p.q.w * p.q.w;
	return finite(p.p) && std::isfinite(n) && n > 1e-12;
}
} // namespace

struct rift_s_motion
{
	std::mutex mutex;
	std::array<Hand, 2> hands;
	Pose body;
	bool body_valid = false;
};

extern "C" struct rift_s_motion *
rift_s_motion_create(void)
{
	return new (std::nothrow) rift_s_motion;
}

extern "C" void
rift_s_motion_destroy(struct rift_s_motion **motion)
{
	if (motion == nullptr)
		return;
	delete *motion;
	*motion = nullptr;
}

extern "C" void
rift_s_motion_reset_hand(struct rift_s_motion *motion, unsigned int hand)
{
	if (motion == nullptr || hand >= 2)
		return;
	std::lock_guard<std::mutex> lock(motion->mutex);
	motion->hands[hand] = Hand{};
	motion->body_valid = motion->hands[1 - hand].torso.valid;
	if (motion->body_valid)
		motion->body = motion->hands[1 - hand].torso.output;
}

extern "C" void
rift_s_motion_update(struct rift_s_motion *motion,
                     unsigned int hand,
                     const struct rift_s_motion_sample *sample,
                     const struct xrt_pose *reference)
{
	if (motion == nullptr || hand >= 2 || sample == nullptr || reference == nullptr || sample->timestamp_ns <= 0)
		return;
	Sample s = sample_from(*sample);
	Pose head = pose_from(*reference);
	if (!finite(s.pose) || !finite(head) || !finite(s.v) || !finite(s.a) || !finite(s.w))
		return;
	s.pose.q = s.pose.q.normalized();
	head.q = head.q.normalized();
	std::lock_guard<std::mutex> lock(motion->mutex);
	Hand &p = motion->hands[hand], &q = motion->hands[1 - hand];
	if (s.t <= p.record.sample.t)
		return;
	p.record.store(s, head);
	p.torso.update(s, head);
	if (p.torso.valid) {
		motion->body = p.torso.output;
		motion->body_valid = true;
		if (q.torso.valid) {
			motion->body.q = motion->body.q *
			                 exp_rotation(log_rotation(motion->body.q.inverse() * q.torso.output.q) * 0.5);
		}
	}
	p.update(q.record, head, motion->body, motion->body_valid);
}

extern "C" enum rift_s_motion_model
rift_s_motion_get(struct rift_s_motion *motion,
                  unsigned int hand,
                  timepoint_ns timestamp_ns,
                  struct xrt_space_relation *out_relation)
{
	if (motion == nullptr || hand >= 2 || out_relation == nullptr)
		return RIFT_S_MOTION_NONE;
	std::lock_guard<std::mutex> lock(motion->mutex);
	const Record &r = motion->hands[hand].record;
	if (r.model == 0)
		return RIFT_S_MOTION_NONE;
	const Sample &s = r.behavior;
	double time = std::clamp(double(timestamp_ns) / U_TIME_1S_IN_NS, s.t, s.t + 1.0);
	*out_relation = {};
	out_relation->pose = pose_to(predict(s, time));
	out_relation->linear_velocity = vec_to(s.v);
	out_relation->angular_velocity = vec_to(s.pose.q.rotate(s.w));
	out_relation->relation_flags = static_cast<xrt_space_relation_flags>(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	    XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
	// Attachment models pass the input matrix through. Rest and rotation models emit -I.
	if (std::all_of(s.variance.begin(), s.variance.end(), [](double v) { return v > 0; })) {
		out_relation->relation_flags = static_cast<xrt_space_relation_flags>(
		    out_relation->relation_flags | XRT_SPACE_RELATION_POSITION_VALID_BIT |
		    XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
	}
	return static_cast<rift_s_motion_model>(r.model);
}
