// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Rejects diverged SLAM poses and supplies the fallback HMD pose.
 *
 * Validate source samples, separately from historical and predicted queries. A lost pose holds
 * position and follows IMU attitude. Ordinary loss retains the map alignment; a backend reset
 * invalidates it. A saved world requires visual relocalization, never a held-head realignment.
 * @ingroup drv_rift_s
 */
#include "rift_s_slam_guard.h"

#include "math/m_api.h"
#include "math/m_vec3.h"

#include <math.h>
#include <string.h>

static const struct xrt_quat identity_quat = {0, 0, 0, 1};

static bool
pose_is_finite(const struct xrt_pose *p)
{
	return isfinite(p->position.x) && isfinite(p->position.y) && isfinite(p->position.z) &&
	       isfinite(p->orientation.x) && isfinite(p->orientation.y) && isfinite(p->orientation.z) &&
	       isfinite(p->orientation.w);
}

bool
rift_s_slam_guard_pose_usable(const struct xrt_pose *p, const struct xrt_quat *fusion)
{
	if (!pose_is_finite(p)) return false;
	float norm = p->orientation.x * p->orientation.x + p->orientation.y * p->orientation.y +
	             p->orientation.z * p->orientation.z + p->orientation.w * p->orientation.w;
	if (!(norm > 0.9f && norm < 1.1f)) return false;
	// Compare gravity in the body frame: world yaw is arbitrary, physical head tilt is not.
	struct xrt_vec3 up = {0, 1, 0}, slam_up, imu_up;
	struct xrt_quat inverse;
	math_quat_invert(&p->orientation, &inverse);
	math_quat_rotate_vec3(&inverse, &up, &slam_up);
	math_quat_invert(fusion, &inverse);
	math_quat_rotate_vec3(&inverse, &up, &imu_up);
	return m_vec3_dot(slam_up, imu_up) >= 0.70710678f;
}

void
rift_s_slam_guard_set_world(struct rift_s_slam_guard *g, const struct xrt_pose *world_from_slam)
{
	g->boundary_confirmed = true;
	g->world_transition = g->world_valid && g->have_good && g->state == RIFT_S_SLAM_GUARD_TRACKING;
	if (g->world_transition) {
		g->transition_from = g->align;
		g->transition_to = *world_from_slam;
		g->transition_start_ns = g->good_ns;
	} else {
		g->align = *world_from_slam;
	}
	g->world_valid = true;
}

static void
apply_align(const struct xrt_pose *align, const struct xrt_pose *raw, struct xrt_pose *out)
{
	struct xrt_pose result;
	math_quat_rotate(&align->orientation, &raw->orientation, &result.orientation);
	math_quat_normalize(&result.orientation);
	math_quat_rotate_vec3(&align->orientation, &raw->position, &result.position);
	result.position = m_vec3_add(result.position, align->position);
	*out = result;
}

//! Checks a step from @p from to @p to against the jump and speed limits.
static enum rift_s_slam_guard_reason
check_step(const struct xrt_vec3 *from, int64_t from_ns, const struct xrt_vec3 *to, int64_t to_ns, float *out_step)
{
	float step = m_vec3_len(m_vec3_sub(*to, *from));
	double dt_s = fabs((double)(to_ns - from_ns)) / 1e9;
	*out_step = step;
	if (step > RIFT_S_SLAM_GUARD_MAX_STEP_M) {
		return RIFT_S_SLAM_GUARD_JUMP;
	}
	if (step > RIFT_S_SLAM_GUARD_STEP_TOLERANCE_M + RIFT_S_SLAM_GUARD_MAX_SPEED_MPS * dt_s) {
		return RIFT_S_SLAM_GUARD_SPEED;
	}
	return RIFT_S_SLAM_GUARD_OK;
}

//! Height outside the room (below the nominal floor or far above it).
static bool
outside_room(const struct xrt_vec3 *p, const struct xrt_vec3 *no_pose_position)
{
	float floor_y = no_pose_position->y - RIFT_S_SLAM_GUARD_EYE_HEIGHT_M;
	return p->y < floor_y - RIFT_S_SLAM_GUARD_BELOW_FLOOR_M || p->y > floor_y + RIFT_S_SLAM_GUARD_ABOVE_FLOOR_M;
}

bool
rift_s_slam_guard_query(const struct rift_s_slam_guard *g, const struct xrt_pose *raw, int64_t query_ns,
                         const struct xrt_quat *fusion, const struct xrt_vec3 *no_pose_position,
                         struct xrt_pose *out)
{
	if (g->state != RIFT_S_SLAM_GUARD_TRACKING || !g->world_valid ||
	    (g->require_relocalization && !g->boundary_confirmed) ||
	    !rift_s_slam_guard_pose_usable(raw, fusion)) return false;
	struct xrt_pose aligned;
	apply_align(&g->align, raw, &aligned);
	float step;
	if (m_vec3_len(aligned.position) > RIFT_S_SLAM_GUARD_MAX_DISTANCE_M ||
	    outside_room(&aligned.position, no_pose_position) ||
	    check_step(&g->good.position, g->good_ns, &aligned.position, query_ns, &step) != RIFT_S_SLAM_GUARD_OK)
		return false;
	*out = aligned;
	return true;
}

static bool
reset_due(const struct rift_s_slam_guard *g, int64_t ts_ns)
{
	if (g->reset_pending) {
		return false;
	}
	return !g->have_reset || (ts_ns - g->last_reset_ns) >= RIFT_S_SLAM_GUARD_RESET_INTERVAL_NS;
}

void
rift_s_slam_guard_init(struct rift_s_slam_guard *g)
{
	memset(g, 0, sizeof(*g));
	g->state = RIFT_S_SLAM_GUARD_ACQUIRING;
	g->good.orientation = identity_quat;
	g->align.orientation = identity_quat;
	g->fallback_yaw = identity_quat;
	g->stable_last.orientation = identity_quat;
}

struct xrt_vec3
rift_s_slam_guard_no_pose_position(float origin_offset_y)
{
	struct xrt_vec3 p = {0, RIFT_S_SLAM_GUARD_EYE_HEIGHT_M - origin_offset_y, 0};
	return p;
}

void
rift_s_slam_guard_yaw_of(const struct xrt_quat *q, struct xrt_quat *out_yaw)
{
	struct xrt_quat yaw = {0, q->y, 0, q->w};
	float len = sqrtf(yaw.y * yaw.y + yaw.w * yaw.w);
	if (!(len > 1e-6f)) {
		*out_yaw = identity_quat;
		return;
	}
	yaw.y /= len;
	yaw.w /= len;
	*out_yaw = yaw;
}

void
rift_s_slam_guard_fallback_pose(const struct rift_s_slam_guard *g,
                                const struct xrt_quat *fusion_orientation,
                                const struct xrt_vec3 *no_pose_position,
                                struct xrt_pose *out_pose)
{
	math_quat_rotate(&g->fallback_yaw, fusion_orientation, &out_pose->orientation);
	float norm = out_pose->orientation.x * out_pose->orientation.x + out_pose->orientation.y * out_pose->orientation.y +
	             out_pose->orientation.z * out_pose->orientation.z + out_pose->orientation.w * out_pose->orientation.w;
	if (isfinite(norm) && norm > 0.5f) math_quat_normalize(&out_pose->orientation);
	else out_pose->orientation = g->have_good ? g->good.orientation : identity_quat;
	out_pose->position = g->have_good ? g->good.position : *no_pose_position;
}

void
rift_s_slam_guard_backend_started(struct rift_s_slam_guard *g, int64_t ts_ns)
{
	g->reset_pending = false;
	g->invalid_since_ns = 0;
	g->have_reset = true;
	g->last_reset_ns = ts_ns;
	g->stable_samples = 0;
}

bool
rift_s_slam_guard_take_reset(struct rift_s_slam_guard *g, int64_t ts_ns)
{
	if (!g->reset_pending) {
		return false;
	}
	g->reset_pending = false;
	g->have_reset = true;
	g->last_reset_ns = ts_ns;
	g->resets++;
	g->world_valid = false;
	g->boundary_confirmed = false;
	g->world_transition = false;
	// Samples queued before the reset belong to the diverged run.
	g->stable_samples = 0;
	return true;
}

static void
enter_diverged(struct rift_s_slam_guard *g,
               int64_t ts_ns,
               const struct xrt_quat *fusion_orientation,
               bool diverged,
               float distance,
               float step)
{
	g->state = RIFT_S_SLAM_GUARD_DIVERGED;
	g->stable_samples = 0;

	// Continue the 3DoF orientation in the heading SLAM last showed.
	struct xrt_quat inv_fusion;
	struct xrt_quat delta;
	math_quat_invert(fusion_orientation, &inv_fusion);
	math_quat_rotate(&g->good.orientation, &inv_fusion, &delta);
	rift_s_slam_guard_yaw_of(&delta, &g->fallback_yaw);

	if (diverged) {
		g->divergences++;
		g->last_reject_distance_m = distance;
		g->last_reject_step_m = step;
		if (reset_due(g, ts_ns)) {
			g->reset_pending = true;
		}
	}
}

void
rift_s_slam_guard_suspend(struct rift_s_slam_guard *g, const struct xrt_quat *fusion_orientation)
{
	if (g->state == RIFT_S_SLAM_GUARD_TRACKING) {
		enter_diverged(g, 0, fusion_orientation, false, 0.0f, 0.0f);
	}
}

//! Feeds a raw sample into the stability run used while acquiring or diverged.
static enum rift_s_slam_guard_reason
settle(struct rift_s_slam_guard *g, bool slam_valid, const struct xrt_pose *raw, int64_t ts_ns)
{
	if (!slam_valid) {
		g->stable_samples = 0;
		if (g->have_good && ts_ns - g->good_ns > RIFT_S_SLAM_GUARD_RESET_INTERVAL_NS && reset_due(g, ts_ns))
			g->reset_pending = true;
		return RIFT_S_SLAM_GUARD_NO_POSE;
	}

	enum rift_s_slam_guard_reason reason = RIFT_S_SLAM_GUARD_OK;
	float step = 0.0f;
	if (!pose_is_finite(raw)) {
		reason = RIFT_S_SLAM_GUARD_NOT_FINITE;
	} else if (m_vec3_len(raw->position) > RIFT_S_SLAM_GUARD_MAX_DISTANCE_M) {
		reason = RIFT_S_SLAM_GUARD_TOO_FAR;
	} else if (g->stable_samples > 0) {
		reason = check_step(&g->stable_last.position, g->stable_last_ns, &raw->position, ts_ns, &step);
	}

	bool far_off = reason == RIFT_S_SLAM_GUARD_NOT_FINITE || reason == RIFT_S_SLAM_GUARD_TOO_FAR;
	bool still_diverging = far_off || (reason != RIFT_S_SLAM_GUARD_OK && g->state == RIFT_S_SLAM_GUARD_DIVERGED);
	if (still_diverging && reset_due(g, ts_ns)) {
		// Retry the re-initialisation at the rate limit.
		g->reset_pending = true;
	}
	if (far_off) {
		g->stable_samples = 0;
		return reason;
	}

	if (reason != RIFT_S_SLAM_GUARD_OK || g->stable_samples == 0) {
		// A jump starts a new run here: a re-initialised SLAM snaps to its new origin.
		g->stable_samples = 0;
		g->stable_start_ns = ts_ns;
	}
	g->stable_samples++;
	g->stable_last = *raw;
	g->stable_last_ns = ts_ns;

	if (reason != RIFT_S_SLAM_GUARD_OK) {
		return reason;
	}
	if (g->stable_samples < RIFT_S_SLAM_GUARD_STABLE_SAMPLES ||
	    (ts_ns - g->stable_start_ns) < RIFT_S_SLAM_GUARD_STABLE_NS) {
		return RIFT_S_SLAM_GUARD_SETTLING;
	}
	return RIFT_S_SLAM_GUARD_OK;
}

bool
rift_s_slam_guard_update(struct rift_s_slam_guard *g,
                         bool slam_valid,
                         const struct xrt_pose *slam_pose,
                         int64_t ts_ns,
                         const struct xrt_quat *fusion_orientation,
                         const struct xrt_vec3 *no_pose_position,
                         struct xrt_pose *out_pose,
                         enum rift_s_slam_guard_reason *out_reason)
{
	if (g->require_relocalization && !g->boundary_confirmed && g->world_valid) {
		// A failed/cancelled setup can restore saved bounds while the fresh frame is still
		// active. Revoke that frame before either source or predicted poses are published.
		g->world_valid = false;
		g->world_transition = false;
		if (g->state == RIFT_S_SLAM_GUARD_TRACKING)
			enter_diverged(g, ts_ns, fusion_orientation, false, 0, 0);
	}
	bool usable = slam_valid && rift_s_slam_guard_pose_usable(slam_pose, fusion_orientation);
	if (usable) {
		g->invalid_since_ns = 0;
	} else {
		if (g->invalid_since_ns == 0)
			g->invalid_since_ns = ts_ns;
		if (ts_ns - g->invalid_since_ns >= RIFT_S_SLAM_GUARD_RESET_INTERVAL_NS && reset_due(g, ts_ns))
			g->reset_pending = true;
	}
	if (slam_valid && pose_is_finite(slam_pose) && !usable) {
		*out_reason = RIFT_S_SLAM_GUARD_TILT;
		if (g->state == RIFT_S_SLAM_GUARD_TRACKING)
			enter_diverged(g, ts_ns, fusion_orientation, true, 0, 0);
		g->stable_samples = 0;
		rift_s_slam_guard_fallback_pose(g, fusion_orientation, no_pose_position, out_pose);
		return false;
	}
	if (g->state == RIFT_S_SLAM_GUARD_TRACKING) {
		// Duplicate/out-of-order reads cannot change the source anchor or trigger a reset.
		if (slam_valid && ts_ns <= g->good_ns) {
			*out_pose = g->good;
			*out_reason = RIFT_S_SLAM_GUARD_OK;
			return true;
		}
		if (!slam_valid || !pose_is_finite(slam_pose)) {
			*out_reason = slam_valid ? RIFT_S_SLAM_GUARD_NOT_FINITE : RIFT_S_SLAM_GUARD_NO_POSE;
			// A lost SLAM pose is not a divergence; a non-finite one is.
			enter_diverged(g, ts_ns, fusion_orientation, slam_valid, NAN, NAN);
			rift_s_slam_guard_fallback_pose(g, fusion_orientation, no_pose_position, out_pose);
			return false;
		}

		if (g->world_transition) {
			// Move the published frame over two seconds. Reframe history too, so this deliberate
			// correction cannot masquerade as a physical head jump in the source guard.
			float t = fminf(1.0f, fmaxf(0.0f, (float)(ts_ns - g->transition_start_ns) / 2e9f));
			struct xrt_pose previous = g->align, inverse, raw_good;
			math_pose_invert(&previous, &inverse);
			math_pose_transform(&inverse, &g->good, &raw_good);
			math_pose_interpolate(&g->transition_from, &g->transition_to, t, &g->align);
			apply_align(&g->align, &raw_good, &g->good);
			g->world_transition = t < 1.0f;
		}
		struct xrt_pose aligned;
		apply_align(&g->align, slam_pose, &aligned);
		float distance = m_vec3_len(aligned.position);
		float step = 0.0f;
		enum rift_s_slam_guard_reason reason =
		    check_step(&g->good.position, g->good_ns, &aligned.position, ts_ns, &step);
		if (distance > RIFT_S_SLAM_GUARD_MAX_DISTANCE_M) {
			reason = RIFT_S_SLAM_GUARD_TOO_FAR;
		} else if (outside_room(&aligned.position, no_pose_position)) {
			reason = RIFT_S_SLAM_GUARD_HEIGHT;
		}

		if (reason == RIFT_S_SLAM_GUARD_OK) {
			g->good = aligned;
			g->good_ns = ts_ns;
			*out_pose = aligned;
			*out_reason = RIFT_S_SLAM_GUARD_OK;
			return true;
		}

		*out_reason = reason;
		enter_diverged(g, ts_ns, fusion_orientation, true, distance, step);
		rift_s_slam_guard_fallback_pose(g, fusion_orientation, no_pose_position, out_pose);
		return false;
	}

	// Acquiring or diverged: publish the fallback until SLAM has been consistent for a while.
	struct xrt_pose fallback;
	rift_s_slam_guard_fallback_pose(g, fusion_orientation, no_pose_position, &fallback);

	if (g->stable_samples > 0 && ts_ns <= g->stable_last_ns) {
		*out_reason = RIFT_S_SLAM_GUARD_SETTLING;
		*out_pose = fallback;
		return false;
	}
	enum rift_s_slam_guard_reason reason = settle(g, slam_valid, slam_pose, ts_ns);
	if (reason != RIFT_S_SLAM_GUARD_OK) {
		*out_reason = reason;
		*out_pose = fallback;
		return false;
	}

	if (!g->world_valid) {
		if (g->require_relocalization && !g->boundary_confirmed) {
			// SteamVR's saved standing origin belongs to the persisted map, not this VIO
			// run's arbitrary yaw/translation. Only a visual match can join those frames.
			*out_reason = RIFT_S_SLAM_GUARD_RELOCALIZING;
			*out_pose = fallback;
			return false;
		}
		// An explicitly new room can start a fresh frame before it has saved landmarks.
		struct xrt_quat inv_raw, delta;
		math_quat_invert(&slam_pose->orientation, &inv_raw);
		math_quat_rotate(&fallback.orientation, &inv_raw, &delta);
		rift_s_slam_guard_yaw_of(&delta, &g->align.orientation);
		struct xrt_vec3 rotated;
		math_quat_rotate_vec3(&g->align.orientation, &slam_pose->position, &rotated);
		g->align.position = m_vec3_sub(fallback.position, rotated);
		g->world_valid = true;
	}
	if (g->state == RIFT_S_SLAM_GUARD_DIVERGED) {
		g->recoveries++;
	}

	g->state = RIFT_S_SLAM_GUARD_TRACKING;
	g->stable_samples = 0;
	apply_align(&g->align, slam_pose, &g->good);
	g->good_ns = ts_ns;
	g->have_good = true;
	*out_pose = g->good;
	*out_reason = RIFT_S_SLAM_GUARD_OK;
	return true;
}

const char *
rift_s_slam_guard_reason_str(enum rift_s_slam_guard_reason reason)
{
	switch (reason) {
	case RIFT_S_SLAM_GUARD_OK: return "ok";
	case RIFT_S_SLAM_GUARD_NO_POSE: return "no_pose";
	case RIFT_S_SLAM_GUARD_NOT_FINITE: return "not_finite";
	case RIFT_S_SLAM_GUARD_TOO_FAR: return "too_far";
	case RIFT_S_SLAM_GUARD_HEIGHT: return "height";
	case RIFT_S_SLAM_GUARD_JUMP: return "jump";
	case RIFT_S_SLAM_GUARD_SPEED: return "speed";
	case RIFT_S_SLAM_GUARD_SETTLING: return "settling";
	case RIFT_S_SLAM_GUARD_TILT: return "gravity_mismatch";
	case RIFT_S_SLAM_GUARD_RELOCALIZING: return "relocalizing";
	}
	return "unknown";
}

enum rift_s_slam_feature_event
rift_s_slam_feature_gate_update(struct rift_s_slam_feature_gate *gate, bool known, int count, int64_t ts_ns)
{
	if (!known || ts_ns <= gate->last_ts_ns) {
		return RIFT_S_SLAM_FEATURES_NO_CHANGE;
	}
	gate->last_ts_ns = ts_ns;
	gate->last_count = count;

	bool low = count < RIFT_S_SLAM_FEATURES_MIN;
	if (low != gate->lost) {
		if (!gate->crossing) {
			gate->crossing = true;
			gate->crossing_since_ns = ts_ns;
		}
		if (ts_ns - gate->crossing_since_ns >= RIFT_S_SLAM_FEATURES_DWELL_NS) {
			gate->lost = low;
			gate->crossing = false;
			return low ? RIFT_S_SLAM_FEATURES_LOST : RIFT_S_SLAM_FEATURES_RETURNED;
		}
	} else {
		gate->crossing = false;
	}
	return RIFT_S_SLAM_FEATURES_NO_CHANGE;
}
