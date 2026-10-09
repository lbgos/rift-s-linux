// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Controller position/velocity filter driven by IMU acceleration and corrected by optical fixes.
 *
 * Three independent per-axis Kalman filters with state [position, velocity, acceleration bias]
 * and the world-frame IMU acceleration as control input. Optical fixes arrive tens of
 * milliseconds after their capture time; they are fused at capture time and the retained IMU
 * inputs are replayed, as the Windows tracker does for delayed vision.
 * @ingroup drv_rift_s
 */
#include "rift_s_position_filter.h"

#include <math.h>
#include <string.h>

//! Velocity decay time constant while coasting without optical fixes.
#define COAST_VELOCITY_TAU_S 0.5
#define INITIAL_VELOCITY_VARIANCE (0.5 * 0.5)
#define INITIAL_BIAS_VARIANCE (0.5 * 0.5)
//! Longest single integration step; larger gaps are dropped radio packets.
#define MAX_STEP_S 0.05

static struct rift_s_position_filter_input *
input_at(struct rift_s_position_filter *f, unsigned int i)
{
	return &f->history[(f->history_start + i) % RIFT_S_POSITION_FILTER_HISTORY];
}

static void
propagate(const struct rift_s_position_filter *f,
          struct rift_s_position_filter_state *s,
          const struct xrt_vec3 *accel,
          timepoint_ns t)
{
	const bool coasting = f->coasting;
	double dt = (double)(t - s->t) / U_TIME_1S_IN_NS;
	s->t = t;
	if (dt <= 0) {
		return;
	}
	if (dt > MAX_STEP_S) {
		dt = MAX_STEP_S;
	}
	if (coasting && f->track_updates < RIFT_S_POSITION_FILTER_MATURE_UPDATES) {
		// A young track's velocity comes from a handful of fixes, possibly a wrong one: hold.
		for (int axis = 0; axis < 3; axis++) {
			s->x[axis][1] = 0;
		}
		return;
	}
	const double a[3] = {accel->x, accel->y, accel->z};
	const double d = coasting ? exp(-dt / COAST_VELOCITY_TAU_S) : 1.0;
	const double F[3][3] = {{1, dt, -0.5 * dt * dt}, {0, d, -d * dt}, {0, 0, 1}};
	for (int axis = 0; axis < 3; axis++) {
		double *x = s->x[axis];
		double u = a[axis] - x[2];
		x[0] += x[1] * dt + 0.5 * u * dt * dt;
		x[1] = d * (x[1] + u * dt);

		double FP[3][3] = {{0}};
		double (*P)[3] = s->P[axis];
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				for (int k = 0; k < 3; k++)
					FP[i][j] += F[i][k] * P[k][j];
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++) {
				double v = 0;
				for (int k = 0; k < 3; k++)
					v += FP[i][k] * F[j][k];
				P[i][j] = v;
			}
		P[0][0] += f->accel_noise_density * dt * dt * dt / 3;
		P[0][1] += f->accel_noise_density * dt * dt / 2;
		P[1][0] += f->accel_noise_density * dt * dt / 2;
		P[1][1] += f->accel_noise_density * dt;
		P[2][2] += f->bias_random_walk * dt;
	}
}

static void
init_state(struct rift_s_position_filter_state *s, timepoint_ns t, const struct xrt_vec3 *z, const struct xrt_vec3 *r)
{
	const double zz[3] = {z->x, z->y, z->z}, rr[3] = {r->x, r->y, r->z};
	s->t = t;
	memset(s->P, 0, sizeof(s->P));
	for (int axis = 0; axis < 3; axis++) {
		s->x[axis][0] = zz[axis];
		s->x[axis][1] = 0;
		s->x[axis][2] = 0;
		s->P[axis][0][0] = rr[axis];
		s->P[axis][1][1] = INITIAL_VELOCITY_VARIANCE;
		s->P[axis][2][2] = INITIAL_BIAS_VARIANCE;
	}
}

static void
update(struct rift_s_position_filter_state *s, const struct xrt_vec3 *z, const struct xrt_vec3 *r)
{
	const double zz[3] = {z->x, z->y, z->z}, rr[3] = {r->x, r->y, r->z};
	for (int axis = 0; axis < 3; axis++) {
		double *x = s->x[axis];
		double (*P)[3] = s->P[axis];
		double S = P[0][0] + rr[axis];
		double K[3] = {P[0][0] / S, P[1][0] / S, P[2][0] / S};
		double innovation = zz[axis] - x[0];
		for (int i = 0; i < 3; i++)
			x[i] += K[i] * innovation;
		double row0[3] = {P[0][0], P[0][1], P[0][2]};
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				P[i][j] -= K[i] * row0[j];
		for (int i = 0; i < 3; i++)
			for (int j = i + 1; j < 3; j++)
				P[i][j] = P[j][i] = 0.5 * (P[i][j] + P[j][i]);
	}
}

void
rift_s_position_filter_reset(struct rift_s_position_filter *f)
{
	memset(f, 0, sizeof(*f));
	f->accel_noise_density = RIFT_S_POSITION_FILTER_ACCEL_NOISE_DENSITY;
	f->bias_random_walk = RIFT_S_POSITION_FILTER_BIAS_RANDOM_WALK;
}

void
rift_s_position_filter_imu(struct rift_s_position_filter *f, timepoint_ns t, const struct xrt_vec3 *accel_world)
{
	if (!f->initialized || t <= f->state.t) {
		f->last_accel = *accel_world;
		return;
	}
	if (f->frozen) {
		f->state.t = t;
	} else {
		propagate(f, &f->state, accel_world, t);
	}
	f->last_accel = *accel_world;
	if (f->initialized && t - f->last_fix_ns > 120 * U_TIME_1MS_IN_NS) {
		// No measurement can correct acceleration/tilt error while occluded. Bound the
		// displacement from the last optical fix, including before coasting was noticed.
		double delta[3] = {f->state.x[0][0] - f->last_fix_position.x, f->state.x[1][0] - f->last_fix_position.y,
		                   f->state.x[2][0] - f->last_fix_position.z};
		double distance = sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
		if (distance > 0.1) {
			const double base[3] = {f->last_fix_position.x, f->last_fix_position.y, f->last_fix_position.z};
			for (int axis = 0; axis < 3; ++axis) {
				f->state.x[axis][0] = base[axis] + delta[axis] * 0.1 / distance;
				f->state.x[axis][1] = 0;
			}
		}
		if (t - f->last_fix_ns > RIFT_S_POSITION_FILTER_BRIDGE_NS)
			rift_s_position_filter_freeze(f);
	}
	if (f->history_count == RIFT_S_POSITION_FILTER_HISTORY) {
		f->history_start = (f->history_start + 1) % RIFT_S_POSITION_FILTER_HISTORY;
		f->history_count--;
	}
	struct rift_s_position_filter_input *in = input_at(f, f->history_count++);
	in->t = t;
	in->accel = *accel_world;
	in->after = f->state;
}

// Continue from @p s (at or before the IMU input @p first) through the retained inputs.
static void
replay(struct rift_s_position_filter *f, struct rift_s_position_filter_state s, unsigned int first)
{
	for (unsigned int i = first; i < f->history_count; i++) {
		struct rift_s_position_filter_input *in = input_at(f, i);
		propagate(f, &s, &in->accel, in->t);
		in->after = s;
	}
	f->state = s;
}

// Count @p position towards replacing the estimate; true once enough consecutive candidates agree.
static bool
confirm_candidate(struct rift_s_position_filter *f,
                  timepoint_ns capture,
                  const struct xrt_vec3 *position,
                  float reset_distance_m)
{
	bool consistent = false;
	if (f->candidate_count > 0 && capture > f->candidate_t &&
	    capture - f->candidate_t <= RIFT_S_POSITION_FILTER_CONFIRM_GAP_NS) {
		struct xrt_vec3 d = {position->x - f->candidate_position.x, position->y - f->candidate_position.y,
		                     position->z - f->candidate_position.z};
		consistent = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z) <= reset_distance_m;
	}
	f->candidate_count = consistent ? f->candidate_count + 1 : 1;
	f->candidate_position = *position;
	f->candidate_t = capture;
	return f->candidate_count >= RIFT_S_POSITION_FILTER_CONFIRM_FIXES;
}

enum rift_s_position_fix_result
rift_s_position_filter_fix(struct rift_s_position_filter *f,
                           timepoint_ns capture,
                           const struct xrt_vec3 *position,
                           const struct xrt_vec3 *variance,
                           float reset_distance_m)
{
	// First retained input after the capture time.
	unsigned int next = 0;
	while (next < f->history_count && input_at(f, next)->t <= capture) {
		next++;
	}
	// A fix needs the state at or before its capture time; the oldest retained input only has
	// the state after it.
	bool reset = !f->initialized || f->frozen;
	bool stale = f->history_count != 0 ? next == 0 : capture < f->state.t;
	if (f->initialized && stale) {
		f->stale++;
		return RIFT_S_POSITION_FIX_STALE;
	}

	struct rift_s_position_filter_state s;
	if (!reset) {
		s = next > 0 ? input_at(f, next - 1)->after : f->state;
		const struct xrt_vec3 *accel = next < f->history_count ? &input_at(f, next)->accel : &f->last_accel;
		propagate(f, &s, accel, capture);
		struct xrt_vec3 innovation = {(float)(position->x - s.x[0][0]), (float)(position->y - s.x[1][0]),
		                              (float)(position->z - s.x[2][0])};
		float distance =
		    sqrtf(innovation.x * innovation.x + innovation.y * innovation.y + innovation.z * innovation.z);
		if (!(distance <= reset_distance_m)) {
			f->gated++;
			reset = true;
		} else {
			f->innovation_sq_sum += (double)distance * distance;
			f->innovation_count++;
		}
	}
	if (reset && f->initialized && !confirm_candidate(f, capture, position, reset_distance_m)) {
		f->rejected++;
		return RIFT_S_POSITION_FIX_PENDING;
	}
	f->candidate_count = 0;
	if (reset) {
		init_state(&s, capture, position, variance);
		f->resets++;
		f->track_updates = 1;
	} else {
		update(&s, position, variance);
		f->updates++;
		f->track_updates++;
	}
	f->initialized = true;
	f->last_fix_ns = capture;
	f->last_fix_position = *position;
	f->frozen = false;
	f->coasting = false;
	// Inputs before the capture keep their states; later ones are replayed from the fused state.
	replay(f, s, next);
	return reset ? RIFT_S_POSITION_FIX_RESET : RIFT_S_POSITION_FIX_UPDATED;
}

void
rift_s_position_filter_freeze(struct rift_s_position_filter *f)
{
	if (!f->initialized || f->frozen) {
		return;
	}
	f->frozen = true;
	for (int axis = 0; axis < 3; axis++) {
		f->state.x[axis][1] = 0;
	}
}

void
rift_s_position_filter_predict(const struct rift_s_position_filter *f,
                               timepoint_ns t,
                               time_duration_ns max_horizon_ns,
                               struct xrt_vec3 *out_position,
                               struct xrt_vec3 *out_velocity)
{
	time_duration_ns ahead = t - f->state.t;
	if (ahead < 0 || f->frozen || (f->initialized && t - f->last_fix_ns > RIFT_S_POSITION_FILTER_BRIDGE_NS)) {
		ahead = 0;
	} else if (ahead > max_horizon_ns) {
		ahead = max_horizon_ns;
	}
	double dt = (double)ahead / U_TIME_1S_IN_NS;
	const struct rift_s_position_filter_state *s = &f->state;
	out_velocity->x = (float)s->x[0][1];
	out_velocity->y = (float)s->x[1][1];
	out_velocity->z = (float)s->x[2][1];
	out_position->x = (float)(s->x[0][0] + s->x[0][1] * dt);
	out_position->y = (float)(s->x[1][0] + s->x[1][1] * dt);
	out_position->z = (float)(s->x[2][0] + s->x[2][1] * dt);
	if (f->initialized && t - f->last_fix_ns > 120 * U_TIME_1MS_IN_NS) {
		// Queries can run beyond the last IMU sample, including during a radio gap.
		double delta[3] = {out_position->x - f->last_fix_position.x, out_position->y - f->last_fix_position.y,
		                   out_position->z - f->last_fix_position.z};
		double distance = sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
		if (distance > 0.1) {
			out_position->x = (float)(f->last_fix_position.x + delta[0] * 0.1 / distance);
			out_position->y = (float)(f->last_fix_position.y + delta[1] * 0.1 / distance);
			out_position->z = (float)(f->last_fix_position.z + delta[2] * 0.1 / distance);
			*out_velocity = (struct xrt_vec3){0};
		}
		if (t - f->last_fix_ns > RIFT_S_POSITION_FILTER_BRIDGE_NS) {
			*out_velocity = (struct xrt_vec3){0};
		}
	}
}

void
rift_s_tracking_loss_durations(uint32_t vision_updates,
                               uint32_t reliable_updates,
                               time_duration_ns *out_position,
                               time_duration_ns *out_velocity)
{
	double position_s = 0, velocity_s = 0;
	if (reliable_updates > 20) {
		position_s = 3.0;
		velocity_s = 5.0;
	} else if (vision_updates > 10) {
		position_s = 2.0;
		velocity_s = 4.0;
	} else if (vision_updates > 2) {
		position_s = 0.5;
		velocity_s = 0.5;
	} else if (vision_updates >= 1) {
		position_s = 0.25;
		velocity_s = 0.25;
	}
	*out_position = (time_duration_ns)(position_s * U_TIME_1S_IN_NS);
	*out_velocity = (time_duration_ns)(velocity_s * U_TIME_1S_IN_NS);
}
