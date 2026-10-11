// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Controller position/velocity filter driven by IMU acceleration and corrected by optical fixes.
 * @ingroup drv_rift_s
 */
#pragma once

#include "xrt/xrt_defines.h"
#include "util/u_time.h"
#include "tracking/t_constellation_motion.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Defaults chosen on the synthetic cases in tests_rift_s_position_filter (noise at rest, delayed
 * fixes during a hand swing, slowly varying acceleration error); no Meta values were recovered.
 * Continuous white acceleration noise density, (m/s^2)^2 per Hz, and bias random walk,
 * (m/s^2)^2 per second.
 */
#define RIFT_S_POSITION_FILTER_ACCEL_NOISE_DENSITY (0.1 * 0.1)
#define RIFT_S_POSITION_FILTER_BIAS_RANDOM_WALK (0.004)

/*!
 * Mutually consistent fixes needed before a fix that disagrees with the estimate (a gated
 * innovation, or any fix after an IMU clock discontinuity) replaces it. A single wrong
 * constellation fit, e.g. a partial ring seen behind the back, must not move the hand.
 */
#define RIFT_S_POSITION_FILTER_CONFIRM_FIXES 3
//! Longest gap between consecutive candidate fixes of one confirmation.
#define RIFT_S_POSITION_FILTER_CONFIRM_GAP_NS T_CONSTELLATION_TRACK_MAX_GAP_NS
//! IMU inputs retained for replaying after a delayed optical fix: 1 s at 500 Hz.
#define RIFT_S_POSITION_FILTER_HISTORY 512

/*!
 * Per axis state: world position, velocity and a world-frame acceleration bias, which absorbs
 * accelerometer bias and residual tilt error while optical fixes arrive.
 */
struct rift_s_position_filter_state
{
	timepoint_ns t;
	double x[3][3];    //!< [axis][p, v, b]
	double P[3][3][3]; //!< [axis] 3x3 covariance
};

struct rift_s_position_filter_input
{
	timepoint_ns t;
	struct xrt_vec3 accel; //!< World-frame acceleration, gravity removed.
	struct rift_s_position_filter_state after; //!< State after integrating up to t.
};

struct rift_s_position_filter
{
	bool initialized;
	//! IMU clock or device discontinuity: stop propagation until a confirmed optical reset.
	bool suspended;
	//! Process noise; set by reset(), may be overridden afterwards.
	double accel_noise_density, bias_random_walk;
	struct rift_s_position_filter_state state;
	struct xrt_vec3 last_accel;
	struct xrt_vec3 last_fix_position;
	timepoint_ns last_fix_ns;

	struct rift_s_position_filter_input history[RIFT_S_POSITION_FILTER_HISTORY];
	unsigned int history_start, history_count;

	//! Fixes fused since the last (re)initialization.
	uint32_t track_updates;
	//! Candidate replacement position awaiting confirmation.
	uint32_t candidate_count;
	struct xrt_vec3 candidate_position;
	timepoint_ns candidate_t;

	uint64_t resets, updates, stale, gated, rejected;
	//! Squared innovation norm of fused fixes, m^2, since the caller last cleared it.
	double innovation_sq_sum;
	uint32_t innovation_count;
};

//! Result of applying one optical position fix.
enum rift_s_position_fix_result
{
	RIFT_S_POSITION_FIX_RESET,   //!< (Re)initialized from the fix.
	RIFT_S_POSITION_FIX_UPDATED, //!< Fused at capture time and replayed.
	RIFT_S_POSITION_FIX_STALE,   //!< Older than the retained IMU history.
	RIFT_S_POSITION_FIX_PENDING, //!< Disagrees with the estimate; held until confirmed.
};

void
rift_s_position_filter_reset(struct rift_s_position_filter *f);

//! Integrate one IMU sample, @p accel_world in m/s^2 with gravity removed, at host time @p t.
void
rift_s_position_filter_imu(struct rift_s_position_filter *f, timepoint_ns t, const struct xrt_vec3 *accel_world);

/*!
 * Fuse an optical position at its capture time. @p variance is the per-axis measurement variance
 * in m^2. The first fix initializes the filter. An innovation beyond @p reset_distance_m, or any fix
 * while suspended, is a candidate: it re-initializes position and velocity (the Windows "6DoF reset")
 * only once RIFT_S_POSITION_FILTER_CONFIRM_FIXES consecutive candidates agree within
 * @p reset_distance_m of each other; until then the estimate is kept and PENDING returned.
 */
enum rift_s_position_fix_result
rift_s_position_filter_fix(struct rift_s_position_filter *f,
                           timepoint_ns capture,
                           const struct xrt_vec3 *position,
                           const struct xrt_vec3 *variance,
                           float reset_distance_m);

//! Seed a Constellation-confirmed track after optical age loss, retaining IMU history and noise.
//! Stale samples and clock/device discontinuities still use the normal rejection/confirmation gates.
enum rift_s_position_fix_result
rift_s_position_filter_reacquire(struct rift_s_position_filter *f,
                                 timepoint_ns capture,
                                 const struct xrt_vec3 *position,
                                 const struct xrt_vec3 *variance,
                                 float reset_distance_m);

//! Suspend integration after an IMU clock/device discontinuity, until the next optical reset.
void
rift_s_position_filter_suspend(struct rift_s_position_filter *f);

/*!
 * Position and velocity at @p t. Times after the last IMU sample extrapolate with constant
 * velocity, at most @p max_horizon_ns; earlier times return the current state.
 */
void
rift_s_position_filter_predict(const struct rift_s_position_filter *f,
                               timepoint_ns t,
                               time_duration_ns max_horizon_ns,
                               struct xrt_vec3 *out_position,
                               struct xrt_vec3 *out_velocity);

/*!
 * Controller loss duration configuration: how long position and velocity
 * stay tracked after the last optical update, by the number of successful and reliable optical
 * updates in the current track.
 */
void
rift_s_tracking_loss_durations(uint32_t vision_updates,
                               uint32_t reliable_updates,
                               time_duration_ns *out_position,
                               time_duration_ns *out_velocity);

#ifdef __cplusplus
}
#endif
