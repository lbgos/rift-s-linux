// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Rejects diverged SLAM poses and supplies the fallback HMD pose.
 * @ingroup drv_rift_s
 */

#pragma once

#include "xrt/xrt_defines.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! SLAM positions farther than this from the tracking origin are treated as divergence.
#define RIFT_S_SLAM_GUARD_MAX_DISTANCE_M 5.0f
//! A position step larger than this between two SLAM samples is treated as divergence.
#define RIFT_S_SLAM_GUARD_MAX_STEP_M 0.5f
//! A head speed above this between two SLAM samples is treated as divergence.
#define RIFT_S_SLAM_GUARD_MAX_SPEED_MPS 5.0f
//! Step allowed on top of the speed limit, for samples queried very close in time.
#define RIFT_S_SLAM_GUARD_STEP_TOLERANCE_M 0.01f
//! Consecutive consistent SLAM samples needed before SLAM is trusted (again).
#define RIFT_S_SLAM_GUARD_STABLE_SAMPLES 30
//! Time over which those samples must stay consistent.
#define RIFT_S_SLAM_GUARD_STABLE_NS (1000LL * 1000LL * 1000LL)
//! Minimum interval between two SLAM re-initialisation requests.
#define RIFT_S_SLAM_GUARD_RESET_INTERVAL_NS (2000LL * 1000LL * 1000LL)
//! Eye height above the floor reported before SLAM ever gave a good position.
#define RIFT_S_SLAM_GUARD_EYE_HEIGHT_M 1.6f
//! A head this far below the (nominal) floor is underground: divergence.
#define RIFT_S_SLAM_GUARD_BELOW_FLOOR_M 0.3f
//! A head this high above the (nominal) floor is divergence.
#define RIFT_S_SLAM_GUARD_ABOVE_FLOOR_M 2.5f

enum rift_s_slam_guard_state
{
	//! No trusted SLAM pose yet since start.
	RIFT_S_SLAM_GUARD_ACQUIRING,
	//! SLAM poses are published.
	RIFT_S_SLAM_GUARD_TRACKING,
	//! SLAM diverged or stopped; 3DoF with the held position until SLAM is consistent again.
	RIFT_S_SLAM_GUARD_DIVERGED,
};

struct rift_s_slam_guard
{
	//! Saved world alignment is usable only until the backend changes its map.
	bool world_valid;
	//! Saved-boundary confirmation is independent of temporary SLAM tracking.
	bool boundary_confirmed;
	bool world_transition;
	struct xrt_pose transition_from, transition_to;
	int64_t transition_start_ns;
	//! A saved standing frame requires visual confirmation before publishing position tracking.
	bool require_relocalization;
	enum rift_s_slam_guard_state state;

	//! Last accepted source SLAM pose in the world frame and its source timestamp.
	bool have_good;
	struct xrt_pose good;
	int64_t good_ns;

	//! Maps raw SLAM poses into the published frame (yaw and translation only).
	struct xrt_pose align;

	//! Yaw applied to the 3DoF orientation while falling back, so it continues the SLAM heading.
	struct xrt_quat fallback_yaw;

	//! Stability run of raw SLAM samples while acquiring or diverged.
	uint32_t stable_samples;
	int64_t stable_start_ns;
	struct xrt_pose stable_last;
	int64_t stable_last_ns;

	//! Re-initialisation bookkeeping.
	bool reset_pending;
	bool have_reset;
	int64_t last_reset_ns;
	//! Invalid output must also recover before a first trusted pose exists.
	int64_t invalid_since_ns;

	//! Diagnostics.
	uint32_t divergences;
	uint32_t resets;
	uint32_t recoveries;
	float last_reject_distance_m;
	float last_reject_step_m;
};

//! Why a SLAM sample was not published.
enum rift_s_slam_guard_reason
{
	RIFT_S_SLAM_GUARD_OK,
	RIFT_S_SLAM_GUARD_NO_POSE,
	RIFT_S_SLAM_GUARD_NOT_FINITE,
	RIFT_S_SLAM_GUARD_TOO_FAR,
	RIFT_S_SLAM_GUARD_HEIGHT,
	RIFT_S_SLAM_GUARD_JUMP,
	RIFT_S_SLAM_GUARD_SPEED,
	RIFT_S_SLAM_GUARD_SETTLING,
	RIFT_S_SLAM_GUARD_TILT,
	RIFT_S_SLAM_GUARD_RELOCALIZING,
};

//! Finite, unit-length orientation whose gravity direction agrees with capture-time IMU fusion.
bool
rift_s_slam_guard_pose_usable(const struct xrt_pose *pose, const struct xrt_quat *fusion);

//! Validate a predicted/historical query without changing source history or world alignment.
bool
rift_s_slam_guard_query(const struct rift_s_slam_guard *g, const struct xrt_pose *raw, int64_t query_ns,
                         const struct xrt_quat *fusion, const struct xrt_vec3 *no_pose_position,
                         struct xrt_pose *out);

//! Install a visually recovered world transform. No head-height or held-position adjustment.
void
rift_s_slam_guard_set_world(struct rift_s_slam_guard *g, const struct xrt_pose *world_from_slam);

void
rift_s_slam_guard_init(struct rift_s_slam_guard *g);

/*!
 * Feed one SLAM sample and get the IMU pose to publish.
 *
 * @param g                 Guard state.
 * @param slam_valid        Whether the SLAM tracker returned a pose with a valid position.
 * @param slam_pose         SLAM IMU pose, already converted to the Monado frame.
 * @param ts_ns             Query time of the sample.
 * @param fusion_orientation Current 3DoF orientation.
 * @param no_pose_position  Position to report while no SLAM position was ever trusted. It is at eye
 *                          height, so it also gives the nominal floor for the height bound.
 * @param[out] out_pose     Pose to publish.
 * @param[out] out_reason   Why SLAM was not used, or OK.
 * @return true when @p out_pose comes from SLAM (position tracked).
 */
bool
rift_s_slam_guard_update(struct rift_s_slam_guard *g,
                         bool slam_valid,
                         const struct xrt_pose *slam_pose,
                         int64_t ts_ns,
                         const struct xrt_quat *fusion_orientation,
                         const struct xrt_vec3 *no_pose_position,
                         struct xrt_pose *out_pose,
                         enum rift_s_slam_guard_reason *out_reason);

/*!
 * The pose to publish without SLAM: the 3DoF orientation, continued in the SLAM heading, at the
 * last good position (or @p no_pose_position).
 */
void
rift_s_slam_guard_fallback_pose(const struct rift_s_slam_guard *g,
                                const struct xrt_quat *fusion_orientation,
                                const struct xrt_vec3 *no_pose_position,
                                struct xrt_pose *out_pose);

/*!
 * Stop publishing SLAM without counting a divergence (3DoF chosen by the user). SLAM is aligned onto
 * the shown pose again once it is consistent.
 */
void
rift_s_slam_guard_suspend(struct rift_s_slam_guard *g, const struct xrt_quat *fusion_orientation);

//! Start the replacement backend's recovery interval without changing the held world pose.
void
rift_s_slam_guard_backend_started(struct rift_s_slam_guard *g, int64_t ts_ns);

/*!
 * Returns true once per required SLAM re-initialisation, and records it as done at @p ts_ns.
 */
bool
rift_s_slam_guard_take_reset(struct rift_s_slam_guard *g, int64_t ts_ns);

/*!
 * Position, in the tracker frame, that puts the head at eye height above the floor when the tracking
 * origin is already offset by @p origin_offset_y.
 */
struct xrt_vec3
rift_s_slam_guard_no_pose_position(float origin_offset_y);

//! The yaw-only part (about +Y) of a rotation.
void
rift_s_slam_guard_yaw_of(const struct xrt_quat *q, struct xrt_quat *out_yaw);

const char *
rift_s_slam_guard_reason_str(enum rift_s_slam_guard_reason reason);

//! Fewer features than this over all SLAM cameras means the cameras see nothing usable.
#define RIFT_S_SLAM_FEATURES_MIN 20
//! How long the count must stay on one side of the limit before the state changes.
#define RIFT_S_SLAM_FEATURES_DWELL_NS (500LL * 1000LL * 1000LL)

//! Tracks whether the SLAM cameras see enough features (not face down, covered or dark).
struct rift_s_slam_feature_gate
{
	bool lost;
	bool crossing;
	int64_t crossing_since_ns;
	int64_t last_ts_ns;
	int last_count;
};

enum rift_s_slam_feature_event
{
	RIFT_S_SLAM_FEATURES_NO_CHANGE,
	//! Features gone: hold the head pose.
	RIFT_S_SLAM_FEATURES_LOST,
	//! Features back: re-initialise SLAM and realign.
	RIFT_S_SLAM_FEATURES_RETURNED,
};

/*!
 * Feed the latest feature count. @p known false (no count yet) changes nothing. Samples with a
 * timestamp not newer than the last one are ignored.
 */
enum rift_s_slam_feature_event
rift_s_slam_feature_gate_update(struct rift_s_slam_feature_gate *gate, bool known, int count, int64_t ts_ns);

#ifdef __cplusplus
}
#endif
