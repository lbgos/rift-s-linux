// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once

#include "xrt/xrt_defines.h"
#include "util/u_time.h"

#ifdef __cplusplus
extern "C" {
#endif

struct rift_s_motion;

// Samples are at the controller IMU origin, before the device/aim/grip lever arms.
struct rift_s_motion_sample
{
	timepoint_ns timestamp_ns;
	struct xrt_pose pose;
	struct xrt_vec3 linear_velocity;
	struct xrt_vec3 linear_acceleration;
	// Angular rate in the controller frame. Relations returned by get use tracking-frame rates.
	struct xrt_vec3 angular_velocity;
	// A negative diagonal marks a position estimate without a reliable filter position.
	double position_variance[3];
};

enum rift_s_motion_model
{
	RIFT_S_MOTION_NONE,
	RIFT_S_MOTION_AT_REST,
	RIFT_S_MOTION_NEAR_HEAD,
	RIFT_S_MOTION_HAND_OVER_HAND,
	RIFT_S_MOTION_BEHIND_BACK,
	RIFT_S_MOTION_OUT_OF_FOV,
};

struct rift_s_motion *
rift_s_motion_create(void);

void
rift_s_motion_destroy(struct rift_s_motion **motion);

void
rift_s_motion_reset_hand(struct rift_s_motion *motion, unsigned int hand);

// hand: 0 left, 1 right. The shared cache retains the other hand's most recent sample.
// The reference pose must share the controller's tracking frame. A missing reference leaves
// the previous cache intact; it cannot initialize a new hand model.
void
rift_s_motion_update(struct rift_s_motion *motion,
                     unsigned int hand,
                     const struct rift_s_motion_sample *sample,
                     const struct xrt_pose *reference);

// Query is read-only: neither render prediction nor lever-arm transforms enter the model cache.
// Rest/rotation estimates have no position confidence; attachment models retain the input confidence.
// The caller keeps its optical result whenever no model is active.
enum rift_s_motion_model
rift_s_motion_get(struct rift_s_motion *motion,
                  unsigned int hand,
                  timepoint_ns timestamp_ns,
                  struct xrt_space_relation *out_relation);

#ifdef __cplusplus
}
#endif
