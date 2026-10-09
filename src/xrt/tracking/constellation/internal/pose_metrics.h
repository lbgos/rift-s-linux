// Copyright 2020-2023 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*
 * Ported from OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
/*!
 * @file
 * @brief  Metrics for constellation tracking poses
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup constellation
 */
#pragma once

#include "xrt/xrt_defines.h"
#include "tracking/t_led_models.h"

#include "blobwatch.h"
#include "camera_model.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_OBJECT_LEDS 64

struct pose_rect
{
	double left;
	double top;
	double right;
	double bottom;
};

XRT_MAYBE_UNUSED static bool
pose_rect_has_area(struct pose_rect *rect)
{
	return rect->left != rect->right && rect->top != rect->bottom;
}

enum pose_match_flags
{
	POSE_MATCH_GOOD = 0x1,      /* A reasonable pose match - most LEDs matched to within a few pixels error */
	POSE_MATCH_STRONG = 0x2,    /* A strong pose match is a match with very low error */
	POSE_MATCH_POSITION = 0x10, /* The position of the pose matched the prior well */
	POSE_MATCH_ORIENT = 0x20,   /* The orientation of the pose matched the prior well */
	POSE_HAD_PRIOR =
	    0x100, /* If a pose prior was supplied when calculating the score, then rot/trans_error are set */
	POSE_MATCH_LED_IDS = 0x200, /* The LED IDs on the blobs all matched the LEDs we thought (or were unassigned) */
};

#define POSE_SET_FLAG(score, f) ((score)->match_flags |= (f))
#define POSE_CLEAR_FLAG(score, f) ((score)->match_flags &= ~(f))
#define POSE_HAS_FLAGS(score, f) (((score)->match_flags & (f)) == (f))

struct pose_metrics
{
	enum pose_match_flags match_flags;

	int matched_blobs;
	int unmatched_blobs;
	int visible_leds;

	double reprojection_error;

	struct xrt_vec3 orient_error; /* Rotation error (compared to a prior) */
	struct xrt_vec3 pos_error;    /* Translation error (compared to a prior) */
};

struct pose_metrics_visible_led_info
{
	struct t_constellation_led *led;
	double led_radius_px;   /* Expected max size of the LED in pixels at that distance */
	struct xrt_vec2 pos_px; /* Projected position of the LED (pixels) */
	struct xrt_vec3 pos_m;  /* Projected physical position of the LED (metres) */
	double facing_dot;      /* Dot product between LED and camera */
	struct blob *matched_blob;
};

struct pose_metrics_blob_match_info
{
	struct pose_metrics_visible_led_info visible_leds[MAX_OBJECT_LEDS];
	int num_visible_leds;

	bool all_led_ids_matched;
	int matched_blobs;
	int unmatched_blobs;

	double reprojection_error;
	struct pose_rect bounds;
};

// Angular disagreement of gravity in model space. Camera-space yaw is unconstrained.
float
pose_metrics_gravity_error(const struct xrt_pose *pose,
                           const struct xrt_pose *prior,
                           const struct xrt_vec3 *camera_gravity);

void
pose_metrics_get_device_bounds(struct xrt_pose *P_cam_obj,
                               struct t_constellation_led_model *led_model,
                               struct camera_model *calib,
                               struct pose_rect *device_bounds,
                               struct xrt_vec2 *visible_points,
                               size_t *num_visible_points);

void
pose_metrics_match_pose_to_blobs(struct xrt_pose *pose,
                                 struct blob *blobs,
                                 int num_blobs,
                                 struct t_constellation_led_model *led_model,
                                 struct camera_model *calib,
                                 struct pose_metrics_blob_match_info *match_info);

void
pose_metrics_evaluate_pose(struct pose_metrics *score,
                           struct xrt_pose *pose,
                           struct blob *blobs,
                           int num_blobs,
                           struct t_constellation_led_model *leds_model,
                           struct camera_model *calib,
                           struct pose_rect *out_bounds);

void
pose_metrics_evaluate_pose_with_prior(struct pose_metrics *score,
                                      struct xrt_pose *pose,
                                      bool prior_must_match,
                                      struct xrt_pose *pose_prior,
                                      const struct xrt_vec3 *pos_error_thresh,
                                      const struct xrt_vec3 *rot_error_thresh,
                                      struct blob *blobs,
                                      int num_blobs,
                                      struct t_constellation_led_model *leds_model,
                                      struct camera_model *calib,
                                      struct pose_rect *out_bounds);

// Additional acquisition path for models with fewer visible LEDs. Caller must validate gravity.
bool
pose_metrics_can_acquire_with_gravity(const struct pose_metrics *score,
                                      const struct t_constellation_led_model *led_model);

bool
pose_metrics_score_is_better_pose(struct pose_metrics *old_score, struct pose_metrics *new_score);

#ifdef __cplusplus
}
#endif
