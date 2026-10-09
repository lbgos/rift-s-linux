// Copyright 2023, Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Constellation tracking details for 1 exposure sample
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup constellation
 */

#pragma once

#include "tracking/t_tracking.h"

#include "xrt/xrt_defines.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_results.h"

#include "blobwatch.h"
#include "pose_metrics.h"
#include "constrained_pose.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONSTELLATION_MAX_DEVICES 4
#define CONSTELLATION_MAX_CAMERAS XRT_TRACKING_MAX_SLAM_CAMS

/* Information about one device being tracked in this sample */
struct tracking_sample_device_state
{
	/* Index into the devices array for this state info */
	int dev_index;

	struct t_constellation_led_model *led_model;

	/* Predicted device pose and error bounds from fusion, in world space */
	struct xrt_pose P_world_obj_prior;
	struct xrt_vec3 prior_pos_error;
	struct xrt_vec3 prior_rot_error;
	float gravity_error_rad; /* Gravity vector uncertainty in radians 0..M_PI */
	bool trusted_position;
	bool reliable_heading;

	/* Last observed pose, in world space */
	bool have_last_seen_pose;
	struct xrt_pose last_seen_pose;

	bool found_device_pose;     /* Set to true when the device was found in this sample */
	int found_pose_view_id;     /* Set to the camera ID where the device was found */
	struct xrt_pose final_pose; /* Global pose that was detected */

	struct constellation_constrained_result joint_result;
	struct pose_metrics score;
	struct pose_metrics_blob_match_info blob_match_info;
};

/* Information about 1 camera frame in this sample */
struct tracking_sample_frame
{
	/* Video frame data we are analysing */
	struct xrt_frame *vframe;

	/* The pose from which this view is observed (cam wrt world) */
	struct xrt_pose P_world_cam;
	/* Inverse of the pose from which this view is observed (world wrt camera) */
	struct xrt_pose P_cam_world;
	/* Gravity vector as observed from this camera */
	struct xrt_vec3 cam_gravity_vector;

	/* blobs observation and the owning blobwatch */
	blobwatch *bw;
	blobservation *bwobs;
};

struct constellation_tracking_sample
{
	uint64_t timestamp; // Exposure timestamp

	/* Device poses at capture time */
	struct tracking_sample_device_state devices[CONSTELLATION_MAX_DEVICES];
	uint8_t n_devices;

	struct tracking_sample_frame views[CONSTELLATION_MAX_CAMERAS];
	uint8_t n_views;

	/* HMD pose at capture time in the OpenXR world, for logging positions relative to the head */
	struct xrt_pose P_xrworld_hmd;
	bool have_hmd_pose;

	bool need_long_analysis;

	bool long_analysis_found_new_blobs;
};

struct constellation_tracking_sample *
constellation_tracking_sample_new(void);
void
constellation_tracking_sample_free(struct constellation_tracking_sample *sample);

// Reject held or failed HMD queries before constructing world-space camera views.
bool
constellation_tracking_sample_set_hmd_pose(struct constellation_tracking_sample *sample,
                                           xrt_result_t result,
                                           const struct xrt_space_relation *relation);

/* Mark the other devices' published assignments in this exposure. Original blob indices are
 * preserved for joint verification and arbitration; inherited temporal labels are ignored. */
unsigned
constellation_tracking_sample_exclude_published(const struct constellation_tracking_sample *sample,
                                                 unsigned device_index,
                                                 struct constellation_constrained_view *views,
                                                 unsigned num_views);

/* A world position relative to the head, in the head's heading frame (OpenXR axes): x right, y up,
 * -z forward. Head pitch and roll are removed, so "forward" and "below" mean the same thing
 * wherever the user looks. */
void
constellation_head_relative_position(const struct xrt_pose *P_xrworld_hmd,
                                     const struct xrt_vec3 *position,
                                     struct xrt_vec3 *out);

#ifdef __cplusplus
}
#endif
