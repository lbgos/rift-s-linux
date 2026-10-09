// Copyright 2020-2023 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*
 * Ported from OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
/*!
 * @file
 * @brief  Ab-initio blob<->LED correspondence search
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup constellation
 */
#pragma once

#include "tracking/t_led_models.h"

#include "blobwatch.h"
#include "camera_model.h"
#include "pose_metrics.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_BLOB_SEARCH_DEPTH 5

enum correspondence_search_flags
{
	CS_FLAG_NONE = 0x0,
	CS_FLAG_SHALLOW_SEARCH = 0x1, /* do quick search @ depth 1-2 neighbour depth */
	CS_FLAG_DEEP_SEARCH = 0x2,    /* do deeper searches @ up to MAX_LED_SEARCH_DEPTH/MAX_BLOB_SEARCH_DEPTH */
	CS_FLAG_STOP_FOR_STRONG_MATCH =
	    0x4, /* Stop searching if a strong match is found, otherwise search all and return best match */
	CS_FLAG_MATCH_ALL_BLOBS =
	    0x8, /* Allow matching against all blobs, not just unlabelled ones or for the current device */
	CS_FLAG_HAVE_POSE_PRIOR = 0x10, /* If the input obj_cam_pose contains a valid prior */
	CS_FLAG_MATCH_GRAVITY = 0x20,   /* Use pose orientation and gravity even without a position prior. */
	CS_FLAG_JOINT_P3P = 0x40, /* Callback verifies three-point roots across the complete camera rig. */
};

struct cs_image_point
{
	struct blob *blob;

	double point_homog[3]; // Homogeneous version of the point
	double size[2];        // w/h of the blob, in homogeneous coordinates
	double max_dist;       // norm of (W/H) for distance checks

	/* List of the nearest blobs, filtered for the active model */
	int num_neighbours;
	struct cs_image_point *neighbours[MAX_BLOB_SEARCH_DEPTH];
};

struct cs_model_info
{
	int id;

	struct t_constellation_search_model *model;

	double best_pose_found_time; /* Time (in secs) at which the best pose was found */
	int best_pose_blob_depth;    /* Blob neighbor depth the best pose is from */
	int best_pose_led_depth;     /* LED neigbour depth the best pose is from */
	struct xrt_pose best_pose;
	enum pose_match_flags match_flags;

	struct pose_metrics best_score;

	/* Search parameters */
	double search_start_time;
	int led_depth;
	int led_index;
	int blob_index;

	enum correspondence_search_flags search_flags;
	int min_led_depth, max_led_depth;
	int max_blob_depth;

	/* Valid when CS_FLAG_HAVE_POSE_PRIOR is set */
	struct xrt_pose pose_prior;
	struct xrt_vec3 *pos_error_thresh;
	struct xrt_vec3 *rot_error_thresh;

	/* Used when CS_FLAG_MATCH_GRAVITY is set */
	struct xrt_vec3 gravity_vector;
	float gravity_tolerance_rad;
};

struct correspondence_search
{
	int num_points;
	struct cs_image_point *points;
	struct blob *blobs; /* Original blobs structs [num_points] */
	//! Optional same-exposure ownership mask, indexed like blobs. Null searches all observations.
	const bool *excluded_blobs;

	//! Optional elapsed-time budget per search, zero preserves the caller default.
	uint64_t max_search_ns;
	uint64_t search_deadline_ns;
	bool budget_exhausted;

	//! Called for each geometry-verified candidate, before selecting a per-view winner.
	void (*pose_candidate_cb)(void *userdata, const struct xrt_pose *pose, const struct pose_metrics *score);
	void *pose_candidate_userdata;

	unsigned int num_trials;
	unsigned int num_pose_checks;
	unsigned int num_gravity_rejects;
	unsigned int num_metric_checks;

	struct camera_model *calib;

	/* List of the nearest blobs for each blob */
	struct cs_image_point *blob_neighbours[MAX_BLOBS_PER_FRAME][MAX_BLOBS_PER_FRAME];
};

struct correspondence_search *
correspondence_search_new(struct camera_model *camera_calib);
void
correspondence_search_set_blobs(struct correspondence_search *cs, struct blob *blobs, int num_blobs);

void
correspondence_search_free(struct correspondence_search *cs);

bool
correspondence_search_find_one_pose(struct correspondence_search *cs,
                                    struct t_constellation_search_model *model,
                                    enum correspondence_search_flags search_flags,
                                    struct xrt_pose *pose,
                                    struct xrt_vec3 *pos_error_thresh,
                                    struct xrt_vec3 *rot_error_thresh,
                                    struct xrt_vec3 *gravity_vector,
                                    float gravity_tolerance_rad,
                                    struct pose_metrics *score);
bool
correspondence_search_have_pose(struct correspondence_search *cs,
                                int model_id,
                                struct xrt_pose *pose,
                                struct pose_metrics *score);

#ifdef __cplusplus
}
#endif
