// Copyright 2020 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Constellation tracking metrics for assessing pose matches
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup constellation
 */
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "math/m_vec3.h"
#include "util/u_logging.h"

#include "pose_metrics.h"

static void
expand_rect(struct pose_rect *bounds, double x, double y, double w, double h)
{
	if (x < bounds->left)
		bounds->left = x;
	if (y < bounds->top)
		bounds->top = y;
	if (x + w > bounds->right)
		bounds->right = x + w;
	if (y + h > bounds->bottom)
		bounds->bottom = y + h;
}

static int
find_best_matching_led(struct pose_metrics_visible_led_info *led_points,
                       int num_leds,
                       struct blob *blob,
                       double *out_sqerror)
{
	double best_z;
	int best_led_index = -1;
	double best_sqerror = 1e20;
	int leds_within_range = 0;

	for (int i = 0; i < num_leds; i++) {
		struct pose_metrics_visible_led_info *led_info = led_points + i;
		struct xrt_vec2 *pos_px = &led_info->pos_px;
		double led_radius_px = led_info->led_radius_px;
		double dx = fabs(pos_px->x - blob->x);
		double dy = fabs(pos_px->y - blob->y);
		double sqerror = dx * dx + dy * dy;

		/* If the blob is much larger than the LED in either dimension,
		 * don't match */
		if (blob->width > led_info->led_radius_px * 4 || blob->height > led_info->led_radius_px * 4) {
			continue;
		}

		/* Check if the LED falls within the bounding box
		 * is closer to the camera (smaller Z), or is at least
		 * led_radius closer to the blob center */
		if (sqerror < (led_radius_px * led_radius_px)) {
			leds_within_range++;

			if (best_led_index < 0 || best_z > led_info->pos_m.z ||
			    (sqerror + led_radius_px) < best_sqerror) {
				best_z = led_info->pos_m.z;
				best_led_index = i;
				best_sqerror = sqerror;
			}
		}
	}

	if (leds_within_range > 1 && u_log_get_global_level() >= U_LOGGING_TRACE) {
		U_LOG_T("Multiple LEDs match blob @ %f, %f. best_sqerror %f LED %d z %f", blob->x, blob->y,
		        best_sqerror, best_led_index, led_points[best_led_index].pos_m.z);
		for (int i = 0; i < num_leds; i++) {
			struct pose_metrics_visible_led_info *led_info = led_points + i;
			struct xrt_vec2 *pos_px = &led_info->pos_px;
			struct xrt_vec3 *pos_m = &led_info->pos_m;
			double led_radius_px = led_info->led_radius_px;
			double dx = fabs(pos_px->x - blob->x);
			double dy = fabs(pos_px->y - blob->y);
			double sqerror = dx * dx + dy * dy;

			/* Check if the LED falls within the bounding box
			 * has smaller error distance, or is closer to the camera (smaller Z) */
			if (sqerror < (led_radius_px * led_radius_px)) {
				U_LOG_T("LED %d sqerror %f pos px %f %f radius %f metres %f %f %f", i, sqerror,
				        pos_px->x, pos_px->y, led_radius_px, pos_m->x, pos_m->y, pos_m->z);
			}
		}
	}

	if (out_sqerror)
		*out_sqerror = best_sqerror;
	return best_led_index;
}

static void
check_pose_prior(struct pose_metrics *score,
                 struct xrt_pose *pose,
                 struct xrt_pose *pose_prior,
                 const struct xrt_vec3 *pos_error_thresh,
                 const struct xrt_vec3 *rot_error_thresh)
{
	struct xrt_quat orient_diff;

	score->match_flags |= POSE_HAD_PRIOR;

	score->pos_error = m_vec3_sub(pose->position, pose_prior->position);

	math_quat_unrotate(&pose->orientation, &pose_prior->orientation, &orient_diff);
	math_quat_normalize(&orient_diff);
	math_quat_ln(&orient_diff, &score->orient_error);

	/* Check each component of position and rotation are within the passed error bound and
	 * clear any return flag that's not set */
	if (pos_error_thresh) {
		score->match_flags |= POSE_MATCH_POSITION;

		if (fabs(score->pos_error.x) > pos_error_thresh->x || fabs(score->pos_error.y) > pos_error_thresh->y ||
		    fabs(score->pos_error.z) > pos_error_thresh->z) {
			score->match_flags &= ~POSE_MATCH_POSITION;
		}
	}

	if (rot_error_thresh) {
		score->match_flags |= POSE_MATCH_ORIENT;

		if (fabs(score->orient_error.x) > rot_error_thresh->x ||
		    fabs(score->orient_error.y) > rot_error_thresh->y ||
		    fabs(score->orient_error.z) > rot_error_thresh->z) {
			score->match_flags &= ~POSE_MATCH_ORIENT;
		}
	}
}

static bool
project_led_points(struct t_constellation_led_model *led_model,
                   struct camera_model *calib,
                   struct xrt_pose *pose,
                   struct xrt_vec3 *out_positions,
                   struct xrt_vec2 *out_points)
{
	for (int i = 0; i < led_model->num_leds; i++) {
		struct xrt_vec3 *tmp = out_positions + i;
		math_pose_transform_point(pose, &led_model->leds[i].pos, tmp);
		if (tmp->z <= 0) {
			out_points[i] = (struct xrt_vec2){NAN, NAN};
			continue; // Visibility below skips LEDs behind the camera.
		}
		if (!camera_model_project(calib, tmp->x, tmp->y, tmp->z, &out_points[i].x, &out_points[i].y))
			return false;
	}
	return true;
}

static void
get_visible_leds_and_bounds(struct xrt_pose *pose,
                            struct t_constellation_led_model *led_model,
                            struct camera_model *calib,
                            struct pose_metrics_visible_led_info *visible_led_points,
                            int *num_visible_leds,
                            struct pose_rect *bounds)
{
	struct xrt_vec3 led_out_positions[MAX_OBJECT_LEDS];
	struct xrt_vec2 led_out_points[MAX_OBJECT_LEDS];
	bool first_visible_led = true;
	int i;
	struct t_constellation_led *leds = led_model->leds;
	const int num_leds = led_model->num_leds;

	*bounds = (struct pose_rect){0, 0, 0, 0};
	/* Project LEDs into the distorted image space */
	if (!project_led_points(led_model, calib, pose, led_out_positions, led_out_points)) {
		*num_visible_leds = 0;
		return;
	}

	/* Compute LED pixel size based on model distance below
	 * using the larger X/Y focal length and LED's Z value */
	double focal_length = MAX(calib->calib.fx, calib->calib.fy);

	// pose is camera->device, I_pose is device->camera
	struct xrt_pose P_obj_cam;
	math_pose_invert(pose, &P_obj_cam);

	/* Calculate the bounding box and visible LEDs */
	*num_visible_leds = 0;
	for (i = 0; i < num_leds; i++) {
		struct xrt_vec2 *led_pos_px = led_out_points + i;
		struct xrt_vec3 *led_pos_m = led_out_positions + i;
		struct t_constellation_led *led = &leds[i];

		/* LEDs behind the camera are not visible */
		if (led_pos_m->z <= 0.0) {
			continue;
		}

		if (led_pos_px->x < 0 || led_pos_px->y < 0 || led_pos_px->x >= calib->width ||
		    led_pos_px->y >= calib->height)
			continue; // Outside the visible screen space

		/* Calculate the expected size of an LED at this distance */
		double led_radius_px = 4.0;
		const double led_radius_mm = led->radius_mm;
		led_radius_px = focal_length * led_radius_mm / led_pos_m->z / 1000.0;
#if 0
		printf("LED id %d led_radius_px %f = focal length %f led_radius %f Z = %f m\n",
		       led_model->leds[i].id, led_radius_px,
		       focal_length, led_radius_mm, led_pos_m->z);
#endif

		/* Convert the position to a unit vector for dot product comparison */
		struct xrt_vec3 view_vec = *led_pos_m;
		struct xrt_vec3 normal;

		math_vec3_normalize(&view_vec);
		math_quat_rotate_vec3(&pose->orientation, &leds[i].dir, &normal);

		double facing_dot = m_vec3_dot(view_vec, normal);

#if 0
		printf ("device %d LED %u pos %f,%f,%f -> %f,%f (pos %f,%f,%f metres) dir %f %f %f dot %f visible %d\n",
			led_model->id, i, leds[i].pos.x, leds[i].pos.y, leds[i].pos.z,
			led_pos_px->x, led_pos_px->y,
			led_pos_m.x, led_pos_m.y, led_pos_m.z,
			normal.x, normal.y, normal.z, facing_dot,
			facing_dot < cos(DEG_TO_RAD(180.0 - LED_ANGLE)));
#endif

		/* The vector to the LED position points out from the camera
		 * to the LED, but the normal points toward the camera, so
		 * we need to compare against 180 - LED_ANGLE here */
		if (facing_dot > cos(DEG_TO_RAD(180.0 - LED_ANGLE))) {
			continue;
		}

		if (led_model->check_led_visibility &&
		    !led_model->check_led_visibility(led_model, i, P_obj_cam.position)) {
			continue;
		}

		struct pose_metrics_visible_led_info *led_info = visible_led_points + (*num_visible_leds);
		led_info->led = leds + i;
		led_info->pos_px = *led_pos_px;
		led_info->pos_m = *led_pos_m;
		led_info->led_radius_px = led_radius_px;
		led_info->matched_blob = NULL;
		led_info->facing_dot = facing_dot;
		(*num_visible_leds)++;

		/* Expand the bounding box */
		if (first_visible_led) {
			bounds->left = led_pos_px->x - led_radius_px;
			bounds->top = led_pos_px->y - led_radius_px;
			bounds->right = led_pos_px->x + 2 * led_radius_px;
			bounds->bottom = led_pos_px->y + 2 * led_radius_px;
			first_visible_led = false;
		} else {
			expand_rect(bounds, led_pos_px->x - led_radius_px, led_pos_px->y - led_radius_px,
			            2 * led_radius_px, 2 * led_radius_px);
		}
	}
}

static bool
project_bounding_points(struct t_constellation_led_model *led_model,
                        struct camera_model *calib,
                        struct xrt_pose *P_cam_obj,
                        struct xrt_vec3 *out_positions,
                        struct xrt_vec2 *out_points)
{
	for (int i = 0; i < led_model->num_bounding_points; i++) {
		struct xrt_vec3 *tmp = out_positions + i;
		math_pose_transform_point(P_cam_obj, &led_model->bounding_points[i].pos, tmp);
		if (tmp->z <= 0) {
			out_points[i] = (struct xrt_vec2){NAN, NAN};
			continue;
		}
		if (!camera_model_project(calib, tmp->x, tmp->y, tmp->z, &out_points[i].x,
		                             &out_points[i].y)) {
			return false;
		}
	}
	return true;
}

void
pose_metrics_get_device_bounds(struct xrt_pose *P_cam_obj,
                               struct t_constellation_led_model *led_model,
                               struct camera_model *calib,
                               struct pose_rect *device_bounds,
                               struct xrt_vec2 *visible_points,
                               size_t *num_visible_points)
{
	bool first_visible_bounding_point = true;

	if (num_visible_points)
		*num_visible_points = 0;

	struct xrt_vec3 bounding_out_positions[MAX_OBJECT_LEDS];
	struct xrt_vec2 bounding_out_points[MAX_OBJECT_LEDS];

	const int num_bounding_points = led_model->num_bounding_points;
	/* Project device bounding points into the distorted image space */
	bool bounding_points_valid =
	    project_bounding_points(led_model, calib, P_cam_obj, bounding_out_positions, bounding_out_points);

	*device_bounds = (struct pose_rect){0, 0, 0, 0};

	for (int i = 0; i < num_bounding_points && bounding_points_valid; i++) {
		struct xrt_vec2 *point_pos_px = bounding_out_points + i;
		struct xrt_vec3 *point_pos_m = bounding_out_positions + i;

		// skip points behind the camera
		if (point_pos_m->z <= 0) {
			continue;
		}

		if (visible_points && num_visible_points)
			visible_points[(*num_visible_points)++] = *point_pos_px;

		struct xrt_vec2 clamped_pos = {CLAMP(point_pos_px->x, 0, calib->width - 1),
		                               CLAMP(point_pos_px->y, 0, calib->height - 1)};

		if (first_visible_bounding_point) {
			*device_bounds = (struct pose_rect){
			    .top = clamped_pos.y,
			    .bottom = clamped_pos.y,
			    .left = clamped_pos.x,
			    .right = clamped_pos.x,
			};
		} else
			expand_rect(device_bounds, clamped_pos.x, clamped_pos.y, 0, 0);

		first_visible_bounding_point = false;
	}
}

void
pose_metrics_match_pose_to_blobs(struct xrt_pose *pose,
                                 struct blob *blobs,
                                 int num_blobs,
                                 struct t_constellation_led_model *led_model,
                                 struct camera_model *calib,
                                 struct pose_metrics_blob_match_info *match_info)
{
	struct pose_rect *bounds = &match_info->bounds;

	match_info->reprojection_error = 0.0;
	match_info->matched_blobs = 0;
	match_info->unmatched_blobs = 0;

	get_visible_leds_and_bounds(pose, led_model, calib, match_info->visible_leds, &match_info->num_visible_leds,
	                            &match_info->bounds);

	// printf("Bounding box for pose is %f,%f -> %f,%f\n", bounds.left, bounds.top, bounds.right, bounds.bottom);

	/* Iterate the blobs and see which ones are within the bounding box and have a matching LED */
	bool all_led_ids_matched = true;

	for (int i = 0; i < num_blobs; i++) {
		struct blob *b = blobs + i;
		uint32_t led_object_id = LED_OBJECT_ID(b->led_id);

		/* Skip blobs which already have an ID not belonging to this device */
		if (led_object_id != LED_INVALID_ID && led_object_id != led_model->id)
			continue;

		if (b->x < bounds->left || b->y < bounds->top || b->x > bounds->right || b->y > bounds->bottom) {
			/* Ignore blobs that are outside the pose bounding box */
			continue;
		}

		double sqerror;

		int match_led_index =
		    find_best_matching_led(match_info->visible_leds, match_info->num_visible_leds, b, &sqerror);
		if (match_led_index < 0) {
			match_info->unmatched_blobs++;
			continue;
		}

		struct pose_metrics_visible_led_info *led_info = match_info->visible_leds + match_led_index;
		/* A split blob or reflection cannot count as another LED correspondence. */
		if (led_info->matched_blob != NULL) {
			match_info->unmatched_blobs++;
			continue;
		}

		match_info->reprojection_error += sqerror;
		match_info->matched_blobs++;

		led_info->matched_blob = b;

		if (b->led_id != LED_INVALID_ID) {
			struct t_constellation_led *match_led = led_info->led;
			uint8_t led_id = match_led->id;
			if (b->led_id != LED_MAKE_ID(led_model->id, led_id)) {
#if 0
				printf("mismatched LED id %d/%d blob %d (@ %f,%f) has %d/%d\n", led_model->id, led_id,
				       i, b->x, b->y, LED_OBJECT_ID(b->led_id), LED_LOCAL_ID(b->led_id));
#endif
				all_led_ids_matched = false;
			}
		}
	}

	match_info->all_led_ids_matched = all_led_ids_matched;
}

void
pose_metrics_evaluate_pose_with_prior(struct pose_metrics *score,
                                      struct xrt_pose *pose,
                                      bool prior_must_match,
                                      struct xrt_pose *pose_prior,
                                      const struct xrt_vec3 *pos_error_thresh,
                                      const struct xrt_vec3 *rot_error_thresh,
                                      struct blob *blobs,
                                      int num_blobs,
                                      struct t_constellation_led_model *led_model,
                                      struct camera_model *calib,
                                      struct pose_rect *out_bounds)
{
	/*
	 * 1. Project the LED points with the provided pose
	 * 2. Build a bounding box for the points
	 * 3. For blobs within the bounding box, see if they match a LED
	 * 4. Count up the matched LED<->Blob correspondences, and the reprojection error
	 */
	struct pose_metrics_blob_match_info blob_match_info;

	pose_metrics_match_pose_to_blobs(pose, blobs, num_blobs, led_model, calib, &blob_match_info);

	assert(led_model->num_leds > 0);
	assert(num_blobs > 0);
	assert(score != NULL);

	score->match_flags = 0;
	score->reprojection_error = blob_match_info.reprojection_error;
	score->matched_blobs = blob_match_info.matched_blobs;
	score->unmatched_blobs = blob_match_info.unmatched_blobs;
	score->visible_leds = blob_match_info.num_visible_leds;

	if (blob_match_info.all_led_ids_matched) {
		score->match_flags |= POSE_MATCH_LED_IDS;
	}

	double error_per_led = score->reprojection_error / score->matched_blobs;

	/* If we have a pose prior, calculate the rotation and translation error and match flags as needed */
	if (pose_prior) {
		/* We can't validate a prior without error bounds */
		assert(pos_error_thresh != NULL);
		assert(rot_error_thresh != NULL);

		check_pose_prior(score, pose, pose_prior, pos_error_thresh, rot_error_thresh);
	}

	/* Don't add GOOD/STRONG flags if matched fewer than 3 blobs */
	if (score->matched_blobs < 3) {
		goto done;
	}

	/* At this point, we have at least 3 LEDs and their blobs matching */
	if (POSE_HAS_FLAGS(score, POSE_MATCH_POSITION | POSE_MATCH_ORIENT)) {
		if (error_per_led < 2.0 && (score->unmatched_blobs * 4 <= score->matched_blobs ||
		                            (2 * score->visible_leds <= 3 * score->matched_blobs))) {
#if 0
			printf("Got good prior match within pos (%f, %f, %f) rot (%f, %f, %f)\n", pos_error_thresh->x,
			       pos_error_thresh->y, pos_error_thresh->z, rot_error_thresh->x, rot_error_thresh->y,
			       rot_error_thresh->z);
#endif
			score->match_flags |= POSE_MATCH_GOOD;

			if (error_per_led < 1.5)
				score->match_flags |= POSE_MATCH_STRONG;
		}
	} else if (prior_must_match) {
		/* If we must match the prior and failed, bail out */
		goto done;
	} else if (score->visible_leds > 6 && score->matched_blobs > 6 && error_per_led < 3.0 &&
	           (score->unmatched_blobs * 4 <= score->matched_blobs ||
	            (2 * score->visible_leds <= 3 * score->matched_blobs))) {
		/* If we matched all the blobs in the pose bounding box (allowing 25% noise / overlapping blobs)
		 * or if we matched a large proportion (2/3) of the LEDs we expect to be visible, then consider this a
		 * good pose match */
		score->match_flags |= POSE_MATCH_GOOD;

		/* If we had no pose prior, but a close reprojection error, allow a STRONG match */
		/* If we had a pose prior and got here, the pose is out of tolerance, so only permit a "GOOD" match */
		if (pose_prior == NULL && error_per_led < 1.5)
			score->match_flags |= POSE_MATCH_STRONG;
	}

#if 0
	printf ("score for pose is %u matched %u unmatched %u visible %f error\n",
		score->matched_blobs, score->unmatched_blobs, score->visible_leds, score->reprojection_error);
#endif

done:
	if (out_bounds)
		*out_bounds = blob_match_info.bounds;
}

void
pose_metrics_evaluate_pose(struct pose_metrics *score,
                           struct xrt_pose *pose,
                           struct blob *blobs,
                           int num_blobs,
                           struct t_constellation_led_model *led_model,
                           struct camera_model *calib,
                           struct pose_rect *out_bounds)
{
	pose_metrics_evaluate_pose_with_prior(score, pose, false, NULL, NULL, NULL, blobs, num_blobs, led_model, calib,
	                                      out_bounds);
}

/* Return true if new_score is for a more likely pose than old_score */
bool
pose_metrics_score_is_better_pose(struct pose_metrics *old_score, struct pose_metrics *new_score)
{
	/* if our previous best pose was "strong", only take better "strong" poses */
	if (POSE_HAS_FLAGS(old_score, POSE_MATCH_STRONG) && !POSE_HAS_FLAGS(new_score, POSE_MATCH_STRONG))
		return false;

	/* If the old score wasn't any good, but the new one is - take the new one */
	if (!POSE_HAS_FLAGS(old_score, POSE_MATCH_GOOD) && POSE_HAS_FLAGS(new_score, POSE_MATCH_GOOD))
		return true;

	double new_error_per_led = new_score->reprojection_error / new_score->matched_blobs;
	double best_error_per_led = 10.0;

	if (old_score->matched_blobs > 0)
		best_error_per_led = old_score->reprojection_error / old_score->matched_blobs;

	if (old_score->matched_blobs < new_score->matched_blobs && (new_error_per_led < best_error_per_led))
		return true; /* prefer more matched blobs with tighter error/LED */

	if (old_score->matched_blobs + 1 < new_score->matched_blobs && (new_error_per_led < best_error_per_led * 1.1))
		return true; /* prefer at least 2 more matched blobs with slightly worse error/LED */

	if (old_score->matched_blobs == new_score->matched_blobs &&
	    new_score->reprojection_error < old_score->reprojection_error)
		return true; /* else, prefer closer reprojection with at least as many matches*/

	/* If both scores have pose priors, prefer the one where the orientation better matches the prior */
	if (POSE_HAS_FLAGS(old_score, POSE_HAD_PRIOR) && POSE_HAS_FLAGS(new_score, POSE_HAD_PRIOR)) {
		if (m_vec3_len(new_score->orient_error) < m_vec3_len(old_score->orient_error)) {
			return true;
		}
	}

	return false;
}

float
pose_metrics_gravity_error(const struct xrt_pose *pose,
                           const struct xrt_pose *prior,
                           const struct xrt_vec3 *camera_gravity)
{
	struct xrt_quat inverse, prior_inverse;
	struct xrt_vec3 gravity, prior_gravity;
	math_quat_invert(&pose->orientation, &inverse);
	math_quat_invert(&prior->orientation, &prior_inverse);
	math_quat_rotate_vec3(&inverse, camera_gravity, &gravity);
	math_quat_rotate_vec3(&prior_inverse, camera_gravity, &prior_gravity);

	// Compare directions in model space, preserving tilt axis and ignoring camera-space yaw.
	float dot = m_vec3_dot(gravity, prior_gravity);
	return isfinite(dot) ? acosf(fmaxf(-1.0f, fminf(1.0f, dot))) : INFINITY;
}

/* The caller must independently validate the IMU gravity prior. */
bool
pose_metrics_can_acquire_with_gravity(const struct pose_metrics *score,
                                      const struct t_constellation_led_model *led_model)
{
	const int min_leds = led_model->min_acquisition_leds;
	return min_leds >= 5 && min_leds < 7 && score->matched_blobs >= min_leds &&
	       POSE_HAS_FLAGS(score, POSE_MATCH_LED_IDS) && score->reprojection_error / score->matched_blobs < 1.5 &&
	       score->unmatched_blobs <= 1 && 5 * score->matched_blobs >= 4 * score->visible_leds;
}
