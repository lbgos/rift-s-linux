/* Copyright 2023, Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */
/*!
 * @file
 * @brief  Constellation tracking details for 1 exposure sample
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup constellation
 */
#include "sample.h"

#include "math/m_api.h"
#include "math/m_vec3.h"

#include <math.h>

struct constellation_tracking_sample *
constellation_tracking_sample_new(void)
{
	struct constellation_tracking_sample *sample = calloc(1, sizeof(struct constellation_tracking_sample));
	return sample;
}

bool
constellation_tracking_sample_set_hmd_pose(struct constellation_tracking_sample *sample,
                                           xrt_result_t result,
                                           const struct xrt_space_relation *relation)
{
	sample->have_hmd_pose = false;
	const enum xrt_space_relation_flags required =
	    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
	if (result != XRT_SUCCESS || (relation->relation_flags & required) != required)
		return false;
	const struct xrt_pose *p = &relation->pose;
	float norm = p->orientation.x * p->orientation.x + p->orientation.y * p->orientation.y +
	             p->orientation.z * p->orientation.z + p->orientation.w * p->orientation.w;
	if (!isfinite(p->position.x) || !isfinite(p->position.y) || !isfinite(p->position.z) ||
	    !isfinite(norm) || norm < 0.9f || norm > 1.1f)
		return false;
	sample->P_xrworld_hmd = *p;
	sample->have_hmd_pose = true;
	return true;
}

void
constellation_tracking_sample_free(struct constellation_tracking_sample *sample)
{
	int i;

	assert(sample->n_views <= CONSTELLATION_MAX_CAMERAS);
	for (i = 0; i < sample->n_views; i++) {
		struct tracking_sample_frame *view = sample->views + i;
		xrt_frame_reference(&view->vframe, NULL);
		if (view->bwobs != NULL) {
			assert(view->bw != NULL);
			blobwatch_release_observation(view->bw, view->bwobs);
		}
	}

	free(sample);
}

unsigned
constellation_tracking_sample_exclude_published(const struct constellation_tracking_sample *sample,
                                                 unsigned device_index,
                                                 struct constellation_constrained_view *views,
                                                 unsigned num_views)
{
	unsigned excluded = 0;
	for (unsigned d = 0; d < sample->n_devices; d++) {
		const struct tracking_sample_device_state *owner = &sample->devices[d];
		if (d == device_index || !owner->found_device_pose) {
			continue;
		}
		for (unsigned a = 0; a < owner->joint_result.num_assignments; a++) {
			const struct constellation_constrained_assignment *claim = &owner->joint_result.assignments[a];
			for (unsigned v = 0; v < num_views; v++) {
				if (views[v].camera_index != claim->camera_index || !views[v].observation ||
				    claim->blob_index >= (unsigned)views[v].observation->num_blobs ||
				    claim->blob_index >= MAX_BLOBS_PER_FRAME || views[v].excluded_blobs[claim->blob_index]) {
					continue;
				}
				views[v].excluded_blobs[claim->blob_index] = true;
				excluded++;
			}
		}
	}
	return excluded;
}

void
constellation_head_relative_position(const struct xrt_pose *P_xrworld_hmd,
                                     const struct xrt_vec3 *position,
                                     struct xrt_vec3 *out)
{
	const struct xrt_vec3 forward = {0, 0, -1}, up = {0, 1, 0};
	struct xrt_vec3 head_forward;
	math_quat_rotate_vec3(&P_xrworld_hmd->orientation, &forward, &head_forward);
	struct xrt_quat inverse_heading;
	math_quat_from_angle_vector(-atan2f(-head_forward.x, -head_forward.z), &up, &inverse_heading);
	struct xrt_vec3 offset = m_vec3_sub(*position, P_xrworld_hmd->position);
	math_quat_rotate_vec3(&inverse_heading, &offset, out);
}
