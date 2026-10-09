// Copyright 2020,2024 Collabora, Ltd.
// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once

#include "math/m_api.h"
#include "math/m_space.h"
#include "openvr_driver.h"
#include <cmath>

inline void
copy_vec3(struct xrt_vec3 *from, double *to)
{
	to[0] = from->x;
	to[1] = from->y;
	to[2] = from->z;
}

inline void
copy_quat(struct xrt_quat *from, vr::HmdQuaternion_t *to)
{
	to->x = from->x;
	to->y = from->y;
	to->z = from->z;
	to->w = from->w;
}

inline void
copy_pose_velocities(struct xrt_space_relation *rel, vr::DriverPose_t *m_pose)
{
	// Controller GetPose reuses DriverPose_t. SteamVR has no velocity validity bits,
	// so an invalid Monado velocity must not leave the previous frame's prediction active.
	for (auto &velocity : m_pose->vecVelocity)
		velocity = 0;
	for (auto &velocity : m_pose->vecAngularVelocity)
		velocity = 0;
	if ((rel->relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT) != 0 &&
	    std::isfinite(rel->linear_velocity.x) && std::isfinite(rel->linear_velocity.y) &&
	    std::isfinite(rel->linear_velocity.z)) {
		// linear velocity in world space
		copy_vec3(&rel->linear_velocity, m_pose->vecVelocity);
	}

	if ((rel->relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT) != 0 &&
	    std::isfinite(rel->angular_velocity.x) && std::isfinite(rel->angular_velocity.y) &&
	    std::isfinite(rel->angular_velocity.z)) {
		// angular velocity reported by monado in world space,
		// expected by steamvr to be in "controller space"
		struct xrt_quat orientation_inv;
		math_quat_invert(&rel->pose.orientation, &orientation_inv);

		struct xrt_vec3 vel;
		math_quat_rotate_derivative(&orientation_inv, &rel->angular_velocity, &vel);

		if (std::isfinite(vel.x) && std::isfinite(vel.y) && std::isfinite(vel.z))
			copy_vec3(&vel, m_pose->vecAngularVelocity);
	}
}

inline bool
finite_pose(const struct xrt_pose &p)
{
	float norm = p.orientation.x * p.orientation.x + p.orientation.y * p.orientation.y +
	             p.orientation.z * p.orientation.z + p.orientation.w * p.orientation.w;
	return std::isfinite(norm) && norm >= 0.9f && norm <= 1.1f && std::isfinite(p.position.x) &&
	       std::isfinite(p.position.y) && std::isfinite(p.position.z);
}

inline void
apply_pose(struct xrt_space_relation *rel,
           vr::DriverPose_t *m_pose,
           enum xrt_space_relation_flags position_flag = XRT_SPACE_RELATION_POSITION_TRACKED_BIT,
           bool require_position = false)
{
	if (!finite_pose(rel->pose)) {
		for (auto &velocity : m_pose->vecVelocity)
			velocity = 0;
		for (auto &velocity : m_pose->vecAngularVelocity)
			velocity = 0;
		m_pose->result = vr::TrackingResult_Running_OutOfRange;
		m_pose->poseIsValid = false;
		return;
	}
	if ((rel->relation_flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) != 0) {
		copy_quat(&rel->pose.orientation, &m_pose->qRotation);
	} else {
		m_pose->result = vr::TrackingResult_Running_OutOfRange;
		m_pose->poseIsValid = false;
	}
	if (require_position && (rel->relation_flags & position_flag) == 0) {
		m_pose->result = vr::TrackingResult_Running_OutOfRange;
		m_pose->poseIsValid = false;
	}
	if ((rel->relation_flags & position_flag) != 0)
		copy_vec3(&rel->pose.position, m_pose->vecPosition);
	copy_pose_velocities(rel, m_pose);
}

inline void
transform_rift_s_touch_relation(struct xrt_space_relation *rel,
                                const struct xrt_pose *grip_to_model,
                                const struct xrt_pose *origin)
{
	const auto position_flags = rel->relation_flags &
	                            (XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
	// m_relation_chain promotes orientation-only relations to positioned 3DoF devices and
	// otherwise discards their estimated coordinates. Transform the estimate, then restore
	// the controller's own confidence instead of publishing that promotion as a tracked fix.
	if ((rel->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0)
		rel->relation_flags = static_cast<enum xrt_space_relation_flags>(
		    rel->relation_flags | XRT_SPACE_RELATION_POSITION_VALID_BIT);
	struct xrt_relation_chain chain = {};
	m_relation_chain_push_pose_if_not_identity(&chain, grip_to_model);
	m_relation_chain_push_relation(&chain, rel);
	m_relation_chain_push_pose_if_not_identity(&chain, origin);
	m_relation_chain_resolve(&chain, rel);
	rel->relation_flags = static_cast<enum xrt_space_relation_flags>(
	    (rel->relation_flags & ~(XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT)) |
	    position_flags);
}

// Oculus' controller adapter admits orientation-valid poses, copies their estimated
// position even without PositionValid, and reports that case as Running_OutOfRange.
inline void
apply_rift_s_touch_pose(struct xrt_space_relation *rel, vr::DriverPose_t *m_pose)
{
	bool connected = m_pose->deviceIsConnected;
	*m_pose = {};
	m_pose->deviceIsConnected = connected;
	bool position_valid = (rel->relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0;
	m_pose->willDriftInYaw = !position_valid;
	m_pose->result = vr::TrackingResult_Uninitialized;
	if ((rel->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0)
		return;
	if (!finite_pose(rel->pose)) {
		m_pose->result = vr::TrackingResult_Running_OutOfRange;
		return;
	}
	m_pose->poseIsValid = true;
	m_pose->result = position_valid ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange;
	copy_quat(&rel->pose.orientation, &m_pose->qRotation);
	copy_vec3(&rel->pose.position, m_pose->vecPosition);
	copy_pose_velocities(rel, m_pose);
}

/*!
 * SteamVR draws a render model with its origin at the device pose, and its OpenXR runtime takes
 * grip from the model's openxr_grip component. Monado reports grip, so the device pose is moved
 * from grip to the model origin. Rift S Touch (oculus_rifts_controller_*): grip is +20.6 degrees
 * about X at (+-0.007, -0.00183, 0.10195) m in the model, in the handle 10 cm behind the origin.
 */
inline struct xrt_pose
rift_s_touch_grip_to_model(enum xrt_hand hand)
{
	struct xrt_pose model_grip = XRT_POSE_IDENTITY;
	const struct xrt_vec3 axis = {1, 0, 0};
	math_quat_from_angle_vector((float)DEG_TO_RAD(20.6), &axis, &model_grip.orientation);
	model_grip.position = {hand == XRT_HAND_LEFT ? 0.007f : -0.007f, -0.00182941f, 0.1019482f};
	struct xrt_pose grip_model;
	math_pose_invert(&model_grip, &grip_model);
	return grip_model;
}

/*!
 * Resting hand position for a controller that has never had a position fix: 0.2 m to the side,
 * 0.5 m below and 0.3 m in front of the head, following its heading only. The pose stays
 * invalid; this only keeps SteamVR from drawing it at the tracking origin under the floor.
 */
inline struct xrt_vec3
nominal_hand_position(const struct xrt_pose &head, enum xrt_hand hand)
{
	struct xrt_vec3 forward;
	const struct xrt_vec3 minus_z = {0, 0, -1};
	math_quat_rotate_vec3(&head.orientation, &minus_z, &forward);
	float length = std::hypot(forward.x, forward.z);
	float fx = length > 1e-3f ? forward.x / length : 0, fz = length > 1e-3f ? forward.z / length : -1;
	// Right is forward x up.
	float side = hand == XRT_HAND_LEFT ? -0.2f : 0.2f;
	return {head.position.x + 0.3f * fx - side * fz, head.position.y - 0.5f, head.position.z + 0.3f * fz + side * fx};
}
