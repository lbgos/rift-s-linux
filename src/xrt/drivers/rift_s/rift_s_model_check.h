// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Consistency checks for Rift S controller firmware models.
 *
 * These checks do not change tracking. They summarise per-device firmware data so left/right
 * identity, normals, IMU placement, IMU axes and mirroring can be compared on real hardware.
 *
 * @ingroup drv_rift_s
 */
#pragma once

#include "rift_s_firmware.h"

#ifdef __cplusplus
extern "C" {
#endif

//! Rotation, reflection and scale summary of a calibration matrix.
struct rift_s_matrix_summary
{
	float determinant;
	//! Angle of the nearest proper rotation (polar decomposition), degrees.
	float rotation_deg;
	//! Singular values: away from 1 means scale or non-orthogonality.
	float singular_min, singular_max;
	//! The matrix contains a reflection (determinant < 0).
	bool reflection;
};

struct rift_s_controller_model_report
{
	unsigned num_leds;
	unsigned num_lensing_models;
	//! LED ids are the PointN indices: always 0..num_leds-1 when parsing succeeded.
	bool ids_sequential;
	float normal_length_min, normal_length_max;
	//! Normals pointing away from the LED centroid (dot(dir, pos - centroid) > 0).
	unsigned outward_normals;
	struct xrt_vec3 centroid;
	struct xrt_vec3 imu_position;
	float imu_to_centroid_m;
	//! Range of the two per-LED angle fields (85/80 so far); Monado uses one global limit.
	float angle_x_min, angle_x_max, angle_y_min, angle_y_max;
	struct rift_s_matrix_summary accel_rectification;
	struct rift_s_matrix_summary gyro_rectification;
	//! Angle between the rotation parts of the accelerometer and gyro rectifications, degrees.
	float accel_gyro_axes_deg;
	//! TrackedObject AccCalibration/GyroCalibration 3x3 part: parsed, not applied by Monado.
	struct rift_s_matrix_summary accel_calibration;
	struct rift_s_matrix_summary gyro_calibration;
	float accel_calibration_offset_norm, gyro_calibration_offset_norm;
	//! Numbers in each firmware array (the 3x3 + offset layout needs 12).
	unsigned accel_calibration_values, gyro_calibration_values;
	//! Angle between the rotation parts of the TrackedObject matrix and the rectification, degrees.
	float accel_calibration_vs_rectification_deg, gyro_calibration_vs_rectification_deg;
};

enum rift_s_hand_relation
{
	RIFT_S_HANDS_IDENTICAL,
	RIFT_S_HANDS_MIRROR_X,
	RIFT_S_HANDS_MIRROR_Y,
	RIFT_S_HANDS_MIRROR_Z,
	RIFT_S_HANDS_RELATION_COUNT,
};

struct rift_s_controller_hand_comparison
{
	//! Nearest-neighbour RMS (metres) from right LEDs to left LEDs under each relation.
	float position_rms_m[RIFT_S_HANDS_RELATION_COUNT];
	//! Mean angle between matched normals under each relation, degrees.
	float normal_mean_deg[RIFT_S_HANDS_RELATION_COUNT];
	//! IMU position difference under each relation, metres.
	float imu_difference_m[RIFT_S_HANDS_RELATION_COUNT];
	enum rift_s_hand_relation best;
};

/*!
 * The resting accelerometer under both firmware calibrations. Monado uses the top-level
 * rectification and offset; the TrackedObject calibration is the alternative. On a stationary
 * controller the right one reads 9.81 m/s^2, and the direction difference is the tilt error the
 * other would introduce.
 */
struct rift_s_controller_gravity_report
{
	//! |g| under acc_m/acc_b (the driver's calibration), m/s^2.
	float rectified_mps2;
	//! |g| under the TrackedObject AccCalibration, m/s^2.
	float tracked_mps2;
	//! Angle between the two gravity directions in the IMU frame, degrees.
	float direction_deg;
	bool valid;
};

//! @p calibrated_mean is the mean resting accelerometer after factory TrackedObject calibration.
void
rift_s_controller_gravity_check(const struct rift_s_controller_imu_calibration *calibration,
                                const struct xrt_vec3 *calibrated_mean,
                                struct rift_s_controller_gravity_report *out);

void
rift_s_controller_model_check(const struct rift_s_controller_imu_calibration *calibration,
                              struct rift_s_controller_model_report *out);

void
rift_s_controller_compare_hands(const struct rift_s_controller_imu_calibration *left,
                                const struct rift_s_controller_imu_calibration *right,
                                struct rift_s_controller_hand_comparison *out);

const char *
rift_s_hand_relation_name(enum rift_s_hand_relation relation);

#ifdef __cplusplus
}
#endif
