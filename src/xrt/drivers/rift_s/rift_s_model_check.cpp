// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Consistency checks for Rift S controller firmware models.
 * @ingroup drv_rift_s
 */
#include "rift_s_model_check.h"

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
using V3 = Eigen::Vector3f;
using M3 = Eigen::Matrix3f;

V3
vec(const xrt_vec3 &v)
{
	return V3(v.x, v.y, v.z);
}

float
degrees(float radians)
{
	return radians * 180.0f / float(M_PI);
}

rift_s_matrix_summary
summarise(const M3 &m)
{
	rift_s_matrix_summary s{};
	s.determinant = m.determinant();
	s.reflection = s.determinant < 0;
	Eigen::JacobiSVD<M3> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
	s.singular_min = svd.singularValues().minCoeff();
	s.singular_max = svd.singularValues().maxCoeff();
	M3 d = M3::Identity();
	d(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant() < 0 ? -1 : 1;
	M3 r = svd.matrixU() * d * svd.matrixV().transpose();
	s.rotation_deg = degrees(std::acos(std::clamp((r.trace() - 1) / 2, -1.0f, 1.0f)));
	return s;
}

M3
nearest_rotation(const M3 &m)
{
	Eigen::JacobiSVD<M3> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
	M3 d = M3::Identity();
	d(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant() < 0 ? -1 : 1;
	return svd.matrixU() * d * svd.matrixV().transpose();
}

/* Firmware matrices are stored row-major, as parsed. */
M3
matrix3(const xrt_matrix_3x3 &m)
{
	M3 out;
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++)
			out(r, c) = m.v[3 * r + c];
	return out;
}

float
rotation_between_deg(const M3 &a, const M3 &b)
{
	M3 between = nearest_rotation(a).transpose() * nearest_rotation(b);
	return degrees(std::acos(std::clamp((between.trace() - 1) / 2, -1.0f, 1.0f)));
}

V3
relate(const V3 &v, rift_s_hand_relation relation)
{
	V3 out = v;
	if (relation != RIFT_S_HANDS_IDENTICAL)
		out(relation - RIFT_S_HANDS_MIRROR_X) = -out(relation - RIFT_S_HANDS_MIRROR_X);
	return out;
}
} // namespace

extern "C" void
rift_s_controller_model_check(const struct rift_s_controller_imu_calibration *c,
                              struct rift_s_controller_model_report *out)
{
	*out = {};
	if (c == nullptr || c->leds == nullptr || c->num_leds == 0)
		return;
	out->num_leds = c->num_leds;
	out->num_lensing_models = c->num_lensing_models;
	// The parser reads "Point%d" for 0..n-1, so a gap fails parsing instead of renumbering.
	out->ids_sequential = true;
	V3 centroid = V3::Zero();
	for (unsigned i = 0; i < c->num_leds; i++)
		centroid += vec(c->leds[i].pos);
	centroid /= float(c->num_leds);
	out->centroid = {centroid.x(), centroid.y(), centroid.z()};
	out->normal_length_min = std::numeric_limits<float>::infinity();
	out->angle_x_min = out->angle_y_min = std::numeric_limits<float>::infinity();
	out->angle_x_max = out->angle_y_max = -std::numeric_limits<float>::infinity();
	for (unsigned i = 0; i < c->num_leds; i++) {
		const rift_s_led &led = c->leds[i];
		float length = vec(led.dir).norm();
		out->normal_length_min = std::min(out->normal_length_min, length);
		out->normal_length_max = std::max(out->normal_length_max, length);
		out->outward_normals += vec(led.dir).dot(vec(led.pos) - centroid) > 0;
		out->angle_x_min = std::min(out->angle_x_min, led.angles.x);
		out->angle_x_max = std::max(out->angle_x_max, led.angles.x);
		out->angle_y_min = std::min(out->angle_y_min, led.angles.y);
		out->angle_y_max = std::max(out->angle_y_max, led.angles.y);
	}
	out->imu_position = c->imu_position;
	out->imu_to_centroid_m = (vec(c->imu_position) - centroid).norm();
	M3 accel = matrix3(c->accel.rectification), gyro = matrix3(c->gyro.rectification);
	out->accel_rectification = summarise(accel);
	out->gyro_rectification = summarise(gyro);
	out->accel_gyro_axes_deg = rotation_between_deg(accel, gyro);
	M3 accel_tracked = matrix3(c->accel_calibration.matrix), gyro_tracked = matrix3(c->gyro_calibration.matrix);
	out->accel_calibration = summarise(accel_tracked);
	out->gyro_calibration = summarise(gyro_tracked);
	out->accel_calibration_offset_norm = vec(c->accel_calibration.offset).norm();
	out->gyro_calibration_offset_norm = vec(c->gyro_calibration.offset).norm();
	out->accel_calibration_values = c->accel_calibration.num_values;
	out->gyro_calibration_values = c->gyro_calibration.num_values;
	out->accel_calibration_vs_rectification_deg = rotation_between_deg(accel, accel_tracked);
	out->gyro_calibration_vs_rectification_deg = rotation_between_deg(gyro, gyro_tracked);
}

extern "C" void
rift_s_controller_gravity_check(const struct rift_s_controller_imu_calibration *c,
                                const struct xrt_vec3 *calibrated_mean,
                                struct rift_s_controller_gravity_report *out)
{
	*out = {};
	if (c == nullptr || calibrated_mean == nullptr || c->accel_calibration.num_values != 12)
		return;
	M3 factory = matrix3(c->accel_calibration.matrix);
	if (std::abs(factory.determinant()) < 1e-6f)
		return;
	// Recover sensor units from the factory-calibrated sample, then compare the legacy fields.
	V3 tracked = vec(*calibrated_mean);
	V3 sample = factory.inverse() * tracked + vec(c->accel_calibration.offset);
	V3 rectified = matrix3(c->accel.rectification) * (sample - vec(c->accel.offset));
	out->rectified_mps2 = rectified.norm();
	out->tracked_mps2 = tracked.norm();
	if (out->rectified_mps2 <= 0 || out->tracked_mps2 <= 0)
		return;
	float dot = rectified.normalized().dot(tracked.normalized());
	out->direction_deg = degrees(std::acos(std::clamp(dot, -1.0f, 1.0f)));
	out->valid = true;
}

extern "C" void
rift_s_controller_compare_hands(const struct rift_s_controller_imu_calibration *left,
                                const struct rift_s_controller_imu_calibration *right,
                                struct rift_s_controller_hand_comparison *out)
{
	*out = {};
	if (left == nullptr || right == nullptr || left->leds == nullptr || right->leds == nullptr ||
	    left->num_leds == 0 || right->num_leds == 0)
		return;
	float best = std::numeric_limits<float>::infinity();
	for (int r = 0; r < RIFT_S_HANDS_RELATION_COUNT; r++) {
		auto relation = rift_s_hand_relation(r);
		double squared = 0, normal = 0;
		for (unsigned i = 0; i < right->num_leds; i++) {
			V3 p = relate(vec(right->leds[i].pos), relation);
			V3 n = relate(vec(right->leds[i].dir), relation).normalized();
			float nearest = std::numeric_limits<float>::infinity();
			unsigned match = 0;
			for (unsigned j = 0; j < left->num_leds; j++) {
				float d = (vec(left->leds[j].pos) - p).norm();
				if (d < nearest) {
					nearest = d;
					match = j;
				}
			}
			squared += double(nearest) * nearest;
			float dot = std::clamp(n.dot(vec(left->leds[match].dir).normalized()), -1.0f, 1.0f);
			normal += degrees(std::acos(dot));
		}
		out->position_rms_m[r] = float(std::sqrt(squared / right->num_leds));
		out->normal_mean_deg[r] = float(normal / right->num_leds);
		out->imu_difference_m[r] = (relate(vec(right->imu_position), relation) - vec(left->imu_position)).norm();
		if (out->position_rms_m[r] < best) {
			best = out->position_rms_m[r];
			out->best = relation;
		}
	}
}

extern "C" const char *
rift_s_hand_relation_name(enum rift_s_hand_relation relation)
{
	switch (relation) {
	case RIFT_S_HANDS_IDENTICAL: return "identical";
	case RIFT_S_HANDS_MIRROR_X: return "mirror_x";
	case RIFT_S_HANDS_MIRROR_Y: return "mirror_y";
	case RIFT_S_HANDS_MIRROR_Z: return "mirror_z";
	default: return "unknown";
	}
}
