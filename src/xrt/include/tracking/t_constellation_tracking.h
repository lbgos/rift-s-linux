// Copyright 2023 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Implementation of LED constellation tracking logic
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup constellation
 */
#pragma once

#include <stdint.h>

#include "os/os_threading.h"
#include "tracking/t_tracking.h"
#include "tracking/t_led_models.h"
#include "util/u_sink.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup constellation LED constellation tracking
 * @ingroup tracking
 *
 * @brief Tracker for devices with LED constellations
 */

/*!
 * @dir tracking/constellation
 *
 * @brief @ref constellation tracking files.
 */

struct t_constellation_tracker;
struct t_constellation_tracked_device_connection;

struct t_constellation_camera
{
	//!< IMU to camera pose
	struct xrt_pose P_imu_cam;
	//! ROI in the full frame mosaic
	struct xrt_rect roi;
	//! Intrinsics and distortion parameters
	struct t_camera_calibration calibration;
	//! Native Fisheye62 coefficients, when a KB4 approximation would discard calibration.
	bool fisheye62_valid;
	float fisheye62_radial[6], fisheye62_p1, fisheye62_p2;
	//! Minimum blob brightness threshold
	uint8_t min_threshold;
	//! Minimum blob brightness threshold for pixel inclusion
	uint8_t blob_min_threshold;
	//! Threshold at which a group of pixels become a detected blob
	uint8_t blob_detect_threshold;
	//! The index into the slam tracking camera array this camera represents
	size_t slam_tracking_index;
};

struct t_constellation_camera_group
{
	int cam_count; //!< Number of cameras
	struct t_constellation_camera cams[XRT_TRACKING_MAX_SLAM_CAMS];
};

int
t_constellation_tracker_create(struct xrt_frame_context *xfctx,
                               struct xrt_device *hmd_xdev,
                               struct t_constellation_camera_group *cams,
                               struct t_constellation_tracker **out_tracker,
                               struct xrt_frame_sink **out_sink,
                               struct xrt_device_masks_sink *controller_mask_sink);

/*!
 * One verified optical measurement of a tracked device at image capture time.
 *
 * The pose is the device pose in the OpenXR world frame, as passed to push_observed_pose.
 * Covariances describe the optical fit only; there is no velocity estimate yet.
 */
struct t_constellation_pose_observation
{
	timepoint_ns capture_ns;
	struct xrt_pose pose;
	//! The optical fit itself constrained yaw. Otherwise orientation is the IMU prior echoed back.
	bool orientation_observed;
	//! Accepted observations (repeated camera views counted) and distinct physical LEDs.
	uint32_t inliers;
	uint32_t distinct_leds;
	//! Information rank of the optical fit: 3 (translation) or 4 (translation and yaw).
	uint32_t information_rank;
	//! Row-major world position covariance in m^2.
	float position_covariance[9];
	//! Yaw variance in rad^2, INFINITY when not observed.
	float yaw_variance;
	//! Object-space RMS cost in metres and the hypothesis that produced the pose.
	float cost_m;
	const char *hypothesis;
};

struct t_constellation_tracked_device_callbacks
{
	bool (*get_led_model)(struct xrt_device *xdev, struct t_constellation_led_model *led_model);
	void (*notify_frame_received)(struct xrt_device *xdev, uint64_t frame_mono_ns, uint64_t frame_sequence);
	void (*push_observed_pose)(struct xrt_device *xdev, timepoint_ns frame_mono_ns, const struct xrt_pose *pose);
	void (*push_brightness_update)(struct xrt_device *xdev, uint8_t average_brightness);
	//! True only when optical heading and capture-time attitude support a rotation prior.
	bool (*has_heading_prior)(struct xrt_device *xdev, timepoint_ns capture_ns);
	//! Optional: preferred over push_observed_pose when set, carries fit quality and yaw observability.
	void (*push_pose_observation)(struct xrt_device *xdev, const struct t_constellation_pose_observation *obs);
};

struct t_constellation_tracked_device_connection *
t_constellation_tracker_add_device(struct t_constellation_tracker *ct,
                                   struct xrt_device *xdev,
                                   struct t_constellation_tracked_device_callbacks *cb);
void
t_constellation_tracked_device_connection_disconnect(struct t_constellation_tracked_device_connection *ctdc);

#ifdef __cplusplus
}
#endif
