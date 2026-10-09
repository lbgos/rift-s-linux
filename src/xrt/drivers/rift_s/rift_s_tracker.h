/*
 * Copyright 2013, Fredrik Hultin.
 * Copyright 2013, Jakob Bornecrantz.
 * Copyright 2016 Philipp Zabel
 * Copyright 2019-2022 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */

/*!
 * @file
 * @brief  HMD tracker handling
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_rift_s
 */

#pragma once

#include "math/m_imu_3dof.h"
#include "math/m_relation_history.h"
#include "os/os_threading.h"
#include "util/u_var.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#include "tracking/t_tracking.h"
#include "tracking/t_constellation_tracking.h"

#include "rift_s_firmware.h"
#include "rift_s_slam_guard.h"
#include "rift_s_world_anchor.h"

/* Oculus Rift S HMD Tracking */
#ifndef RIFT_S_TRACKER_H
#define RIFT_S_TRACKER_H

struct rift_s_hmd_config;

enum rift_s_tracker_pose
{
	RIFT_S_TRACKER_POSE_IMU,
	RIFT_S_TRACKER_POSE_LEFT_CAMERA,
	RIFT_S_TRACKER_POSE_DEVICE,
};

struct rift_s_tracker
{
	struct xrt_device base;

	//! Protects shared access to 3dof and pose storage
	struct os_mutex mutex;

	//! Serializes camera submission and SLAM session replacement.
	struct os_mutex slam_mutex;
	//! Latest-frame queue; feature matching never blocks USB, SLAM or controller ingestion.
	struct os_thread_helper world_thread;
	bool world_thread_initialized;
	struct rift_s_world_anchor *world_anchor;
	struct xrt_frame *world_frames[RIFT_S_CAMERA_COUNT];
	timepoint_ns world_last_queued_ns;
	bool world_setup;
	bool world_map_available;
	bool world_storage_failed;
	uint64_t world_command_id;
	uint32_t world_confirmations;
	struct xrt_pose world_candidate;
	uint64_t world_candidate_generation;
	timepoint_ns world_candidate_ns;

	//! Camera arrival watchdog uses the host clock, not predicted pose time.
	timepoint_ns last_camera_arrival_ns;
	//! Proximity sensor reports the headset off-head.
	bool standby;
	//! SLAM input stopped (standby or camera timeout).
	bool slam_paused;
	//! Re-initialise SLAM before submitting the next fresh camera frame.
	bool slam_restart_pending;
	uint64_t slam_generation;
	timepoint_ns slam_min_sample_ns;

	//! Whether the SLAM cameras see enough features. Under @ref mutex.
	struct rift_s_slam_feature_gate feature_gate;

	//! Last published IMU pose, frozen while the head pose is held.
	bool have_published_imu_pose;
	timepoint_ns last_published_pose_log_ns;
	struct xrt_pose published_imu_pose;

	//! Don't process IMU / video until started
	bool ready_for_data;

	struct
	{
		//! Main fusion calculator.
		struct m_imu_3dof i3dof;
		struct m_relation_history *history;

		//! The last angular velocity from the IMU, for prediction.
		struct xrt_vec3 last_angular_velocity;

		//! When did we get the last IMU sample, device clock
		uint64_t last_imu_timestamp_ns;

		//! Last IMU sample local system clock
		timepoint_ns last_imu_local_timestamp_ns;
	} fusion;

	//! Fields related to camera-based tracking (SLAM and hand tracking)
	struct
	{
		//! SLAM tracker.
		//! @todo Right now, we are not consistent in how we interface with
		//! trackers. In particular, we have a @ref xrt_tracked_slam field but not
		//! an equivalent for hand tracking.
		struct xrt_tracked_slam *slam;

		//! Set at start. Whether the SLAM tracker was initialized.
		bool slam_enabled;

		//! Set at start. Whether the hand tracker was initialized.
		bool hand_enabled;

	} tracking;

	// Correction offset poses from firmware
	struct xrt_pose device_from_imu;
	struct xrt_pose left_cam_from_imu;

	//!< Estimated offset from HMD device timestamp to local monotonic clock
	uint64_t valid_clock_observations;
	bool have_hw2mono;
	time_duration_ns hw2mono;
	timepoint_ns last_frame_time;

	//! Adjustment to apply to camera timestamps to bring them into the
	// same 32-bit range as the IMU times
	int64_t camera_ts_offset;

	//! Whether to track the HMD with 6dof SLAM or fallback to the `fusion` 3dof tracker
	bool slam_over_3dof;

	//! Last tracked pose
	struct xrt_pose pose;

	//! Rejects diverged SLAM poses and holds the position while falling back to 3DoF. Under @ref mutex.
	struct rift_s_slam_guard slam_guard;

	/* Stereo calibration for the front 2 cameras */
	struct t_stereo_camera_calibration *stereo_calib;
	struct t_slam_calibration slam_calib;

	/* Input sinks that the camera delivers SLAM frames to */
	struct xrt_slam_sinks in_slam_sinks;

	/* SLAM/HT sinks we deliver imu and frame data to */
	struct xrt_slam_sinks slam_sinks;

	struct xrt_device *handtracker;

	//! Calibration data for constellation tracking
	struct t_constellation_camera_group constellation_calib;
	// Constellation tracker
	struct t_constellation_tracker *controller_tracker;
	struct xrt_frame_sink *controller_sink; //!< Sink to send controller frames to tracker

	struct
	{
		struct u_var_button hmd_screen_enable_btn;
		struct u_var_button switch_tracker_btn;
		char hand_status[128];
		char slam_status[128];
	} gui;
};

#ifdef __cplusplus
extern "C" {
#endif

struct rift_s_tracker *
rift_s_tracker_create(struct xrt_tracking_origin *origin,
                      struct xrt_frame_context *xfctx,
                      struct rift_s_hmd_config *hmd_config);
void
rift_s_tracker_start(struct rift_s_tracker *t);
void
rift_s_tracker_set_standby(struct rift_s_tracker *t, bool standby);
void
rift_s_tracker_destroy(struct rift_s_tracker *t);
void
rift_s_tracker_add_debug_ui(struct rift_s_tracker *t, void *root);

struct t_constellation_tracked_device_connection *
rift_s_tracker_add_controller(struct rift_s_tracker *t,
                              struct xrt_device *xdev,
                              struct t_constellation_tracked_device_callbacks *cb);

struct xrt_slam_sinks *
rift_s_tracker_get_slam_sinks(struct rift_s_tracker *t);
struct xrt_device *
rift_s_tracker_get_hand_tracking_device(struct rift_s_tracker *t);

void
rift_s_tracker_clock_update(struct rift_s_tracker *t, uint64_t device_timestamp_ns, timepoint_ns local_timestamp_ns);

void
rift_s_tracker_imu_update(struct rift_s_tracker *t,
                          uint64_t device_timestamp_ns,
                          const struct xrt_vec3 *accel,
                          const struct xrt_vec3 *gyro);

void
rift_s_tracker_imu_finish(struct rift_s_tracker *t);

void
rift_s_tracker_push_slam_frames(struct rift_s_tracker *t,
                                uint64_t frame_ts_ns,
                                struct xrt_frame *frames[RIFT_S_CAMERA_COUNT]);

void
rift_s_tracker_push_controller_frameset(struct rift_s_tracker *t, uint64_t frame_ts_ns, struct xrt_frame *frameset);

void
rift_s_tracker_get_tracked_pose(struct rift_s_tracker *t,
                                enum rift_s_tracker_pose pose,
                                uint64_t at_timestamp_ns,
                                struct xrt_space_relation *out_relation);

#ifdef __cplusplus
}
#endif

#endif
