/*
 * Copyright 2020 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/*!
 * @file
 * @brief  Oculus Rift S Touch Controller interface
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_rift_s
 */

#ifndef RIFT_S_CONTROLLER_H
#define RIFT_S_CONTROLLER_H

#include "math/m_imu_3dof.h"

#include "os/os_time.h"
#include "tracking/t_constellation_tracking.h"
#include "xrt/xrt_device.h"

#include "rift_s.h"
#include "rift_s_position_filter.h"

#define MAX_LOG_SIZE 1024

// About two seconds at the observed 500 Hz report rate. Not a runtime constant.
#define RIFT_S_ATTITUDE_HISTORY_CAPACITY 1024

struct rift_s_attitude_sample
{
	timepoint_ns device_capture_ns;
	timepoint_ns capture_ns;
	timepoint_ns arrival_ns;
	struct xrt_quat orientation;
	struct xrt_vec3 gyro;
	bool tilt_valid;
};

// Monado policy: a heading prior needs an optical yaw measurement no older than this.
#define RIFT_S_HEADING_PRIOR_MAX_AGE_NS (2 * U_TIME_1S_IN_NS)

//! Why an IMU sample's time was rejected; the parts of reject_clock.
enum rift_s_clock_reject
{
	//! The 32-bit device timestamp went backwards.
	RIFT_S_CLOCK_BACKWARDS,
	//! Device capture time not after the last attitude history sample.
	RIFT_S_CLOCK_HISTORY_ORDER,
	//! The skew tracker could not map device time to host time.
	RIFT_S_CLOCK_UNMAPPED,
	//! The mapping jumped by more than 2 ms; the attitude history restarts.
	RIFT_S_CLOCK_DISCONTINUITY,
	//! The mapped host capture time was not after the previous one.
	RIFT_S_CLOCK_MAPPED_ORDER,
	RIFT_S_CLOCK_REJECT_COUNT,
};

enum rift_s_attitude_rejection
{
	RIFT_S_ATTITUDE_EMPTY,
	RIFT_S_ATTITUDE_INVALID_TIME,
	RIFT_S_ATTITUDE_TOO_OLD,
	RIFT_S_ATTITUDE_TOO_NEW,
	RIFT_S_ATTITUDE_TILT_UNREADY,
	RIFT_S_ATTITUDE_RADIO_GAP,
	RIFT_S_ATTITUDE_REJECT_COUNT,
};

struct rift_s_fusion_diagnostics
{
	//! Optical observations that updated position.
	uint64_t optical_accepted;
	//! Accepted observations whose fit measured yaw from at least three distinct LEDs.
	uint64_t optical_reliable;
	//! Accepted observations whose orientation was the IMU prior (position-only update).
	uint64_t optical_position_only;
	//! Entries into the unacquired position state.
	uint64_t fallback_entries;
	//! Calibration JSON blocks that failed to parse (bad transfer or unexpected FlsVersion).
	uint64_t calibration_parse_failures;
	timepoint_ns last_summary_ns;
	uint64_t reject_stale;
	uint64_t reject_nonfinite;
	//! Device timestamp went backwards, or the host clock mapping was discontinuous.
	uint64_t reject_clock;
	uint64_t reject_clock_by[RIFT_S_CLOCK_REJECT_COUNT];
	//! Device time step of the last accepted IMU sample, and consecutive backwards samples.
	uint32_t last_step_us;
	uint32_t backwards_run;
	//! Device clock re-anchored after a run of backwards samples.
	uint32_t clock_resyncs;
	timepoint_ns last_clock_log_ns;
	//! Repeated IMU reports with the same device timestamp (radio retransmits); harmless.
	uint64_t reject_duplicate;
	uint64_t reject_history;
	uint64_t attitude_rejected[RIFT_S_ATTITUDE_REJECT_COUNT];
	enum rift_s_attitude_rejection last_attitude_reject;
	timepoint_ns last_attitude_reject_ns;
	timepoint_ns last_history_log_ns;
	timepoint_ns last_health_log_ns;
	const char *last_optical_result;
	timepoint_ns last_optical_processing_ns;

};

/*!
 * Resting accelerometer magnitude across the session. A calibrated accelerometer reads 9.81 m/s^2
 * at rest in every orientation; a magnitude that changes with orientation means an uncorrected
 * bias or per-axis scale, which also tilts the IMU attitude by up to a few degrees.
 */
struct rift_s_rest_stats
{
	// Current stationary window.
	struct xrt_vec3 sum;
	unsigned int samples;
	timepoint_ns start_ns, last_ns;
	// Completed 250 ms windows.
	unsigned int windows;
	float min_mps2, max_mps2;
	//! Gravity direction in the IMU frame at the smallest and largest reading.
	struct xrt_vec3 min_dir, max_dir;
	//! Mean acceleration of the window completed last.
	struct xrt_vec3 last_mean;
};

#define RIFT_S_ACCEL_BIAS_WINDOWS 16

/*!
 * Accelerometer bias from rest windows in different orientations. At rest the corrected reading
 * must have the magnitude of gravity whatever the orientation, so windows spread over enough
 * directions determine the bias along them. A weak prior keeps unobserved axes at zero. A
 * measured residual bias of 2 m/s^2 can tilt the IMU attitude by up to 12 degrees depending on
 * orientation.
 */
struct rift_s_accel_bias
{
	//! Rest-window means, at most one per 15 degree cone of directions.
	struct xrt_vec3 means[RIFT_S_ACCEL_BIAS_WINDOWS];
	unsigned int count;
	struct xrt_vec3 bias;
	//! Enough spread and a consistent fit: the bias is applied.
	bool valid;
	//! RMS of |mean - bias| - g over the stored windows, before and after the correction.
	float rms_before_mps2, rms_after_mps2;
	//! Largest angle between two stored window directions.
	float spread_deg;
};

/*!
 * Accelerometer offset from rest windows at a known attitude: the window mean minus gravity
 * rotated into the IMU frame gives the full offset from one window, without the spread of
 * orientations rift_s_accel_bias needs. The attitude is levelled by optical tilt.
 */
#define RIFT_S_ATTITUDE_BIAS_SAMPLES 32
#define RIFT_S_ATTITUDE_BIAS_MIN_SAMPLES 8

struct rift_s_attitude_bias
{
	struct xrt_vec3 samples[RIFT_S_ATTITUDE_BIAS_SAMPLES];
	unsigned int count;
	struct xrt_vec3 bias;
	//! RMS distance of the samples from their mean, m/s^2.
	float rms_mps2;
	bool valid;
};

struct rift_s_controller
{
	struct xrt_device base;

	struct os_mutex mutex;

	struct xrt_pose pose;

	/* The system this controller belongs to / receives reports from */
	struct rift_s_system *sys;

	uint64_t device_id;
	rift_s_device_type device_type;

	//! Controller tracker connection that is doing 6dof tracking of this controller
	struct t_constellation_tracked_device_connection *tracking_connection;

	//! Last timestamp of tracked pose from optical controller tracking
	timepoint_ns last_tracked_pose_ts;
	//! Last tracked pose from optical controller tracking
	struct xrt_pose last_tracked_pose;
	//! debug boolean - enable yaw updates
	bool update_yaw_from_optical;
	bool optical_heading_valid;
	//! Capture time of the last observation whose fit measured yaw.
	timepoint_ns last_optical_yaw_ts;
	//! Position covariance (m^2, row-major world) of the last accepted optical fit.
	float last_position_covariance[9];
	//! Aim/grip position: IMU acceleration integrated at IMU rate, corrected by optical fixes.
	struct rift_s_position_filter position_filter;
	//! Presentation-only translation correction after held position reacquires optics.
	struct xrt_vec3 reacquire_offset;
	timepoint_ns reacquire_started_ns;
	//! Successful and reliable optical fixes in the current track (inputs of the loss ladder).
	uint32_t track_vision_updates, track_reliable_updates;
	//! Capture time of the newest optical fix given to position_filter.
	timepoint_ns last_fix_ns;
	//! Host capture time and world-frame acceleration (gravity removed) of the newest IMU sample.
	timepoint_ns last_imu_capture_ns;
	struct xrt_vec3 world_accel;
	bool world_accel_valid;
	//! Last reported aim/grip position source, for logging transitions.
	const char *position_source;
	//! No optical position has been acquired yet.
	bool fallback_active;
	struct rift_s_fusion_diagnostics diagnostics;

	/* Debug logs */
	/* 0x04 = new log line
	 * 0x02 = parity bit, toggles each line when receiving log chars
	 * other bits, unknown */
	uint8_t log_flags;
	int log_bytes;
	uint8_t log[MAX_LOG_SIZE];

	/* IMU tracking */
	bool imu_time_valid;
	uint32_t imu_timestamp32;
	timepoint_ns last_imu_device_time_ns;
	timepoint_ns last_imu_local_time_ns;
	//! Device-to-host IMU clock estimate (minimum transport delay); allocated on the first sample.
	struct m_clock_windowed_skew_tracker *imu_clock;
	//! Current offset from imu_clock, kept to detect discontinuities.
	time_duration_ns imu_to_host_ns;
	unsigned int clock_stable_samples;
	struct rift_s_attitude_sample attitude_history[RIFT_S_ATTITUDE_HISTORY_CAPACITY];
	unsigned int attitude_history_start;
	unsigned int attitude_history_count;

	uint16_t imu_unknown_varying2;
	int16_t raw_accel[3];
	int16_t raw_gyro[3];

	struct xrt_vec3 accel;
	struct xrt_vec3 gyro;
	struct xrt_vec3 mag;
	struct m_imu_3dof fusion;

	// Gyro-compensated startup gravity, before constellation uses the tilt prior.
	bool gravity_initialized;
	bool gravity_moving;
	struct rift_s_rest_stats rest;
	struct rift_s_accel_bias accel_bias;
	timepoint_ns last_bias_log_ns;
	/*!
	 * The stationary startup reading was not gravity's magnitude: the accelerometer carries an
	 * offset that tilts its gravity direction by up to ~12 degrees depending on orientation.
	 * Until a bias is known, the fusion does not level to it; optical tilt levels instead.
	 */
	bool accel_offset;
	struct rift_s_attitude_bias attitude_bias;
	//! Reliable optical tilt corrections while accel_offset, their latest capture time and a
	//! smoothed residual tilt in degrees.
	uint32_t optical_tilt_updates;
	timepoint_ns last_optical_tilt_ns;
	float optical_tilt_residual_deg;
	timepoint_ns gravity_start_ns;
	unsigned int gravity_samples;
	struct xrt_vec3 gravity_accel_sum;
	struct xrt_vec3 gravity_gyro_sum;
	float gravity_accel_scale;

	//! Offset for IMU pose from device (from json calibration)
	struct xrt_pose P_device_imu;
	//! Offset for device pose from imu
	struct xrt_pose P_imu_device;

	//! Published OpenXR aim and grip in the firmware LED-model frame.
	struct xrt_pose P_device_aim;
	struct xrt_pose P_device_grip;

	//! Rate limit for the unconfirmed-fix debug line.
	timepoint_ns last_candidate_log_ns;

	/* Controls / buttons state */
	timepoint_ns last_controls_local_time_ns;

	/* 0x8, 0x0c 0x0d or 0xe block */
	uint8_t mask08;
	uint8_t buttons;
	uint8_t fingers;
	uint8_t mask0e;

	uint16_t trigger;
	uint16_t grip;

	int16_t joystick_x;
	int16_t joystick_y;

	uint8_t capsense_a_x;
	uint8_t capsense_b_y;
	uint8_t capsense_joystick;
	uint8_t capsense_trigger;

	uint8_t extra_bytes_len;
	uint8_t extra_bytes[48];

	bool reading_config;
	bool have_config;
	//! Earliest time to re-request the calibration block after a parse failure.
	timepoint_ns calibration_retry_ns;
	rift_s_controller_config config;
	//! Attempt 0x31 once per physical controller, with unchanged gyro calibration on failure.
	bool imu_descriptor_requested;
	//! The IR LED timing (register 0x28) was queued for this physical controller.
	bool irled_requested;
	//! The IMU may start: the descriptor was read, or waiting for it gave up.
	bool imu_descriptor_done;
	//! IMU samples wait for the descriptor until this time, then use the factory gyro calibration.
	timepoint_ns imu_descriptor_deadline_ns;
	char imu_descriptor[RIFT_S_CONTROLLER_IMU_DESCRIPTOR_SIZE + 1];
	struct rift_s_tracked_imu_calibration gyro_calibration;

	bool reading_calibration;
	bool have_calibration;
	struct rift_s_controller_imu_calibration calibration;
};

#ifdef __cplusplus
extern "C" {
#endif

struct rift_s_controller *
rift_s_controller_create(struct rift_s_system *sys, enum xrt_device_type device_type);

/*!
 * Rift S Touch frames in the SteamVR render-model frame (oculus_rifts_controller_left/right):
 * the firmware LED-model ("device") frame and the OpenXR grip.
 */
void
rift_s_controller_model_frames(bool left, struct xrt_pose *out_model_device, struct xrt_pose *out_model_grip);

//! OpenXR aim and grip in the firmware LED-model frame.
void
rift_s_controller_pose_frames(bool left, struct xrt_pose *device_aim, struct xrt_pose *device_grip);

// Bootstrap tilt from gyro-compensated acceleration; estimate gyro bias only at rest.
void
rift_s_controller_update_fusion(struct rift_s_controller *ctrl, timepoint_ns timestamp_ns);

// Record the current fused attitude with both clock readings; caller holds the mutex.
bool
rift_s_controller_record_attitude(struct rift_s_controller *ctrl,
                                  timepoint_ns device_capture_ns,
                                  timepoint_ns arrival_ns);

//! Feed one rectified, unscaled accelerometer sample and the gyro rate. True when a window completed.
bool
rift_s_rest_stats_push(struct rift_s_rest_stats *s,
                       timepoint_ns timestamp_ns,
                       const struct xrt_vec3 *accel,
                       const struct xrt_vec3 *gyro);

//! Add one completed rest window and re-estimate the bias.
void
rift_s_accel_bias_add_window(struct rift_s_accel_bias *b, const struct xrt_vec3 *mean);

//! Consecutive backwards IMU samples that re-anchor the device clock.
#define RIFT_S_IMU_RESYNC_RUN 16
//! A backwards step this large re-anchors after two samples in a row.
#define RIFT_S_IMU_RESYNC_BACKWARDS_US 1000000u

//! Whether a backwards IMU timestamp means the device clock anchor is wrong.
bool
rift_s_imu_clock_needs_resync(uint32_t backwards_us, uint32_t backwards_run);

//! Add a rest-window mean taken at a known IMU @p attitude (IMU to world).
void
rift_s_attitude_bias_add(struct rift_s_attitude_bias *b,
                         const struct xrt_vec3 *rest_mean,
                         const struct xrt_quat *attitude);

/*!
 * World-frame rotation that moves @p imu's tilt a @p fraction of the way to @p optical's, leaving
 * heading alone. Returns the full tilt difference in radians.
 */
float
rift_s_optical_tilt_correction(const struct xrt_quat *imu,
                               const struct xrt_quat *optical,
                               float fraction,
                               struct xrt_quat *out_correction);

// Interpolate a capture-time attitude, with up to 20 ms of gyro prediction for radio scheduling.
bool
rift_s_controller_get_attitude(struct rift_s_controller *ctrl,
                               timepoint_ns capture_ns,
                               struct rift_s_attitude_sample *out);

// A heading prior requires recent optical yaw agreement as well as capture-time tilt.
bool
rift_s_controller_has_heading_prior(struct rift_s_controller *ctrl, timepoint_ns capture_ns);

void
rift_s_controller_push_observed_pose(struct xrt_device *xdev, timepoint_ns frame_mono_ns, const struct xrt_pose *pose);
// Build the constellation LED model in the OpenCV-flipped IMU frame, firmware order as LED id.
bool
rift_s_controller_get_led_model(struct xrt_device *xdev, struct t_constellation_led_model *led_model);
// Accept an optical fit. Yaw is corrected only when the fit itself observed it.
void
rift_s_controller_push_pose_observation(struct xrt_device *xdev, const struct t_constellation_pose_observation *obs);

// Controller pose callback. Optical loss preserves IMU orientation and estimates hand position.
xrt_result_t
rift_s_controller_get_tracked_pose(struct xrt_device *xdev,
                                   enum xrt_input_name name,
                                   int64_t at_timestamp_ns,
                                   struct xrt_space_relation *out_relation);

void
rift_s_controller_update_configuration(struct rift_s_controller *ctrl, uint64_t device_id);
bool
rift_s_controller_handle_report(struct rift_s_controller *ctrl,
                                timepoint_ns local_ts,
                                rift_s_controller_report_t *report);

#ifdef __cplusplus
}
#endif

#endif
