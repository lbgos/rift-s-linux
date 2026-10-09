/*
 * Copyright 2013, Fredrik Hultin.
 * Copyright 2013, Jakob Bornecrantz.
 * Copyright 2016 Philipp Zabel
 * Copyright 2019-2022 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 */
/*!
 * @file
 * @brief  Driver code for Oculus Rift S headsets
 *
 * Implementation for the HMD communication, calibration and
 * IMU integration.
 *
 * Ported from OpenHMD
 *
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_rift_s
 */

/* Oculus Rift S Driver - HID/USB Driver Implementation */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <assert.h>
#include <ctype.h>
#include <unistd.h>

#include "math/m_api.h"
#include "util/u_debug.h"
#include "math/m_vec3.h"

#include "os/os_time.h"

#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"

#include "xrt/xrt_device.h"

#include "rift_s_hmd.h"


DEBUG_GET_ONCE_BOOL_OPTION(steamvr_bench, "MONADO_STEAMVR_BENCH", false)

static xrt_result_t
rift_s_get_tracked_pose(struct xrt_device *xdev,
                        enum xrt_input_name name,
                        int64_t at_timestamp_ns,
                        struct xrt_space_relation *out_relation)
{
	struct rift_s_hmd *hmd = (struct rift_s_hmd *)(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&hmd->base, rift_s_log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	U_ZERO(out_relation);

	rift_s_tracker_get_tracked_pose(hmd->tracker, RIFT_S_TRACKER_POSE_DEVICE, at_timestamp_ns, out_relation);
	return XRT_SUCCESS;
}

void
rift_s_hmd_handle_report(struct rift_s_hmd *hmd, timepoint_ns local_ts, rift_s_hmd_report_t *report)
{
	struct rift_s_imu_config_info_t *imu_config = &hmd->config->imu_config_info;
	struct rift_s_imu_calibration *imu_calibration = &hmd->config->imu_calibration;

	const uint32_t TICK_LEN_US = 1000000 / imu_config->imu_hz;
	uint32_t dt = TICK_LEN_US;

	int n_samples = 0;

	for (int i = 0; i < 3; i++) {
		rift_s_hmd_imu_sample_t *s = report->samples + i;
		if (s->marker & 0x80)
			break; /* Sample (and remaining ones) are invalid */
		n_samples++;
	}

	/* Check that there's at least 1 valid sample */
	if (n_samples == 0)
		return;

	if (hmd->last_imu_timestamp_ns != 0) {
		/* Avoid wrap-around on 32-bit device times */
		dt = report->timestamp - hmd->last_imu_timestamp32;
	} else {
		hmd->last_imu_timestamp_ns = (timepoint_ns)(report->timestamp) * OS_NS_PER_USEC;
		hmd->last_imu_timestamp32 = report->timestamp;
	}

	/* Give the tracker an update for matching local clock to device. The sample ts we're
	 * given seems to be the time the first IMU sample was captured, but the local_ts
	 * is USB packet arrival time, which is after the last IMU sample was captured,
	 * so calculate the correct imu timestamp accordingly */
	uint64_t packet_duration_us = (n_samples - 1) * TICK_LEN_US + dt;
	uint64_t end_imu_timestamp_ns = hmd->last_imu_timestamp_ns + (OS_NS_PER_USEC * packet_duration_us);

	rift_s_tracker_clock_update(hmd->tracker, end_imu_timestamp_ns, local_ts);

	const float gyro_scale = 1.0 / imu_config->gyro_scale;
	const float accel_scale = MATH_GRAVITY_M_S2 / imu_config->accel_scale;
	const float temperature_scale = 1.0 / imu_config->temperature_scale;
	const float temperature_offset = imu_config->temperature_offset;

	for (int i = 0; i < 3; i++) {
		rift_s_hmd_imu_sample_t *s = report->samples + i;

		if (s->marker & 0x80)
			break; /* Sample (and remaining ones) are invalid */

		struct xrt_vec3 raw_accel, raw_gyro;
		struct xrt_vec3 accel, gyro;

		raw_gyro.x = DEG_TO_RAD(gyro_scale * s->gyro[0]);
		raw_gyro.y = DEG_TO_RAD(gyro_scale * s->gyro[1]);
		raw_gyro.z = DEG_TO_RAD(gyro_scale * s->gyro[2]);

		raw_accel.x = accel_scale * s->accel[0];
		raw_accel.y = accel_scale * s->accel[1];
		raw_accel.z = accel_scale * s->accel[2];

		/* Apply correction offsets first, then rectify */
		accel = m_vec3_sub(raw_accel, imu_calibration->accel.offset_at_0C);
		gyro = m_vec3_sub(raw_gyro, imu_calibration->gyro.offset);

		math_matrix_3x3_transform_vec3(&imu_calibration->accel.rectification, &accel, &accel);
		math_matrix_3x3_transform_vec3(&imu_calibration->gyro.rectification, &gyro, &gyro);

		/* FIXME: This doesn't seem to produce the right numbers, but it's OK - we don't use it anyway */
		hmd->temperature = temperature_scale * (s->temperature - temperature_offset) + 25;

#if 0
		RIFT_S_DEBUG("Sample %d dt %f ts %" PRIu64 " report ts %u "
			"accel %f %f %f (len %f) gyro %f %f %f",
			i, (double)(dt) / (1000000), hmd->last_imu_timestamp_ns,
			report->timestamp,
			accel.x, accel.y, accel.z, m_vec3_len(raw_accel),
			gyro.x, gyro.y, gyro.z);
#endif

		// Send the sample to the pose tracker
		rift_s_tracker_imu_update(hmd->tracker, hmd->last_imu_timestamp_ns, &accel, &gyro);

		hmd->last_imu_timestamp_ns += (uint64_t)dt * OS_NS_PER_USEC;
		hmd->last_imu_timestamp32 += dt;
		dt = TICK_LEN_US;
	}
}

static xrt_result_t
rift_s_compute_distortion(struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *result)
{
	struct rift_s_hmd *hmd = (struct rift_s_hmd *)(xdev);
	rift_s_optics_compute(&hmd->optics[view], u, v, result);
	return XRT_SUCCESS;
}

#if 0
static int
dump_fw_block(struct os_hid_device *handle, uint8_t block_id) {
	int res;
	char *data = NULL;
	int len;

	res = rift_s_read_firmware_block (handle, block_id, &data, &len);
	if (res	< 0)
			return res;

	free (data);
	return 0;
}
#endif

/* Parse a positive projection-tangent scale from the environment; 1.0 if unset or invalid. */
DEBUG_GET_ONCE_NUM_OPTION(rift_s_ipd_mm, "RIFT_S_IPD_MM", 63.5)

static float
parse_fov_scale(const char *name)
{
	const char *text = getenv(name);
	if (text == NULL) {
		return 1.0f;
	}
	char *end;
	const float parsed = strtof(text, &end);
	const bool consumed = end != text;
	while (isspace((unsigned char)*end)) {
		end++;
	}
	struct rift_s_optics check;
	if (consumed && *end == '\0' && rift_s_optics_init(&check, 0, 0.0635f, parsed, parsed)) {
		return parsed;
	}
	RIFT_S_WARN("Invalid %s; using 1.0", name);
	return 1.0f;
}

/* Re-read the IPD file at most twice a second; ignores missing or out-of-range values. */
static void
rift_s_poll_ipd(struct rift_s_hmd *hmd)
{
	int64_t now = os_monotonic_get_ns();
	if (now < hmd->ipd_next_poll_ns || hmd->ipd_path[0] == '\0') {
		return;
	}
	hmd->ipd_next_poll_ns = now + 500 * 1000000ll;
	FILE *f = fopen(hmd->ipd_path, "r");
	if (f == NULL) {
		return;
	}
	float mm = 0;
	if (fscanf(f, "%f", &mm) == 1 && mm >= 50.0f && mm <= 80.0f && fabsf(mm / 1000.0f - hmd->ipd_m) > 1e-5f) {
		hmd->ipd_m = mm / 1000.0f;
		RIFT_S_INFO("Rift S IPD set to %.1f mm", mm);
	}
	fclose(f);
}

static xrt_result_t
rift_s_get_view_poses(struct xrt_device *xdev,
                      const struct xrt_vec3 *default_eye_relation,
                      int64_t at_timestamp_ns,
                      enum xrt_view_type view_type,
                      uint32_t view_count,
                      struct xrt_space_relation *out_head_relation,
                      struct xrt_fov *out_fovs,
                      struct xrt_pose *out_poses)
{
	struct rift_s_hmd *hmd = (struct rift_s_hmd *)xdev;
	rift_s_poll_ipd(hmd);
	struct xrt_vec3 eye_relation = *default_eye_relation;
	eye_relation.x = hmd->ipd_m;
	return u_device_get_view_poses(xdev, &eye_relation, at_timestamp_ns, view_type, view_count, out_head_relation,
	                               out_fovs, out_poses);
}

static void
rift_s_hmd_destroy(struct xrt_device *xdev)
{
	struct rift_s_hmd *hmd = (struct rift_s_hmd *)(xdev);

	DRV_TRACE_MARKER();

	RIFT_S_DEBUG("Destroying HMD");

	/* Tell the tracker there will be no more IMU */
	rift_s_tracker_imu_finish(hmd->tracker);

	/* Remove this device from the system */
	rift_s_system_remove_hmd(hmd->sys);

	/* Drop the reference to the system */
	rift_s_system_reference(&hmd->sys, NULL);

	u_var_remove_root(hmd);

	u_device_free(&hmd->base);
}

struct rift_s_hmd *
rift_s_hmd_create(struct rift_s_system *sys, const unsigned char *hmd_serial_no, struct rift_s_hmd_config *config)
{
	DRV_TRACE_MARKER();

	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	struct rift_s_hmd *hmd = U_DEVICE_ALLOCATE(struct rift_s_hmd, flags, 1, 0);
	if (hmd == NULL) {
		return NULL;
	}

	/* Take a reference to the rift_s_system */
	rift_s_system_reference(&hmd->sys, sys);

	hmd->config = config;

	hmd->base.tracking_origin = &sys->base;

	hmd->base.update_inputs = u_device_noop_update_inputs;
	hmd->base.get_tracked_pose = rift_s_get_tracked_pose;
	hmd->base.get_view_poses = rift_s_get_view_poses;
	hmd->ipd_m = debug_get_num_option_rift_s_ipd_mm() / 1000.0f;
	if (hmd->ipd_m < 0.05f || hmd->ipd_m > 0.08f) {
		hmd->ipd_m = 0.0635f;
	}
	snprintf(hmd->ipd_path, sizeof(hmd->ipd_path), "%s/rift-s-ipd.conf", getenv("HOME"));
	rift_s_poll_ipd(hmd);
	RIFT_S_INFO("Rift S IPD %.1f mm", hmd->ipd_m * 1000.0f);
	hmd->base.destroy = rift_s_hmd_destroy;
	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;

	hmd->tracker = rift_s_system_get_tracker(sys);
	hmd->base.camera = hmd->tracker->base.camera;
	hmd->base.supported.orientation_tracking = true;
	hmd->base.supported.position_tracking = hmd->tracker->tracking.slam_enabled;

	// Print name.
	snprintf(hmd->base.str, XRT_DEVICE_NAME_LEN, "Oculus Rift S");
	snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "%s", hmd_serial_no);

	// Setup input.
	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;

	hmd->last_imu_timestamp_ns = 0;

	struct os_hid_device *hid_hmd = rift_s_system_hid_handle(hmd->sys);

	RIFT_S_DEBUG("Configuring firmware provided proximity sensor threshold %u", config->proximity_threshold);

	if (rift_s_protocol_set_proximity_threshold(hid_hmd, (uint16_t)config->proximity_threshold) < 0)
		goto cleanup;

#if 0
	dump_fw_block(hid_hmd, 0xB);
	dump_fw_block(hid_hmd, 0xF);
	dump_fw_block(hid_hmd, 0x10);
	dump_fw_block(hid_hmd, 0x12);
#endif

	// Native display layout and 80 Hz refresh rate.
	hmd->base.hmd->screens[0].nominal_frame_interval_ns = time_s_to_ns(1.0f / 80.0f);

	/* In the Rift S, there's one panel that is rotated
	 * to the right, so reported to the OS as a
	 * 1440x2560 wxh panel that needs to be split
	 * in two and each view rotated for rendering */
	const int view_w = 1440;
	const int view_h = 1280;

	/* screen is the physical width/height of the panel
	 * as presented to the OS */
	hmd->base.hmd->screens[0].w_pixels = view_w;
	hmd->base.hmd->screens[0].h_pixels = view_h * 2;

	// Left, Right eye view setup
	for (uint8_t eye = 0; eye < 2; ++eye) {
		// Display w/h need to be swapped, as the client sees / renders
		hmd->base.hmd->views[eye].display.w_pixels = view_h;
		hmd->base.hmd->views[eye].display.h_pixels = view_w;
		// Viewport is position on the output panel
		hmd->base.hmd->views[eye].viewport.y_pixels = 0;
		hmd->base.hmd->views[eye].viewport.w_pixels = view_w;
		hmd->base.hmd->views[eye].viewport.h_pixels = view_h;
		hmd->base.hmd->views[eye].rot = u_device_rotation_right;
	}
	// left eye starts at y=0, right eye starts at y=view_height
	hmd->base.hmd->views[0].viewport.y_pixels = 0;
	hmd->base.hmd->views[1].viewport.y_pixels = view_h;

	// RIFT_S_FOV_SCALE scales all projection tangents, RIFT_S_FOV_SCALE_Y only the vertical ones.
	const float fov_scale = parse_fov_scale("RIFT_S_FOV_SCALE");
	const float fov_scale_y = parse_fov_scale("RIFT_S_FOV_SCALE_Y");
	for (uint8_t eye = 0; eye < 2; eye++) {
		// The lens centre follows the IPD at creation; the mesh is built once.
		if (!rift_s_optics_init(&hmd->optics[eye], eye, hmd->ipd_m, fov_scale, fov_scale_y)) {
			RIFT_S_ERROR("Failed to initialise Rift S optics");
			goto cleanup;
		}
		hmd->base.hmd->distortion.fov[eye] = hmd->optics[eye].fov;
		const struct xrt_fov *fov = &hmd->optics[eye].fov;
		RIFT_S_INFO("Rift S eye %u FOV tan: left=%.6f right=%.6f up=%.6f down=%.6f; scale=%.6f, vertical scale=%.6f",
		            eye, tanf(fov->angle_left), tanf(fov->angle_right), tanf(fov->angle_up), tanf(fov->angle_down),
		            fov_scale, fov_scale_y);
		RIFT_S_INFO("Rift S eye %u lens centre (%.2f, %.2f) px at IPD %.1f mm", eye,
		            hmd->optics[eye].lens_center_px.x, hmd->optics[eye].lens_center_px.y, hmd->ipd_m * 1000.0f);
	}

	hmd->base.hmd->distortion.models = XRT_DISTORTION_MODEL_COMPUTE;
	hmd->base.hmd->distortion.preferred = XRT_DISTORTION_MODEL_COMPUTE;
	hmd->base.compute_distortion = rift_s_compute_distortion;
	u_distortion_mesh_fill_in_compute(&hmd->base);

	/* Set Opaque blend mode */
	hmd->base.hmd->blend_modes[0] = XRT_BLEND_MODE_OPAQUE;
	hmd->base.hmd->blend_mode_count = 1;

	// Setup variable tracker: Optional but useful for debugging
	u_var_add_root(hmd, "Oculus Rift S", true);

	/* Add tracker variables to the HMD debug */
	rift_s_tracker_add_debug_ui(hmd->tracker, hmd);

	u_var_add_gui_header(hmd, NULL, "Misc");
	u_var_add_log_level(hmd, &rift_s_log_level, "log_level");

	RIFT_S_DEBUG("Oculus Rift S HMD serial %s initialised.", hmd_serial_no);

	return hmd;

cleanup:
	rift_s_system_reference(&hmd->sys, NULL);
	return NULL;
}

void
rift_s_hmd_set_proximity(struct rift_s_hmd *hmd, bool prox_sensor)
{
	if (debug_get_bool_option_steamvr_bench()) {
		prox_sensor = true;
	}
	rift_s_tracker_set_standby(hmd->tracker, !prox_sensor);

	/* Enable the screen if the prox sensor is triggered, or turn it off otherwise. */
	if (prox_sensor != hmd->display_on) {
		struct os_hid_device *hid_hmd = rift_s_system_hid_handle(hmd->sys);

		rift_s_set_screen_enable(hid_hmd, prox_sensor);
		hmd->display_on = prox_sensor;
	}
}
