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
 * @brief  Oculus Rift S headset tracking system
 *
 * The Rift S system provides the HID/USB polling thread
 * and dispatches incoming packets to the HMD and controller
 * implementations.
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
#include <errno.h>

#include "math/m_api.h"
#include "math/m_vec3.h"

#include "os/os_time.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"

#ifdef XRT_OS_LINUX
#include "util/u_linux.h"
#endif

#include "xrt/xrt_device.h"

#include "rift_s.h"
#include "rift_s_hmd.h"
#include "rift_s_controller.h"
#include "rift_s_camera.h"
#include "rift_s_haptics.h"
#include "rift_s_interface.h"
#include "rift_s_usb.h"

DEBUG_GET_ONCE_BOOL_OPTION(steamvr_bench, "MONADO_STEAMVR_BENCH", false)

static void
stop_haptics(struct rift_s_system *sys);

static void *
rift_s_run_thread(void *ptr);
static void
rift_s_system_free(struct rift_s_system *sys);

static int
read_camera_calibration(struct os_hid_device *hid_hmd, struct rift_s_camera_calibration_block *calibration)
{
	char *json = NULL;
	int json_len = 0;

	int ret = rift_s_read_firmware_block(hid_hmd, RIFT_S_FIRMWARE_BLOCK_CAMERA_CALIB, &json, &json_len);
	if (ret < 0)
		return ret;

	ret = rift_s_parse_camera_calibration_block(json, calibration);
	free(json);

	return ret;
}

static int
read_hmd_fw_imu_calibration(struct os_hid_device *hid_hmd, struct rift_s_imu_calibration *imu_calibration)
{
	char *json = NULL;
	int json_len = 0;

	int ret = rift_s_read_firmware_block(hid_hmd, RIFT_S_FIRMWARE_BLOCK_IMU_CALIB, &json, &json_len);
	if (ret < 0)
		return ret;

	ret = rift_s_parse_imu_calibration(json, imu_calibration);
	free(json);

	return ret;
}

static int
read_hmd_proximity_threshold(struct os_hid_device *hid_hmd, int *proximity_threshold)
{
	char *json = NULL;
	int json_len = 0;

	int ret = rift_s_read_firmware_block(hid_hmd, RIFT_S_FIRMWARE_BLOCK_THRESHOLD, &json, &json_len);
	if (ret < 0)
		return ret;

	ret = rift_s_parse_proximity_threshold(json, proximity_threshold);
	free(json);

	return ret;
}

static int
read_hmd_config(struct os_hid_device *hid_hmd, struct rift_s_hmd_config *config)
{
	int ret;

	ret = rift_s_read_firmware_version(hid_hmd);
	if (ret < 0) {
		RIFT_S_ERROR("Failed to read Rift S firmware version");
		return ret;
	}

	ret = rift_s_read_panel_info(hid_hmd, &config->panel_info);
	if (ret < 0) {
		RIFT_S_ERROR("Failed to read Rift S device info");
		return ret;
	}

	ret = rift_s_read_imu_config_info(hid_hmd, &config->imu_config_info);
	if (ret < 0) {
		RIFT_S_ERROR("Failed to read IMU configuration block");
		return ret;
	}

	ret = read_hmd_fw_imu_calibration(hid_hmd, &config->imu_calibration);
	if (ret < 0) {
		RIFT_S_ERROR("Failed to read IMU configuration block");
		return ret;
	}

	/* Configure the proximity sensor threshold */
	ret = read_hmd_proximity_threshold(hid_hmd, &config->proximity_threshold);
	if (ret < 0) {
		RIFT_S_ERROR("Failed to read proximity sensor firmware block");
		return ret;
	}

	ret = read_camera_calibration(hid_hmd, &config->camera_calibration);
	if (ret < 0) {
		RIFT_S_ERROR("Failed to read HMD camera calibration block");
		return ret;
	}

	/* Optional and informational: the optics use the Oculus runtime lens centres, not these */
	char *lens_json = NULL;
	int lens_json_len = 0;
	if (rift_s_read_firmware_block(hid_hmd, RIFT_S_FIRMWARE_BLOCK_LENS_CALIB, &lens_json, &lens_json_len) >= 0) {
		config->lens_centers_valid = rift_s_parse_lens_centers(lens_json, config->lens_center_px) == 0;
		free(lens_json);
	}
	if (!config->lens_centers_valid) {
		RIFT_S_DEBUG("No lens calibration block");
	}

	return 0;
}

struct rift_s_system *
rift_s_system_create(struct xrt_prober *xp,
                     const unsigned char *hmd_serial_no,
                     struct os_hid_device *hid_hmd,
                     struct os_hid_device *hid_status,
                     struct os_hid_device *hid_controllers)
{
	int ret;

	DRV_TRACE_MARKER();

	struct rift_s_system *sys = U_TYPED_CALLOC(struct rift_s_system);
	if (sys == NULL)
		return NULL;
	sys->prober = xp;
	snprintf(sys->serial, sizeof(sys->serial), "%s", hmd_serial_no);
	sys->base.type = XRT_TRACKING_TYPE_NONE;
	sys->base.initial_offset.orientation.w = 1.0f;

	/* Init refcount */
	sys->ref.count = 1;

	sys->handles[HMD_HID] = hid_hmd;
	sys->handles[STATUS_HID] = hid_status;
	sys->handles[CONTROLLER_HID] = hid_controllers;

	ret = os_mutex_init(&sys->dev_mutex);
	if (ret != 0) {
		RIFT_S_ERROR("Failed to init device mutex");
		goto cleanup;
	}
	//
	// Thread and other state.
	ret = os_thread_helper_init(&sys->oth);
	if (ret != 0) {
		RIFT_S_ERROR("Failed to init packet processing thread");
		goto cleanup;
	}

	sys->motion = rift_s_motion_create();
	if (sys->motion == NULL) {
		RIFT_S_ERROR("Failed to allocate controller motion cache");
		goto cleanup;
	}

	if (read_hmd_config(hid_hmd, &sys->hmd_config) < 0) {
		RIFT_S_ERROR("Failed to read HMD configuration");
		goto cleanup;
	}

	sys->tracker = rift_s_tracker_create(&sys->base, &sys->xfctx, &sys->hmd_config);
	if (sys->tracker == NULL) {
		RIFT_S_ERROR("Failed to init tracking");
		goto cleanup;
	}

	rift_s_radio_state_init(&sys->radio_state);
	for (unsigned hand = 0; hand < MAX_TRACKED_DEVICES; ++hand) {
		sys->haptics[hand].sys = sys;
		sys->haptics[hand].hand = hand;
	}

	/* Create the HMD now. Controllers are created in the
	 * rift_s_system_get_controller() call later */
	struct rift_s_hmd *hmd = rift_s_hmd_create(sys, hmd_serial_no, &sys->hmd_config);
	if (hmd == NULL) {
		RIFT_S_ERROR("Failed to create Oculus Rift S device.");
		goto cleanup;
	}

	sys->hmd = hmd;

	/* Turn on the headset and display connection */
	if (rift_s_hmd_enable(sys->handles[HMD_HID], true) < 0) {
		RIFT_S_ERROR("Failed to enable Rift S");
		goto cleanup;
	}

	// Bench sessions must power the panel even if no new proximity packet arrives.
	if (debug_get_bool_option_steamvr_bench()) {
		rift_s_hmd_set_proximity(sys->hmd, true);
	}

	// Allow time for enumeration of available displays by host system, so the compositor can select among them.
	RIFT_S_INFO(
	    "Sleeping until the HMD display is powered up so, the available displays "
	    "can be enumerated by the host system.");

	// Two seconds seems to be needed for the display connection to stabilise
	os_nanosleep((uint64_t)U_TIME_1S_IN_NS * 2);

	// Start the camera input
	struct rift_s_camera *cam =
	    rift_s_camera_create(xp, &sys->camera_context, (const char *)hmd_serial_no, sys->handles[HMD_HID], sys->tracker,
	                         &sys->hmd_config.camera_calibration);
	if (cam == NULL) {
		RIFT_S_ERROR("Failed to open Rift S camera device");
		goto cleanup;
	}
	os_mutex_lock(&sys->dev_mutex);
	sys->cam = cam;
	os_mutex_unlock(&sys->dev_mutex);

	rift_s_tracker_start(sys->tracker);
	xrt_atomic_s32_store(&sys->transport_online, 1);
	ret = os_thread_helper_start(&sys->oth, rift_s_run_thread, sys);
	if (ret != 0) {
		RIFT_S_ERROR("Failed to start packet processing thread");
		goto cleanup;
	}

	RIFT_S_DEBUG("Oculus Rift S driver ready");

	return sys;

cleanup:
	if (sys->hmd != NULL) {
		xrt_device_destroy((struct xrt_device **)&sys->hmd);
	}
	rift_s_system_reference(&sys, NULL);
	return NULL;
}

static void
rift_s_system_free(struct rift_s_system *sys)
{
	DRV_TRACE_MARKER();

	RIFT_S_DEBUG("Freeing Rift S system");

	/* Stop the packet reading thread */
	os_thread_helper_destroy(&sys->oth);

	/* The world worker reads SLAM and camera buffers owned by the frame context. */
	if (sys->tracker != NULL) {
		rift_s_tracker_stop(sys->tracker);
	}

	/* Stop all the frame processing (has to happen before the cameras
	 * and tracker are destroyed */
	rift_s_camera_stop(sys->cam);
	xrt_frame_context_destroy_nodes(&sys->camera_context);
	xrt_frame_context_destroy_nodes(&sys->xfctx);

	rift_s_radio_state_clear(&sys->radio_state);
	stop_haptics(sys);

	if (sys->handles[HMD_HID]) {
		if (rift_s_hmd_enable(sys->handles[HMD_HID], false) < 0) {
			RIFT_S_WARN("Failed to disable Rift S");
		}
	}

	for (int i = 0; i < 3; i++) {
		if (sys->handles[i] != NULL)
			os_hid_destroy(sys->handles[i]);
	}

	/* Free the camera */
	if (sys->cam != NULL) {
		rift_s_camera_destroy(sys->cam);
	}

	if (sys->tracker != NULL) {
		rift_s_tracker_destroy(sys->tracker);
	}

	os_mutex_destroy(&sys->dev_mutex);

	rift_s_motion_destroy(&sys->motion);
	free(sys);
}

/* Reference count handling for rift_s_system */
void
rift_s_system_reference(struct rift_s_system **dst, struct rift_s_system *src)
{
	struct rift_s_system *old_dst = *dst;

	if (old_dst == src) {
		return;
	}

	if (src) {
		xrt_reference_inc(&src->ref);
	}

	*dst = src;

	if (old_dst) {
		if (xrt_reference_dec_and_is_zero(&old_dst->ref)) {
			rift_s_system_free(old_dst);
		}
	}
}

struct os_hid_device *
rift_s_system_hid_handle(struct rift_s_system *sys)
{
	return sys->handles[HMD_HID];
}

rift_s_radio_state *
rift_s_system_radio(struct rift_s_system *sys)
{
	return &sys->radio_state;
}

struct rift_s_tracker *
rift_s_system_get_tracker(struct rift_s_system *sys)
{
	return sys->tracker;
}

bool
rift_s_system_is_online(struct rift_s_system *sys)
{
	return xrt_atomic_s32_load(&sys->transport_online) != 0;
}

struct xrt_device *
rift_s_system_get_hmd(struct rift_s_system *sys)
{
	return (struct xrt_device *)sys->hmd;
}

void
rift_s_system_remove_hmd(struct rift_s_system *sys)
{
	os_thread_helper_stop_and_wait(&sys->oth);
	xrt_atomic_s32_store(&sys->transport_online, 0);
	rift_s_radio_state_clear(&sys->radio_state);
	os_mutex_lock(&sys->dev_mutex);
	sys->hmd = NULL;
	os_mutex_unlock(&sys->dev_mutex);
}

struct xrt_device *
rift_s_system_get_controller(struct rift_s_system *sys, int index)
{
	assert(index >= 0 && index < MAX_TRACKED_DEVICES);
	assert(sys->controllers[index] == NULL); // Ensure only called once per controller

	os_mutex_lock(&sys->dev_mutex);
	if (index == 0) {
		sys->controllers[0] = rift_s_controller_create(sys, XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER);
	} else {
		sys->controllers[1] = rift_s_controller_create(sys, XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER);
	}
	os_mutex_unlock(&sys->dev_mutex);

	return (struct xrt_device *)sys->controllers[index];
}

void
rift_s_system_remove_controller(struct rift_s_system *sys, struct rift_s_controller *ctrl)
{
	// Registered devices are only destroyed at session teardown. Join the producer
	// and finish radio callbacks while every controller is still alive.
	os_thread_helper_stop_and_wait(&sys->oth);
	xrt_atomic_s32_store(&sys->transport_online, 0);
	rift_s_radio_state_clear(&sys->radio_state);
	os_mutex_lock(&sys->dev_mutex);

	for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
		if (sys->controllers[i] == ctrl) {
			sys->haptics[i].end_ns = 0;
			sys->haptics[i].amplitude = 0;
			sys->controllers[i] = NULL;
			break;
		}
	}

	os_mutex_unlock(&sys->dev_mutex);
}

struct xrt_device *
rift_s_system_get_hand_tracking_device(struct rift_s_system *sys)
{
	return rift_s_tracker_get_hand_tracking_device(sys->tracker);
}

xrt_result_t
rift_s_system_set_haptic(struct rift_s_system *sys, unsigned hand, uint64_t device_id,
                         const struct xrt_output_value_vibration *value)
{
	if (hand >= MAX_TRACKED_DEVICES)
		return XRT_ERROR_OUTPUT_UNSUPPORTED;
	uint8_t packet[5];
	rift_s_encode_haptic_raw(value->amplitude, packet);
	timepoint_ns now = os_monotonic_get_ns();
	os_mutex_lock(&sys->dev_mutex);
	struct rift_s_haptic_radio *h = &sys->haptics[hand];
	if (device_id != 0)
		h->device_id = device_id;
	h->amplitude = packet[4];
	h->end_ns = now + rift_s_haptic_duration(value);
	os_mutex_unlock(&sys->dev_mutex);
	// The raw register has no frequency field. Use native motor frequency rather
	// than guessing the buffered protocol's device-clock timestamp.
	RIFT_S_DEBUG("Haptic request hand=%s amplitude=%u duration_ms=%.3f requested_hz=%.1f mode=raw",
	             hand ? "RIGHT" : "LEFT", packet[4], (double)rift_s_haptic_duration(value) / U_TIME_1MS_IN_NS,
	             value->frequency);
	return XRT_SUCCESS;
}

static void
haptic_complete(bool success, uint8_t *response, int size, void *data)
{
	struct rift_s_haptic_radio *h = data;
	uint32_t status = UINT32_MAX;
	bool ok = success && size >= 0 && rift_s_controller_write_succeeded(response, (size_t)size, &status);
	os_mutex_lock(&h->sys->dev_mutex);
	h->in_flight = false;
	if (ok) {
		h->applied_amplitude = h->sent_amplitude;
		h->applied_device_id = h->sent_device_id;
		h->refresh_ns = os_monotonic_get_ns() + 500 * U_TIME_1MS_IN_NS;
	} else {
		h->retry_ns = os_monotonic_get_ns() + 20 * U_TIME_1MS_IN_NS;
	}
	uint8_t amplitude = h->sent_amplitude;
	unsigned hand = h->hand;
	os_mutex_unlock(&h->sys->dev_mutex);
	RIFT_S_DEBUG("Haptic radio hand=%s amplitude=%u ack=%s status=0x%x", hand ? "RIGHT" : "LEFT",
	             amplitude, ok ? "ok" : "failed", status);
}

// Only called by the USB worker, between report handling and radio update. Keep
// at most one command per hand and coalesce requests before touching the queue.
static void
update_haptics(struct rift_s_system *sys)
{
	if (sys->radio_state.pending_commands != NULL)
		return; // Existing calibration/LED transactions keep their ordering.
	for (unsigned hand = 0; hand < MAX_TRACKED_DEVICES; ++hand) {
		struct rift_s_haptic_radio *h = &sys->haptics[hand];
		timepoint_ns now = os_monotonic_get_ns();
		os_mutex_lock(&sys->dev_mutex);
		uint8_t desired;
		uint64_t id;
		bool needed = rift_s_haptic_next(h, now, &id, &desired);
		os_mutex_unlock(&sys->dev_mutex);
		if (!needed)
			continue;
		if (id == 0) {
			// Pairing identity exists even when a desk controller is asleep and
			// has not sent its first IMU packet. Do not change tracking readiness.
			rift_s_devices_list_t list;
			if (rift_s_read_devices_list(sys->handles[HMD_HID], &list) >= 0) {
				for (unsigned i = 0; i < list.num_devices; ++i) {
					rift_s_device_type type = hand ? RIFT_S_DEVICE_RIGHT_CONTROLLER : RIFT_S_DEVICE_LEFT_CONTROLLER;
					if (list.devices[i].device_type == type)
						id = list.devices[i].device_id;
				}
			}
		}
		os_mutex_lock(&sys->dev_mutex);
		// The request may have changed while the pairing list was read.
		if (h->device_id == 0)
			h->device_id = id;
		if (!rift_s_haptic_next(h, os_monotonic_get_ns(), &id, &desired)) {
			os_mutex_unlock(&sys->dev_mutex);
			continue;
		}
		if (id == 0 || (desired == 0 && h->applied_amplitude == 0)) {
			h->retry_ns = os_monotonic_get_ns() + 100 * U_TIME_1MS_IN_NS;
			os_mutex_unlock(&sys->dev_mutex);
			continue;
		}
		h->sent_device_id = id;
		h->sent_amplitude = desired;
		h->in_flight = true;
		os_mutex_unlock(&sys->dev_mutex);
		uint8_t packet[5];
		rift_s_encode_haptic_raw((float)desired / 255.0f, packet);
		rift_s_radio_queue_report(&sys->radio_state, RIFT_S_RADIO_REPORT_WRITE, id, packet, sizeof(packet),
		                          haptic_complete, h);
		return; // One feature transaction this tick; the other hand runs next.
	}
}

// The raw register latches amplitude. After joining the USB worker, send zero
// to every identity that could have received a request, including an unacked one.
// This is best effort on USB/radio failure and never waits indefinitely at exit.
static void
stop_haptics(struct rift_s_system *sys)
{
	if (sys->handles[HMD_HID] == NULL)
		return;
	uint8_t packet[5];
	rift_s_encode_haptic_raw(0, packet);
	uint64_t ids[MAX_TRACKED_DEVICES * 3] = {0};
	unsigned count = 0;
	for (unsigned hand = 0; hand < MAX_TRACKED_DEVICES; ++hand) {
		struct rift_s_haptic_radio *h = &sys->haptics[hand];
		uint64_t candidates[] = {h->device_id, h->sent_device_id, h->applied_device_id};
		for (unsigned i = 0; i < 3; ++i) {
			bool duplicate = candidates[i] == 0;
			for (unsigned j = 0; j < count; ++j)
				duplicate |= ids[j] == candidates[i];
			if (!duplicate)
				ids[count++] = candidates[i];
		}
	}
	for (unsigned i = 0; i < count; ++i)
		rift_s_radio_queue_report(&sys->radio_state, RIFT_S_RADIO_REPORT_WRITE, ids[i], packet, sizeof(packet),
		                          NULL, NULL);
	timepoint_ns deadline = os_monotonic_get_ns() + 100 * U_TIME_1MS_IN_NS;
	while (sys->radio_state.pending_commands != NULL && os_monotonic_get_ns() < deadline) {
		rift_s_radio_update(&sys->radio_state, sys->handles[HMD_HID]);
		os_nanosleep(U_TIME_1MS_IN_NS / 2);
	}
	if (sys->radio_state.pending_commands != NULL)
		RIFT_S_WARN("Controller haptics stop did not complete before shutdown");
	rift_s_radio_state_clear(&sys->radio_state);
}

/* Packet reading / handling */
static int
update_tracked_device_types(struct rift_s_system *sys)
{
	int res;
	rift_s_devices_list_t dev_list;
	struct os_hid_device *hid = sys->handles[HMD_HID];

	res = rift_s_read_devices_list(hid, &dev_list);
	if (res < 0)
		return res;

	for (int i = 0; i < dev_list.num_devices; i++) {
		rift_s_device_type_record_t *dev = dev_list.devices + i;
		int d;

		for (d = 0; d < sys->num_active_tracked_devices; d++) {
			if (sys->tracked_device[d].device_id == dev->device_id) {
				if (sys->tracked_device[d].device_type != dev->device_type) {
					sys->tracked_device[d].device_type = dev->device_type;
					RIFT_S_DEBUG("Tracked device 0x%16" PRIx64 " type %u now online",
					             dev->device_id, dev->device_type);
				}
				break;
			}
		}

		if (d == sys->num_active_tracked_devices) {
			RIFT_S_WARN("Got a device type record for an unknown device 0x%16" PRIx64 "\n", dev->device_id);
		}
	}

	return 0;
}

static void
handle_hmd_report(struct rift_s_system *sys, timepoint_ns local_ts, const unsigned char *buf, int size)
{
	rift_s_hmd_report_t report;

	if (!rift_s_parse_hmd_report(&report, buf, size)) {
		return;
	}

	os_mutex_lock(&sys->dev_mutex);
	if (sys->hmd != NULL) {
		rift_s_hmd_handle_report(sys->hmd, local_ts, &report);
	}
	os_mutex_unlock(&sys->dev_mutex);
}

static void
handle_controller_report(struct rift_s_system *sys, timepoint_ns local_ts, const unsigned char *buf, int size)
{
	rift_s_controller_report_t report;

	if (!rift_s_parse_controller_report(&report, buf, size)) {
		rift_s_hexdump_buffer("Invalid Controller Report", buf, size);
		return;
	}

	if (report.device_id == 0x00) {
		/* placeholder report. Ignore it */
		return;
	}

	int i;
	struct rift_s_tracked_device *td = NULL;

	for (i = 0; i < sys->num_active_tracked_devices; i++) {
		if (sys->tracked_device[i].device_id == report.device_id) {
			td = sys->tracked_device + i;
			break;
		}
	}

	if (td == NULL) {
		if (sys->num_active_tracked_devices == MAX_TRACKED_DEVICES) {
			RIFT_S_ERROR("Too many controllers. Can't add %08" PRIx64 "\n", report.device_id);
			return;
		}

		/* Add a new controller to the online list */
		td = sys->tracked_device + sys->num_active_tracked_devices;
		sys->num_active_tracked_devices++;

		memset(td, 0, sizeof(struct rift_s_tracked_device));
		td->device_id = report.device_id;

		update_tracked_device_types(sys);
	}

	struct rift_s_controller *ctrl = NULL;

	os_mutex_lock(&sys->dev_mutex);

	switch (td->device_type) {
	/* If we didn't already succeed in reading the type for this device, try again */
	case RIFT_S_DEVICE_TYPE_UNKNOWN: update_tracked_device_types(sys); break;
	case RIFT_S_DEVICE_LEFT_CONTROLLER: ctrl = sys->controllers[0]; break;
	case RIFT_S_DEVICE_RIGHT_CONTROLLER: ctrl = sys->controllers[1]; break;
	default: break; /* Ignore unknown device type */
	}

	if (ctrl != NULL) {
		rift_s_controller_update_configuration(ctrl, td->device_id);

		if (!rift_s_controller_handle_report(ctrl, local_ts, &report)) {
			rift_s_hexdump_buffer("Invalid Controller Report Content", buf, size);
		}
	}
	os_mutex_unlock(&sys->dev_mutex);
}

static enum rift_s_usb_result
handle_packets(struct rift_s_system *sys)
{
	unsigned char buf[FEATURE_BUFFER_SIZE];
	enum rift_s_usb_result ret = RIFT_S_USB_OK;

	// Handle keep alive messages
	timepoint_ns now = os_monotonic_get_ns();

	if ((now - sys->last_keep_alive) / U_TIME_1MS_IN_NS >= KEEPALIVE_INTERVAL_MS) {
		// send keep alive message
		rift_s_send_keepalive(sys->handles[HMD_HID]);
		// Update the time of the last keep alive we have sent.
		sys->last_keep_alive = now;
	}

	/* Poll each of the 3 HID interfaces for messages and process them */
	for (int i = 0; i < 3; i++) {
		if (sys->handles[i] == NULL)
			continue;
		if (now < sys->read_retry_ns[i]) {
			ret = RIFT_S_USB_RETRY;
			continue;
		}

		while (true) {
			errno = 0;
			int size = os_hid_read(sys->handles[i], buf, FEATURE_BUFFER_SIZE, 0);
			if (size < 0) {
				int error = errno;
				uint32_t errors = ++sys->read_errors[i];
				enum rift_s_usb_result result = rift_s_usb_read_error(error, errors);
				if (result == RIFT_S_USB_LOST) {
					RIFT_S_WARN("USB interface %d lost errno=%d", i, error);
					return result;
				}
				sys->read_retry_ns[i] = os_monotonic_get_ns() + rift_s_usb_backoff_ns(errors);
				if (errors == 1)
					RIFT_S_DEBUG("USB interface %d interrupted errno=%d; retrying", i, error);
				ret = RIFT_S_USB_RETRY;
				break;
			}
			sys->read_errors[i] = 0;
			sys->read_retry_ns[i] = 0;
			if (size == 0) {
				break; // No more messages, return.
			}

			now = os_monotonic_get_ns();

			if (buf[0] == 0x65)
				handle_hmd_report(sys, now, buf, size);
			else if (buf[0] == 0x67)
				handle_controller_report(sys, now, buf, size);
			else if (buf[0] == 0x66 && size >= 2) {
				/* System state packet. Enable the screen if the prox sensor is
				 * triggered. */
				bool prox_sensor = (buf[1] == 0) ? false : true;
				if (prox_sensor != sys->physical_proximity || now - sys->last_proximity_log_ns >= 5000000000ULL) {
					RIFT_S_INFO("HMD physical proximity=%d bench=%d", prox_sensor,
					            debug_get_bool_option_steamvr_bench());
					sys->physical_proximity = prox_sensor;
					sys->last_proximity_log_ns = now;
				}
				os_mutex_lock(&sys->dev_mutex);
				if (sys->hmd != NULL) {
					rift_s_hmd_set_proximity(sys->hmd, prox_sensor);
				}
				os_mutex_unlock(&sys->dev_mutex);
			} else {
				RIFT_S_WARN("Unknown Rift S report 0x%02x!", buf[0]);
			}
		}
	}

	return ret;
}

static void
close_transport(struct rift_s_system *sys)
{
	xrt_atomic_s32_store(&sys->transport_online, 0);
	rift_s_tracker_device_lost(sys->tracker);
	// Do not hold dev_mutex across a frameserver join or a SLAM restart.
	rift_s_camera_stop(sys->cam);
	xrt_frame_context_destroy_nodes(&sys->camera_context);
	rift_s_radio_state_clear(&sys->radio_state);
	rift_s_radio_state_init(&sys->radio_state);
	os_mutex_lock(&sys->dev_mutex);
	if (sys->cam != NULL) {
		rift_s_camera_destroy(sys->cam);
		sys->cam = NULL;
	}
	for (unsigned i = 0; i < MAX_TRACKED_DEVICES; i++) {
		if (sys->controllers[i] != NULL)
			rift_s_controller_device_lost(sys->controllers[i]);
		sys->haptics[i] = (struct rift_s_haptic_radio){.sys = sys, .hand = i};
		rift_s_motion_reset_hand(sys->motion, i);
	}
	if (sys->hmd != NULL) {
		sys->hmd->last_imu_timestamp_ns = 0;
		sys->hmd->last_imu_timestamp32 = 0;
		sys->hmd->display_on = false;
	}
	for (unsigned i = 0; i < 3; i++) {
		if (sys->handles[i] != NULL)
			os_hid_destroy(sys->handles[i]);
		sys->handles[i] = NULL;
	}
	sys->num_active_tracked_devices = 0;
	memset(sys->tracked_device, 0, sizeof(sys->tracked_device));
	memset(sys->read_errors, 0, sizeof(sys->read_errors));
	memset(sys->read_retry_ns, 0, sizeof(sys->read_retry_ns));
	os_mutex_unlock(&sys->dev_mutex);
	RIFT_S_WARN("Rift S transport closed; waiting for serial %s", sys->serial);
}

static bool
open_transport(struct rift_s_system *sys)
{
	// Reprobe only after startup is complete; the instance owns this prober.
	struct xrt_prober_device **devices = NULL;
	size_t count = 0;
	if (strcmp(sys->serial, "Unknown") == 0 || xrt_prober_probe(sys->prober) != XRT_SUCCESS ||
	    xrt_prober_lock_list(sys->prober, &devices, &count) != XRT_SUCCESS)
		return false;
	struct os_hid_device *handles[3] = {0};
	bool found = false;
	for (size_t i = 0; i < count; i++) {
		struct xrt_prober_device *dev = devices[i];
		if (dev->vendor_id != OCULUS_VR_INC_VID || dev->product_id != OCULUS_RIFT_S_PID ||
		    dev->bus != XRT_BUS_TYPE_USB)
			continue;
		unsigned char serial[XRT_DEVICE_NAME_LEN] = {0};
		if (xrt_prober_get_string_descriptor(sys->prober, dev, XRT_PROBER_STRING_SERIAL_NUMBER,
		                                     serial, sizeof(serial)) < 0 ||
		    strcmp(sys->serial, (const char *)serial) != 0)
			continue;
		found = true;
		for (unsigned iface = 0; iface < 3; iface++) {
			if (xrt_prober_open_hid_interface(sys->prober, dev, 6 + iface, &handles[iface]) != 0) {
				found = false;
				break;
			}
		}
		break;
	}
	xrt_prober_unlock_list(sys->prober, &devices);
	if (!found)
		goto fail;
	// Calibration belongs to the matched serial. Reapply volatile hardware state.
	// Reset volatile radio state even after a live I/O failure, so a late response
	// cannot enter the next command epoch.
	if (rift_s_hmd_enable(handles[HMD_HID], false) < 0)
		goto fail;
	if (rift_s_protocol_set_proximity_threshold(handles[HMD_HID], sys->hmd_config.proximity_threshold) < 0 ||
	    rift_s_hmd_enable(handles[HMD_HID], true) < 0 ||
	    rift_s_set_screen_enable(handles[HMD_HID], true) < 0)
		goto fail;
	// Power the panel during enumeration even off-head. Proximity packets restore
	// ordinary standby after the DP link and camera input have returned.
	os_nanosleep(2ULL * U_TIME_1S_IN_NS);
	if (!os_thread_helper_is_running(&sys->oth))
		goto fail;
	struct rift_s_camera *cam = rift_s_camera_create(sys->prober, &sys->camera_context, sys->serial,
	                                                handles[HMD_HID], sys->tracker,
	                                                &sys->hmd_config.camera_calibration);
	if (cam == NULL) {
		xrt_frame_context_destroy_nodes(&sys->camera_context);
		goto fail;
	}
	os_mutex_lock(&sys->dev_mutex);
	memcpy(sys->handles, handles, sizeof(handles));
	sys->cam = cam;
	sys->last_keep_alive = 0;
	if (sys->hmd != NULL)
		sys->hmd->display_on = true;
	os_mutex_unlock(&sys->dev_mutex);
	rift_s_tracker_start(sys->tracker);
	xrt_atomic_s32_store(&sys->transport_online, 1);
	RIFT_S_WARN("Rift S serial %s reconnected; camera/radio resumed", sys->serial);
	return true;
fail:
	for (unsigned i = 0; i < 3; i++) {
		if (handles[i] != NULL)
			os_hid_destroy(handles[i]);
	}
	return false;
}

static void *
rift_s_run_thread(void *ptr)
{
	DRV_TRACE_MARKER();

	struct rift_s_system *sys = (struct rift_s_system *)ptr;

#ifdef XRT_OS_LINUX
	// Try to raise priority of this thread.
	u_linux_try_to_set_realtime_priority_on_thread(rift_s_log_level, "Rift S: USB-HMD");
#endif

	os_thread_helper_lock(&sys->oth);
	while (os_thread_helper_is_running_locked(&sys->oth)) {
		os_thread_helper_unlock(&sys->oth);

		if (!rift_s_system_is_online(sys)) {
			if (!open_transport(sys))
				os_nanosleep(U_TIME_1S_IN_NS);
		} else {
			enum rift_s_usb_result result = handle_packets(sys);
			if (result == RIFT_S_USB_LOST || !rift_s_camera_is_running(sys->cam)) {
				close_transport(sys);
			} else if (result == RIFT_S_USB_OK) {
				update_haptics(sys);
				rift_s_radio_update(&sys->radio_state, sys->handles[HMD_HID]);
				if (sys->radio_state.recovery_required) {
					close_transport(sys);
				} else {

					os_mutex_lock(&sys->dev_mutex);
					if (sys->cam != NULL && sys->physical_proximity) {
						rift_s_camera_update(sys->cam, sys->handles[HMD_HID]);
					}
					os_mutex_unlock(&sys->dev_mutex);
				}
			}
		}

		os_thread_helper_lock(&sys->oth);

		if (os_thread_helper_is_running_locked(&sys->oth)) {
			os_nanosleep(U_TIME_1MS_IN_NS / 2);
		}
	}
	os_thread_helper_unlock(&sys->oth);

	RIFT_S_DEBUG("Exiting packet reading thread");

	return NULL;
}
