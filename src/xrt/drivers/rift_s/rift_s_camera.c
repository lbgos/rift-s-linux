/*
 * Copyright 2021, Collabora, Ltd.
 * Copyright 2022 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 */

/*!
 * @file
 * @brief  Oculus Rift S camera handling
 *
 * The Rift S camera module, handles reception and dispatch
 * of camera frames.
 *
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_rift_s
 */
#include <string.h>
#include <stdio.h>

#include "rift_s.h"
#include "rift_s_camera.h"
#include "rift_s_camera_metadata.h"

#include "os/os_threading.h"

#include "xrt/xrt_defines.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_frameserver.h"

#include "util/u_autoexpgain.h"
#include "util/u_debug.h"
#include "util/u_var.h"
#include "util/u_sink.h"
#include "util/u_frame.h"
#include "util/u_trace_marker.h"

#define DEFAULT_EXPOSURE 6000
#define DEFAULT_GAIN 127

#define RIFT_S_MIN_EXPOSURE 38
#define RIFT_S_MAX_EXPOSURE 14022

#define RIFT_S_MIN_GAIN 16
#define RIFT_S_MAX_GAIN 255

//! Specifies whether the user wants to enable autoexposure from the start.
DEBUG_GET_ONCE_BOOL_OPTION(rift_s_autoexposure, "RIFT_S_AUTOEXPOSURE", true)

// Save up to ten raw LED frames, one per second, to an existing directory.
DEBUG_GET_ONCE_OPTION(rift_s_controller_dump_dir, "RIFT_S_CONTROLLER_DUMP_DIR", NULL)

struct rift_s_camera
{
	struct os_mutex lock;

	struct rift_s_tracker *tracker;

	struct rift_s_camera_calibration_block *camera_calibration;

	uint64_t last_frame_ts_ns;
	uint64_t last_diag_ns[2];
	uint64_t last_arrival_ns[2];
	uint64_t max_arrival_gap_ns[2];
	bool have_frame_counter;
	uint16_t last_frame_counter;
	struct
	{
		uint64_t accepted[2]; // controller, SLAM
		uint64_t bad_layout, bad_magic, bad_crc, bad_timestamp;
		uint64_t missing_frames, counter_discontinuities, bad_crop, bad_guard;
	} metadata_stats;
	uint32_t controller_dump_frames;

	struct xrt_frame_sink in_sink; // Receive raw frames and split them

	struct u_sink_debug debug_sinks[2];

	rift_s_camera_report_t camera_report;

	uint16_t last_slam_exposure, target_exposure;
	uint8_t last_slam_gain, target_gain;

	bool manual_control;                    //!< Whether to control exp/gain manually or with aeg
	struct u_var_draggable_u16 exposure_ui; //! Widget to control `exposure` value
	struct u_autoexpgain *aeg;
};

struct rift_s_camera_finder
{
	const char *hmd_serial_no;

	struct xrt_fs *xfs;
	struct xrt_frame_context *xfctx;
};

static void
update_expgain(struct rift_s_camera *cam, struct xrt_frame *xf);

static void
receive_cam_frame(struct xrt_frame_sink *sink, struct xrt_frame *xf);

static void
on_video_device(struct xrt_prober *xp,
                struct xrt_prober_device *pdev,
                const char *product,
                const char *manufacturer,
                const char *serial,
                void *ptr)
{
	struct rift_s_camera_finder *finder = (struct rift_s_camera_finder *)ptr;

	/* Already found a device? */
	if (finder->xfs != NULL)
		return;

	if (product == NULL || manufacturer == NULL || serial == NULL) {
		return;
	}

	RIFT_S_TRACE("Inspecting video device %s - %s serial %s", manufacturer, product, serial);

	if ((strcmp(product, "Rift S Sensor") == 0) && (strcmp(manufacturer, "Oculus VR") == 0)) {
		// && (strcmp(serial, finder->hmd_serial_no) == 0)) {
		// Serial no seems to be all zeros right now, so ignore it
		xrt_prober_open_video_device(xp, pdev, finder->xfctx, &finder->xfs);
		return;
	}
}

struct rift_s_camera *
rift_s_camera_create(struct xrt_prober *xp,
                     struct xrt_frame_context *xfctx,
                     const char *hmd_serial_no,
                     struct os_hid_device *hid,
                     struct rift_s_tracker *tracker,
                     struct rift_s_camera_calibration_block *camera_calibration)
{
	struct rift_s_camera_finder finder = {
	    0,
	};

	DRV_TRACE_MARKER();

	/* Set up the finder with the HMD serial number and frame server context we want */
	finder.xfctx = xfctx;
	finder.hmd_serial_no = hmd_serial_no;

	/* Re-probe devices. The v4l2 camera device should have appeared by now */
	int retry_count = 5;
	do {
		xrt_result_t xret = xrt_prober_probe(xp);

		if (xret != XRT_SUCCESS) {
			return NULL;
		}

		xrt_prober_list_video_devices(xp, on_video_device, &finder);
		if (finder.xfs != NULL) {
			break;
		}

		/* Sleep 1 second before retry */
		os_nanosleep((uint64_t)U_TIME_1S_IN_NS);
	} while (retry_count-- > 0);

	if (finder.xfs == NULL) {
		RIFT_S_ERROR("Didn't find Rift S camera device");
		return NULL;
	}

	struct rift_s_camera *cam = U_TYPED_CALLOC(struct rift_s_camera);

	if (os_mutex_init(&cam->lock) != 0) {
		RIFT_S_ERROR("Failed to init camera configuration mutex");
		goto cleanup;
	}

	// Store the tracker
	cam->tracker = tracker;
	cam->camera_calibration = camera_calibration;

	/* Configure default camera settings */
	rift_s_protocol_camera_report_init(&cam->camera_report);
	cam->camera_report.uvc_enable = 0x1;
	cam->camera_report.radio_sync_flag = 0x1;

	/* Store the defaults from the init() call into our current settings */
	cam->last_slam_exposure = cam->camera_report.slam_frame_exposures[0];
	cam->last_slam_gain = cam->camera_report.slam_frame_gains[0];

	cam->target_exposure = DEFAULT_EXPOSURE;
	cam->target_gain = DEFAULT_GAIN;

	rift_s_camera_update(cam, hid);

	cam->in_sink.push_frame = receive_cam_frame;

	bool enable_aeg = debug_get_bool_option_rift_s_autoexposure();
	int frame_delay = 2; // Exposure updates take effect on the 2nd frame after sending
	cam->aeg = u_autoexpgain_create(U_AEG_STRATEGY_TRACKING, enable_aeg, frame_delay);

	u_sink_debug_init(&cam->debug_sinks[0]);
	u_sink_debug_init(&cam->debug_sinks[1]);

	struct xrt_frame_sink *tmp = &cam->in_sink;

	struct xrt_fs_mode *modes = NULL;
	uint32_t count;

	xrt_fs_enumerate_modes(finder.xfs, &modes, &count);

	bool found_mode = false;
	uint32_t selected_mode = 0;

	for (; selected_mode < count; selected_mode++) {
		if (modes[selected_mode].format == XRT_FORMAT_YUYV422) {
			found_mode = true;
			break;
		}
		if (modes[selected_mode].format == XRT_FORMAT_MJPEG) {
			u_sink_create_format_converter(xfctx, XRT_FORMAT_L8, tmp, &tmp);
			found_mode = true;
			break;
		}
	}

	if (!found_mode) {
		selected_mode = 0;
		RIFT_S_ERROR("Couldn't find compatible camera input format.");
		goto cleanup;
	}

	free(modes);

	u_var_add_root(cam, "Oculus Rift S Cameras", true);

	u_var_add_bool(cam, &cam->manual_control, "Manual exposure and gain control");
	cam->exposure_ui.val = &cam->target_exposure;
	cam->exposure_ui.min = RIFT_S_MIN_EXPOSURE;
	cam->exposure_ui.max = RIFT_S_MAX_EXPOSURE;
	cam->exposure_ui.step = 25;

	u_var_add_draggable_u16(cam, &cam->exposure_ui, "Exposure");
	u_var_add_u8(cam, &cam->target_gain, "Gain");
	u_var_add_gui_header(cam, NULL, "Auto exposure and gain control");
	u_autoexpgain_add_vars(cam->aeg, cam, "");

	u_var_add_gui_header(cam, NULL, "Camera Streams");
	u_var_add_sink_debug(cam, &cam->debug_sinks[0], "Tracking Streams");
	u_var_add_sink_debug(cam, &cam->debug_sinks[1], "Controller Streams");

	/* Finally, start the video feed */
	xrt_fs_stream_start(finder.xfs, tmp, XRT_FS_CAPTURE_TYPE_TRACKING, selected_mode);

	return cam;

cleanup:
	rift_s_camera_destroy(cam);
	return NULL;
}

void
rift_s_camera_destroy(struct rift_s_camera *cam)
{
	u_var_remove_root(cam);
	u_autoexpgain_destroy(&cam->aeg);
	os_mutex_destroy(&cam->lock);
	free(cam);
}

static bool
crop_in_bounds(const struct xrt_frame *xf, struct xrt_rect roi)
{
	if (roi.offset.w < 0 || roi.offset.h < 0 || roi.extent.w <= 0 || roi.extent.h <= 0) {
		return false;
	}
	uint32_t x = roi.offset.w, y = roi.offset.h, w = roi.extent.w, h = roi.extent.h;
	if (x > xf->width || y > xf->height || w > xf->width - x || h > xf->height - y || xf->stride < xf->width ||
	    xf->size < x + w || xf->stride == 0) {
		return false;
	}
	return (size_t)y + h - 1 <= (xf->size - x - w) / xf->stride;
}

// Neighboring black rows are transport guards, not LED segmentation thresholds.
static bool
crop_guards_valid(const struct xrt_frame *xf, struct xrt_rect roi)
{
	uint32_t rows[2] = {roi.offset.h - 1, roi.offset.h + roi.extent.h};
	for (int i = 0; i < 2; i++) {
		if (rows[i] >= xf->height) {
			continue;
		}
		struct xrt_rect guard = {.offset = {roi.offset.w, rows[i]}, .extent = {roi.extent.w, 1}};
		if (!crop_in_bounds(xf, guard)) {
			return false;
		}
		const uint8_t *row = xf->data + rows[i] * xf->stride + roi.offset.w;
		for (int x = 0; x < roi.extent.w; x++) {
			if (row[x] > 20) {
				return false;
			}
		}
	}
	return true;
}

static int
get_y_offset(struct rift_s_camera *cam, enum rift_s_camera_id cam_id, struct rift_s_frame_metadata *row_data)
{
	/* There's a magic formula for computing the vertical offset of each camera view
	 * based on exposure, due to some internals of the headset. This formula extracted
	 * through trial and error */
	int exposure = row_data->exposure[cam_id];
	int y_offset = (exposure + 275) / 38;

	if (y_offset > 375) {
		y_offset = 375;
	} else if (y_offset < 8) {
		y_offset = 8;
	}

	return y_offset;
}

static struct xrt_frame *
rift_s_camera_extract_frame(struct rift_s_camera *cam,
                            enum rift_s_camera_id cam_id,
                            struct xrt_frame *full_frame,
                            struct rift_s_frame_metadata *row_data)
{
	struct rift_s_camera_calibration *calib = &cam->camera_calibration->cameras[cam_id];
	struct xrt_rect roi = calib->roi;

	roi.offset.h = get_y_offset(cam, cam_id, row_data);

	struct xrt_frame *xf_crop = NULL;
	if (!crop_in_bounds(full_frame, roi)) {
		cam->metadata_stats.bad_crop++;
		RIFT_S_DEBUG("DIAG camcrop camera=%u rejected=bounds", cam_id);
		return NULL;
	}
	if (!crop_guards_valid(full_frame, roi)) {
		cam->metadata_stats.bad_guard++;
		// The empirical SLAM crop formula has no raw-frame fixture yet. Count the guard
		// failure without suppressing SLAM until its neighboring rows are verified.
		RIFT_S_DEBUG("DIAG camcrop camera=%u guard=nonblack y=%d count=%" PRIu64, cam_id, roi.offset.h,
		             cam->metadata_stats.bad_guard);
	}
	u_frame_create_roi(full_frame, roi, &xf_crop);

	return xf_crop;
}

static void
dump_controller_frame(struct rift_s_camera *cam, struct xrt_frame *xf, uint64_t hw_ns)
{
	const char *dir = debug_get_option_rift_s_controller_dump_dir();
	if (dir == NULL || cam->controller_dump_frames >= 300) {
		return;
	}
	if (cam->controller_dump_frames++ % 30 != 0) {
		return;
	}

	char path[1024];
	int len = snprintf(path, sizeof(path), "%s/led-%" PRIu64 ".pgm", dir, hw_ns);
	if (len < 0 || (size_t)len >= sizeof(path)) {
		return;
	}
	FILE *file = fopen(path, "wb");
	if (file == NULL) {
		RIFT_S_WARN("Unable to write controller frame to %s", path);
		return;
	}
	bool ok = fprintf(file, "P5\n%u %u\n255\n", xf->width, xf->height) > 0;
	for (uint32_t y = 0; ok && y < xf->height; y++) {
		ok = fwrite(xf->data + y * xf->stride, 1, xf->width, file) == xf->width;
	}
	if (fclose(file) != 0 || !ok) {
		RIFT_S_WARN("Incomplete controller frame at %s", path);
	}
}

static void
receive_cam_frame(struct xrt_frame_sink *sink, struct xrt_frame *xf)
{
	struct rift_s_camera *cam = container_of(sink, struct rift_s_camera, in_sink);
	bool release_xf = false;

	RIFT_S_TRACE("cam img t=%" PRIu64 " source_t=%" PRIu64, xf->timestamp, xf->source_timestamp);

	// If the format is YUYV422 we need to override it to L8 and double the width
	// because the v4l2 device provides the wrong format description for the actual video
	// data
	if (xf->format == XRT_FORMAT_YUYV422) {
		struct xrt_rect roi = {.offset = {0, 0}, .extent = {.w = xf->width, .h = xf->height}};
		struct xrt_frame *xf_l8 = NULL;

		u_frame_create_roi(xf, roi, &xf_l8);
		xf_l8->width = 2 * xf->width;
		xf_l8->format = XRT_FORMAT_L8;

		xf = xf_l8;
		release_xf = true;
	}

	struct rift_s_frame_metadata row_data;
	enum rift_s_metadata_status status =
	    rift_s_metadata_extract(xf->data, xf->width, xf->height, xf->stride, xf->size, &row_data);
	if (status != RIFT_S_METADATA_OK) {
		if (status == RIFT_S_METADATA_BAD_LAYOUT)
			cam->metadata_stats.bad_layout++;
		if (status == RIFT_S_METADATA_BAD_MAGIC)
			cam->metadata_stats.bad_magic++;
		if (status == RIFT_S_METADATA_BAD_CRC)
			cam->metadata_stats.bad_crc++;
		RIFT_S_DEBUG("DIAG cammeta rejected=%s layout=%" PRIu64 " magic=%" PRIu64 " crc=%" PRIu64,
		             status == RIFT_S_METADATA_BAD_LAYOUT  ? "layout"
		             : status == RIFT_S_METADATA_BAD_MAGIC ? "magic"
		                                                   : "crc",
		             cam->metadata_stats.bad_layout, cam->metadata_stats.bad_magic,
		             cam->metadata_stats.bad_crc);
		goto out;
	}
	if (row_data.frame_ts_us > UINT64_MAX / OS_NS_PER_USEC) {
		cam->metadata_stats.bad_timestamp++;
		RIFT_S_DEBUG("DIAG cammeta rejected=timestamp_overflow ctr=%u", row_data.frame_ctr);
		goto out;
	}
	uint64_t frame_ts_ns = row_data.frame_ts_us * OS_NS_PER_USEC;
	if (frame_ts_ns == 0 || frame_ts_ns <= cam->last_frame_ts_ns) {
		cam->metadata_stats.bad_timestamp++;
		RIFT_S_DEBUG("DIAG cammeta rejected=timestamp ctr=%u hw_us=%" PRIu64, row_data.frame_ctr,
		             row_data.frame_ts_us);
		goto out;
	}
	cam->last_frame_ts_ns = frame_ts_ns;
	if (cam->have_frame_counter) {
		uint16_t missing;
		if (rift_s_metadata_counter_delta(cam->last_frame_counter, row_data.frame_ctr, &missing)) {
			cam->metadata_stats.missing_frames += missing;
		} else {
			cam->metadata_stats.counter_discontinuities++;
		}
	}
	cam->last_frame_counter = row_data.frame_ctr;
	cam->have_frame_counter = true;
	unsigned int kind = (row_data.frame_type & 0x80) != 0;
	cam->metadata_stats.accepted[kind]++;
	uint64_t now_ns = os_monotonic_get_ns();
	if (cam->last_arrival_ns[kind] != 0) {
		cam->max_arrival_gap_ns[kind] = MAX(cam->max_arrival_gap_ns[kind], now_ns - cam->last_arrival_ns[kind]);
	}
	cam->last_arrival_ns[kind] = now_ns;
	if (now_ns - cam->last_diag_ns[kind] >= 5 * (uint64_t)U_TIME_1S_IN_NS) {
		cam->last_diag_ns[kind] = now_ns;
		RIFT_S_INFO("Camera timing kind=%s arrival_ns=%" PRIu64 " hw_us=%" PRIu64
		             " max_arrival_gap_ms=%.3f missing=%" PRIu64 " discontinuities=%" PRIu64
		             " exposure=%u,%u,%u,%u,%u gain16=%u,%u,%u,%u,%u",
		             kind ? "slam" : "controller", now_ns, row_data.frame_ts_us,
		             (double)cam->max_arrival_gap_ns[kind] / U_TIME_1MS_IN_NS,
		             cam->metadata_stats.missing_frames, cam->metadata_stats.counter_discontinuities,
		             row_data.exposure[0], row_data.exposure[1], row_data.exposure[2], row_data.exposure[3], row_data.exposure[4],
		             row_data.gain[0], row_data.gain[1], row_data.gain[2], row_data.gain[3], row_data.gain[4]);
		cam->max_arrival_gap_ns[kind] = 0;
	}

	RIFT_S_DEBUG("DIAG cammeta kind=%s ctr=%u pair=%u hw_us=%" PRIu64 " controller=%" PRIu64 " slam=%" PRIu64
	             " missing=%" PRIu64 " discontinuities=%" PRIu64,
	             kind ? "slam" : "controller", row_data.frame_ctr, row_data.frame_ctr2, row_data.frame_ts_us,
	             cam->metadata_stats.accepted[0], cam->metadata_stats.accepted[1],
	             cam->metadata_stats.missing_frames, cam->metadata_stats.counter_discontinuities);
	for (int i = 0; i < RIFT_S_CAMERA_COUNT; i++) {
		RIFT_S_TRACE("DIAG cammeta camera=%d kind=%s ctr=%u exposure=%u gain=%f", i,
		             kind ? "slam" : "controller", row_data.frame_ctr, row_data.exposure[i],
		             row_data.gain[i] / 16.0f);
	}

	// Route by metadata kind, independent of arrival order or pixel brightness.
	if (kind) {
		int y_offset = get_y_offset(cam, 0, &row_data);
		struct xrt_rect roi = {.offset = {0, y_offset}, .extent = {.w = xf->width, .h = 480}};

		struct xrt_frame *xf_crop = NULL;
		if (crop_in_bounds(xf, roi)) {
			u_frame_create_roi(xf, roi, &xf_crop);
			u_sink_debug_push_frame(&cam->debug_sinks[0], xf_crop);
		}
		xrt_frame_reference(&xf_crop, NULL);

		/* Extract camera frames and push to the tracker */
		struct xrt_frame *frames[RIFT_S_CAMERA_COUNT] = {0};
		bool valid = true;
		for (int i = 0; i < RIFT_S_CAMERA_COUNT; i++) {
			frames[i] = rift_s_camera_extract_frame(cam, CAM_IDX_TO_ID[i], xf, &row_data);
			valid &= frames[i] != NULL;
		}
		if (!valid) {
			for (int i = 0; i < RIFT_S_CAMERA_COUNT; i++)
				xrt_frame_reference(&frames[i], NULL);
			goto out;
		}

		/* Update the exposure for all cameras based on the auto exposure for the left camera view */
		//! @todo Update expgain independently for each camera like in WMR
		update_expgain(cam, frames[0]);

		rift_s_tracker_push_slam_frames(cam->tracker, frame_ts_ns, frames);

		for (int i = 0; i < RIFT_S_CAMERA_COUNT; i++) {
			xrt_frame_reference(&frames[i], NULL);
		}
	} else {
		dump_controller_frame(cam, xf, frame_ts_ns);
		struct xrt_rect roi = {.offset = {0, 40}, .extent = {.w = xf->width, .h = 480}};
		struct xrt_frame *xf_crop = NULL;

		if (!crop_in_bounds(xf, roi)) {
			cam->metadata_stats.bad_crop++;
			RIFT_S_DEBUG("DIAG camcrop kind=controller rejected=bounds count=%" PRIu64,
			             cam->metadata_stats.bad_crop);
			goto out;
		}
		if (!crop_guards_valid(xf, roi)) {
			cam->metadata_stats.bad_guard++;
			RIFT_S_DEBUG("DIAG camcrop kind=controller rejected=guard count=%" PRIu64,
			             cam->metadata_stats.bad_guard);
			goto out;
		}
		u_frame_create_roi(xf, roi, &xf_crop);
		u_sink_debug_push_frame(&cam->debug_sinks[1], xf_crop);

		rift_s_tracker_push_controller_frameset(cam->tracker, frame_ts_ns, xf_crop);
		xrt_frame_reference(&xf_crop, NULL);
	}
out:
	if (release_xf)
		xrt_frame_reference(&xf, NULL);
}

static void
update_expgain(struct rift_s_camera *cam, struct xrt_frame *xf)
{
	if (!cam->manual_control && xf != NULL) {
		u_autoexpgain_update(cam->aeg, xf);

		uint16_t new_target_exposure;
		uint8_t new_target_gain;

		new_target_exposure =
		    CLAMP(u_autoexpgain_get_exposure(cam->aeg), RIFT_S_MIN_EXPOSURE, RIFT_S_MAX_EXPOSURE);
		new_target_gain = CLAMP(u_autoexpgain_get_gain(cam->aeg), RIFT_S_MIN_GAIN, RIFT_S_MAX_GAIN);

		if (cam->target_exposure != new_target_exposure || cam->target_gain != new_target_gain) {
			RIFT_S_DEBUG("AEG exposure now %u (cur %u) gain %u (cur %u)", new_target_exposure,
			             cam->target_exposure, new_target_gain, cam->target_gain);

			os_mutex_lock(&cam->lock);
			cam->target_exposure = new_target_exposure;
			cam->target_gain = new_target_gain;
			os_mutex_unlock(&cam->lock);
		}
	}
}

/* Called from the Rift S system device USB loop, so we can check
 * and send an exposure/gain change command if needed */
void
rift_s_camera_update(struct rift_s_camera *cam, struct os_hid_device *hid)
{
	bool need_update = false;
	int i;

	os_mutex_lock(&cam->lock);
	if (cam->target_exposure != cam->last_slam_exposure) {
		for (i = 0; i < 5; i++) {
			cam->camera_report.slam_frame_exposures[i] = cam->target_exposure;
		}
		cam->last_slam_exposure = cam->target_exposure;
		need_update = true;
	}

	if (cam->target_gain != cam->last_slam_gain) {
		for (i = 0; i < 5; i++) {
			cam->camera_report.slam_frame_gains[i] = cam->target_gain;
		}
		cam->last_slam_gain = cam->target_gain;
		need_update = true;
	}
	os_mutex_unlock(&cam->lock);

	if (need_update) {
		RIFT_S_DEBUG("Updating AEG exposure to %u gain %u", cam->target_exposure, cam->target_gain);
		if (rift_s_protocol_send_camera_report(hid, &cam->camera_report) < 0) {
			RIFT_S_WARN("Failed to update camera settings");
		}
	}
}
