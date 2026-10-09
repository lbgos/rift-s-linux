/*
 * Copyright 2013, Fredrik Hultin.
 * Copyright 2013, Jakob Bornecrantz.
 * Copyright 2016 Philipp Zabel
 * Copyright 2019-2022 Jan Schmidt
 * Copyright 2023, Collabora, Ltd.
 * SPDX-License-Identifier: BSL-1.0
 *
 */
/*!
 * @file
 * @brief  Driver code for Oculus Rift S headsets
 *
 * Implementation for the HMD 3dof and 6dof tracking
 *
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_rift_s
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <assert.h>

#include "math/m_api.h"
#include "math/m_clock_tracking.h"
#include "math/m_space.h"
#include "math/m_vec3.h"

#include "os/os_time.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_sink.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"
#include "util/u_file.h"
#include "rift_s_world_control.h"

#include "xrt/xrt_config_build.h"
#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_device.h"

#ifdef XRT_BUILD_DRIVER_HANDTRACKING
#include "../drivers/ht/ht_interface.h"
#include "../multi_wrapper/multi.h"
#endif

#include "rift_s.h"
#include "rift_s_interface.h"
#include "rift_s_util.h"
#include "rift_s_tracker.h"
#include "rift_s_passthrough.h"

#ifdef XRT_FEATURE_SLAM
static const bool slam_supported = true;
#else
static const bool slam_supported = false;
#endif

#ifdef XRT_BUILD_DRIVER_HANDTRACKING
static const bool hand_supported = true;
#else
static const bool hand_supported = false;
#endif

//! Specifies whether the user wants to use a SLAM tracker.
DEBUG_GET_ONCE_BOOL_OPTION(rift_s_slam, "RIFT_S_SLAM", true)

//! Specifies whether the user wants to use the hand tracker.
DEBUG_GET_ONCE_BOOL_OPTION(rift_s_handtracking, "RIFT_S_HANDTRACKING", true)

#ifdef XRT_FEATURE_SLAM
DEBUG_GET_ONCE_OPTION(slam_submit_from_start, "SLAM_SUBMIT_FROM_START", NULL)
#endif

static void *
rift_s_world_thread(void *ptr);

static xrt_result_t
rift_s_tracker_get_tracked_pose_imu(struct xrt_device *xdev,
                                    enum xrt_input_name name,
                                    int64_t at_timestamp_ns,
                                    struct xrt_space_relation *out_relation);

static void
rift_s_tracker_switch_method_cb(void *t_ptr)
{
	DRV_TRACE_MARKER();

	struct rift_s_tracker *t = t_ptr;
	t->slam_over_3dof = !t->slam_over_3dof;
	struct u_var_button *btn = &t->gui.switch_tracker_btn;

	if (t->slam_over_3dof) { // Use SLAM
		snprintf(btn->label, sizeof(btn->label), "Switch to 3DoF Tracking");
	} else { // Use 3DoF
		snprintf(btn->label, sizeof(btn->label), "Switch to SLAM Tracking");

		os_mutex_lock(&t->mutex);
		m_imu_3dof_reset(&t->fusion.i3dof);
		t->fusion.i3dof.rot = t->pose.orientation;
		rift_s_slam_guard_suspend(&t->slam_guard, &t->fusion.i3dof.rot);
		os_mutex_unlock(&t->mutex);
	}
}

XRT_MAYBE_UNUSED void
rift_s_fill_slam_imu_calibration(struct rift_s_tracker *t, struct rift_s_hmd_config *hmd_config)
{
	/* FIXME: Validate these hard coded standard deviations against
	 * some actual at-rest IMU measurements */
	const double a_bias_std = 0.001;
	const double a_noise_std = 0.016;

	const double g_bias_std = 0.0001;
	const double g_noise_std = 0.000282;

	/* we pass already corrected accel and gyro
	 * readings to Basalt, so the transforms and
	 * offsets are just identity / zero matrices */
	struct t_imu_calibration imu_calib = {
	    .accel =
	        {
	            .transform =
	                {
	                    {1.0, 0.0, 0.0},
	                    {0.0, 1.0, 0.0},
	                    {0.0, 0.0, 1.0},
	                },
	            .offset =
	                {
	                    0,
	                },
	            .bias_std = {a_bias_std, a_bias_std, a_bias_std},
	            .noise_std = {a_noise_std, a_noise_std, a_noise_std},
	        },
	    .gyro =
	        {
	            .transform =
	                {
	                    {1.0, 0.0, 0.0},
	                    {0.0, 1.0, 0.0},
	                    {0.0, 0.0, 1.0},
	                },
	            .offset =
	                {
	                    0,
	                },
	            .bias_std = {g_bias_std, g_bias_std, g_bias_std},
	            .noise_std = {g_noise_std, g_noise_std, g_noise_std},
	        },
	};

	struct t_slam_imu_calibration calib = {
	    .base = imu_calib,
	    .frequency = hmd_config->imu_config_info.imu_hz,
	};

	t->slam_calib.imu = calib;
}

//! Extended camera calibration for SLAM
static void
rift_s_fill_slam_cameras_calibration(struct rift_s_tracker *t, struct rift_s_hmd_config *hmd_config)
{
	/* SLAM frames are every 2nd frame of 60Hz camera feed */
	const int CAMERA_FREQUENCY = 30;

	struct rift_s_camera_calibration_block *camera_calibration = &hmd_config->camera_calibration;

	/* Compute the IMU from cam transform for each cam */
	struct xrt_pose device_from_imu, imu_from_device;
	math_pose_from_isometry(&hmd_config->imu_calibration.device_from_imu, &device_from_imu);
	math_pose_invert(&device_from_imu, &imu_from_device);

	t->slam_calib.cam_count = RIFT_S_CAMERA_COUNT;
	for (int i = 0; i < RIFT_S_CAMERA_COUNT; i++) {
		enum rift_s_camera_id cam_id = CAM_IDX_TO_ID[i];
		struct rift_s_camera_calibration *cam = &camera_calibration->cameras[cam_id];

		RIFT_S_DEBUG("DIAG FW cam%d id%d %ux%u fx=%.9g fy=%.9g cx=%.9g cy=%.9g", i, cam_id,
		             cam->roi.extent.w, cam->roi.extent.h, cam->projection.fx, cam->projection.fy,
		             cam->projection.cx, cam->projection.cy);
		for (int j = 0; j < 16; j++) RIFT_S_DEBUG("DIAG FW cam%d device_from_cam[%d]=%.9g", i, j, cam->device_from_camera.v[j]);
		for (int j = 0; j < 6; j++) RIFT_S_DEBUG("DIAG FW cam%d k[%d]=%.9g", i, j, cam->distortion.k[j]);
		RIFT_S_DEBUG("DIAG FW cam%d p1=%.9g p2=%.9g", i, cam->distortion.p1, cam->distortion.p2);
		struct xrt_pose device_from_cam;
		math_pose_from_isometry(&cam->device_from_camera, &device_from_cam);

		struct xrt_pose P_imu_cam;
		math_pose_transform(&imu_from_device, &device_from_cam, &P_imu_cam);

		struct xrt_matrix_4x4 T_imu_cam;
		math_matrix_4x4_isometry_from_pose(&P_imu_cam, &T_imu_cam);

		RIFT_S_DEBUG("IMU cam%d cam pose %f %f %f orient %f %f %f %f", i, P_imu_cam.position.x,
		             P_imu_cam.position.y, P_imu_cam.position.z, P_imu_cam.orientation.x,
		             P_imu_cam.orientation.y, P_imu_cam.orientation.z, P_imu_cam.orientation.w);

		struct t_slam_camera_calibration calib = {
		    .base = rift_s_get_cam_calib(&hmd_config->camera_calibration, cam_id),
		    .frequency = CAMERA_FREQUENCY,
		    .T_imu_cam = T_imu_cam,
		};
		t->slam_calib.cams[i] = calib;
	}
}

static void
rift_s_fill_slam_calibration(struct rift_s_tracker *t, struct rift_s_hmd_config *hmd_config)
{
	const struct rift_s_imu_calibration *c = &hmd_config->imu_calibration;
	RIFT_S_DEBUG("DIAG FW accel offset %.9g %.9g %.9g tempcoeff %.9g %.9g %.9g gyro offset %.9g %.9g %.9g",
	             c->accel.offset_at_0C.x, c->accel.offset_at_0C.y, c->accel.offset_at_0C.z,
	             c->accel.temp_coeff.x, c->accel.temp_coeff.y, c->accel.temp_coeff.z,
	             c->gyro.offset.x, c->gyro.offset.y, c->gyro.offset.z);
	for (int i = 0; i < 9; i++) RIFT_S_DEBUG("DIAG FW rect[%d] accel=%.9g gyro=%.9g", i, c->accel.rectification.v[i], c->gyro.rectification.v[i]);
	for (int i = 0; i < 16; i++) RIFT_S_DEBUG("DIAG FW device_from_imu[%d]=%.9g", i, c->device_from_imu.v[i]);
	rift_s_fill_slam_imu_calibration(t, hmd_config);
	rift_s_fill_slam_cameras_calibration(t, hmd_config);
}

static struct xrt_slam_sinks *
rift_s_create_slam_tracker(struct rift_s_tracker *t, struct xrt_frame_context *xfctx)
{
	DRV_TRACE_MARKER();

	struct xrt_slam_sinks *sinks = NULL;

#ifdef XRT_FEATURE_SLAM
	struct t_slam_tracker_config config = {0};
	t_slam_fill_default_config(&config);

	/* No need to refcount these parameters */
	config.cam_count = RIFT_S_CAMERA_COUNT;
	config.slam_calib = &t->slam_calib;
	if (debug_get_option_slam_submit_from_start() == NULL) {
		config.submit_from_start = true;
	}

	int create_status = t_slam_create(xfctx, &config, &t->tracking.slam, &sinks);
	if (create_status != 0) {
		return NULL;
	}

	int start_status = t_slam_start(t->tracking.slam);
	if (start_status != 0) {
		return NULL;
	}

	RIFT_S_DEBUG("Rift S SLAM tracker successfully started");
#endif

	return sinks;
}

static int
rift_s_create_hand_tracker(struct rift_s_tracker *t,
                           struct xrt_frame_context *xfctx,
                           struct xrt_device_masks_sink *masks_sink,
                           struct xrt_slam_sinks **out_sinks,
                           struct xrt_device **out_device)
{
	DRV_TRACE_MARKER();

	struct xrt_slam_sinks *sinks = NULL;
	struct xrt_device *device = NULL;

#ifdef XRT_BUILD_DRIVER_HANDTRACKING

	//!@todo What's a sensible boundary for Rift S?
	struct t_camera_extra_info extra_camera_info = {
	    0,
	};
	extra_camera_info.views[0].boundary_type = HT_IMAGE_BOUNDARY_NONE;
	extra_camera_info.views[1].boundary_type = HT_IMAGE_BOUNDARY_NONE;

	extra_camera_info.views[0].camera_orientation = CAMERA_ORIENTATION_90;
	extra_camera_info.views[1].camera_orientation = CAMERA_ORIENTATION_90;

	struct t_hand_tracking_create_info create_info = {.cams_info = extra_camera_info, .masks_sink = masks_sink};

	int create_status = ht_device_create(xfctx,           //
	                                     t->stereo_calib, //
	                                     create_info,     //
	                                     &sinks,          //
	                                     &device);
	if (create_status != 0) {
		return create_status;
	}

	if (device != NULL) {
		// Attach tracking override that links hand pose to the SLAM tracked position
		// The hand poses need to be rotated 90° because of the way we passed
		// the stereo camera configuration to the hand tracker.
		struct xrt_pose left_cam_rotated_from_imu;
		struct xrt_pose cam_rotate = {.orientation = {.x = 1.0, .y = 0.0, .z = 0.0, .w = 0.0},
		                              .position = {0, 0, 0}};
		math_pose_transform(&cam_rotate, &t->left_cam_from_imu, &left_cam_rotated_from_imu);

		device = multi_create_tracking_override(XRT_TRACKING_OVERRIDE_ATTACHED, device, &t->base,
		                                        XRT_INPUT_GENERIC_TRACKER_POSE, &left_cam_rotated_from_imu);
	}

	RIFT_S_DEBUG("Rift S HMD hand tracker successfully created");
#endif

	*out_sinks = sinks;
	*out_device = device;

	return 0;
}

static void
rift_s_fill_constellation_calibration(struct rift_s_tracker *t, struct rift_s_hmd_config *hmd_config)
{
/* Rift S thresholds for min brightness and min-blob-required magnitude. Quite high thresholds,
 * due to bright LED pulses and a lot of light bleed */
#define BLOB_PIXEL_THRESHOLD 0x60
#define BLOB_THRESHOLD_MIN 0x80

	struct rift_s_camera_calibration_block *camera_calibration = &hmd_config->camera_calibration;
	struct t_constellation_camera_group *out = &t->constellation_calib;

	struct xrt_pose device_from_imu, imu_from_device;
	math_pose_from_isometry(&hmd_config->imu_calibration.device_from_imu, &device_from_imu);
	math_pose_invert(&device_from_imu, &imu_from_device);

	out->cam_count = RIFT_S_CAMERA_COUNT;
	for (int i = 0; i < RIFT_S_CAMERA_COUNT; i++) {
		enum rift_s_camera_id cam_id = CAM_IDX_TO_ID[i];
		struct rift_s_camera_calibration *cam = &camera_calibration->cameras[cam_id];

		/* Compute the cam from IMU transform for each cam */
		RIFT_S_DEBUG("DIAG FW cam%d id%d %ux%u fx=%.9g fy=%.9g cx=%.9g cy=%.9g", i, cam_id,
		             cam->roi.extent.w, cam->roi.extent.h, cam->projection.fx, cam->projection.fy,
		             cam->projection.cx, cam->projection.cy);
		for (int j = 0; j < 16; j++) RIFT_S_DEBUG("DIAG FW cam%d device_from_cam[%d]=%.9g", i, j, cam->device_from_camera.v[j]);
		for (int j = 0; j < 6; j++) RIFT_S_DEBUG("DIAG FW cam%d k[%d]=%.9g", i, j, cam->distortion.k[j]);
		RIFT_S_DEBUG("DIAG FW cam%d p1=%.9g p2=%.9g", i, cam->distortion.p1, cam->distortion.p2);
		struct xrt_pose device_from_cam;
		math_pose_from_isometry(&cam->device_from_camera, &device_from_cam);

		struct xrt_pose P_imu_cam;
		math_pose_transform(&imu_from_device, &device_from_cam, &P_imu_cam);

		const struct xrt_pose P_YZ_flip = {
		    {1.0, 0.0, 0.0, 0.0},
		    {0.0, 0.0, 0.0},
		};

		struct xrt_pose P_imu_camcv;
		math_pose_transform(&P_YZ_flip, &P_imu_cam, &P_imu_camcv);

		struct xrt_rect roi = (struct xrt_rect){
		    .offset = {.w = cam_id * 640, .h = 0},
		    .extent = {.w = 640, .h = 480},
		};
		out->cams[i] = (struct t_constellation_camera){
		    .P_imu_cam = P_imu_camcv,
		    .slam_tracking_index = i,
		    .roi = roi,
		    .calibration = rift_s_get_cam_calib(&hmd_config->camera_calibration, cam_id),
		    .fisheye62_valid = true,
		    .fisheye62_radial = {cam->distortion.k[0], cam->distortion.k[1], cam->distortion.k[2],
		                         cam->distortion.k[3], cam->distortion.k[4], cam->distortion.k[5]},
		    .fisheye62_p1 = cam->distortion.p1,
		    .fisheye62_p2 = cam->distortion.p2,
		    .blob_min_threshold = BLOB_PIXEL_THRESHOLD,
		    .blob_detect_threshold = BLOB_THRESHOLD_MIN};

		RIFT_S_DEBUG("Constellation IMU cam%d cam pose %f %f %f orient %f %f %f %f", cam_id,
		             P_imu_cam.position.x, P_imu_cam.position.y, P_imu_cam.position.z, P_imu_cam.orientation.x,
		             P_imu_cam.orientation.y, P_imu_cam.orientation.z, P_imu_cam.orientation.w);
	}
}

static int
rift_s_create_constellation_tracker(struct rift_s_tracker *t, struct xrt_frame_context *xfctx)
{
	struct xrt_frame_sink *controller_sink = NULL;

	if (t_constellation_tracker_create(xfctx, &t->base, &t->constellation_calib, &t->controller_tracker,
	                                   &controller_sink, NULL /* @todo */) != 0) {
		RIFT_S_WARN("Failed to create Controller Tracker. Controllers will not be 6dof");
		return -1;
	}
	t->controller_sink = controller_sink;
	return 0;
}

void
rift_s_tracker_add_debug_ui(struct rift_s_tracker *t, void *root)
{
	u_var_add_gui_header(root, NULL, "Tracking");

	if (t->tracking.slam_enabled) {
		t->gui.switch_tracker_btn.cb = rift_s_tracker_switch_method_cb;
		t->gui.switch_tracker_btn.ptr = t;
		u_var_add_button(root, &t->gui.switch_tracker_btn, "Switch to 3DoF Tracking");
	}

	u_var_add_pose(root, &t->pose, "Tracked Pose");

	u_var_add_gui_header(root, NULL, "3DoF Tracking");
	m_imu_3dof_add_vars(&t->fusion.i3dof, root, "");

	u_var_add_gui_header(root, NULL, "SLAM Tracking");
	u_var_add_ro_text(root, t->gui.slam_status, "Tracker status");

	u_var_add_gui_header(root, NULL, "Hand Tracking");
	u_var_add_ro_text(root, t->gui.hand_status, "Tracker status");
}

/*!
 * Procedure to setup trackers: 3dof, SLAM and hand tracking.
 *
 * Determines which trackers to initialize
 *
 * @param xfctx the frame server that will own processing nodes
 * @param hmd_config HMD configuration and firmware info
 *
 * @return initialised tracker on success, NULL if creation fails
 */
struct rift_s_tracker *
rift_s_tracker_create(struct xrt_tracking_origin *origin,
                      struct xrt_frame_context *xfctx,
                      struct rift_s_hmd_config *hmd_config)
{
	struct rift_s_tracker *t = U_DEVICE_ALLOCATE(struct rift_s_tracker, U_DEVICE_ALLOC_TRACKING_NONE, 1, 0);
	if (t == NULL) {
		return NULL;
	}

	t->base.tracking_origin = origin;
	t->base.get_tracked_pose = rift_s_tracker_get_tracked_pose_imu;

	// Pose / state lock
	int ret = os_mutex_init(&t->mutex);
	if (ret != 0) {
		RIFT_S_ERROR("Failed to init mutex!");
		free(t);
		return NULL;
	}

	ret = os_mutex_init(&t->slam_mutex);
	if (ret != 0) {
		os_mutex_destroy(&t->mutex);
		free(t);
		return NULL;
	}
	rift_s_slam_guard_init(&t->slam_guard);
	t->slam_guard.require_relocalization = false; // Set from the saved map below; no map means a fresh world.
	t->world_command_id = os_monotonic_get_ns();

	// Compute IMU and camera device poses for get_tracked_pose relations
	math_pose_from_isometry(&hmd_config->imu_calibration.device_from_imu, &t->device_from_imu);

	struct xrt_pose device_from_left_cam;
	struct rift_s_camera_calibration *left_cam = &hmd_config->camera_calibration.cameras[RIFT_S_CAMERA_FRONT_LEFT];
	math_pose_from_isometry(&left_cam->device_from_camera, &device_from_left_cam);

	struct xrt_pose left_cam_from_device;
	math_pose_invert(&device_from_left_cam, &left_cam_from_device);
	math_pose_transform(&left_cam_from_device, &t->device_from_imu, &t->left_cam_from_imu);

	// Decide whether to initialize the SLAM tracker
	bool slam_wanted = debug_get_bool_option_rift_s_slam();
	bool slam_enabled = slam_supported && slam_wanted;

	// Decide whether to initialize the hand tracker
	bool hand_wanted = debug_get_bool_option_rift_s_handtracking();
	bool hand_enabled = hand_supported && hand_wanted;

	t->tracking.slam_enabled = slam_enabled;
	t->tracking.hand_enabled = hand_enabled;

	t->slam_over_3dof = slam_enabled; // We prefer SLAM over 3dof tracking if possible

	const char *slam_status = t->tracking.slam_enabled ? "Enabled"
	                          : !slam_wanted           ? "Disabled by the user (envvar set to false)"
	                          : !slam_supported        ? "Unavailable (not built)"
	                                                   : NULL;

	const char *hand_status = t->tracking.hand_enabled ? "Enabled"
	                          : !hand_wanted           ? "Disabled by the user (envvar set to false)"
	                          : !hand_supported        ? "Unavailable (not built)"
	                                                   : NULL;

	assert(slam_status != NULL && hand_status != NULL);

	(void)snprintf(t->gui.slam_status, sizeof(t->gui.slam_status), "%s", slam_status);
	(void)snprintf(t->gui.hand_status, sizeof(t->gui.hand_status), "%s", hand_status);

	// Initialize 3DoF tracker
	m_imu_3dof_init(&t->fusion.i3dof, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
	m_relation_history_create(&t->fusion.history);

	t->pose.orientation.w = 1.0f; // All other values set to zero by U_DEVICE_ALLOCATE (which calls U_CALLOC)

	// Construct the stereo camera calibration for the front cameras
	t->stereo_calib = rift_s_create_stereo_camera_calib_rotated(&hmd_config->camera_calibration);
	rift_s_fill_slam_calibration(t, hmd_config);

	// Initialize the input sinks for the camera to send to

	// Initialize SLAM tracker
	struct xrt_slam_sinks *slam_sinks = NULL;
	if (t->tracking.slam_enabled) {
		slam_sinks = rift_s_create_slam_tracker(t, xfctx);
		if (slam_sinks == NULL) {
			RIFT_S_WARN("Unable to setup the SLAM tracker");
			rift_s_tracker_destroy(t);
			return NULL;
		}
	}

	// Initialize hand tracker
	struct xrt_slam_sinks *hand_sinks = NULL;
	struct xrt_device *hand_device = NULL;
	struct xrt_device_masks_sink *masks_sink = slam_sinks ? slam_sinks->hand_masks : NULL;
	if (t->tracking.hand_enabled) {
		int hand_status = rift_s_create_hand_tracker(t, xfctx, masks_sink, &hand_sinks, &hand_device);
		if (hand_status != 0 || hand_sinks == NULL || hand_device == NULL) {
			RIFT_S_WARN("Unable to setup the hand tracker");
			rift_s_tracker_destroy(t);
			return NULL;
		}
	}

	// Initialize controller constellation tracking
	rift_s_fill_constellation_calibration(t, hmd_config);

	int constellation_status = rift_s_create_constellation_tracker(t, xfctx);
	if (constellation_status != 0) {
		RIFT_S_WARN("Unable to setup the controller constellation tracker");
		rift_s_tracker_destroy(t);
		return NULL;
	}

	// Setup sinks depending on tracking configuration
	struct xrt_slam_sinks entry_sinks = {0};
	if (slam_enabled && hand_enabled) {
		struct xrt_frame_sink *entry_cam0_sink = NULL;
		struct xrt_frame_sink *entry_cam1_sink = NULL;

		u_sink_split_create(xfctx, slam_sinks->cams[0], hand_sinks->cams[0], &entry_cam0_sink);
		u_sink_split_create(xfctx, slam_sinks->cams[1], hand_sinks->cams[1], &entry_cam1_sink);

		entry_sinks = *slam_sinks;
		entry_sinks.cams[0] = entry_cam0_sink;
		entry_sinks.cams[1] = entry_cam1_sink;
	} else if (slam_enabled) {
		entry_sinks = *slam_sinks;
	} else if (hand_enabled) {
		entry_sinks = *hand_sinks;
	} else {
		entry_sinks = (struct xrt_slam_sinks){0};
	}

	t->base.camera = rift_s_passthrough_create(&hmd_config->camera_calibration);
	t->slam_sinks = entry_sinks;
	t->handtracker = hand_device;
	struct rift_s_world_anchor_config world_config = {.camera_count = RIFT_S_CAMERA_COUNT,
	                                                  .require_boundary = true};
	struct xrt_pose imu_from_device;
	math_pose_invert(&t->device_from_imu, &imu_from_device);
	for (int eye = 0; eye < RIFT_S_CAMERA_COUNT; eye++) {
		world_config.camera[eye] = hmd_config->camera_calibration.cameras[CAM_IDX_TO_ID[eye]];
		struct xrt_pose device_from_camera;
		math_pose_from_isometry(&world_config.camera[eye].device_from_camera, &device_from_camera);
		math_pose_transform(&imu_from_device, &device_from_camera, &world_config.imu_from_camera[eye]);
	}
	char map_path[1024];
	const char *bench = getenv("MONADO_STEAMVR_BENCH");
	const char *bench_map = getenv("RIFT_S_WORLD_BENCH_MAP");
	if (bench && strcmp(bench, "1") == 0 && bench_map && bench_map[0] == '/') {
		// Offline test maps use their capture's SLAM gauge, independent of the live boundary.
		world_config.require_boundary = false;
		t->world_anchor = rift_s_world_anchor_create(&world_config, bench_map);
		RIFT_S_INFO("World bench map override active");
	} else if (u_file_get_path_in_config_dir("rift-s-world.bin", map_path, sizeof(map_path)) > 0) {
		t->world_anchor = rift_s_world_anchor_create(&world_config, map_path);
	}
	t->world_map_available = rift_s_world_anchor_has_map(t->world_anchor);
	t->slam_guard.require_relocalization =
	    t->world_map_available || rift_s_world_anchor_has_boundary(t->world_anchor);
	if (t->world_anchor && os_thread_helper_init(&t->world_thread) == 0) {
		t->world_thread_initialized = true;
		if (os_thread_helper_start(&t->world_thread, rift_s_world_thread, t) != 0) {
			os_thread_helper_destroy(&t->world_thread);
			t->world_thread_initialized = false;
		}
	}
	RIFT_S_INFO("World worker started=%d map_available=%d require_relocalization=%d", t->world_thread_initialized,
	            t->world_map_available, t->slam_guard.require_relocalization);
	if (t->slam_guard.require_relocalization && !t->world_map_available)
		RIFT_S_WARN("Saved boundary has no compatible visual map; room setup is required to replace its frame");

	return t;
}

void
rift_s_tracker_destroy(struct rift_s_tracker *t)
{
	if (t->world_thread_initialized)
		os_thread_helper_destroy(&t->world_thread);
	for (int eye = 0; eye < RIFT_S_CAMERA_COUNT; eye++)
		xrt_frame_reference(&t->world_frames[eye], NULL);
	rift_s_world_anchor_destroy(t->world_anchor);
	rift_s_passthrough_destroy(t->base.camera);
	t_stereo_camera_calibration_reference(&t->stereo_calib, NULL);

	m_imu_3dof_close(&t->fusion.i3dof);
	m_relation_history_destroy(&t->fusion.history);
	os_mutex_destroy(&t->slam_mutex);
	os_mutex_destroy(&t->mutex);
}

struct xrt_slam_sinks *
rift_s_tracker_get_slam_sinks(struct rift_s_tracker *t)
{
	return &t->in_slam_sinks;
}

struct xrt_device *
rift_s_tracker_get_hand_tracking_device(struct rift_s_tracker *t)
{
	return t->handtracker;
}

void
rift_s_tracker_clock_update(struct rift_s_tracker *t, uint64_t device_timestamp_ns, timepoint_ns local_timestamp_ns)
{
	os_mutex_lock(&t->mutex);
	time_duration_ns last_hw2mono = t->hw2mono;
	const float freq = 250.0;

	m_clock_offset_a2b(freq, device_timestamp_ns, local_timestamp_ns, &t->hw2mono);

	if (!t->have_hw2mono) {
		/* At startup, Rift S can send old data that throws off
		 * our clock estimation and confuses the SLAM tracker into
		 * not consuming data correctly. This code tries to
		 * ensure the clock has stabilised before uncorking
		 * the SLAM + IMU processing */
		time_duration_ns change_ns = last_hw2mono - t->hw2mono;
		if (change_ns >= -U_TIME_HALF_MS_IN_NS && change_ns <= U_TIME_HALF_MS_IN_NS) {
			t->valid_clock_observations++;
			if (t->valid_clock_observations > 500) {
				RIFT_S_INFO("HMD device to local clock map stabilised");
				t->have_hw2mono = true;
			}
		} else {
			/* Invalid clock observation. Start again */
			t->valid_clock_observations = 0;
			t->hw2mono = 0;
			m_clock_offset_a2b(freq, device_timestamp_ns, local_timestamp_ns, &t->hw2mono);
		}
	}
	os_mutex_unlock(&t->mutex);
}

//! Camera specific logic for clock conversion
static void
clock_hw2mono_get(struct rift_s_tracker *t, uint64_t device_ts, timepoint_ns *out)
{
	*out = t->hw2mono + device_ts;
}

/* Three missing 30 Hz SLAM frames are enough to stop submitting IMU before
 * Basalt's bounded IMU queue fills. Caller holds mutex. */
#define RIFT_S_CAMERA_TIMEOUT_NS (100 * U_TIME_1MS_IN_NS)

static void
pause_slam_locked(struct rift_s_tracker *t)
{
	if (t->slam_paused) {
		return;
	}
	t->slam_paused = true;
	t->slam_restart_pending = t->tracking.slam_enabled && t->last_camera_arrival_ns != 0;
	// SLAM is realigned onto the held pose once it is consistent again.
	rift_s_slam_guard_suspend(&t->slam_guard, &t->pose.orientation);
	RIFT_S_INFO("Head tracking paused (%s); holding last pose", t->standby ? "standby" : "camera timeout");
}

static void
check_camera_timeout_locked(struct rift_s_tracker *t, timepoint_ns now_ns)
{
	if (t->standby ||
	    (t->last_camera_arrival_ns != 0 && now_ns - t->last_camera_arrival_ns > RIFT_S_CAMERA_TIMEOUT_NS)) {
		pause_slam_locked(t);
	}
}

void
rift_s_tracker_set_standby(struct rift_s_tracker *t, bool standby)
{
	os_mutex_lock(&t->mutex);
	if (standby != t->standby) {
		t->standby = standby;
		if (standby) {
			pause_slam_locked(t);
		} else {
			RIFT_S_INFO("Headset awake; waiting for fresh camera frames");
		}
	}
	os_mutex_unlock(&t->mutex);
}

void
rift_s_tracker_imu_update(struct rift_s_tracker *t,
                          uint64_t device_timestamp_ns,
                          const struct xrt_vec3 *accel,
                          const struct xrt_vec3 *gyro)
{
	os_mutex_lock(&t->mutex);

	check_camera_timeout_locked(t, os_monotonic_get_ns());

	/* Ignore packets before we're ready and clock is stable */
	if (!t->ready_for_data || !t->have_hw2mono || t->last_frame_time == 0) {
		os_mutex_unlock(&t->mutex);
		return;
	}

	/* Get the smoothed monotonic time estimate for this IMU sample */
	timepoint_ns local_timestamp_ns;

	clock_hw2mono_get(t, device_timestamp_ns, &local_timestamp_ns);

	if (t->fusion.last_imu_local_timestamp_ns != 0 && local_timestamp_ns < t->fusion.last_imu_local_timestamp_ns) {
		RIFT_S_WARN("IMU time went backward by %" PRId64 " ns",
		            local_timestamp_ns - t->fusion.last_imu_local_timestamp_ns);
	} else {
		m_imu_3dof_update(&t->fusion.i3dof, local_timestamp_ns, accel, gyro);
	}

	RIFT_S_TRACE("IMU timestamp %" PRIu64 " (dt %f) hw2mono local ts %" PRIu64 " (dt %f) offset %" PRId64,
	             device_timestamp_ns,
	             (double)(device_timestamp_ns - t->fusion.last_imu_timestamp_ns) / 1000000000.0, local_timestamp_ns,
	             (double)(local_timestamp_ns - t->fusion.last_imu_local_timestamp_ns) / 1000000000.0, t->hw2mono);

	if (local_timestamp_ns / U_TIME_1S_IN_NS != t->fusion.last_imu_local_timestamp_ns / U_TIME_1S_IN_NS) {
		RIFT_S_TRACE("HMD IMU accel=%f,%f,%f fusion=%f,%f,%f,%f", accel->x, accel->y, accel->z,
		             t->fusion.i3dof.rot.x, t->fusion.i3dof.rot.y, t->fusion.i3dof.rot.z,
		             t->fusion.i3dof.rot.w);
	}

	uint64_t sample_generation = t->slam_generation;
	t->fusion.last_angular_velocity = *gyro;
	t->fusion.last_imu_timestamp_ns = device_timestamp_ns;
	t->fusion.last_imu_local_timestamp_ns = local_timestamp_ns;

	t->pose.orientation = t->fusion.i3dof.rot;
	struct xrt_space_relation fusion_relation = XRT_SPACE_RELATION_ZERO;
	fusion_relation.pose.orientation = t->pose.orientation;
	math_quat_rotate_derivative(&t->pose.orientation, gyro, &fusion_relation.angular_velocity);
	fusion_relation.relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                 XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                                 XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
	if (t->fusion.history) m_relation_history_push(t->fusion.history, &fusion_relation, local_timestamp_ns);

	os_mutex_unlock(&t->mutex);

	// A delayed old-generation IMU must never enter the replacement backend. Do not
	// stall HID ingestion while the camera thread is replacing a session.
	if (os_mutex_trylock(&t->slam_mutex) != 0)
		return;
	os_mutex_lock(&t->mutex);
	bool submit = !t->standby && !t->slam_paused && sample_generation == t->slam_generation;
	os_mutex_unlock(&t->mutex);
	if (submit && t->slam_sinks.imu) {
		/* Push IMU sample to the SLAM tracker */
		struct xrt_vec3_f64 accel64 = {accel->x, accel->y, accel->z};
		struct xrt_vec3_f64 gyro64 = {gyro->x, gyro->y, gyro->z};
		struct xrt_imu_sample sample = {
		    .timestamp_ns = local_timestamp_ns, .accel_m_s2 = accel64, .gyro_rad_secs = gyro64};

		xrt_sink_push_imu(t->slam_sinks.imu, &sample);
	}
	os_mutex_unlock(&t->slam_mutex);
}

#define UPPER_32BITS(x) ((x) & 0xffffffff00000000ULL)

// Called with tracker mutex held
static timepoint_ns
raw_frame_ts_to_mono_ts(struct rift_s_tracker *t, uint64_t frame_ts_ns)
{
	timepoint_ns frame_time;

	/* Ensure the input timestamp is within 32-bits of the IMU
	 * time, because the timestamps are reported and extended to 64-bits
	 * separately and can end up in different epochs */
	uint64_t adj_frame_ts_ns = frame_ts_ns + t->camera_ts_offset;
	int64_t frame_to_imu_uS = (adj_frame_ts_ns / 1000 - t->fusion.last_imu_timestamp_ns / 1000);

	if (frame_to_imu_uS < -(int64_t)(1ULL << 31) || frame_to_imu_uS > (int64_t)(1ULL << 31)) {
		t->camera_ts_offset =
		    (UPPER_32BITS(t->fusion.last_imu_timestamp_ns / 1000) - UPPER_32BITS(frame_ts_ns / 1000)) * 1000;
		RIFT_S_DEBUG("Applying epoch offset to frame times of %" PRId64 " (frame->imu was %" PRId64 " µS)",
		             t->camera_ts_offset, frame_to_imu_uS);
	}
	frame_ts_ns += t->camera_ts_offset;

	clock_hw2mono_get(t, frame_ts_ns, &frame_time);
	return frame_time;
}

void
rift_s_tracker_push_slam_frames(struct rift_s_tracker *t,
                                uint64_t frame_ts_ns,
                                struct xrt_frame *frames[RIFT_S_CAMERA_COUNT])
{
	os_mutex_lock(&t->slam_mutex);
	os_mutex_lock(&t->mutex);

	/* Ignore packets before we're ready, and drop any frames before we have IMU */
	if (!t->ready_for_data || !t->have_hw2mono) {
		os_mutex_unlock(&t->mutex);
		os_mutex_unlock(&t->slam_mutex);
		return;
	}

	timepoint_ns now_ns = os_monotonic_get_ns();
	timepoint_ns frame_mono_ns = raw_frame_ts_to_mono_ts(t, frame_ts_ns);
	if (frame_mono_ns < t->last_frame_time) {
		RIFT_S_WARN("Camera frame time went backward by %" PRId64 " ns", frame_mono_ns - t->last_frame_time);
		os_mutex_unlock(&t->mutex);
		os_mutex_unlock(&t->slam_mutex);
		return;
	}
	RIFT_S_TRACE("SLAM frame timestamp %" PRIu64 " local %" PRIu64, frame_ts_ns, frame_mono_ns);
	t->last_frame_time = frame_mono_ns;

	/* Queued pre-standby frames must not wake tracking. */
	bool fresh =
	    now_ns - frame_mono_ns <= RIFT_S_CAMERA_TIMEOUT_NS && frame_mono_ns - now_ns <= RIFT_S_CAMERA_TIMEOUT_NS;
	bool submit = fresh && !t->standby;
	bool restart = submit && t->tracking.slam_enabled && t->slam_restart_pending;
	if (restart) {
		t->slam_generation++;
		t->slam_min_sample_ns = frame_mono_ns;
		t->slam_guard.world_valid = false;
		t->slam_guard.boundary_confirmed = false;
		t->slam_guard.world_transition = false;
	}
	os_mutex_unlock(&t->mutex);

	for (int i = 0; i < RIFT_S_CAMERA_COUNT; i++) {
		frames[i]->timestamp = frame_mono_ns;
	}
	rift_s_passthrough_push(t->base.camera, frames[0], frames[1]);
	if (fresh && t->world_thread_initialized && frame_mono_ns - t->world_last_queued_ns >= 200 * U_TIME_1MS_IN_NS) {
		os_thread_helper_lock(&t->world_thread);
		for (int eye = 0; eye < RIFT_S_CAMERA_COUNT; eye++)
			xrt_frame_reference(&t->world_frames[eye], frames[eye]);
		t->world_last_queued_ns = frame_mono_ns;
		os_thread_helper_unlock(&t->world_thread);
	}

	if (restart) {
		/* Safety-patched Basalt advertises recreation support. Replacement clears flow,
		 * estimator and queues; unknown backends keep the estimator-reset fallback. */
		int ret = -1;
		const char *how = "none";
		timepoint_ns restart_begin_ns = os_monotonic_get_ns();
#ifdef XRT_FEATURE_SLAM
		if (t->tracking.slam->restart != NULL && t->tracking.slam->restart(t->tracking.slam) == 0) {
			ret = 0;
			how = "recreate";
		} else if (t_slam_reset(t->tracking.slam) == 0) {
			ret = 0;
			how = "reset";
		}
#endif
		os_mutex_lock(&t->mutex);
		t->slam_restart_pending = false;
		/* Samples from before the restart belong to the old session. */
		t->slam_guard.stable_samples = 0;
		if (ret == 0)
			rift_s_slam_guard_backend_started(&t->slam_guard, os_monotonic_get_ns());
		t->feature_gate = (struct rift_s_slam_feature_gate){0};
		os_mutex_unlock(&t->mutex);
		if (ret == 0) {
			RIFT_S_INFO("SLAM session restarted method=%s elapsed_ms=%.3f; waiting for consistent poses",
			            how, (os_monotonic_get_ns() - restart_begin_ns) / 1e6);
		} else {
			RIFT_S_WARN("Unable to restart SLAM; the guard keeps holding until its poses are consistent");
		}
	}

	if (submit) {
		os_mutex_lock(&t->mutex);
		/* Proximity may change while the backend is restarting. */
		submit = !t->standby;
		if (submit) {
			t->last_camera_arrival_ns = os_monotonic_get_ns();
			t->slam_paused = false;
		}
		os_mutex_unlock(&t->mutex);
	}

	if (submit) {
		for (int i = 0; i < RIFT_S_CAMERA_COUNT; i++) {
			if (t->slam_sinks.cams[i]) {
				xrt_sink_push_frame(t->slam_sinks.cams[i], frames[i]);
			}
		}
	}
	os_mutex_unlock(&t->slam_mutex);
}

struct t_constellation_tracked_device_connection *
rift_s_tracker_add_controller(struct rift_s_tracker *t,
                              struct xrt_device *xdev,
                              struct t_constellation_tracked_device_callbacks *cb)
{
	if (t->controller_tracker != NULL) {
		return t_constellation_tracker_add_device(t->controller_tracker, xdev, cb);
	}
	return NULL;
}

void
rift_s_tracker_push_controller_frameset(struct rift_s_tracker *t, uint64_t frame_ts_ns, struct xrt_frame *frameset)
{
	os_mutex_lock(&t->mutex);

	/* Ignore packets before we're ready */
	if (!t->ready_for_data) {
		os_mutex_unlock(&t->mutex);
		return;
	}

	if (!t->have_hw2mono) {
		/* Drop any frames before we have IMU */
		os_mutex_unlock(&t->mutex);
		return;
	}

	timepoint_ns frame_mono_ns = raw_frame_ts_to_mono_ts(t, frame_ts_ns);
	bool usable = !t->standby && !t->slam_paused &&
	              (timepoint_ns)os_monotonic_get_ns() - frame_mono_ns <= RIFT_S_CAMERA_TIMEOUT_NS;
	os_mutex_unlock(&t->mutex);
	if (!usable) {
		return;
	}

	if (t->controller_sink) {
		RIFT_S_TRACE("Controller frame seq=%" PRIu64 " hw=%" PRIu64 " mono=%" PRIu64 " age_ms=%.3f",
		             frameset->source_sequence, frame_ts_ns, frame_mono_ns,
		             (double)((int64_t)os_monotonic_get_ns() - frame_mono_ns) / U_TIME_1MS_IN_NS);
		frameset->timestamp = frame_mono_ns;
		xrt_sink_push_frame(t->controller_sink, frameset);
	}
}

//! Specific pose correction for Basalt to OpenXR coordinates
XRT_MAYBE_UNUSED static inline void
rift_s_tracker_correct_pose_from_basalt(struct xrt_pose *pose)
{
	struct xrt_quat q = {0.70710678, 0, 0, -0.70710678};
	math_quat_rotate(&q, &pose->orientation, &pose->orientation);
	math_quat_rotate_vec3(&q, &pose->position, &pose->position);
}

static xrt_result_t
rift_s_tracker_get_tracked_pose_imu(struct xrt_device *xdev,
                                    enum xrt_input_name name,
                                    int64_t at_timestamp_ns,
                                    struct xrt_space_relation *out_relation)
{
	struct rift_s_tracker *tracker = (struct rift_s_tracker *)(xdev);
	if (name != XRT_INPUT_GENERIC_TRACKER_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&tracker->base, rift_s_log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	rift_s_tracker_get_tracked_pose(tracker, RIFT_S_TRACKER_POSE_IMU, at_timestamp_ns, out_relation);

	return XRT_SUCCESS;
}

void
rift_s_tracker_get_tracked_pose(struct rift_s_tracker *t,
                                enum rift_s_tracker_pose pose,
                                uint64_t at_timestamp_ns,
                                struct xrt_space_relation *out_relation)
{
	struct xrt_relation_chain xrc = {0};

	if (pose == RIFT_S_TRACKER_POSE_DEVICE) {
		m_relation_chain_push_inverted_pose_if_not_identity(&xrc, &t->device_from_imu);
	} else if (pose == RIFT_S_TRACKER_POSE_LEFT_CAMERA) {
		m_relation_chain_push_inverted_pose_if_not_identity(&xrc, &t->left_cam_from_imu);
	}

	// Off-head (proximity) or without camera frames, SLAM input is stopped and the head pose is held.
	os_mutex_lock(&t->mutex);
	check_camera_timeout_locked(t, os_monotonic_get_ns());
	bool query_slam = t->tracking.slam_enabled && t->slam_over_3dof;
	bool paused = t->tracking.slam_enabled && (t->standby || t->slam_paused || t->slam_restart_pending);
	uint64_t sample_generation = t->slam_generation;
	os_mutex_unlock(&t->mutex);

	// Basalt can diverge (tens of metres to ~1e7 m away within seconds, seen with the headset left idle).
	// The guard publishes SLAM only while it stays near the origin and moves like a head; otherwise it
	// falls back to the 3DoF orientation at the last good position and asks for a SLAM re-initialisation.
	struct xrt_space_relation slam_relation = XRT_SPACE_RELATION_ZERO;
	struct xrt_space_relation query_relation = XRT_SPACE_RELATION_ZERO;
	timepoint_ns sample_ts = 0;
	bool slam_valid = false;
	bool features_known = false;
	int feature_count = 0;
	timepoint_ns feature_ts = 0;
	if (query_slam && !paused) {
#ifdef XRT_FEATURE_SLAM
		slam_valid = t_slam_get_latest_sample(t->tracking.slam, at_timestamp_ns, &sample_ts,
		                                     &slam_relation, &query_relation) &&
		             (slam_relation.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0;
		timepoint_ns now_ns = os_monotonic_get_ns();
		slam_valid = slam_valid && sample_ts <= now_ns && now_ns - sample_ts < 250 * U_TIME_1MS_IN_NS;
		// !todo Correct pose depending on the VIT system in use, this should be done in the system itself.
		// For now, assume that we are using Basalt.
		rift_s_tracker_correct_pose_from_basalt(&slam_relation.pose);
		rift_s_tracker_correct_pose_from_basalt(&query_relation.pose);
		struct xrt_quat basalt_to_xr = {0.70710678, 0, 0, -0.70710678};
		math_quat_rotate_vec3(&basalt_to_xr, &slam_relation.linear_velocity, &slam_relation.linear_velocity);
		math_quat_rotate_vec3(&basalt_to_xr, &slam_relation.angular_velocity, &slam_relation.angular_velocity);
		math_quat_rotate_vec3(&basalt_to_xr, &query_relation.linear_velocity, &query_relation.linear_velocity);
		math_quat_rotate_vec3(&basalt_to_xr, &query_relation.angular_velocity, &query_relation.angular_velocity);
		features_known = t_slam_get_feature_count(t->tracking.slam, &feature_count, &feature_ts);
#endif
	}

	// Without any good SLAM position, put the head at eye height above the floor. The tracking origin
	// may already carry that height as its offset (the Rift S builder sets 1.6 m).
	struct xrt_vec3 no_pose_position = rift_s_slam_guard_no_pose_position(
	    t->base.tracking_origin != NULL ? t->base.tracking_origin->initial_offset.position.y : 0.0f);

	struct xrt_space_relation imu_relation = XRT_SPACE_RELATION_ZERO;
	const enum xrt_space_relation_flags held_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT);

	os_mutex_lock(&t->mutex);
	math_quat_normalize(&t->pose.orientation);
	paused = paused || t->standby || t->slam_paused || t->slam_restart_pending ||
	         sample_generation != t->slam_generation;
	slam_valid = slam_valid && sample_ts >= t->slam_min_sample_ns;
	struct xrt_quat sample_fusion = t->pose.orientation;
	struct xrt_quat query_fusion = t->pose.orientation;
	if (t->fusion.history) {
		struct xrt_space_relation queried;
		if (m_relation_history_get(t->fusion.history, at_timestamp_ns, &queried) != M_RELATION_HISTORY_RESULT_INVALID)
			query_fusion = queried.pose.orientation;
		if (m_relation_history_get(t->fusion.history, sample_ts, &queried) != M_RELATION_HISTORY_RESULT_INVALID)
			sample_fusion = queried.pose.orientation;
	}
	struct rift_s_slam_guard *g = &t->slam_guard;
	uint32_t divergences = g->divergences;
	uint32_t recoveries = g->recoveries;
	enum rift_s_slam_guard_state previous_state = g->state;
	uint32_t resets = g->resets;
	int64_t previous_good_ns = g->good_ns;
	enum rift_s_slam_guard_reason reason = RIFT_S_SLAM_GUARD_OK;

	enum rift_s_slam_feature_event feature_event = RIFT_S_SLAM_FEATURES_NO_CHANGE;
	if (query_slam && !paused) {
		feature_event = rift_s_slam_feature_gate_update(&t->feature_gate, features_known, feature_count,
		                                                (int64_t)feature_ts);
		if (feature_event == RIFT_S_SLAM_FEATURES_LOST) {
			rift_s_slam_guard_suspend(g, &t->pose.orientation);
		} else if (feature_event == RIFT_S_SLAM_FEATURES_RETURNED) {
			t->slam_restart_pending = true;
		}
	}
	bool hold = paused || (query_slam && t->feature_gate.lost);
	if (query_slam && !paused && t->feature_gate.lost) {
		// Keep the health clock running through a feature hold without accepting new world poses.
		if (slam_valid && rift_s_slam_guard_pose_usable(&slam_relation.pose, &sample_fusion)) {
			g->invalid_since_ns = 0;
		} else {
			struct xrt_pose ignored;
			rift_s_slam_guard_update(g, slam_valid, &slam_relation.pose,
			                         slam_valid ? sample_ts : os_monotonic_get_ns(), &sample_fusion,
			                         &no_pose_position, &ignored, &reason);
			if (rift_s_slam_guard_take_reset(g, os_monotonic_get_ns()))
				t->slam_restart_pending = true;
		}
	}

	if (hold) {
		// Freeze the head where it was last shown: untracked, no motion.
		if (!t->have_published_imu_pose) {
			rift_s_slam_guard_fallback_pose(g, &t->pose.orientation, &no_pose_position,
			                                &t->published_imu_pose);
			t->have_published_imu_pose = true;
		}
		imu_relation.pose = t->published_imu_pose;
		imu_relation.relation_flags = held_flags;
	} else {
		bool use_slam = false;
		if (query_slam) {
			use_slam = rift_s_slam_guard_update(g, slam_valid, &slam_relation.pose,
			                                    slam_valid ? sample_ts : os_monotonic_get_ns(), &sample_fusion,
			                                    &no_pose_position, &imu_relation.pose, &reason);
			if (rift_s_slam_guard_take_reset(g, os_monotonic_get_ns())) {
				t->slam_restart_pending = true;
			}
		} else {
			// TODO: Estimate pose at timestamp at_timestamp_ns
			rift_s_slam_guard_fallback_pose(g, &t->pose.orientation, &no_pose_position, &imu_relation.pose);
		}

		if (use_slam && rift_s_slam_guard_query(g, &query_relation.pose, at_timestamp_ns, &query_fusion,
		                                     &no_pose_position, &imu_relation.pose)) {
			math_quat_rotate_vec3(&g->align.orientation, &query_relation.linear_velocity,
			                      &imu_relation.linear_velocity);
			math_quat_rotate_vec3(&g->align.orientation, &query_relation.angular_velocity,
			                      &imu_relation.angular_velocity);
			imu_relation.relation_flags =
			    (enum xrt_space_relation_flags)(held_flags | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
			                                    XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
		} else {
			rift_s_slam_guard_fallback_pose(g, &query_fusion, &no_pose_position, &imu_relation.pose);
			imu_relation.angular_velocity = t->fusion.last_angular_velocity;
			imu_relation.relation_flags =
			    (enum xrt_space_relation_flags)(held_flags | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
		}
		if (pose == RIFT_S_TRACKER_POSE_DEVICE || !t->have_published_imu_pose) {
			t->published_imu_pose = imu_relation.pose;
			t->have_published_imu_pose = true;
		}
	}

	if (at_timestamp_ns < (uint64_t)t->slam_min_sample_ns)
		imu_relation.relation_flags = held_flags;

	if (feature_event == RIFT_S_SLAM_FEATURES_LOST) {
		RIFT_S_WARN("SLAM cameras see %d features (< %d); holding head pose", feature_count,
		            RIFT_S_SLAM_FEATURES_MIN);
	} else if (feature_event == RIFT_S_SLAM_FEATURES_RETURNED) {
		RIFT_S_INFO("SLAM cameras see %d features again; re-initialising SLAM", feature_count);
	}
	if (g->divergences != divergences) {
		RIFT_S_WARN(
		    "SLAM diverged reason=%s distance_m=%.2f step_m=%.2f slam=(%.2f,%.2f,%.2f) "
		    "held=(%.2f,%.2f,%.2f) query_ns=%lld previous_ns=%lld delta_ms=%.3f pose_kind=%d divergences=%u: 3DoF until SLAM is consistent again",
		    rift_s_slam_guard_reason_str(reason), g->last_reject_distance_m, g->last_reject_step_m,
		    slam_relation.pose.position.x, slam_relation.pose.position.y, slam_relation.pose.position.z,
		    imu_relation.pose.position.x, imu_relation.pose.position.y, imu_relation.pose.position.z,
		    (long long)at_timestamp_ns, (long long)previous_good_ns,
		    ((int64_t)at_timestamp_ns - previous_good_ns) / 1e6, pose, g->divergences);
	}
	if (g->recoveries != recoveries) {
		RIFT_S_INFO("SLAM recovered position=(%.2f,%.2f,%.2f) recoveries=%u resets=%u",
		            imu_relation.pose.position.x, imu_relation.pose.position.y, imu_relation.pose.position.z,
		            g->recoveries, g->resets);
	}
	if (previous_state != g->state || recoveries != g->recoveries || resets != g->resets) {
		float origin_y = t->base.tracking_origin ? t->base.tracking_origin->initial_offset.position.y : 0;
		RIFT_S_INFO("SLAM alignment state=%d->%d query_ns=%lld origin_y=%.6f nominal_floor_y=%.6f align_p=(%.6f,%.6f,%.6f) align_q=(%.6f,%.6f,%.6f,%.6f) raw_p=(%.6f,%.6f,%.6f) published_p=(%.6f,%.6f,%.6f) recoveries=%u resets=%u",
		            previous_state, g->state, (long long)at_timestamp_ns, origin_y,
		            no_pose_position.y - RIFT_S_SLAM_GUARD_EYE_HEIGHT_M,
		            g->align.position.x, g->align.position.y, g->align.position.z,
		            g->align.orientation.x, g->align.orientation.y, g->align.orientation.z, g->align.orientation.w,
		            slam_relation.pose.position.x, slam_relation.pose.position.y, slam_relation.pose.position.z,
		            imu_relation.pose.position.x, imu_relation.pose.position.y, imu_relation.pose.position.z,
		            g->recoveries, g->resets);
	}
	timepoint_ns pose_log_now = os_monotonic_get_ns();
	if (pose == RIFT_S_TRACKER_POSE_DEVICE &&
	    pose_log_now - t->last_published_pose_log_ns >= 5LL * U_TIME_1S_IN_NS) {
		t->last_published_pose_log_ns = pose_log_now;
		RIFT_S_INFO("HMD pose flags=0x%x paused=%d slam_valid=%d sample_age_ms=%.1f guard=%d "
		            "boundary_confirmed=%d position=(%.3f,%.3f,%.3f)",
		            imu_relation.relation_flags, hold, slam_valid,
		            sample_ts ? (pose_log_now - sample_ts) / 1e6 : -1.0, g->state,
		            g->boundary_confirmed, imu_relation.pose.position.x,
		            imu_relation.pose.position.y, imu_relation.pose.position.z);
	}
	os_mutex_unlock(&t->mutex);

	m_relation_chain_push_relation(&xrc, &imu_relation);
	m_relation_chain_resolve(&xrc, out_relation);
}

// Run feature matching off the camera/pose threads. A single latest-frame pair bounds backlog.
static float
world_rotation_difference(const struct xrt_quat *a, const struct xrt_quat *b)
{
	float dot = fabsf(a->x * b->x + a->y * b->y + a->z * b->z + a->w * b->w);
	return 2 * acosf(fminf(1, dot));
}

static void
world_write_status(struct rift_s_tracker *t)
{
	os_mutex_lock(&t->mutex);
	const char *state = t->world_storage_failed                                                 ? "save_failed"
	                    : t->world_setup                                                        ? "setup"
	                    : (t->slam_guard.boundary_confirmed && !t->slam_guard.world_transition) ? "localized"
	                    : t->world_map_available                                                ? "relocalizing"
	                                                                                            : "unanchored";
	os_mutex_unlock(&t->mutex);
	static timepoint_ns last_write_ns;
	static const char *last_state;
	timepoint_ns now_ns = os_monotonic_get_ns();
	if (state == last_state && now_ns - last_write_ns < 500 * U_TIME_1MS_IN_NS) return;
	last_write_ns = now_ns;
	last_state = state;
	char path[1024], temp[1050];
	rift_s_world_control_path("status", path, sizeof(path));
	snprintf(temp, sizeof(temp), "%s.tmp", path);
	FILE *file = fopen(temp, "w");
	if (!file) return;
	fprintf(file, "%lld %s\n", (long long)os_monotonic_get_ns(), state);
	if (fclose(file) == 0) rename(temp, path);
}

static void
world_read_command(struct rift_s_tracker *t)
{
	char path[1024], command[16] = {0};
	rift_s_world_control_path("command", path, sizeof(path));
	FILE *file = fopen(path, "r");
	if (!file)
		return;
	unsigned long long id = 0;
	int count = fscanf(file, "%llu %15s", &id, command);
	fclose(file);
	if (count != 2 || id <= t->world_command_id)
		return;
	t->world_command_id = id;
	if (strcmp(command, "begin") == 0) {
		rift_s_world_anchor_clear(t->world_anchor);
		os_mutex_lock(&t->mutex);
		t->world_setup = true;
		t->slam_guard.boundary_confirmed = false;
		t->slam_guard.require_relocalization = false;
		t->world_confirmations = 0;
		os_mutex_unlock(&t->mutex);
		RIFT_S_INFO("New room setup may establish a world; saved anchors retained until Accept");
	} else if (strcmp(command, "save") == 0) {
		os_mutex_lock(&t->mutex);
		bool setup = t->world_setup;
		os_mutex_unlock(&t->mutex);
		if (!setup) {
			RIFT_S_WARN("Ignoring world save without a room setup begin");
			return;
		}
		bool saved = rift_s_world_anchor_save(t->world_anchor);
		bool requires_relocalization =
		    rift_s_world_anchor_has_map(t->world_anchor) || rift_s_world_anchor_has_boundary(t->world_anchor);
		os_mutex_lock(&t->mutex);
		t->world_storage_failed = !saved;
		t->slam_guard.boundary_confirmed = saved && rift_s_world_anchor_has_map(t->world_anchor);
		t->world_setup = false;
		t->world_map_available = rift_s_world_anchor_has_map(t->world_anchor);
		t->slam_guard.require_relocalization = requires_relocalization;
		if (!saved || !t->world_map_available)
			t->slam_guard.boundary_confirmed = false;
		t->slam_guard.world_transition = false;
		os_mutex_unlock(&t->mutex);
		RIFT_S_INFO("Boundary world anchors saved=%d map_available=%d", saved, t->world_map_available);
	} else if (strcmp(command, "cancel") == 0) {
		rift_s_world_anchor_reload(t->world_anchor);
		bool requires_relocalization =
		    rift_s_world_anchor_has_map(t->world_anchor) || rift_s_world_anchor_has_boundary(t->world_anchor);
		os_mutex_lock(&t->mutex);
		if (t->world_setup) {
			t->slam_guard.world_valid = false;
			t->slam_guard.boundary_confirmed = false;
			t->slam_guard.world_transition = false;
			rift_s_slam_guard_suspend(&t->slam_guard, &t->pose.orientation);
		}
		t->world_setup = false;
		t->world_map_available = rift_s_world_anchor_has_map(t->world_anchor);
		t->slam_guard.require_relocalization = requires_relocalization;
		t->world_confirmations = 0;
		os_mutex_unlock(&t->mutex);
	}
}

static void *
rift_s_world_thread(void *ptr)
{
	struct rift_s_tracker *t = ptr;
	timepoint_ns last_attempt_log_ns = 0;
	uint64_t attempts = 0, usable = 0;
	os_thread_helper_lock(&t->world_thread);
	while (os_thread_helper_is_running_locked(&t->world_thread)) {
		struct xrt_frame *frames[RIFT_S_CAMERA_COUNT] = {0};
		for (int eye = 0; eye < RIFT_S_CAMERA_COUNT; eye++) {
			frames[eye] = t->world_frames[eye];
			t->world_frames[eye] = NULL;
		}
		os_thread_helper_unlock(&t->world_thread);
		world_read_command(t);
		if (frames[0] && frames[1] && t->tracking.slam_enabled) {
			timepoint_ns capture_ns = frames[0]->timestamp;
			struct xrt_space_relation source = XRT_SPACE_RELATION_ZERO, capture = XRT_SPACE_RELATION_ZERO;
			timepoint_ns source_ns = 0;
			os_mutex_lock(&t->mutex);
			uint64_t generation = t->slam_generation;
			os_mutex_unlock(&t->mutex);
			bool valid = false;
#ifdef XRT_FEATURE_SLAM
			// Wait briefly for this exposure's SLAM sample, rather than triangulate against a prediction.
			for (int attempt = 0; attempt < 12; attempt++) {
				valid = t_slam_get_latest_sample(t->tracking.slam, capture_ns, &source_ns, &source,
				                                 &capture);
				if (valid && source_ns >= capture_ns)
					break;
				os_nanosleep(10 * U_TIME_1MS_IN_NS);
			}
			rift_s_tracker_correct_pose_from_basalt(&capture.pose);
#endif
			os_mutex_lock(&t->mutex);
			struct xrt_quat fusion = t->pose.orientation;
			struct xrt_space_relation fusion_capture;
			if (m_relation_history_get(t->fusion.history, capture_ns, &fusion_capture) !=
			    M_RELATION_HISTORY_RESULT_INVALID)
				fusion = fusion_capture.pose.orientation;
			valid = valid && source_ns >= capture_ns && source_ns - capture_ns <= 150 * U_TIME_1MS_IN_NS &&
			        (capture.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) &&
			        capture_ns >= t->slam_min_sample_ns && generation == t->slam_generation &&
			        !t->standby && !t->slam_paused && !t->slam_restart_pending &&
			        rift_s_slam_guard_pose_usable(&capture.pose, &fusion);
			bool standby = t->standby;
			struct xrt_pose world_capture;
			struct xrt_vec3 no_pose_position = rift_s_slam_guard_no_pose_position(
			    t->base.tracking_origin ? t->base.tracking_origin->initial_offset.position.y : 0.0f);
			// Recording must pass the same displacement/gravity checks as a displayed pose.
			bool record = valid && (t->world_setup || t->slam_guard.boundary_confirmed) &&
			              !t->slam_guard.world_transition &&
			              rift_s_slam_guard_query(&t->slam_guard, &capture.pose, capture_ns, &fusion,
			                                      &no_pose_position, &world_capture);
			os_mutex_unlock(&t->mutex);
			rift_s_world_anchor_record_rig_frames(t->world_anchor, (const struct xrt_frame *const *)frames,
			                                      RIFT_S_CAMERA_COUNT, valid ? &capture.pose : NULL);
			struct rift_s_world_anchor_result result = {0};
			bool processed = (record || rift_s_world_anchor_has_map(t->world_anchor)) &&
			                 rift_s_world_anchor_process_rig_gravity(
			                     t->world_anchor, (const struct xrt_frame *const *)frames,
			                     RIFT_S_CAMERA_COUNT, record ? &world_capture : NULL,
			                     valid ? &capture.pose.orientation : NULL, &result);
			attempts++;
			usable += valid;
			if (capture_ns - last_attempt_log_ns >= U_TIME_1S_IN_NS) {
				last_attempt_log_ns = capture_ns;
				RIFT_S_INFO("World attempt=%" PRIu64 " usable=%" PRIu64
				            " pose_valid=%d sample_gap_ms=%.1f"
				            " standby=%d keyframes=%u features=%u/%u/%u/%u/%u matches=%u/%u/%u/%u/%u "
				            "inliers=%u/%u/%u/%u/%u"
				            " points_by_camera=%u/%u/%u/%u/%u support=%u stereo=%u triangulated=%u "
				            "error_px=%.3f record=%d matched=%d confirmations=%u rejection=%s",
				            attempts, usable, valid,
				            source_ns > 0 ? (double)(source_ns - capture_ns) / U_TIME_1MS_IN_NS : -1.0,
				            standby, result.keyframes, result.features[0], result.features[1],
				            result.features[2], result.features[3], result.features[4],
				            result.matches[0], result.matches[1], result.matches[2], result.matches[3],
				            result.matches[4], result.inliers[0], result.inliers[1], result.inliers[2],
				            result.inliers[3], result.inliers[4], result.camera_points[0],
				            result.camera_points[1], result.camera_points[2], result.camera_points[3],
				            result.camera_points[4], result.supporting_cameras, result.stereo_matches,
				            result.triangulated, result.max_reprojection_px, record, result.relocalized,
				            t->world_confirmations, result.rejection ? result.rejection : "none");
			}
			if (processed && result.recorded) {
				os_mutex_lock(&t->mutex);
				bool setup = t->world_setup;
				t->world_map_available = true;
				os_mutex_unlock(&t->mutex);
				if (!setup) {
					bool saved = rift_s_world_anchor_save(t->world_anchor);
					os_mutex_lock(&t->mutex);
					t->world_storage_failed = !saved;
					os_mutex_unlock(&t->mutex);
				}
				RIFT_S_INFO("World keyframe recorded count=%u", result.keyframes);
			}
			if (processed && result.relocalized && valid && !record) {
				struct xrt_pose inverse, candidate;
				math_pose_invert(&capture.pose, &inverse);
				math_pose_transform(&result.world_from_imu, &inverse, &candidate);
				struct xrt_vec3 up = {0, 1, 0}, world_up;
				math_quat_rotate_vec3(&candidate.orientation, &up, &world_up);
				os_mutex_lock(&t->mutex);
				if (generation == t->slam_generation && !t->standby && !t->slam_restart_pending &&
				    world_up.y > 0.9961947f) {
					rift_s_slam_guard_yaw_of(&candidate.orientation, &candidate.orientation);
					struct xrt_vec3 rotated;
					math_quat_rotate_vec3(&candidate.orientation, &capture.pose.position, &rotated);
					candidate.position = m_vec3_sub(result.world_from_imu.position, rotated);
					bool consistent =
					    t->world_confirmations > 0 && t->world_candidate_generation == generation &&
					    capture_ns > t->world_candidate_ns &&
					    capture_ns - t->world_candidate_ns <= 3LL * U_TIME_1S_IN_NS &&
					    m_vec3_len(m_vec3_sub(candidate.position, t->world_candidate.position)) <
					        0.04f &&
					    world_rotation_difference(&candidate.orientation,
					                              &t->world_candidate.orientation) < DEG_TO_RAD(3);
					t->world_confirmations = consistent ? t->world_confirmations + 1 : 1;
					t->world_candidate = candidate;
					t->world_candidate_generation = generation;
					t->world_candidate_ns = capture_ns;
					if (t->world_confirmations >= 3 && !t->slam_guard.boundary_confirmed) {
						rift_s_slam_guard_set_world(&t->slam_guard, &candidate);
						RIFT_S_INFO(
						    "World relocalized inliers=%u/%u error_px=%.3f "
						    "transform=(%.4f,%.4f,%.4f)",
						    result.inliers[0], result.inliers[1], result.max_reprojection_px,
						    candidate.position.x, candidate.position.y, candidate.position.z);
					}
				}
				os_mutex_unlock(&t->mutex);
			} else if (!record && (!valid || capture_ns - t->world_candidate_ns > 3LL * U_TIME_1S_IN_NS)) {
				os_mutex_lock(&t->mutex);
				t->world_confirmations = 0;
				os_mutex_unlock(&t->mutex);
			}
		}
		for (int eye = 0; eye < RIFT_S_CAMERA_COUNT; eye++)
			xrt_frame_reference(&frames[eye], NULL);
		world_write_status(t);
		os_nanosleep(20 * U_TIME_1MS_IN_NS);
		os_thread_helper_lock(&t->world_thread);
	}
	os_thread_helper_unlock(&t->world_thread);
	return NULL;
}

void
rift_s_tracker_start(struct rift_s_tracker *t)
{
	os_mutex_lock(&t->mutex);
	t->ready_for_data = true;
	os_mutex_unlock(&t->mutex);
}

void
rift_s_tracker_imu_finish(struct rift_s_tracker *t)
{
	os_mutex_lock(&t->mutex);
	/* Clearing the hw2mono clock disabled sending camera frames, preventing deadlocks on shutdown
	 * if the IMU data stops before camera frames do */
	t->have_hw2mono = false;
	t->valid_clock_observations = 0;
	os_mutex_unlock(&t->mutex);
}
