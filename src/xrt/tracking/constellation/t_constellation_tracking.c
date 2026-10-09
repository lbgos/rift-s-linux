// Copyright 2023 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Implementation of LED constellation tracking
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup constellation
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#ifdef __linux__
#include <sys/prctl.h>
#endif

#include "os/os_threading.h"

#include "tracking/t_led_models.h"
#include "tracking/t_constellation_tracking.h"

#include "util/u_debug.h"
#include "util/u_frame.h"
#include "util/u_logging.h"
#include "util/u_sink.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"

#include "internal/blobwatch.h"
#include "internal/camera_model.h"
#include "internal/correspondence_search.h"
#include "internal/constrained_pose.h"
#include "internal/debug_draw.h"
#include "internal/ransac_pnp.h"
#include "internal/sample.h"

DEBUG_GET_ONCE_LOG_OPTION(ct_log, "CONSTELLATION_LOG", U_LOGGING_INFO)
DEBUG_GET_ONCE_FLOAT_OPTION(ct_score_gate, "CONSTELLATION_SCORE_GATE_M", 0.004)
DEBUG_GET_ONCE_FLOAT_OPTION(ct_association_gate, "CONSTELLATION_ASSOCIATION_GATE_M", 0.15)
DEBUG_GET_ONCE_FLOAT_OPTION(ct_ray_noise, "CONSTELLATION_RAY_NOISE_M", 0.001)
DEBUG_GET_ONCE_FLOAT_OPTION(ct_max_depth, "CONSTELLATION_MAX_DEPTH_M", 1.75)
DEBUG_GET_ONCE_FLOAT_OPTION(ct_bearing_noise, "CONSTELLATION_BEARING_NOISE_RAD", 0.0026)
DEBUG_GET_ONCE_NUM_OPTION(ct_cold_inliers, "CONSTELLATION_COLD_MIN_INLIERS", 4)
// Standard deviation of the bounded IMU tilt adjustment in degrees; 0 keeps the IMU tilt.
DEBUG_GET_ONCE_FLOAT_OPTION(ct_tilt_prior, "CONSTELLATION_TILT_PRIOR_DEG", 3.0)

#define MIN_ROT_ERROR DEG_TO_RAD(30)
#define MIN_POS_ERROR 0.10

#define CT_TRACE(c, ...) U_LOG_IFL_T(c->log_level, __VA_ARGS__)
#define CT_DEBUG(c, ...) U_LOG_IFL_D(c->log_level, __VA_ARGS__)
#define CT_INFO(c, ...) U_LOG_IFL_I(c->log_level, __VA_ARGS__)
#define CT_WARN(c, ...) U_LOG_IFL_W(c->log_level, __VA_ARGS__)
#define CT_ERROR(c, ...) U_LOG_IFL_E(c->log_level, __VA_ARGS__)

/* Maximum number of frames to permit waiting in the fast-processing queue */
#define MAX_FAST_QUEUE_SIZE 2

//! When projecting poses into the camera, we need
// extra flip around the X axis because OpenCV camera space coordinates have
// +Y down and +Z away from the user
static void
pose_flip_YZ(const struct xrt_pose *in, struct xrt_pose *dest)
{
	const struct xrt_pose P_YZ_flip = {
	    {1.0, 0.0, 0.0, 0.0},
	    {0.0, 0.0, 0.0},
	};

	struct xrt_pose tmp;
	math_pose_transform(&P_YZ_flip, in, &tmp);
	math_pose_transform(&tmp, &P_YZ_flip, dest);
}

struct t_constellation_tracked_device_connection
{
	/* Device and tracker each hold a reference to the connection.
	 * It's only cleaned up once both release it. */
	struct xrt_reference ref;

	/* Index in the devices array for this device */
	int id;

	/* Protect access when around API calls and disconnects */
	struct os_mutex lock;
	bool disconnected; /* Set to true once disconnect() is called */

	// Callbacks to the tracked device
	struct xrt_device *xdev;
	struct t_constellation_tracked_device_callbacks *cb;

	struct t_constellation_tracker *tracker; //! Parent tracker instance
};

enum ct_stage
{
	CT_STAGE_FAST,
	CT_STAGE_COLD,
	CT_STAGE_P3P,
	CT_STAGE_COUNT,
};

#define CT_REJECTION_COUNT (CONSTELLATION_CONSTRAINED_INCONSISTENT + 1)
#define CT_ORIGIN_COUNT (CONSTELLATION_CONSTRAINED_P3P + 1)

//! Per-device acquisition counters, reported periodically at DEBUG.
struct constellation_device_diagnostics
{
	uint64_t exposures;
	//! No capture-time attitude from the device: the prior is unavailable.
	uint64_t no_attitude;
	//! No blobs in any camera at all.
	uint64_t no_blobs;
	//! Exposures where the position prior was not trusted, so the fast path could not run.
	uint64_t untrusted_prior;
	uint64_t attempts[CT_STAGE_COUNT];
	uint64_t accepted[CT_STAGE_COUNT];
	uint64_t rejected[CT_STAGE_COUNT][CT_REJECTION_COUNT];
	uint64_t accepted_origin[CT_ORIGIN_COUNT];
	uint64_t published;
	uint64_t vision_yaw;
	uint64_t shared_blob;
	//! shared_blob split by the kind of claim that won the blob.
	uint64_t shared_by[CONSTELLATION_CLAIM_REJECTED + 1];
	uint64_t stale;
	//! Verified results not published because they did not continue the track, by origin.
	uint64_t jumps[CT_ORIGIN_COUNT];
	//! Results held for confirmation, and holds replaced by an inconsistent next result.
	uint64_t confirming;
	uint64_t unconfirmed;
	//! Tracks confirmed, and tracks ended by a timeout or repeated jumps.
	uint64_t confirmed;
	uint64_t lost;
	//! No IMU attitude for the continuity check.
	uint64_t track_no_attitude;
	//! Tracks ended because another device's clearly stronger candidate claimed their blobs.
	uint64_t contested;
	//! Best hypothesis inliers per attempt, accepted or not: 0, 1-3, 4-7, 8+.
	uint64_t best_inliers[4];
	//! Published poses each camera contributed inliers to.
	uint64_t published_by_camera[XRT_TRACKING_MAX_SLAM_CAMS];
	uint64_t last_pose_log_ns;
	uint64_t last_shared_log_ns;
	uint64_t last_reject_log_ns;
	uint64_t last_summary_ns;
	const char *last_failure;
	uint64_t last_failure_capture_ns;
	uint64_t last_failure_processing_ns;
	uint64_t published_processing_ns;
	uint64_t missing_model;
	uint64_t publication_timeouts;

	//! Head-relative position of the last published pose, reported when its track ends.
	struct xrt_vec3 last_published_head;
	bool have_last_published_head;
};

struct constellation_tracker_device
{
	struct t_constellation_tracked_device_connection *connection;
	//! Stable private alias for logs ("left", "right"); never a serial number.
	const char *alias;
	char name[64];
	struct constellation_device_diagnostics diagnostics;

	bool have_led_model;
	struct t_constellation_led_model led_model;
	struct t_constellation_search_model *search_led_model;

	bool have_last_seen_pose;
	uint64_t last_seen_pose_ts;
	struct xrt_pose last_seen_pose; // global pose
	int last_matched_blobs;
	int last_matched_cam;
	struct xrt_pose last_matched_cam_pose; // Camera-relative pose

	//! Publication state, world in OpenCV coordinates. Protected by tracked_device_lock.
	struct constellation_track track;
	uint64_t jumps_since_log;
	uint64_t last_jump_log_ns;
};

struct constellation_tracker_camera_state
{
	//! Distortion params
	struct camera_model camera_model;
	//! ROI in the full frame mosaic
	struct xrt_rect roi;
	//! Camera's pose relative to the HMD GENERIC_TRACKER_POSE (IMU)
	struct xrt_pose P_imu_cam;

	//! Constellation tracking - fast tracking thread
	struct os_mutex bw_lock; /* Protects blobwatch process vs release from long thread */
	blobwatch *bw;
	int last_num_blobs;

	//! Full search / pose recovery thread
	struct correspondence_search *cs;

	//! Debug output
	struct u_sink_debug debug_sink;
	struct xrt_pose debug_last_pose;
	struct xrt_vec3 debug_last_gravity_vector;

	//! The index into the slam tracking camera array this camera represents
	size_t slam_tracking_index;
};

/*!
 * An @ref xrt_frame_sink that analyses video frame groups for LED constellation tracking
 * @implements xrt_frame_sink
 * @implements xrt_frame_node
 */
struct t_constellation_tracker
{
	//! Receive (mosaic) frames from the camera
	struct xrt_frame_sink base;
	//! frame node to insert in the xfctx
	struct xrt_frame_node node;

	/*! HMD device we get observation base poses from
	 * and that owns the xfctx keeping this node alive */
	struct xrt_device *hmd_xdev;

	struct os_mutex tracked_device_lock;

	//! Tracked device communication connections
	int num_devices;
	struct constellation_tracker_device devices[CONSTELLATION_MAX_DEVICES];

	//!< Tracking camera entries
	struct constellation_tracker_camera_state cam[XRT_TRACKING_MAX_SLAM_CAMS];
	int cam_count;

	/* Debug */
	enum u_logging_level log_level;
	bool fast_thread_named;
	uint64_t long_samples_replaced;
	uint64_t last_queue_log_ns;
	bool debug_draw_normalise;
	bool debug_draw_blob_tint;
	bool debug_draw_blob_circles;
	bool debug_draw_blob_ids;
	bool debug_draw_blob_unique_ids;
	bool debug_draw_leds;
	bool debug_draw_prior_leds;
	bool debug_draw_last_leds;
	bool debug_draw_pose_bounds;
	bool debug_draw_device_bounds;

	uint64_t last_frame_timestamp;

	uint64_t last_fast_analysis_ms;
	uint64_t last_blob_analysis_ms;
	uint64_t last_long_analysis_ms;

	//! Per-camera blob statistics for the periodic DEBUG summary, reset after each report.
	uint64_t camera_exposures[XRT_TRACKING_MAX_SLAM_CAMS];
	uint64_t camera_blobs[XRT_TRACKING_MAX_SLAM_CAMS];
	uint64_t camera_dark_blobs[XRT_TRACKING_MAX_SLAM_CAMS];
	uint64_t camera_blob_area[XRT_TRACKING_MAX_SLAM_CAMS];
	uint64_t camera_blob_brightness[XRT_TRACKING_MAX_SLAM_CAMS];
	uint64_t last_camera_summary_ns;

	struct u_var_button full_search_button;
	bool do_full_search;

	// Fast tracking thread
	struct xrt_frame_sink *fast_q_sink;
	struct xrt_frame_sink fast_process_sink;

	// Long analysis / recovery thread
	struct os_thread_helper long_analysis_thread;
	struct constellation_tracking_sample *long_analysis_pending_sample;

	struct xrt_device_masks_sample controller_masks_sample;
	struct xrt_device_masks_sink *controller_masks_sink;
};

static void
constellation_tracked_device_connection_notify_frame(struct t_constellation_tracked_device_connection *ctdc,
                                                     uint64_t frame_mono_ns,
                                                     uint64_t frame_sequence)
{
	os_mutex_lock(&ctdc->lock);
	if (!ctdc->disconnected && ctdc->cb->notify_frame_received) {
		ctdc->cb->notify_frame_received(ctdc->xdev, frame_mono_ns, frame_sequence);
	}
	os_mutex_unlock(&ctdc->lock);
}

static void
constellation_tracked_device_connection_notify_brightness_update(struct t_constellation_tracked_device_connection *ctdc,
                                                                 uint8_t average_brightness)
{
	os_mutex_lock(&ctdc->lock);
	if (!ctdc->disconnected && ctdc->cb->push_brightness_update) {
		ctdc->cb->push_brightness_update(ctdc->xdev, average_brightness);
	}
	os_mutex_unlock(&ctdc->lock);
}

static bool
constellation_tracked_device_connection_get_led_model(struct t_constellation_tracked_device_connection *ctdc,
                                                      struct t_constellation_led_model *led_model)
{
	bool ret = false;

	os_mutex_lock(&ctdc->lock);
	if (!ctdc->disconnected && ctdc->cb->get_led_model) {
		ret = ctdc->cb->get_led_model(ctdc->xdev, led_model);
	}
	os_mutex_unlock(&ctdc->lock);

	return ret;
}

static bool
constellation_tracked_device_connection_get_tracked_pose(struct t_constellation_tracked_device_connection *ctdc,
                                                         uint64_t timestamp_ns,
                                                         struct xrt_space_relation *xsr)
{
	bool ret = false;

	os_mutex_lock(&ctdc->lock);
	if (!ctdc->disconnected) {
		struct xrt_device *xdev = ctdc->xdev;
		xrt_result_t result = xrt_device_get_tracked_pose(xdev, XRT_INPUT_GENERIC_TRACKER_POSE, timestamp_ns, xsr);
		ret = result == XRT_SUCCESS &&
		      (xsr->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT);
	}
	os_mutex_unlock(&ctdc->lock);

	return ret;
}

static void
constellation_tracker_receive_frame(struct xrt_frame_sink *sink, struct xrt_frame *xf)
{
	struct t_constellation_tracker *ct = container_of(sink, struct t_constellation_tracker, base);

	assert(xf->format == XRT_FORMAT_L8);

	// Tell the controllers about the frame so they can their timesync estimate
	os_mutex_lock(&ct->tracked_device_lock);
	for (int i = 0; i < ct->num_devices; i++) {
		constellation_tracked_device_connection_notify_frame(ct->devices[i].connection, xf->timestamp,
		                                                     xf->source_sequence);
	}
	os_mutex_unlock(&ct->tracked_device_lock);

	ct->last_frame_timestamp = xf->timestamp;
	xrt_sink_push_frame(ct->fast_q_sink, xf);
}

static void
constellation_tracker_node_break_apart(struct xrt_frame_node *node)
{
	DRV_TRACE_MARKER();
}

static bool
constellation_tracked_device_connection_has_heading(struct t_constellation_tracked_device_connection *ctdc,
                                                    timepoint_ns capture_ns)
{
	bool ret = false;
	os_mutex_lock(&ctdc->lock);
	if (!ctdc->disconnected && ctdc->cb->has_heading_prior) {
		ret = ctdc->cb->has_heading_prior(ctdc->xdev, capture_ns);
	}
	os_mutex_unlock(&ctdc->lock);
	return ret;
}

static void
constellation_tracked_device_connection_notify_observation(struct t_constellation_tracked_device_connection *ctdc,
                                                           const struct t_constellation_pose_observation *obs)
{
	os_mutex_lock(&ctdc->lock);
	if (!ctdc->disconnected) {
		if (ctdc->cb->push_pose_observation) {
			ctdc->cb->push_pose_observation(ctdc->xdev, obs);
		} else if (ctdc->cb->push_observed_pose) {
			ctdc->cb->push_observed_pose(ctdc->xdev, obs->capture_ns, &obs->pose);
		}
	}
	os_mutex_unlock(&ctdc->lock);
}

static const char *ct_stage_names[CT_STAGE_COUNT] = {"fast", "constrained", "p3p"};

static unsigned
joint_views(struct t_constellation_tracker *ct,
            struct constellation_tracking_sample *sample,
            struct constellation_constrained_view views[CONSTELLATION_MAX_CAMERAS])
{
	unsigned n = 0;
	for (unsigned v = 0; v < sample->n_views; v++) {
		struct tracking_sample_frame *view = &sample->views[v];
		if (!view->bwobs) {
			continue;
		}
		views[n++] = (struct constellation_constrained_view){
		    .camera_index = v,
		    .P_world_cam = view->P_world_cam,
		    .P_cam_world = view->P_cam_world,
		    .calib = &ct->cam[v].camera_model,
		    .observation = view->bwobs,
		};
	}
	return n;
}

static unsigned
sample_blob_count(struct constellation_tracking_sample *sample)
{
	unsigned blobs = 0;
	for (unsigned v = 0; v < sample->n_views; v++) {
		if (sample->views[v].bwobs) {
			blobs += sample->views[v].bwobs->num_blobs;
		}
	}
	return blobs;
}

/* Device position of an LED model pose (OpenCV world) relative to the head at the exposure. */
static bool
head_relative(const struct constellation_tracking_sample *sample,
              const struct constellation_tracker_device *device,
              const struct xrt_pose *P_world_model,
              struct xrt_vec3 *out)
{
	if (!sample->have_hmd_pose) {
		return false;
	}
	struct xrt_pose P_xrworld_model, P_xrworld_device;
	pose_flip_YZ(P_world_model, &P_xrworld_model);
	math_pose_transform(&P_xrworld_model, &device->led_model.P_model_device, &P_xrworld_device);
	constellation_head_relative_position(&sample->P_xrworld_hmd, &P_xrworld_device.position, out);
	return true;
}

static void
format_camera_inliers(const struct t_constellation_tracker *ct,
                      const struct constellation_constrained_result *r,
                      char *buf,
                      size_t size)
{
	unsigned camera_inliers[XRT_TRACKING_MAX_SLAM_CAMS] = {0};
	for (unsigned i = 0; i < r->num_assignments; i++) {
		if (r->assignments[i].camera_index < XRT_TRACKING_MAX_SLAM_CAMS) {
			camera_inliers[r->assignments[i].camera_index]++;
		}
	}
	size_t used = 0;
	buf[0] = '\0';
	for (int c = 0; c < ct->cam_count && used < size; c++) {
		used += snprintf(buf + used, size - used, "%s%u", c ? "," : "", camera_inliers[c]);
	}
}

/* Once a second per device at DEBUG, the best rejected hypothesis and where it was, so a worn log
 * shows where the hand was when it could not be published. */
static void
log_rejected_hypothesis(struct t_constellation_tracker *ct,
                        struct constellation_tracking_sample *sample,
                        struct tracking_sample_device_state *state,
                        const char *stage,
                        const char *rejection)
{
	struct constellation_tracker_device *device = &ct->devices[state->dev_index];
	struct constellation_constrained_result *r = &state->joint_result;
	if (ct->log_level > U_LOGGING_DEBUG || r->num_assignments == 0 || strcmp(rejection, "none") == 0) {
		return;
	}
	uint64_t now = os_monotonic_get_ns();
	os_mutex_lock(&ct->tracked_device_lock);
	bool due = now - device->diagnostics.last_reject_log_ns >= U_TIME_1S_IN_NS;
	if (due) {
		device->diagnostics.last_reject_log_ns = now;
	}
	os_mutex_unlock(&ct->tracked_device_lock);
	if (!due) {
		return;
	}
	struct xrt_vec3 head = {NAN, NAN, NAN};
	head_relative(sample, device, &r->P_world_model, &head);
	char per_camera[64];
	format_camera_inliers(ct, r, per_camera, sizeof(per_camera));
	CT_DEBUG(ct,
	         "Tracking reject device=%s capture=%" PRIu64
	         " stage=%s reason=%s hypothesis=%s head=%.3f,%.3f,%.3f inliers=%u distinct=%u"
	         " camera_inliers=%s unexplained=%u fit_chi2=%.1f/%.1f tilt_correction_deg=%.1f",
	         device->alias, sample->timestamp, stage, rejection, constellation_constrained_origin_name(r->origin),
	         head.x, head.y, head.z, r->inliers, r->distinct_leds, per_camera, r->unexplained_blobs,
	         r->fit_chi_square, r->fit_chi_square_limit, RAD_TO_DEG(r->tilt_correction_rad));
}

/* Per-exposure detail at TRACE: one line per attempt and one per camera. */
static void
log_joint_result(struct t_constellation_tracker *ct,
                 struct constellation_tracking_sample *sample,
                 struct tracking_sample_device_state *state,
                 const char *stage,
                 const char *rejection)
{
	log_rejected_hypothesis(ct, sample, state, stage, rejection);
	if (ct->log_level > U_LOGGING_TRACE) {
		return;
	}
	struct constellation_tracker_device *device = &ct->devices[state->dev_index];
	struct constellation_constrained_result *r = &state->joint_result;
	CT_TRACE(ct,
	         "Tracking device=%s capture=%" PRIu64 " process=%" PRIu64
	         " kind=controller stage=%s hypothesis=%s trusted_position=%d heading=%d candidates=%u unique=%u"
	         " inliers=%u visible=%u unexplained=%u cost_m=%.6f second_m=%.6f ambiguity_chi2=%.3f"
	         " fit_chi2=%.2f/%.2f rank=%u vision_yaw=%d pos_var_m2=%g,%g,%g yaw_var_rad2=%g pair_trials=%u"
	         " hypotheses=%u gravity=%u heading=%u reject_degenerate=%u reject_unresolved_yaw=%u"
	         " reject_no_root=%u reject_depth=%u reject_seed=%u reject_count=%u reject_visibility=%u"
	         " reject_range=%u reject_prior_distance=%u reject_prior_rotation=%u budget_exhausted=%d reject=%s",
	         device->alias, sample->timestamp, os_monotonic_get_ns(), stage,
	         constellation_constrained_origin_name(r->origin), state->trusted_position, state->reliable_heading,
	         r->candidate_observations, r->distinct_leds, r->inliers, r->projected_visible_leds,
	         r->unexplained_blobs, r->cost_m, r->second_best_cost_m, r->ambiguity_likelihood_gap, r->fit_chi_square,
	         r->fit_chi_square_limit, r->information_rank, r->vision_determines_yaw, r->covariance[0],
	         r->covariance[5], r->covariance[10], r->covariance[15], r->pair_trials, r->hypotheses,
	         r->gravity_hypotheses, r->heading_hypotheses, r->degenerate_pairs, r->unresolved_yaw_pairs,
	         r->no_root_pairs, r->depth_pairs, r->seed_rejections, r->insufficient_hypotheses,
	         r->invisible_rejections, r->depth_rejections, r->prior_distance_rejections, r->prior_rotation_rejections, r->budget_exhausted,
	         rejection);
	CT_TRACE(ct, "Tracking candidate device=%s capture=%" PRIu64 " stage=%s position=%.5f,%.5f,%.5f"
	             " orientation=%.5f,%.5f,%.5f,%.5f prior_orientation=%.5f,%.5f,%.5f,%.5f",
	         device->alias, sample->timestamp, stage, r->P_world_model.position.x, r->P_world_model.position.y,
	         r->P_world_model.position.z, r->P_world_model.orientation.x, r->P_world_model.orientation.y,
	         r->P_world_model.orientation.z, r->P_world_model.orientation.w, state->P_world_obj_prior.orientation.x,
	         state->P_world_obj_prior.orientation.y, state->P_world_obj_prior.orientation.z,
	         state->P_world_obj_prior.orientation.w);
	for (unsigned i = 0; i < r->num_assignments; i++) {
		const struct constellation_constrained_assignment *a = &r->assignments[i];
		if (a->camera_index >= sample->n_views || !sample->views[a->camera_index].bwobs ||
		    a->blob_index >= (unsigned)sample->views[a->camera_index].bwobs->num_blobs) {
			continue;
		}
		const struct blob *b = &sample->views[a->camera_index].bwobs->blobs[a->blob_index];
		CT_TRACE(ct, "Tracking assignment device=%s capture=%" PRIu64 " stage=%s camera=%u blob=%u"
		             " led=%u temporal_label=%u pixel=%.3f,%.3f residual_m=%.6f",
		         device->alias, sample->timestamp, stage, a->camera_index, a->blob_index, a->led_id, b->led_id,
		         b->x, b->y, a->residual_m);
	}
	for (unsigned v = 0; v < sample->n_views; v++) {
		struct blobservation *obs = sample->views[v].bwobs;
		unsigned inliers = 0, distinct = 0, eligible = 0, visible = 0, observations = 0;
		for (unsigned k = 0; k < r->num_camera_diagnostics; k++) {
			if (r->camera_diagnostics[k].camera_index == v) {
				eligible = r->camera_diagnostics[k].eligible_blobs;
				visible = r->camera_diagnostics[k].projected_visible_leds;
				observations = r->camera_diagnostics[k].candidate_observations;
			}
		}
		bool seen[MAX_OBJECT_LEDS] = {0};
		for (unsigned i = 0; i < r->num_assignments; i++) {
			struct constellation_constrained_assignment *a = &r->assignments[i];
			if (a->camera_index == v) {
				inliers++;
				if (a->led_index < MAX_OBJECT_LEDS && !seen[a->led_index]) {
					seen[a->led_index] = true;
					distinct++;
				}
			}
		}
		CT_TRACE(ct,
		         "Tracking device=%s capture=%" PRIu64 " kind=controller stage=%s camera=%u detected=%d"
		         " eligible=%u dark_rejected=%d visible=%u candidates=%u unique=%u inliers=%u",
		         device->alias, sample->timestamp, stage, v, obs ? obs->num_blobs : 0, eligible,
		         obs ? obs->dropped_dark_blobs : 0, visible, observations, distinct, inliers);
	}
}

/* Periodic per-device summary at DEBUG, so a worn session separates missing observations,
 * solver rejection, blob arbitration and driver-side rejection for each hand. Caller holds
 * tracked_device_lock. */
static void
log_device_summary_locked(struct t_constellation_tracker *ct, struct constellation_tracker_device *device, bool force)
{
	struct constellation_device_diagnostics *d = &device->diagnostics;
	uint64_t now = os_monotonic_get_ns();
	if (!force && (ct->log_level > U_LOGGING_INFO || now - d->last_summary_ns < 5 * (uint64_t)U_TIME_1S_IN_NS)) {
		return;
	}
	d->last_summary_ns = now;
	CT_INFO(ct, "Tracking health device=%s model_missing=%" PRIu64 " publication_timeouts=%" PRIu64
	            " last_failure=%s capture_ns=%" PRIu64 " processing_ns=%" PRIu64
	            " latency_ms=%.1f since_publication_ms=%.1f",
	        device->alias, d->missing_model, d->publication_timeouts,
	        d->last_failure ? d->last_failure : "none", d->last_failure_capture_ns,
	        d->last_failure_processing_ns,
	        d->last_failure_capture_ns ? (double)(d->last_failure_processing_ns - d->last_failure_capture_ns) / U_TIME_1MS_IN_NS : 0,
	        d->published_processing_ns ? (double)(now - d->published_processing_ns) / U_TIME_1MS_IN_NS : -1);
	char reasons[512] = "";
	size_t used = 0;
	for (unsigned stage = 0; stage < CT_STAGE_COUNT; stage++) {
		for (unsigned r = 1; r < CT_REJECTION_COUNT; r++) {
			if (d->rejected[stage][r] && used < sizeof(reasons)) {
				used += snprintf(reasons + used, sizeof(reasons) - used, " %s.%s=%" PRIu64,
				                 ct_stage_names[stage],
				                 constellation_constrained_rejection_name(
				                     (enum constellation_constrained_rejection)r),
				                 d->rejected[stage][r]);
			}
		}
	}
	char cameras[128] = "";
	size_t written = 0;
	for (int c = 0; c < ct->cam_count && written < sizeof(cameras); c++) {
		written += snprintf(cameras + written, sizeof(cameras) - written, "%s%" PRIu64, c ? "," : "",
		                    d->published_by_camera[c]);
	}
	CT_INFO(ct,
	         "Tracking summary device=%s exposures=%" PRIu64 " no_attitude=%" PRIu64 " no_blobs=%" PRIu64
	         " untrusted_prior=%" PRIu64 " fast=%" PRIu64 "/%" PRIu64 " constrained=%" PRIu64 "/%" PRIu64
	         " p3p=%" PRIu64 "/%" PRIu64 " published=%" PRIu64 " vision_yaw=%" PRIu64 " origin_gravity=%" PRIu64
	         " origin_heading=%" PRIu64 " origin_prior=%" PRIu64 " origin_p3p=%" PRIu64 " shared_blob=%" PRIu64
	         " shared_published=%" PRIu64 " shared_candidate=%" PRIu64 " shared_rejected=%" PRIu64 " stale=%" PRIu64
	         " track=%s confirming=%" PRIu64 " unconfirmed=%" PRIu64 " confirmed=%" PRIu64 " lost=%" PRIu64
	         " jump_gravity=%" PRIu64 " jump_heading=%" PRIu64 " jump_prior=%" PRIu64 " jump_p3p=%" PRIu64
	         " track_no_attitude=%" PRIu64 " contested=%" PRIu64 " best_inliers=%" PRIu64 "/%" PRIu64 "/%" PRIu64
	         "/%" PRIu64 " published_by_camera=%s rejects:%s",
	         device->alias, d->exposures, d->no_attitude, d->no_blobs, d->untrusted_prior,
	         d->accepted[CT_STAGE_FAST], d->attempts[CT_STAGE_FAST], d->accepted[CT_STAGE_COLD],
	         d->attempts[CT_STAGE_COLD], d->accepted[CT_STAGE_P3P], d->attempts[CT_STAGE_P3P], d->published,
	         d->vision_yaw, d->accepted_origin[CONSTELLATION_CONSTRAINED_GRAVITY],
	         d->accepted_origin[CONSTELLATION_CONSTRAINED_HEADING],
	         d->accepted_origin[CONSTELLATION_CONSTRAINED_PRIOR], d->accepted_origin[CONSTELLATION_CONSTRAINED_P3P],
	         d->shared_blob, d->shared_by[CONSTELLATION_CLAIM_PUBLISHED],
	         d->shared_by[CONSTELLATION_CLAIM_CANDIDATE], d->shared_by[CONSTELLATION_CLAIM_REJECTED], d->stale,
	         constellation_track_state_name(device->track.state), d->confirming, d->unconfirmed, d->confirmed,
	         d->lost, d->jumps[CONSTELLATION_CONSTRAINED_GRAVITY], d->jumps[CONSTELLATION_CONSTRAINED_HEADING],
	         d->jumps[CONSTELLATION_CONSTRAINED_PRIOR], d->jumps[CONSTELLATION_CONSTRAINED_P3P],
	         d->track_no_attitude, d->contested, d->best_inliers[0], d->best_inliers[1], d->best_inliers[2],
	         d->best_inliers[3], cameras, used ? reasons : " none");
}

static void
count_attempt(struct t_constellation_tracker *ct,
              struct tracking_sample_device_state *state,
              enum ct_stage stage,
              uint64_t capture_ns,
              bool accepted,
              enum constellation_constrained_rejection rejection)
{
	os_mutex_lock(&ct->tracked_device_lock);
	struct constellation_device_diagnostics *d = &ct->devices[state->dev_index].diagnostics;
	d->attempts[stage]++;
	unsigned inliers = state->joint_result.inliers;
	d->best_inliers[inliers == 0 ? 0 : inliers < 4 ? 1 : inliers < 8 ? 2 : 3]++;
	if (accepted) {
		d->accepted[stage]++;
	} else if ((unsigned)rejection < CT_REJECTION_COUNT) {
		d->rejected[stage][rejection]++;
		if (capture_ns >= d->last_failure_capture_ns) {
			d->last_failure = constellation_constrained_rejection_name(rejection);
			d->last_failure_capture_ns = capture_ns;
			d->last_failure_processing_ns = os_monotonic_get_ns();
		}
		log_device_summary_locked(ct, &ct->devices[state->dev_index], false);
	}
	os_mutex_unlock(&ct->tracked_device_lock);
}

/* Caller holds tracked_device_lock. */
static void
log_track_transition_locked(struct t_constellation_tracker *ct,
                            struct constellation_tracker_device *device,
                            enum constellation_track_state before,
                            uint64_t capture_ns,
                            const char *reason)
{
	if (before == device->track.state) {
		return;
	}
	const struct constellation_device_diagnostics *d = &device->diagnostics;
	if (before == CONSTELLATION_TRACK_TRACKING) {
		CT_INFO(ct, "Tracking drop device=%s reason=%s last_failure=%s failure_capture_ns=%" PRIu64
		            " failure_processing_ns=%" PRIu64 " publication_age_ms=%.1f no_attitude=%" PRIu64
		            " no_blobs=%" PRIu64 " track_no_attitude=%" PRIu64
		            " jumps=%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 " shared=%" PRIu64,
		        device->alias, reason, d->last_failure ? d->last_failure : "none", d->last_failure_capture_ns,
		        d->last_failure_processing_ns, (double)(capture_ns - device->last_seen_pose_ts) / U_TIME_1MS_IN_NS,
		        d->no_attitude, d->no_blobs, d->track_no_attitude, d->jumps[0], d->jumps[1],
		        d->jumps[2], d->jumps[3], d->shared_blob);
		log_device_summary_locked(ct, device, true);
	}
	if (before == CONSTELLATION_TRACK_TRACKING && d->have_last_published_head) {
		// Where the hand was last published: the edge of the tracked region when it ends here.
		CT_DEBUG(ct,
		         "Tracking device=%s capture=%" PRIu64
		         " track_state=%s->%s reason=%s last_published_head=%.3f,%.3f,%.3f",
		         device->alias, capture_ns, constellation_track_state_name(before),
		         constellation_track_state_name(device->track.state), reason, d->last_published_head.x,
		         d->last_published_head.y, d->last_published_head.z);
	} else {
		CT_DEBUG(ct, "Tracking device=%s capture=%" PRIu64 " track_state=%s->%s reason=%s", device->alias,
		         capture_ns, constellation_track_state_name(before),
		         constellation_track_state_name(device->track.state), reason);
	}
	device->diagnostics.confirmed += device->track.state == CONSTELLATION_TRACK_TRACKING;
	device->diagnostics.lost += before == CONSTELLATION_TRACK_TRACKING;
}

/* Caller holds tracked_device_lock. */
static void
expire_track_locked(struct t_constellation_tracker *ct,
                    struct constellation_tracker_device *device,
                    uint64_t capture_ns)
{
	enum constellation_track_state before = device->track.state;
	if (constellation_track_expire(&device->track, (int64_t)capture_ns)) {
		device->diagnostics.publication_timeouts++;
		log_track_transition_locked(ct, device, before, capture_ns, "timeout");
	}
}

/* IMU attitude of the LED model in the OpenCV world at a capture time, from the device's current
 * (yaw-corrected) attitude history. */
static bool
device_model_attitude(struct constellation_tracker_device *device, uint64_t capture_ns, struct xrt_quat *out)
{
	struct xrt_space_relation xsr;
	if (!constellation_tracked_device_connection_get_tracked_pose(device->connection, capture_ns, &xsr)) {
		return false;
	}
	struct xrt_pose P_xrworld_model, P_world_model;
	math_pose_transform(&xsr.pose, &device->led_model.P_device_model, &P_xrworld_model);
	pose_flip_YZ(&P_xrworld_model, &P_world_model);
	*out = P_world_model.orientation;
	return true;
}

/* Decide whether a verified result may be published. Returns NULL to publish, otherwise the
 * reason it is held back. Caller holds tracked_device_lock. */
static const char *
track_gate_locked(struct t_constellation_tracker *ct,
                  struct constellation_tracker_device *device,
                  const struct constellation_constrained_result *r,
                  uint64_t capture_ns)
{
	struct constellation_device_diagnostics *d = &device->diagnostics;
	struct constellation_track_point next = {r->P_world_model, {0, 0, 0, 1}, (int64_t)capture_ns};
	if (!device_model_attitude(device, capture_ns, &next.imu_world)) {
		d->track_no_attitude++;
		return "track_no_attitude";
	}
	struct xrt_quat ref_imu;
	bool have_ref_imu = device->track.state != CONSTELLATION_TRACK_LOST &&
	                    device_model_attitude(device, (uint64_t)device->track.ref.capture_ns, &ref_imu);
	enum constellation_track_state before = device->track.state;
	double excess, rotation;
	int64_t ref_capture_ns = device->track.ref.capture_ns;
	bool ref_yaw_observed = device->track.ref_orientation_observed;
	struct xrt_vec3 delta = {
	    next.P_world_model.position.x - device->track.ref.P_world_model.position.x,
	    next.P_world_model.position.y - device->track.ref.P_world_model.position.y,
	    next.P_world_model.position.z - device->track.ref.P_world_model.position.z,
	};
	double distance = sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
	enum constellation_track_verdict verdict = constellation_track_update(
	    &device->track, &next, r->vision_determines_yaw, have_ref_imu ? &ref_imu : NULL, &excess, &rotation);
	switch (verdict) {
	case CONSTELLATION_TRACK_PUBLISH: break;
	case CONSTELLATION_TRACK_HOLD_STALE: d->stale++; break;
	case CONSTELLATION_TRACK_HOLD_UNCONFIRMED: d->unconfirmed++; /* fallthrough */
	case CONSTELLATION_TRACK_HOLD_CONFIRMING: d->confirming++; break;
	case CONSTELLATION_TRACK_HOLD_JUMP:
	case CONSTELLATION_TRACK_HOLD_RESTART: {
		d->jumps[r->origin]++;
		device->jumps_since_log++;
		uint64_t now = os_monotonic_get_ns();
		if (verdict == CONSTELLATION_TRACK_HOLD_RESTART || now - device->last_jump_log_ns >= U_TIME_1S_IN_NS) {
			CT_INFO(ct,
			        "Tracking device=%s capture=%" PRIu64
			        " reject=jump hypothesis=%s inliers=%u"
			        " position_excess_m=%.3f rotation_error_deg=%.1f have_ref_attitude=%d"
			        " jumps_since_last_log=%" PRIu64
			        " ref_capture_ns=%" PRId64 " dt_ms=%.3f distance_m=%.3f"
			        " ref_yaw_observed=%d yaw_observed=%d gate=%s processing_ns=%" PRIu64,
			        device->alias, capture_ns, constellation_constrained_origin_name(r->origin),
			        r->inliers, excess, rotation * 180.0 / M_PI, have_ref_imu, device->jumps_since_log,
			        ref_capture_ns, ((int64_t)capture_ns - ref_capture_ns) / 1e6, distance,
			        ref_yaw_observed, r->vision_determines_yaw, constellation_track_verdict_name(verdict), now);
			device->last_jump_log_ns = now;
			device->jumps_since_log = 0;
		}
		d->confirming += verdict == CONSTELLATION_TRACK_HOLD_RESTART;
		break;
	}
	}
	if (verdict != CONSTELLATION_TRACK_PUBLISH) {
		d->last_failure = constellation_track_verdict_name(verdict);
		d->last_failure_capture_ns = capture_ns;
		d->last_failure_processing_ns = os_monotonic_get_ns();
	}
	log_track_transition_locked(ct, device, before, capture_ns,
	                            verdict == CONSTELLATION_TRACK_HOLD_RESTART ? "jumps"
	                            : verdict == CONSTELLATION_TRACK_PUBLISH    ? "confirmed"
	                                                                        : "candidate");
	return verdict == CONSTELLATION_TRACK_PUBLISH ? NULL : constellation_track_verdict_name(verdict);
}

/* World (OpenCV) covariance into the OpenXR frame: the YZ flip is diag(1,-1,-1). */
static void
observation_from_result(const struct constellation_constrained_result *r,
                        timepoint_ns capture_ns,
                        const struct xrt_pose *P_xrworld_device,
                        struct t_constellation_pose_observation *obs)
{
	static const float flip[3] = {1, -1, -1};
	*obs = (struct t_constellation_pose_observation){
	    .capture_ns = capture_ns,
	    .pose = *P_xrworld_device,
	    .orientation_observed = r->vision_determines_yaw,
	    .inliers = r->inliers,
	    .distinct_leds = r->distinct_leds,
	    .information_rank = r->information_rank,
	    .yaw_variance = r->vision_determines_yaw ? (float)r->covariance[15] : INFINITY,
	    .cost_m = (float)r->cost_m,
	    .hypothesis = constellation_constrained_origin_name(r->origin),
	};
	for (unsigned i = 0; i < 3; i++) {
		for (unsigned j = 0; j < 3; j++) {
			obs->position_covariance[3 * i + j] = (float)r->covariance[4 * i + j] * flip[i] * flip[j];
		}
	}
}

/* Publish one rig pose per exposure. Labels are evidence from this accepted hypothesis,
 * never independent ownership carried from an earlier frame. */
static bool
submit_joint_pose(struct t_constellation_tracker *ct,
                  struct tracking_sample_device_state *state,
                  struct constellation_tracking_sample *sample,
                  const char *stage)
{
	struct constellation_tracker_device *device = &ct->devices[state->dev_index];
	struct constellation_constrained_result *r = &state->joint_result;
	os_mutex_lock(&ct->tracked_device_lock);
	const char *held = NULL;
	if (device->have_last_seen_pose && sample->timestamp <= device->last_seen_pose_ts) {
		device->diagnostics.stale++;
		held = "stale_exposure";
	} else {
		held = track_gate_locked(ct, device, r, sample->timestamp);
	}
	if (held != NULL) {
		device->diagnostics.last_failure = held;
		device->diagnostics.last_failure_capture_ns = sample->timestamp;
		device->diagnostics.last_failure_processing_ns = os_monotonic_get_ns();
		log_device_summary_locked(ct, device, false);
		os_mutex_unlock(&ct->tracked_device_lock);
		log_joint_result(ct, sample, state, stage, held);
		return false;
	}
	device->have_last_seen_pose = true;
	device->last_seen_pose_ts = sample->timestamp;
	device->last_seen_pose = r->P_world_model;
	device->last_matched_blobs = r->inliers;
	device->diagnostics.published++;
	device->diagnostics.published_processing_ns = os_monotonic_get_ns();
	device->diagnostics.last_failure = NULL;
	device->diagnostics.last_failure_capture_ns = device->diagnostics.last_failure_processing_ns = 0;
	device->diagnostics.vision_yaw += r->vision_determines_yaw;
	unsigned camera_inliers[XRT_TRACKING_MAX_SLAM_CAMS] = {0};
	for (unsigned i = 0; i < r->num_assignments; i++) {
		if (r->assignments[i].camera_index < XRT_TRACKING_MAX_SLAM_CAMS) {
			camera_inliers[r->assignments[i].camera_index]++;
		}
	}
	for (int c = 0; c < XRT_TRACKING_MAX_SLAM_CAMS; c++) {
		device->diagnostics.published_by_camera[c] += camera_inliers[c] > 0;
	}
	device->diagnostics.accepted_origin[r->origin]++;
	state->final_pose = r->P_world_model;
	state->found_device_pose = true;
	state->found_pose_view_id = r->assignments[0].camera_index;
	device->last_matched_cam = state->found_pose_view_id;
	math_pose_transform(&sample->views[state->found_pose_view_id].P_cam_world, &state->final_pose,
	                    &device->last_matched_cam_pose);
	state->score.matched_blobs = r->inliers;
	state->score.visible_leds = r->projected_visible_leds;
	state->score.match_flags = POSE_MATCH_GOOD | POSE_MATCH_LED_IDS;

	uint32_t brightness = 0;
	for (unsigned v = 0; v < sample->n_views; v++) {
		struct tracking_sample_frame *view = &sample->views[v];
		if (!view->bwobs) {
			continue;
		}
		for (int b = 0; b < view->bwobs->num_blobs; b++) {
			struct blob *blob = &view->bwobs->blobs[b];
			if (LED_OBJECT_ID(blob->led_id) == device->led_model.id) {
				blob->led_id = LED_INVALID_ID;
			}
		}
		for (unsigned i = 0; i < r->num_assignments; i++) {
			struct constellation_constrained_assignment *a = &r->assignments[i];
			if (a->camera_index != v) {
				continue;
			}
			assert(a->blob_index < (unsigned)view->bwobs->num_blobs);
			struct blob *blob = &view->bwobs->blobs[a->blob_index];
			blob->led_id = LED_MAKE_ID(device->led_model.id, a->led_id);
			brightness += blob->brightness;
		}
		os_mutex_lock(&ct->cam[v].bw_lock);
		blobwatch_update_labels(ct->cam[v].bw, view->bwobs, device->led_model.id);
		os_mutex_unlock(&ct->cam[v].bw_lock);
	}

	struct xrt_pose P_xrworld_model, P_xrworld_device;
	pose_flip_YZ(&state->final_pose, &P_xrworld_model);
	math_pose_transform(&P_xrworld_model, &device->led_model.P_model_device, &P_xrworld_device);
	struct t_constellation_pose_observation obs;
	observation_from_result(r, sample->timestamp, &P_xrworld_device, &obs);
	struct xrt_vec3 head = {NAN, NAN, NAN};
	if (head_relative(sample, device, &r->P_world_model, &head)) {
		device->diagnostics.last_published_head = head;
		device->diagnostics.have_last_published_head = true;
	}
	// Once a second per device, where the hand was and which cameras saw it, so a worn log can
	// relate dropouts to regions of the play space.
	uint64_t now = os_monotonic_get_ns();
	if (ct->log_level <= U_LOGGING_DEBUG && now - device->diagnostics.last_pose_log_ns >= U_TIME_1S_IN_NS) {
		device->diagnostics.last_pose_log_ns = now;
		char per_camera[64] = "";
		size_t used = 0;
		for (int c = 0; c < ct->cam_count && used < sizeof(per_camera); c++) {
			used += snprintf(per_camera + used, sizeof(per_camera) - used, "%s%u", c ? "," : "",
			                 camera_inliers[c]);
		}
		double pixel_error_squared = 0, maximum_pixel_error = 0;
		unsigned projected = 0;
		for (unsigned i = 0; i < r->num_assignments; i++) {
			const struct constellation_constrained_assignment *a = &r->assignments[i];
			const struct tracking_sample_frame *view = &sample->views[a->camera_index];
			struct xrt_vec3 world_led, camera_led;
			math_pose_transform_point(&r->P_world_model, &device->led_model.leds[a->led_index].pos, &world_led);
			math_pose_transform_point(&view->P_cam_world, &world_led, &camera_led);
			struct xrt_vec2 pixel;
			if (!camera_model_project(&ct->cam[a->camera_index].camera_model,
			                          camera_led.x, camera_led.y, camera_led.z, &pixel.x, &pixel.y)) continue;
			const struct blob *blob = &view->bwobs->blobs[a->blob_index];
			double error = hypot(pixel.x - blob->x, pixel.y - blob->y);
			pixel_error_squared += error * error;
			maximum_pixel_error = fmax(maximum_pixel_error, error);
			projected++;
		}
		CT_DEBUG(ct,
		         "Tracking pose device=%s capture=%" PRIu64
		         " position=%.3f,%.3f,%.3f head=%.3f,%.3f,%.3f hypothesis=%s inliers=%u"
		         " distinct=%u camera_inliers=%s tilt_correction_deg=%.1f reprojection_rms_px=%.4f"
		         " reprojection_max_px=%.4f fit_chi2=%.3f/%.3f cost_m=%.6f",
		         device->alias, sample->timestamp, P_xrworld_device.position.x, P_xrworld_device.position.y,
		         P_xrworld_device.position.z, head.x, head.y, head.z,
		         constellation_constrained_origin_name(r->origin), r->inliers, r->distinct_leds, per_camera,
		         RAD_TO_DEG(r->tilt_correction_rad), projected ? sqrt(pixel_error_squared / projected) : NAN,
		         projected ? maximum_pixel_error : NAN, r->fit_chi_square, r->fit_chi_square_limit, r->cost_m);
	}
	constellation_tracked_device_connection_notify_brightness_update(device->connection, brightness / r->inliers);
	constellation_tracked_device_connection_notify_observation(device->connection, &obs);

	if (ct->controller_masks_sink) {
		for (unsigned v = 0; v < sample->n_views; v++) {
			struct constellation_tracker_camera_state *cam = &ct->cam[v];
			struct xrt_pose P_cam_model;
			math_pose_transform(&sample->views[v].P_cam_world, &state->final_pose, &P_cam_model);
			struct xrt_device_masks_sample_device *mask =
			    &ct->controller_masks_sample.views[cam->slam_tracking_index].devices[state->dev_index];
			struct pose_rect bounds;
			pose_metrics_get_device_bounds(&P_cam_model, &device->led_model, &cam->camera_model, &bounds,
			                               NULL, NULL);
			mask->enabled = pose_rect_has_area(&bounds);
			if (mask->enabled) {
				mask->rect = (struct xrt_rect_f32){bounds.left, bounds.top, bounds.right - bounds.left,
				                                   bounds.bottom - bounds.top};
			}
		}
		xrt_sink_push_device_masks(ct->controller_masks_sink, &ct->controller_masks_sample);
	}
	log_device_summary_locked(ct, device, false);
	os_mutex_unlock(&ct->tracked_device_lock);
	log_joint_result(ct, sample, state, stage, "none");
	return true;
}

/* A competing candidate keeps a shared blob only with this much more inlier support. */
#define ARBITRATION_SUPPORT_MARGIN 2

/* Resolve all devices before notifying fusion. A blob owned by a published pose, a clearly
 * better-supported candidate, or an at-least-as-supported consistent hypothesis cannot verify
 * an acquisition. Decisions use the original claims, so device order does not matter. */
static void
arbitrate_and_submit(struct t_constellation_tracker *ct,
                     struct constellation_tracking_sample *sample,
                     bool candidates[CONSTELLATION_MAX_DEVICES],
                     const char *stage)
{
	const struct constellation_constrained_result *claims[CONSTELLATION_MAX_DEVICES];
	bool published[CONSTELLATION_MAX_DEVICES] = {0};
	enum constellation_constrained_claim blocked[CONSTELLATION_MAX_DEVICES];
	unsigned blockers[CONSTELLATION_MAX_DEVICES];
	for (unsigned d = 0; d < sample->n_devices; d++) {
		claims[d] = &sample->devices[d].joint_result;
		published[d] = sample->devices[d].found_device_pose;
	}
	constellation_constrained_arbitrate_with_owners(claims, published, candidates, blocked, blockers,
	                                                sample->n_devices, ARBITRATION_SUPPORT_MARGIN);
	for (unsigned d = 0; d < sample->n_devices; d++) {
		struct tracking_sample_device_state *state = &sample->devices[d];
		if (blocked[d] != CONSTELLATION_CLAIM_NONE) {
			os_mutex_lock(&ct->tracked_device_lock);
			ct->devices[state->dev_index].diagnostics.shared_blob++;
			ct->devices[state->dev_index].diagnostics.shared_by[blocked[d]]++;
			struct constellation_device_diagnostics *loser = &ct->devices[state->dev_index].diagnostics;
			uint64_t now = os_monotonic_get_ns();
			unsigned w = blockers[d];
			if (w < sample->n_devices && now - loser->last_shared_log_ns >= U_TIME_1S_IN_NS) {
				loser->last_shared_log_ns = now;
				CT_DEBUG(ct,
				         "Tracking device=%s capture=%" PRIu64
				         " reject=shared_blob claim=%s by=%s inliers=%u/%u"
				         " distinct=%u/%u hypothesis=%s/%s",
				         ct->devices[state->dev_index].alias, sample->timestamp,
				         constellation_constrained_claim_name(blocked[d]),
				         ct->devices[sample->devices[w].dev_index].alias, claims[d]->inliers,
				         claims[w]->inliers, claims[d]->distinct_leds, claims[w]->distinct_leds,
				         constellation_constrained_origin_name(claims[d]->origin),
				         constellation_constrained_origin_name(claims[w]->origin));
			}
			for (unsigned p = 0; blocked[d] == CONSTELLATION_CLAIM_PUBLISHED && p < sample->n_devices;
			     p++) {
				struct constellation_tracker_device *owner = &ct->devices[sample->devices[p].dev_index];
				if (p == d || !published[p] || owner->track.state == CONSTELLATION_TRACK_LOST ||
				    !constellation_constrained_contests_published(claims[d], claims[p],
				                                                  ARBITRATION_SUPPORT_MARGIN)) {
					continue;
				}
				// The published pose stands for this exposure, but its prior must not keep
				// steering association onto blobs another device explains better.
				enum constellation_track_state before = owner->track.state;
				constellation_track_reset(&owner->track);
				owner->diagnostics.contested++;
				log_track_transition_locked(ct, owner, before, sample->timestamp, "contested");
				CT_DEBUG(ct,
				         "Tracking device=%s capture=%" PRIu64
				         " contested_by=%s inliers=%u/%u distinct=%u/%u",
				         owner->alias, sample->timestamp, ct->devices[state->dev_index].alias,
				         claims[p]->inliers, claims[d]->inliers, claims[p]->distinct_leds,
				         claims[d]->distinct_leds);
			}
			os_mutex_unlock(&ct->tracked_device_lock);
			log_joint_result(ct, sample, state, stage,
			                 blocked[d] == CONSTELLATION_CLAIM_PUBLISHED   ? "shared_blob_published"
			                 : blocked[d] == CONSTELLATION_CLAIM_CANDIDATE ? "shared_blob_candidate"
			                                                               : "shared_blob_rejected");
		}
		if (candidates[d]) {
			submit_joint_pose(ct, state, sample, stage);
		}
	}
}

static void
joint_config(struct constellation_constrained_config *config, uint64_t capture_ns, bool fast)
{
	constellation_constrained_default_config(config);
	config->score_gate_m = debug_get_float_option_ct_score_gate();
	config->association_gate_m = debug_get_float_option_ct_association_gate();
	config->ray_noise_std_m = debug_get_float_option_ct_ray_noise();
	config->maximum_depth_m = debug_get_float_option_ct_max_depth();
	config->bearing_noise_std_rad = debug_get_float_option_ct_bearing_noise();
	config->minimum_cold_inliers = (unsigned)debug_get_num_option_ct_cold_inliers();
	config->tilt_prior_std_rad = DEG_TO_RAD(debug_get_float_option_ct_tilt_prior());
	config->seed = capture_ns;
	if (fast) {
		// The blob thread runs at the exposure rate; keep its search bounded.
		config->time_budget_us = 3000;
	}
}

#define MAX_P3P_JOINT_CANDIDATES 24
struct joint_p3p_context
{
	struct t_constellation_led_model *model;
	struct constellation_constrained_view *views;
	unsigned num_views;
	struct constellation_constrained_config *config;
	struct tracking_sample_frame *generating_view;
	struct tracking_sample_device_state *state;
	unsigned count;
	bool overflow;
	struct constellation_constrained_result candidates[MAX_P3P_JOINT_CANDIDATES];
};

static void
joint_p3p_candidate(void *userdata, const struct xrt_pose *camera_pose, const struct pose_metrics *score)
{
	(void)score;
	struct joint_p3p_context *ctx = userdata;
	struct xrt_pose world_pose;
	math_pose_transform(&ctx->generating_view->P_world_cam, camera_pose, &world_pose);
	// Filter discontinuous identities before they can outrank the continuous track.
	if (ctx->state->trusted_position) {
		struct xrt_vec3 delta = m_vec3_sub(world_pose.position, ctx->state->P_world_obj_prior.position);
		if (m_vec3_len(delta) > ctx->config->maximum_prior_distance_m)
			return;
		float dot = fminf(1.f, fabsf(math_quat_dot(&world_pose.orientation, &ctx->state->P_world_obj_prior.orientation)));
		if (ctx->state->reliable_heading && 2 * acosf(dot) > ctx->config->maximum_prior_rotation_rad)
			return;
	}
	struct constellation_constrained_result *result = malloc(sizeof(*result));
	if (result == NULL) {
		return;
	}
	if (!constellation_constrained_verify_pose(ctx->model, ctx->views, ctx->num_views, &world_pose,
	                                          CONSTELLATION_CONSTRAINED_P3P, ctx->config, result)) {
		free(result);
		return;
	}
	/* Do not fill the bounded competitor set with the same P3P root generated
	 * from many quadruples. Matching the same support/identities and pose is evidence once. */
	for (unsigned c = 0; c < ctx->count; c++) {
		struct constellation_constrained_result *old = &ctx->candidates[c];
		if (old->num_assignments != result->num_assignments) {
			continue;
		}
		bool identical = true;
		for (unsigned a = 0; a < result->num_assignments && identical; a++) {
			struct constellation_constrained_assignment *claim = &result->assignments[a];
			bool found = false;
			for (unsigned b = 0; b < old->num_assignments; b++) {
				struct constellation_constrained_assignment *other = &old->assignments[b];
				if (claim->camera_index == other->camera_index && claim->blob_index == other->blob_index &&
				    claim->led_id == other->led_id) {
					found = true;
					break;
				}
			}
			identical = found;
		}
		struct xrt_vec3 delta = m_vec3_sub(old->P_world_model.position, result->P_world_model.position);
		float dot = fabsf(math_quat_dot(&old->P_world_model.orientation, &result->P_world_model.orientation));
		if (identical && m_vec3_len(delta) < 0.0001f && dot > cosf(0.001f / 2)) {
			if (result->cost_m < old->cost_m) {
				*old = *result;
			}
			free(result);
			return;
		}
	}
	if (ctx->count == MAX_P3P_JOINT_CANDIDATES) {
		ctx->overflow = true;
	} else {
		ctx->candidates[ctx->count++] = *result;
	}
	free(result);
}

/* Unconstrained P3P fallback within a bounded budget. Every geometry-verified per-view root is
 * jointly verified across cameras; the result keeps the cold claim if P3P produces nothing. */
static bool
joint_p3p_fallback(struct t_constellation_tracker *ct,
                   struct constellation_tracking_sample *sample,
                   struct tracking_sample_device_state *state,
                   struct constellation_constrained_view *views,
                   unsigned n_views,
                   struct constellation_constrained_config *config,
                   enum constellation_constrained_rejection *rejection)
{
	// Without a verified P3P root, the cold result stays as the claim but is not P3P's verdict.
	*rejection = CONSTELLATION_CONSTRAINED_NO_PAIR;
	struct joint_p3p_context *ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		return false;
	}
	ctx->model = state->led_model;
	ctx->state = state;
	ctx->views = views;
	ctx->num_views = n_views;
	ctx->config = config;
	uint64_t deadline = os_monotonic_get_ns() + 25 * U_TIME_1MS_IN_NS;
	for (unsigned v = 0; v < sample->n_views; v++) {
		struct tracking_sample_frame *view = &sample->views[v];
		uint64_t now = os_monotonic_get_ns();
		if (now >= deadline || ctx->overflow) {
			break;
		}
		if (!view->bwobs || view->bwobs->num_blobs < 3) {
			continue;
		}
		struct constellation_tracker_camera_state *cam = &ct->cam[v];
		struct constellation_tracker_device *device = &ct->devices[state->dev_index];
		ctx->generating_view = view;
		cam->cs->max_search_ns = deadline - now;
		cam->cs->pose_candidate_cb = joint_p3p_candidate;
		cam->cs->pose_candidate_userdata = ctx;
		for (unsigned k = 0; k < n_views; k++) {
			if (views[k].camera_index == v) {
				cam->cs->excluded_blobs = views[k].excluded_blobs;
				break;
			}
		}
		correspondence_search_set_blobs(cam->cs, view->bwobs->blobs, view->bwobs->num_blobs);
		struct xrt_pose pose;
		struct pose_metrics score;
		math_pose_transform(&view->P_cam_world, &state->P_world_obj_prior, &pose);
		/* Every qualifying hypothesis reaches the joint evaluator. Do not stop on
		 * the first strong single-view result before checking competing roots. */
		correspondence_search_find_one_pose(cam->cs, device->search_led_model,
		                                    CS_FLAG_SHALLOW_SEARCH | CS_FLAG_DEEP_SEARCH |
		                                        (state->trusted_position ? CS_FLAG_HAVE_POSE_PRIOR : 0) |
		                                        CS_FLAG_MATCH_GRAVITY | CS_FLAG_MATCH_ALL_BLOBS |
		                                        CS_FLAG_JOINT_P3P,
		                                    &pose, &state->prior_pos_error, &state->prior_rot_error,
		                                    &view->cam_gravity_vector, state->gravity_error_rad, &score);
		cam->cs->pose_candidate_cb = NULL;
		cam->cs->pose_candidate_userdata = NULL;
		cam->cs->excluded_blobs = NULL;
		cam->cs->max_search_ns = 0;
		CT_TRACE(ct,
		         "Tracking device=%s capture=%" PRIu64 " camera=%u hypothesis=p3p trials=%u"
		         " checks=%u reject_gravity=%u metrics=%u verified=%u budget_exhausted=%d",
		         device->alias, sample->timestamp, v, cam->cs->num_trials, cam->cs->num_pose_checks,
		         cam->cs->num_gravity_rejects, cam->cs->num_metric_checks, ctx->count,
		         cam->cs->budget_exhausted);
	}
	bool accepted = false;
	if (ctx->count) {
		accepted = constellation_constrained_select_results(ctx->candidates, ctx->count, config,
		                                                    &state->joint_result);
		*rejection = state->joint_result.rejection;
	}
	if (ctx->overflow) {
		state->joint_result.rejection = *rejection = CONSTELLATION_CONSTRAINED_AMBIGUOUS;
		accepted = false;
	}
	free(ctx);
	return accepted;
}

// Fast frame processing: blob extraction and match to existing predictions
static void
constellation_tracker_process_frame_fast(struct xrt_frame_sink *sink, struct xrt_frame *xf)
{
	struct t_constellation_tracker *ct = container_of(sink, struct t_constellation_tracker, fast_process_sink);
#ifdef __linux__
	if (!ct->fast_thread_named) {
		prctl(PR_SET_NAME, "ct-fast", 0, 0, 0);
		ct->fast_thread_named = true;
	}
#endif
	struct xrt_space_relation xsr_base_pose = XRT_SPACE_RELATION_ZERO;

	/* Allocate a tracking sample for everything we're about to process */
	struct constellation_tracking_sample *sample = constellation_tracking_sample_new();
	uint64_t fast_analysis_start_ts = os_monotonic_get_ns();

	CT_DEBUG(ct, "Starting analysis of frame %" PRIu64 " TS %" PRIu64, xf->source_sequence, xf->timestamp);

	/* Get the HMD's pose so we can calculate the camera view poses */
	xrt_result_t pose_result =
	    xrt_device_get_tracked_pose(ct->hmd_xdev, XRT_INPUT_GENERIC_TRACKER_POSE, xf->timestamp, &xsr_base_pose);
	if (!constellation_tracking_sample_set_hmd_pose(sample, pose_result, &xsr_base_pose)) {
		CT_DEBUG(ct, "Skipping exposure TS %" PRIu64 " without a tracked HMD camera pose result=%d flags=0x%x",
		         xf->timestamp, pose_result, xsr_base_pose.relation_flags);
		constellation_tracking_sample_free(sample);
		return;
	}

	/* Split out camera views and collect blobs across all cameras */
	assert(ct->cam_count <= XRT_TRACKING_MAX_SLAM_CAMS);
	sample->n_views = ct->cam_count;
	sample->timestamp = xf->timestamp;

	for (int i = 0; i < ct->cam_count; i++) {
		struct constellation_tracker_camera_state *cam = ct->cam + i;
		struct tracking_sample_frame *view = sample->views + i;

		// Flip the input pose to CV coords, so we can do all our operations
		// in OpenCV coords
		struct xrt_pose P_cvworld_hmdimu;
		pose_flip_YZ(&xsr_base_pose.pose, &P_cvworld_hmdimu);

		math_pose_transform(&P_cvworld_hmdimu, &cam->P_imu_cam, &view->P_world_cam);

		CT_DEBUG(ct,
		         "Prepare transforms for cam %d "
		         " HMD pose %f,%f,%f,%f pos %f,%f,%f "
		         " P_imu_cam %f,%f,%f,%f pos %f,%f,%f "
		         " P_world_cam %f,%f,%f,%f pos %f,%f,%f ",
		         i, xsr_base_pose.pose.orientation.x, xsr_base_pose.pose.orientation.y,
		         xsr_base_pose.pose.orientation.z, xsr_base_pose.pose.orientation.w,
		         xsr_base_pose.pose.position.x, xsr_base_pose.pose.position.y, xsr_base_pose.pose.position.z,

		         cam->P_imu_cam.orientation.x, cam->P_imu_cam.orientation.y, cam->P_imu_cam.orientation.z,
		         cam->P_imu_cam.orientation.w, cam->P_imu_cam.position.x, cam->P_imu_cam.position.y,
		         cam->P_imu_cam.position.z,

		         view->P_world_cam.orientation.x, view->P_world_cam.orientation.y,
		         view->P_world_cam.orientation.z, view->P_world_cam.orientation.w, view->P_world_cam.position.x,
		         view->P_world_cam.position.y, view->P_world_cam.position.z);

		// Calculate inverse from cam back to world coords
		math_pose_invert(&view->P_world_cam, &view->P_cam_world);

		const struct xrt_vec3 gravity_vector = {0.0, 1.0, 0.0};
		math_quat_rotate_vec3(&view->P_cam_world.orientation, &gravity_vector, &view->cam_gravity_vector);

		u_frame_create_roi(xf, cam->roi, &view->vframe);
		view->bw = cam->bw;

		os_mutex_lock(&cam->bw_lock);
		blobwatch_process(cam->bw, view->vframe, &view->bwobs);
		os_mutex_unlock(&cam->bw_lock);

		if (view->bwobs == NULL) {
			cam->last_num_blobs = 0;
			continue;
		}

		blobservation *bwobs = view->bwobs;
		cam->last_num_blobs = bwobs->num_blobs;

		CT_TRACE(ct, "frame %" PRIu64 " TS %" PRIu64 " cam %d ROI %d,%d w/h %d,%d Blobs: %d",
		         xf->source_sequence, xf->timestamp, i, cam->roi.offset.w, cam->roi.offset.h, cam->roi.extent.w,
		         cam->roi.extent.h, bwobs->num_blobs);

#if 0
		for (int index = 0; index < bwobs->num_blobs; index++) {
			printf("  Blob[%d]: %f,%f %dx%d id %d age %u\n", index, bwobs->blobs[index].x,
			       bwobs->blobs[index].y, bwobs->blobs[index].width, bwobs->blobs[index].height,
			       bwobs->blobs[index].led_id, bwobs->blobs[index].age);
		}
#endif
	}
	for (int i = 0; i < ct->cam_count; i++) {
		const blobservation *obs = sample->views[i].bwobs;
		ct->camera_exposures[i]++;
		if (obs == NULL) {
			continue;
		}
		ct->camera_blobs[i] += obs->num_blobs;
		ct->camera_dark_blobs[i] += obs->dropped_dark_blobs;
		for (int b = 0; b < obs->num_blobs; b++) {
			ct->camera_blob_area[i] += obs->blobs[b].area;
			ct->camera_blob_brightness[i] += obs->blobs[b].brightness;
		}
	}
	uint64_t blob_extract_finish_ts = os_monotonic_get_ns();
	if (ct->log_level <= U_LOGGING_DEBUG &&
	    blob_extract_finish_ts - ct->last_camera_summary_ns >= 5 * (uint64_t)U_TIME_1S_IN_NS) {
		ct->last_camera_summary_ns = blob_extract_finish_ts;
		/* Per camera over the last window: blobs kept per exposure, blobs dropped as too dark per
		 * exposure, and the mean area (pixels) and peak brightness of the kept blobs. */
		char per_camera[512] = "";
		size_t used = 0;
		for (int i = 0; i < ct->cam_count && used < sizeof(per_camera); i++) {
			double exposures = (double)(ct->camera_exposures[i] ? ct->camera_exposures[i] : 1);
			double blobs = (double)(ct->camera_blobs[i] ? ct->camera_blobs[i] : 1);
			used += snprintf(per_camera + used, sizeof(per_camera) - used, " cam%d=%.2f/%.2f/%.1f/%.0f", i,
			                 ct->camera_blobs[i] / exposures, ct->camera_dark_blobs[i] / exposures,
			                 ct->camera_blob_area[i] / blobs, ct->camera_blob_brightness[i] / blobs);
			ct->camera_exposures[i] = ct->camera_blobs[i] = ct->camera_dark_blobs[i] = 0;
			ct->camera_blob_area[i] = ct->camera_blob_brightness[i] = 0;
		}
		CT_DEBUG(ct, "Tracking cameras window_s=5 blobs/dark_dropped/area_px/brightness per exposure:%s",
		         per_camera);
	}
	ct->last_blob_analysis_ms = (blob_extract_finish_ts - fast_analysis_start_ts) / U_TIME_1MS_IN_NS;

	// Ready to start processing device poses now. Make sure we have the
	// LED models and collect the best estimate of the current pose
	// for each target device
	os_mutex_lock(&ct->tracked_device_lock);
	assert(ct->num_devices <= CONSTELLATION_MAX_DEVICES);

	for (int d = 0; d < ct->num_devices; d++) {
		struct constellation_tracker_device *device = ct->devices + d;

		if (!device->have_led_model) {
			if (!constellation_tracked_device_connection_get_led_model(device->connection,
			                                                           &device->led_model)) {
				device->diagnostics.missing_model++;
				device->diagnostics.last_failure = "model_unavailable";
				device->diagnostics.last_failure_capture_ns = xf->timestamp;
				device->diagnostics.last_failure_processing_ns = os_monotonic_get_ns();
				expire_track_locked(ct, device, xf->timestamp);
				log_device_summary_locked(ct, device, false);
				continue; // Can't do anything without the LED info
			}

			CT_INFO(ct, "Constellation Tracker: Retrieved controller LED model for device %u",
			        device->led_model.id);
			for (int i = 0; i < device->led_model.num_leds; i++) {
				struct t_constellation_led *led = &device->led_model.leds[i];
				CT_TRACE(ct, "Model device=%d led=%d pos=%f,%f,%f dir=%f,%f,%f", device->led_model.id,
				         i, led->pos.x, led->pos.y, led->pos.z, led->dir.x, led->dir.y, led->dir.z);
			}
			device->search_led_model = t_constellation_search_model_new(&device->led_model);
			device->have_led_model = true;
		}

		struct tracking_sample_device_state *dev_state = sample->devices + sample->n_devices;

		// Collect the controller prior pose
		struct xrt_space_relation xsr;
		device->diagnostics.exposures++;
		if (!constellation_tracked_device_connection_get_tracked_pose(device->connection, xf->timestamp,
		                                                              &xsr)) {
			device->diagnostics.no_attitude++;
			device->diagnostics.last_failure = "attitude_history";
			device->diagnostics.last_failure_capture_ns = xf->timestamp;
			device->diagnostics.last_failure_processing_ns = os_monotonic_get_ns();
			expire_track_locked(ct, device, xf->timestamp);
			CT_TRACE(ct,
			         "Tracking device=%s capture=%" PRIu64 " process=%" PRIu64
			         " kind=controller reject=attitude_history",
			         device->alias, xf->timestamp, os_monotonic_get_ns());
			log_device_summary_locked(ct, device, false);
			continue; // No capture-time attitude: disconnected, not yet calibrated, or history gap
		}

		// Apply device -> LED model pose from xsr = P_world_device + P_device_model = P_world_model
		struct xrt_pose P_xrworld_model;
		math_pose_transform(&xsr.pose, &device->led_model.P_device_model, &P_xrworld_model);

		// Incoming controller pose is in OpenXR. Flip it to OpenCV for all our operations
		pose_flip_YZ(&P_xrworld_model, &dev_state->P_world_obj_prior);

		//! @todo: Get actual error bounds from fusion
		dev_state->prior_pos_error.x = dev_state->prior_pos_error.y = dev_state->prior_pos_error.z =
		    MIN_POS_ERROR;
		dev_state->prior_rot_error.x = dev_state->prior_rot_error.y = dev_state->prior_rot_error.z =
		    MIN_ROT_ERROR;
		dev_state->gravity_error_rad = MIN_ROT_ERROR;
		// Keep a recent optical position bound while re-confirming after a rejected jump.
		expire_track_locked(ct, device, xf->timestamp);
		dev_state->trusted_position = (xsr.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0 &&
		                              (device->track.state == CONSTELLATION_TRACK_TRACKING ||
		                               device->track.state == CONSTELLATION_TRACK_CONFIRMING);
		dev_state->reliable_heading =
		    constellation_tracked_device_connection_has_heading(device->connection, xf->timestamp);

		dev_state->have_last_seen_pose = device->have_last_seen_pose;
		dev_state->last_seen_pose = device->last_seen_pose;

		dev_state->dev_index = d;
		dev_state->led_model = &device->led_model;

		sample->n_devices++;
	}
	os_mutex_unlock(&ct->tracked_device_lock);

	/* Fast path: a trusted position prior (recent optical fix) bounds association, so the
	 * stock low-count gates apply. Everything else goes to the long search. */
	struct constellation_constrained_view views[CONSTELLATION_MAX_CAMERAS];
	unsigned n_views = joint_views(ct, sample, views);
	unsigned blob_count = sample_blob_count(sample);
	bool candidates[CONSTELLATION_MAX_DEVICES] = {0};
	for (unsigned i = 0; i < sample->n_devices; i++) {
		struct tracking_sample_device_state *state = &sample->devices[i];
		if (!state->trusted_position || blob_count == 0) {
			os_mutex_lock(&ct->tracked_device_lock);
			ct->devices[state->dev_index].diagnostics.untrusted_prior += !state->trusted_position;
			os_mutex_unlock(&ct->tracked_device_lock);
			continue;
		}
		struct constellation_constrained_config config;
		joint_config(&config, sample->timestamp, true);
		candidates[i] = constellation_constrained_search(state->led_model, views, n_views,
		                                                 &state->P_world_obj_prior, state->reliable_heading,
		                                                 true, &config, &state->joint_result);
		count_attempt(ct, state, CT_STAGE_FAST, sample->timestamp, candidates[i], state->joint_result.rejection);
		if (!candidates[i]) {
			log_joint_result(ct, sample, state, ct_stage_names[CT_STAGE_FAST],
			                 constellation_constrained_rejection_name(state->joint_result.rejection));
		}
	}
	arbitrate_and_submit(ct, sample, candidates, ct_stage_names[CT_STAGE_FAST]);
	bool need_full_search = false;
	for (unsigned i = 0; i < sample->n_devices; i++) {
		need_full_search |= !sample->devices[i].found_device_pose;
	}

	uint64_t fast_analysis_finish_ts = os_monotonic_get_ns();
	ct->last_fast_analysis_ms = (fast_analysis_finish_ts - fast_analysis_start_ts) / U_TIME_1MS_IN_NS;
	CT_TRACE(ct, "Fast analysis seq=%" PRIu64 " ts=%" PRIu64 " blobs_ms=%.3f total_ms=%.3f age_ms=%.3f",
	         xf->source_sequence, xf->timestamp,
	         (double)(blob_extract_finish_ts - fast_analysis_start_ts) / U_TIME_1MS_IN_NS,
	         (double)(fast_analysis_finish_ts - fast_analysis_start_ts) / U_TIME_1MS_IN_NS,
	         (double)((int64_t)fast_analysis_finish_ts - (int64_t)xf->timestamp) / U_TIME_1MS_IN_NS);

	/* Send analysis results to debug view if needed */
	enum debug_draw_flag debug_flags = DEBUG_DRAW_FLAG_NONE;
	if (ct->debug_draw_normalise)
		debug_flags |= DEBUG_DRAW_FLAG_NORMALISE;
	if (ct->debug_draw_blob_tint)
		debug_flags |= DEBUG_DRAW_FLAG_BLOB_TINT;
	if (ct->debug_draw_blob_circles)
		debug_flags |= DEBUG_DRAW_FLAG_BLOB_CIRCLE;
	if (ct->debug_draw_blob_ids)
		debug_flags |= DEBUG_DRAW_FLAG_BLOB_IDS;
	if (ct->debug_draw_blob_unique_ids)
		debug_flags |= DEBUG_DRAW_FLAG_BLOB_UNIQUE_IDS;
	if (ct->debug_draw_leds)
		debug_flags |= DEBUG_DRAW_FLAG_LEDS;
	if (ct->debug_draw_prior_leds)
		debug_flags |= DEBUG_DRAW_FLAG_PRIOR_LEDS;
	if (ct->debug_draw_last_leds)
		debug_flags |= DEBUG_DRAW_FLAG_LAST_SEEN_LEDS;
	if (ct->debug_draw_pose_bounds)
		debug_flags |= DEBUG_DRAW_FLAG_POSE_BOUNDS;
	if (ct->debug_draw_device_bounds)
		debug_flags |= DEBUG_DRAW_FLAG_DEVICE_BOUNDS;

	for (int i = 0; i < sample->n_views; i++) {
		struct constellation_tracker_camera_state *cam = ct->cam + i;
		struct tracking_sample_frame *view = sample->views + i;

		cam->debug_last_pose = view->P_world_cam;
		cam->debug_last_gravity_vector = view->cam_gravity_vector;

		if (u_sink_debug_is_active(&cam->debug_sink)) {
			struct xrt_frame *xf_src = view->vframe;
			struct xrt_frame *xf_dbg = NULL;

			u_frame_create_one_off(XRT_FORMAT_R8G8B8, xf_src->width, xf_src->height, &xf_dbg);
			xf_dbg->timestamp = xf_src->timestamp;

			// if (view->bwobs != NULL) {
			debug_draw_blobs_leds(xf_dbg, xf_src, debug_flags, view, i, &cam->camera_model, sample->devices,
			                      sample->n_devices);
			//}

			u_sink_debug_push_frame(&cam->debug_sink, xf_dbg);
			xrt_frame_reference(&xf_dbg, NULL);
		}
	}

	/* Grab the 'do_full_search' value in case the user clicked the
	 * button in the debug UI */
	if (ct->do_full_search || need_full_search) {
		ct->do_full_search = false;

		/* Send the sample for long analysis */
		os_thread_helper_lock(&ct->long_analysis_thread);
		if (ct->long_analysis_pending_sample != NULL) {
			ct->long_samples_replaced++;
			uint64_t now = os_monotonic_get_ns();
			if (now - ct->last_queue_log_ns >= 5 * (uint64_t)U_TIME_1S_IN_NS) {
				ct->last_queue_log_ns = now;
				CT_INFO(ct, "Tracking backlog replaced=%" PRIu64 " pending_capture_ns=%" PRIu64
				            " incoming_capture_ns=%" PRIu64 " incoming_age_ms=%.1f",
				        ct->long_samples_replaced, ct->long_analysis_pending_sample->timestamp,
				        sample->timestamp, (double)(now - sample->timestamp) / U_TIME_1MS_IN_NS);
			}
			constellation_tracking_sample_free(ct->long_analysis_pending_sample);
		}
		ct->long_analysis_pending_sample = sample;
		os_thread_helper_signal_locked(&ct->long_analysis_thread);
		os_thread_helper_unlock(&ct->long_analysis_thread);
	} else {
		/* not sending for long analysis: free it */
		constellation_tracking_sample_free(sample);
	}
}

static void
constellation_tracker_process_frame_long(struct t_constellation_tracker *ct,
                                         struct constellation_tracking_sample *sample)
{
	CT_DEBUG(ct, "Starting long analysis of frame TS %" PRIu64, sample->timestamp);

	/* Long path: cold constrained acquisition over all cameras, then the bounded P3P fallback.
	 * Gravity hypotheses are always generated; heading adds translation hypotheses. */
	struct constellation_constrained_view views[CONSTELLATION_MAX_CAMERAS];
	unsigned n_views = joint_views(ct, sample, views);
	unsigned blob_count = sample_blob_count(sample);
	bool candidates[CONSTELLATION_MAX_DEVICES] = {0};
	for (unsigned d = 0; d < sample->n_devices; d++) {
		struct tracking_sample_device_state *state = &sample->devices[d];
		if (state->found_device_pose) {
			continue;
		}
		if (blob_count == 0) {
			os_mutex_lock(&ct->tracked_device_lock);
			struct constellation_tracker_device *device = &ct->devices[state->dev_index];
			device->diagnostics.no_blobs++;
			device->diagnostics.last_failure = "no_blobs";
			device->diagnostics.last_failure_capture_ns = sample->timestamp;
			device->diagnostics.last_failure_processing_ns = os_monotonic_get_ns();
			log_device_summary_locked(ct, device, false);
			os_mutex_unlock(&ct->tracked_device_lock);
			continue;
		}
		struct constellation_constrained_config config;
		joint_config(&config, sample->timestamp, false);
		candidates[d] = constellation_constrained_search(state->led_model, views, n_views,
		                                                 &state->P_world_obj_prior, state->reliable_heading,
		                                                 state->trusted_position, &config, &state->joint_result);
		count_attempt(ct, state, CT_STAGE_COLD, sample->timestamp, candidates[d], state->joint_result.rejection);
		if (candidates[d]) {
			continue;
		}
		log_joint_result(ct, sample, state, ct_stage_names[CT_STAGE_COLD],
		                 constellation_constrained_rejection_name(state->joint_result.rejection));
		/* An ambiguous constrained result cannot be made trustworthy by asking
		 * another solver to select one of the same competing identities. */
		if (state->joint_result.rejection != CONSTELLATION_CONSTRAINED_AMBIGUOUS) {
			enum constellation_constrained_rejection p3p_rejection;
			candidates[d] = joint_p3p_fallback(ct, sample, state, views, n_views, &config, &p3p_rejection);
			count_attempt(ct, state, CT_STAGE_P3P, sample->timestamp, candidates[d], p3p_rejection);
			if (!candidates[d]) {
				log_joint_result(ct, sample, state, ct_stage_names[CT_STAGE_P3P],
				                 constellation_constrained_rejection_name(p3p_rejection));
			}
		}
	}
	arbitrate_and_submit(ct, sample, candidates, "long");

	/* A cold winner can keep explaining the already tracked ring, so arbitration rejects it
	 * forever without ever trying the other ring. After the unrestricted pass (which still
	 * detects contested owners), retry only the observations not published in this exposure.
	 * Keep indices and all confirmation/ambiguity gates; temporal labels are not ownership. */
	memset(candidates, 0, sizeof(candidates));
	for (unsigned d = 0; d < sample->n_devices; d++) {
		struct tracking_sample_device_state *state = &sample->devices[d];
		if (state->found_device_pose) {
			continue;
		}
		struct constellation_constrained_view remaining[CONSTELLATION_MAX_CAMERAS];
		memcpy(remaining, views, n_views * sizeof(*views));
		unsigned excluded = constellation_tracking_sample_exclude_published(sample, d, remaining, n_views);
		if (excluded == 0) {
			continue;
		}
		struct constellation_constrained_config config;
		joint_config(&config, sample->timestamp, false);
		candidates[d] = constellation_constrained_search(state->led_model, remaining, n_views,
		                                                 &state->P_world_obj_prior, state->reliable_heading,
		                                                 state->trusted_position, &config, &state->joint_result);
		count_attempt(ct, state, CT_STAGE_COLD, sample->timestamp, candidates[d], state->joint_result.rejection);
		if (!candidates[d] && state->joint_result.rejection != CONSTELLATION_CONSTRAINED_AMBIGUOUS) {
			enum constellation_constrained_rejection rejection;
			candidates[d] = joint_p3p_fallback(ct, sample, state, remaining, n_views, &config, &rejection);
			count_attempt(ct, state, CT_STAGE_P3P, sample->timestamp, candidates[d], rejection);
		}
		CT_TRACE(ct, "Tracking device=%s capture=%" PRIu64 " stage=unclaimed excluded=%u accepted=%d",
		         ct->devices[state->dev_index].alias, sample->timestamp, excluded, candidates[d]);
	}
	arbitrate_and_submit(ct, sample, candidates, "unclaimed");

	for (int d = 0; d < sample->n_devices; d++) {
		if (!sample->devices[d].found_device_pose) {
			struct tracking_sample_device_state *state = &sample->devices[d];
			for (unsigned v = 0; v < sample->n_views; v++) {
				struct tracking_sample_frame *view = &sample->views[v];
				if (!view->bwobs) continue;
				for (int b = 0; b < view->bwobs->num_blobs; b++) {
					struct blob *blob = &view->bwobs->blobs[b];
					if (LED_OBJECT_ID(blob->led_id) == state->led_model->id) blob->led_id = LED_INVALID_ID;
				}
				os_mutex_lock(&ct->cam[v].bw_lock);
				blobwatch_update_labels(ct->cam[v].bw, view->bwobs, state->led_model->id);
				os_mutex_unlock(&ct->cam[v].bw_lock);
			}

			// if a long analysis did not find the device at all, then we push that it has no brightness
			constellation_tracked_device_connection_notify_brightness_update(
			    ct->devices[sample->devices[d].dev_index].connection, 0);

			// update the controller masks for this controller to mark it as not active
			if (ct->controller_masks_sink) {
				os_mutex_lock(&ct->tracked_device_lock);

				for (int i = 0; i < ct->cam_count; i++) {
					struct constellation_tracker_camera_state *cam = &ct->cam[i];

					struct xrt_device_masks_sample_camera *sample_camera =
					    &ct->controller_masks_sample.views[cam->slam_tracking_index];

					struct xrt_device_masks_sample_device *device_mask =
					    &sample_camera->devices[sample->devices[d].dev_index];

					device_mask->enabled = false;
				}

				xrt_sink_push_device_masks(ct->controller_masks_sink, &ct->controller_masks_sample);
				os_mutex_unlock(&ct->tracked_device_lock);
			}
		}
	}
}

static void *
constellation_tracking_long_analysis_thread(void *ptr)
{
#ifdef __linux__
	prctl(PR_SET_NAME, "ct-long", 0, 0, 0);
#endif
	U_TRACE_SET_THREAD_NAME("Constellation tracker: device long recovery thread");
	struct t_constellation_tracker *ct = (struct t_constellation_tracker *)(ptr);

	os_thread_helper_lock(&ct->long_analysis_thread);
	while (os_thread_helper_is_running_locked(&ct->long_analysis_thread)) {
		/* Wait for a sample to analyse, or for shutdown */
		if (ct->long_analysis_pending_sample == NULL) {
			os_thread_helper_wait_locked(&ct->long_analysis_thread);
			if (!os_thread_helper_is_running_locked(&ct->long_analysis_thread)) {
				break;
			}
		}

		/* Take ownership of any pending sample */
		struct constellation_tracking_sample *sample = ct->long_analysis_pending_sample;
		ct->long_analysis_pending_sample = NULL;

		os_thread_helper_unlock(&ct->long_analysis_thread);
		if (sample != NULL) {
			uint64_t long_analysis_start_ts = os_monotonic_get_ns();
			constellation_tracker_process_frame_long(ct, sample);
			uint64_t long_analysis_finish_ts = os_monotonic_get_ns();

			CT_TRACE(ct, "Long analysis ts=%" PRIu64 " total_ms=%.3f age_ms=%.3f", sample->timestamp,
			         (double)(long_analysis_finish_ts - long_analysis_start_ts) / U_TIME_1MS_IN_NS,
			         (double)((int64_t)long_analysis_finish_ts - (int64_t)sample->timestamp) /
			             U_TIME_1MS_IN_NS);
			constellation_tracking_sample_free(sample);

			ct->last_long_analysis_ms =
			    (long_analysis_finish_ts - long_analysis_start_ts) / U_TIME_1MS_IN_NS;
		}

		os_thread_helper_lock(&ct->long_analysis_thread);
	}
	os_thread_helper_unlock(&ct->long_analysis_thread);

	return NULL;
}

static void
constellation_tracker_node_destroy(struct xrt_frame_node *node)
{
	struct t_constellation_tracker *ct = container_of(node, struct t_constellation_tracker, node);

	DRV_TRACE_MARKER();
	CT_DEBUG(ct, "Destroying constellation tracker");

	/* Make sure the long analysis thread isn't running */
	os_thread_helper_stop_and_wait(&ct->long_analysis_thread);

	// Unlink the device connections and release the models
	os_mutex_lock(&ct->tracked_device_lock);
	for (int i = 0; i < ct->num_devices; i++) {
		struct constellation_tracker_device *device = ct->devices + i;
		log_device_summary_locked(ct, device, true);

		// Clean up the LED tracking model
		t_constellation_led_model_clear(&device->led_model);
		if (device->search_led_model) {
			t_constellation_search_model_free(device->search_led_model);
		}

		if (device->connection != NULL) {
			t_constellation_tracked_device_connection_disconnect(device->connection);
			device->connection = NULL;
		}
	}
	os_mutex_unlock(&ct->tracked_device_lock);
	os_mutex_destroy(&ct->tracked_device_lock);

	/* Clean up any pending sample */
	os_thread_helper_lock(&ct->long_analysis_thread);
	if (ct->long_analysis_pending_sample != NULL) {
		constellation_tracking_sample_free(ct->long_analysis_pending_sample);
		ct->long_analysis_pending_sample = NULL;
	}
	os_thread_helper_unlock(&ct->long_analysis_thread);
	/* Then release the thread helper */
	os_thread_helper_destroy(&ct->long_analysis_thread);

	//! Clean up
	for (int i = 0; i < ct->cam_count; i++) {
		struct constellation_tracker_camera_state *cam = ct->cam + i;

		u_sink_debug_destroy(&cam->debug_sink);

		if (cam->cs) {
			correspondence_search_free(cam->cs);
		}

		os_mutex_destroy(&cam->bw_lock);
		if (cam->bw) {
			blobwatch_free(cam->bw);
		}
	}

	u_var_remove_root(ct);
	free(ct);
}

static void
ct_full_search_btn_cb(void *ct_ptr)
{
	struct t_constellation_tracker *ct = (struct t_constellation_tracker *)ct_ptr;
	ct->do_full_search = true;
}

int
t_constellation_tracker_create(struct xrt_frame_context *xfctx,
                               struct xrt_device *hmd_xdev,
                               struct t_constellation_camera_group *cams,
                               struct t_constellation_tracker **out_tracker,
                               struct xrt_frame_sink **out_sink,
                               struct xrt_device_masks_sink *controller_mask_sink)
{
	DRV_TRACE_MARKER();

	int ret;
	struct t_constellation_tracker *ct = calloc(1, sizeof(struct t_constellation_tracker));

	ct->log_level = debug_get_log_option_ct_log();
	ct->debug_draw_blob_tint = true;
	ct->debug_draw_blob_ids = true;
	ct->hmd_xdev = hmd_xdev;
	ct->controller_masks_sink = controller_mask_sink;

	// Set up the per-camera constellation tracking pieces config and pose
	ct->cam_count = cams->cam_count;
	for (int i = 0; i < ct->cam_count; i++) {
		struct constellation_tracker_camera_state *cam = ct->cam + i;
		struct t_constellation_camera *cam_cfg = cams->cams + i;

		cam->roi = cam_cfg->roi;
		cam->P_imu_cam = cam_cfg->P_imu_cam;
		cam->slam_tracking_index = cam_cfg->slam_tracking_index;

		/* Init the camera model with size and distortion */
		cam->camera_model.width = cam_cfg->roi.extent.w;
		cam->camera_model.height = cam_cfg->roi.extent.h;
		t_camera_model_params_from_t_camera_calibration(&cam_cfg->calibration, &cam->camera_model.calib);

		cam->camera_model.fisheye62_valid = cam_cfg->fisheye62_valid;
		memcpy(cam->camera_model.fisheye62_radial, cam_cfg->fisheye62_radial, sizeof(cam_cfg->fisheye62_radial));
		cam->camera_model.fisheye62_p1 = cam_cfg->fisheye62_p1;
		cam->camera_model.fisheye62_p2 = cam_cfg->fisheye62_p2;

		os_mutex_init(&cam->bw_lock);
		cam->bw = blobwatch_new(cam_cfg->blob_min_threshold, cam_cfg->blob_detect_threshold);
		CT_TRACE(ct, "Camera %d fx=%f fy=%f cx=%f cy=%f kb4=%f,%f,%f,%f", i, cam->camera_model.calib.fx,
		         cam->camera_model.calib.fy, cam->camera_model.calib.cx, cam->camera_model.calib.cy,
		         cam->camera_model.calib.fisheye.k1, cam->camera_model.calib.fisheye.k2,
		         cam->camera_model.calib.fisheye.k3, cam->camera_model.calib.fisheye.k4);
		cam->cs = correspondence_search_new(&cam->camera_model);
	}

	// Set up frame receiver
	ct->base.push_frame = constellation_tracker_receive_frame;

	// Setup node
	struct xrt_frame_node *xfn = &ct->node;
	xfn->break_apart = constellation_tracker_node_break_apart;
	xfn->destroy = constellation_tracker_node_destroy;

	ret = os_mutex_init(&ct->tracked_device_lock);
	if (ret != 0) {
		CT_ERROR(ct, "Failed to init tracked device mutex!");
		constellation_tracker_node_destroy(&ct->node);
		return -1;
	}

	ret = os_thread_helper_init(&ct->long_analysis_thread);
	if (ret != 0) {
		CT_ERROR(ct, "constellation tracker: Failed to init long analysis thread");
		constellation_tracker_node_destroy(&ct->node);
		return -1;
	}

	// Fast processing thread
	ct->fast_process_sink.push_frame = constellation_tracker_process_frame_fast;

	if (!u_sink_queue_create(xfctx, MAX_FAST_QUEUE_SIZE, &ct->fast_process_sink, &ct->fast_q_sink)) {
		CT_ERROR(ct, "Failed to init fast analysis queue!");
		constellation_tracker_node_destroy(&ct->node);
		return -1;
	}

	// Long match recovery thread
	ret = os_thread_helper_start(&ct->long_analysis_thread, constellation_tracking_long_analysis_thread, ct);
	if (ret != 0) {
		CT_ERROR(ct, "constellation tracker: Failed to start long analysis thread!");
		constellation_tracker_node_destroy(&ct->node);
		return -1;
	}

	// Debug UI
	ct->full_search_button.cb = ct_full_search_btn_cb;
	ct->full_search_button.ptr = ct;

	u_var_add_root(ct, "Constellation Tracker", false);
	u_var_add_log_level(ct, &ct->log_level, "Log Level");
	u_var_add_ro_i32(ct, &ct->num_devices, "Num Devices");
	u_var_add_ro_u64(ct, &ct->last_frame_timestamp, "Last Frame Timestamp");
	u_var_add_ro_u64(ct, &ct->last_blob_analysis_ms, "Blob tracking time (ms)");
	u_var_add_ro_u64(ct, &ct->last_fast_analysis_ms, "Fast analysis time (ms)");
	u_var_add_ro_u64(ct, &ct->last_long_analysis_ms, "Long analysis time (ms)");
	u_var_add_button(ct, &ct->full_search_button, "Trigger ab-initio search");

	u_var_add_bool(ct, &ct->debug_draw_normalise, "Debug: Normalise source frame");
	u_var_add_bool(ct, &ct->debug_draw_blob_tint, "Debug: Tint blobs by device assignment");
	u_var_add_bool(ct, &ct->debug_draw_blob_circles, "Debug: Draw circles around blobs");
	u_var_add_bool(ct, &ct->debug_draw_blob_ids, "Debug: Draw LED id labels for blobs");
	u_var_add_bool(ct, &ct->debug_draw_blob_unique_ids, "Debug: Draw blobs tracking ID");
	u_var_add_bool(ct, &ct->debug_draw_leds, "Debug: Draw LED position markers for found poses");
	u_var_add_bool(ct, &ct->debug_draw_prior_leds, "Debug: Draw LED markers for prior poses");
	u_var_add_bool(ct, &ct->debug_draw_last_leds, "Debug: Draw LED markers for last observed poses");
	u_var_add_bool(ct, &ct->debug_draw_pose_bounds, "Debug: Draw LED bounds rect for found poses");
	u_var_add_bool(ct, &ct->debug_draw_device_bounds, "Debug: Draw device bounds rect for found poses");

	for (int i = 0; i < ct->cam_count; i++) {
		struct constellation_tracker_camera_state *cam = ct->cam + i;
		u_var_add_ro_i32(ct, &cam->last_num_blobs, "Num Blobs");
		u_var_add_pose(ct, &cam->debug_last_pose, "Last view pose");
		u_var_add_vec3_f32(ct, &cam->debug_last_gravity_vector, "Last gravity vector");

		char cam_name[64];
		sprintf(cam_name, "Cam %u", i);
		u_sink_debug_init(&cam->debug_sink);
		u_var_add_sink_debug(ct, &cam->debug_sink, cam_name);
	}

	// Hand ownership to the frame context
	xrt_frame_context_add(xfctx, &ct->node);

	CT_DEBUG(ct, "Constellation tracker created");

	*out_tracker = ct;
	*out_sink = &ct->base;

	return 0;
}

static void
constellation_tracked_device_connection_destroy(struct t_constellation_tracked_device_connection *ctdc)
{
	DRV_TRACE_MARKER();

	os_mutex_destroy(&ctdc->lock);
	free(ctdc);
}

static struct t_constellation_tracked_device_connection *
constellation_tracked_device_connection_create(int id,
                                               struct xrt_device *xdev,
                                               struct t_constellation_tracked_device_callbacks *cb,
                                               struct t_constellation_tracker *tracker)
{
	DRV_TRACE_MARKER();

	assert(xdev != NULL);
	assert(cb != NULL);

	struct t_constellation_tracked_device_connection *ctdc =
	    calloc(1, sizeof(struct t_constellation_tracked_device_connection));

	ctdc->id = id;
	ctdc->xdev = xdev;
	ctdc->cb = cb;
	ctdc->tracker = tracker;

	/* Init 2 references - one for the tracked device, one for the tracker */
	xrt_reference_inc(&ctdc->ref);
	xrt_reference_inc(&ctdc->ref);

	int ret = os_mutex_init(&ctdc->lock);
	if (ret != 0) {
		CT_ERROR(tracker, "Constellation tracker device connection: Failed to init mutex!");
		constellation_tracked_device_connection_destroy(ctdc);
		return NULL;
	}

	return ctdc;
}

struct t_constellation_tracked_device_connection *
t_constellation_tracker_add_device(struct t_constellation_tracker *ct,
                                   struct xrt_device *xdev,
                                   struct t_constellation_tracked_device_callbacks *cb)
{
	os_mutex_lock(&ct->tracked_device_lock);
	assert(ct->num_devices < CONSTELLATION_MAX_DEVICES);

	CT_DEBUG(ct, "Constellation tracker: Adding device %d", ct->num_devices);

	struct t_constellation_tracked_device_connection *ctdc =
	    constellation_tracked_device_connection_create(ct->num_devices, xdev, cb, ct);
	if (ctdc != NULL) {
		struct constellation_tracker_device *device = ct->devices + ct->num_devices;
		device->connection = ctdc;
		device->last_matched_cam = -1;
		constellation_track_init(&device->track);
		ct->num_devices++;

		const char *device_type;
		switch (xdev->device_type) {
		case XRT_DEVICE_TYPE_HMD: device_type = "HMD"; device->alias = "hmd"; break;
		case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: device_type = "Right"; device->alias = "right"; break;
		case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: device_type = "Left"; device->alias = "left"; break;
		case XRT_DEVICE_TYPE_ANY_HAND_CONTROLLER: device_type = "Any"; device->alias = "any"; break;
		case XRT_DEVICE_TYPE_GENERIC_TRACKER: device_type = "Tracker"; device->alias = "tracker"; break;
		default: device_type = "Unknown"; device->alias = "unknown"; break;
		}

		snprintf(device->name, sizeof(device->name), "Device %u - %s", ct->num_devices, device_type);
		u_var_add_ro_text(ct, "Device", device->name);
		u_var_add_pose(ct, &device->last_seen_pose, "Last observed global pose");
		u_var_add_u64(ct, &device->last_seen_pose_ts, "Last observed pose");
		u_var_add_ro_i32(ct, &device->last_matched_blobs, "Last matched Blobs");
		u_var_add_ro_i32(ct, &device->last_matched_cam, "Last observed camera #");
		u_var_add_pose(ct, &device->last_matched_cam_pose, "Last observed camera pose");
		u_var_add_ro_u64(ct, &device->diagnostics.published, "Published optical poses");
		u_var_add_ro_u64(ct, &device->diagnostics.shared_blob, "Rejected: shared blobs");
	}

	os_mutex_unlock(&ct->tracked_device_lock);
	return ctdc;
}

void
t_constellation_tracked_device_connection_disconnect(struct t_constellation_tracked_device_connection *ctdc)
{
	os_mutex_lock(&ctdc->lock);
	ctdc->disconnected = true;
	os_mutex_unlock(&ctdc->lock);

	if (xrt_reference_dec_and_is_zero(&ctdc->ref)) {
		constellation_tracked_device_connection_destroy(ctdc);
	}
}
