// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once

#include "blobwatch.h"
#include "camera_model.h"
#include "tracking/t_led_models.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONSTELLATION_CONSTRAINED_MAX_VIEWS 5
#define CONSTELLATION_CONSTRAINED_MAX_ASSIGNMENTS (CONSTELLATION_CONSTRAINED_MAX_VIEWS * MAX_BLOBS_PER_FRAME)

/* Both transforms use the camera's optical (+Z forward) coordinates. */
struct constellation_constrained_view
{
	unsigned camera_index;
	struct xrt_pose P_world_cam;
	struct xrt_pose P_cam_world;
	const struct camera_model *calib;
	const struct blobservation *observation;
	//! Same-exposure published claims to omit from acquisition; temporal LED labels stay advisory.
	bool excluded_blobs[MAX_BLOBS_PER_FRAME];
};

enum constellation_constrained_origin
{
	CONSTELLATION_CONSTRAINED_GRAVITY,
	CONSTELLATION_CONSTRAINED_HEADING,
	CONSTELLATION_CONSTRAINED_PRIOR,
	CONSTELLATION_CONSTRAINED_P3P,
};

enum constellation_constrained_rejection
{
	CONSTELLATION_CONSTRAINED_ACCEPTED,
	CONSTELLATION_CONSTRAINED_INVALID_INPUT,
	CONSTELLATION_CONSTRAINED_NO_PAIR,
	CONSTELLATION_CONSTRAINED_TOO_FEW_INLIERS,
	CONSTELLATION_CONSTRAINED_RANK_DEFICIENT,
	CONSTELLATION_CONSTRAINED_UNCERTAIN,
	CONSTELLATION_CONSTRAINED_AMBIGUOUS,
	CONSTELLATION_CONSTRAINED_SHARED_BLOB,
	//! Cold fit fails the angular goodness-of-fit test or leaves blobs unexplained inside its footprint.
	CONSTELLATION_CONSTRAINED_INCONSISTENT,
};

//! Outcome of one minimal two-point solve, so degenerate geometry is never silently resolved.
enum constellation_constrained_solver_status
{
	CONSTELLATION_SOLVER_OK,
	CONSTELLATION_SOLVER_INVALID_INPUT,
	//! Bearings closer than the minimum separation (or anti-parallel).
	CONSTELLATION_SOLVER_DEGENERATE_BEARINGS,
	//! The two model points coincide.
	CONSTELLATION_SOLVER_DEGENERATE_MODEL,
	//! The pair carries no heading information: every yaw explains it equally.
	CONSTELLATION_SOLVER_UNRESOLVED_YAW,
	//! No real yaw root exists for this assignment.
	CONSTELLATION_SOLVER_NO_ROOT,
	//! Roots exist but put an LED behind the camera or leave translation rank deficient.
	CONSTELLATION_SOLVER_NONPOSITIVE_DEPTH,
};

struct constellation_constrained_config
{
	//! Object-space score gate in metres (stock default, not an active setting).
	double score_gate_m;
	//! Broad object-space association gate in metres for candidate edges.
	double association_gate_m;
	/* With a trusted prior, hypotheses whose model origin lands farther than this from the prior
	 * position are discarded. The prior only selects candidate edges; without this bound a
	 * hypothesis anywhere in space could win on inlier count and either be published as a wrong
	 * identity or make the correct one look ambiguous. */
	double maximum_prior_distance_m;
	//! Bound association rotation only while a confirmed track has a verified heading.
	double maximum_prior_rotation_rad;
	/* Refinement may change the hypothesis' tilt (from the IMU, or from P3P) under a Gaussian
	 * prior of this standard deviation, and never by more than the maximum: the accelerometer and
	 * the attitude history at the exposure time are off by a few degrees, which otherwise fails
	 * the goodness-of-fit test on well-observed rings. Zero disables the adjustment, and fits of
	 * fewer distinct LEDs than the minimum keep the prior tilt. */
	double tilt_prior_std_rad;
	double maximum_tilt_correction_rad;
	unsigned minimum_tilt_distinct_leds;
	double minimum_bearing_separation_rad;
	double minimum_depth_m;
	double maximum_depth_m;
	/* Proposed noise/uncertainty policy, not device calibration. */
	double ray_noise_std_m;
	double maximum_position_std_m;
	double maximum_yaw_std_rad;
	//! Minimum likelihood gap (chi-square units of ray_noise_std_m) to the best competing assignment.
	double ambiguity_chi_square;
	/* Monado verification policy, beyond the stock scorer count (inliers > sample size):
	 * accepted observations required when acquiring without a trusted position prior, and the
	 * distinct physical LEDs among them. Repeated views of one LED do not fix its identity. */
	unsigned minimum_cold_inliers;
	unsigned minimum_cold_distinct_leds;
	/* The same counts for searches around a trusted prior. The stock minimum (inliers > sample
	 * size, so 2 for the retained prior) let one stale prior keep re-verifying itself on two blobs
	 * in worn use; a published pose now needs this much evidence on every path. */
	unsigned minimum_tracking_inliers;
	unsigned minimum_tracking_distinct_leds;
	/* Unmatched blobs within conflict_gate_m (object space) of an LED the hypothesis predicts
	 * visible, ignoring fragments next to an accepted LED. A wrong fit typically puts predicted
	 * LEDs next to the ring's own blobs without explaining them. Applies on every path. */
	unsigned maximum_cold_unexplained_blobs;
	double conflict_gate_m;
	/* Goodness of fit on every path: angular residuals divided by this centroid bearing noise
	 * must pass a chi-square test at the given standard-normal quantile (Wilson-Hilferty). The noise
	 * is a proposed value pending measurement from replayed frames; zero disables the test. */
	double bearing_noise_std_rad;
	double cold_fit_quantile_z;
	//! Cap on minimal-sample pair trials without a trusted position prior.
	unsigned max_pair_trials;
	//! Cap on pair trials drawn from edges near a trusted position prior.
	unsigned max_prior_pair_trials;
	unsigned time_budget_us;
	//! Unused by the deterministic enumeration; kept so callers can record the exposure identity.
	uint64_t seed;
};

struct constellation_constrained_assignment
{
	unsigned camera_index;
	unsigned blob_index;
	unsigned led_index;
	uint8_t led_id;
	double residual_m;
};

struct constellation_constrained_result
{
	struct xrt_pose P_world_model;
	enum constellation_constrained_origin origin;
	enum constellation_constrained_rejection rejection;
	unsigned candidate_observations;
	unsigned inliers;
	unsigned distinct_leds;
	unsigned projected_visible_leds;
	unsigned unexplained_blobs;
	//! Chi-square of angular residuals and its threshold for the selected hypothesis.
	double fit_chi_square;
	double fit_chi_square_limit;
	unsigned pair_trials;
	unsigned hypotheses;
	unsigned gravity_hypotheses;
	unsigned heading_hypotheses;
	unsigned degenerate_pairs;
	unsigned unresolved_yaw_pairs;
	unsigned no_root_pairs;
	unsigned depth_pairs;
	unsigned seed_rejections;
	unsigned insufficient_hypotheses;
	unsigned invisible_rejections;
	unsigned depth_rejections;
	//! Trusted-prior hypotheses discarded for landing outside maximum_prior_distance_m.
	unsigned prior_distance_rejections;
	unsigned prior_rotation_rejections;
	unsigned num_camera_diagnostics;
	struct
	{
		unsigned camera_index;
		unsigned detected_blobs;
		unsigned eligible_blobs;
		unsigned projected_visible_leds;
		unsigned candidate_observations;
		unsigned inliers;
	} camera_diagnostics[CONSTELLATION_CONSTRAINED_MAX_VIEWS];
	bool budget_exhausted;
	/* Cost is clipped object-space RMS in metres, normalized by candidates - sample size. */
	double cost_m;
	double second_best_cost_m;
	double ambiguity_likelihood_gap;
	unsigned information_rank;
	/* Row-major [world translation, yaw about world +Y]; absent yaw has infinite variance. */
	double covariance[16];
	//! True only when the optical fit itself constrains yaw within maximum_yaw_std_rad.
	bool vision_determines_yaw;
	//! How far refinement moved the tilt from the hypothesis' prior tilt.
	double tilt_correction_rad;
	unsigned num_assignments;
	struct constellation_constrained_assignment assignments[CONSTELLATION_CONSTRAINED_MAX_ASSIGNMENTS];
};

void
constellation_constrained_default_config(struct constellation_constrained_config *config);

const char *
constellation_constrained_origin_name(enum constellation_constrained_origin origin);

const char *
constellation_constrained_rejection_name(enum constellation_constrained_rejection rejection);

const char *
constellation_constrained_solver_status_name(enum constellation_constrained_solver_status status);

/* Enumerate every real same-camera yaw/depth root for a gravity-aligned tilt. Returns the
 * number of poses (model to camera). Zero roots always come with a non-OK status. */
unsigned
constellation_constrained_gravity_two_point(const struct xrt_vec3 points[2],
                                            const struct xrt_vec3 bearings[2],
                                            const struct xrt_quat *tilt_model_to_cam,
                                            const struct xrt_vec3 *gravity_cam,
                                            double minimum_bearing_separation_rad,
                                            struct xrt_pose out_poses[2],
                                            enum constellation_constrained_solver_status *status);

/* Translation from two bearings with a known model-to-camera rotation. */
bool
constellation_constrained_heading_two_point(const struct xrt_vec3 points[2],
                                            const struct xrt_vec3 bearings[2],
                                            const struct xrt_quat *model_to_cam,
                                            double minimum_bearing_separation_rad,
                                            struct xrt_pose *out_pose,
                                            enum constellation_constrained_solver_status *status);

/* Low-count acquisition. Gravity-constrained two-point hypotheses are always generated from
 * the tilt of prior_world. With reliable_heading, known-orientation translation hypotheses are
 * added. With trusted_position, pairs come from edges near the prior and, if heading is also
 * reliable, the prior pose itself is scored as a one-point hypothesis. */
bool
constellation_constrained_search(struct t_constellation_led_model *model,
                                 const struct constellation_constrained_view *views,
                                 unsigned num_views,
                                 const struct xrt_pose *prior_world,
                                 bool reliable_heading,
                                 bool trusted_position,
                                 const struct constellation_constrained_config *config,
                                 struct constellation_constrained_result *result);

/* Jointly verify/refine an existing hypothesis. Its sample size follows origin:
 * prior 1, gravity/heading 2, P3P 3. P3P hypotheses are cold acquisitions and also get the
 * cold verification policy. */
bool
constellation_constrained_verify_pose(struct t_constellation_led_model *model,
                                      const struct constellation_constrained_view *views,
                                      unsigned num_views,
                                      const struct xrt_pose *pose_world,
                                      enum constellation_constrained_origin origin,
                                      const struct constellation_constrained_config *config,
                                      struct constellation_constrained_result *result);

bool
constellation_constrained_results_share_blob(const struct constellation_constrained_result *a,
                                             const struct constellation_constrained_result *b);

//! What kind of claim on a shared blob blocked a candidate.
enum constellation_constrained_claim
{
	CONSTELLATION_CLAIM_NONE,
	//! Another device already published a pose using the blob in this exposure.
	CONSTELLATION_CLAIM_PUBLISHED,
	//! Another candidate explains the blob and neither has clearly more support.
	CONSTELLATION_CLAIM_CANDIDATE,
	//! Another device's rejected hypothesis passed the consistency test and explains at least as
	//! many observations.
	CONSTELLATION_CLAIM_REJECTED,
};

/* Cross-device ownership. A candidate whose observations share a blob with another device's
 * claim loses (candidates[i] cleared, blocked[i] set) when the claim is a published pose; when it
 * is another candidate that does not have clearly less support (minimum_support_margin more
 * inliers and no fewer distinct LEDs wins); or when it is a competitive rejected hypothesis, that
 * is one rejected only for count, ambiguity or uncertainty with at least as many inliers.
 * Hypotheses rejected as inconsistent or without a pair are not evidence of ownership.
 * Published results are never revoked. */
void
constellation_constrained_arbitrate(const struct constellation_constrained_result *const *results,
                                    const bool *published,
                                    bool *candidates,
                                    enum constellation_constrained_claim *blocked,
                                    unsigned count,
                                    unsigned minimum_support_margin);

// Optional blocker indices name the claim actually responsible for rejection; count means none.
void
constellation_constrained_arbitrate_with_owners(const struct constellation_constrained_result *const *results,
                                                const bool *published,
                                                bool *candidates,
                                                enum constellation_constrained_claim *blocked,
                                                unsigned *blockers,
                                                unsigned count,
                                                unsigned minimum_support_margin);

const char *
constellation_constrained_claim_name(enum constellation_constrained_claim claim);

/* True when a candidate blocked by a published pose (CONSTELLATION_CLAIM_PUBLISHED) shares a blob
 * with it and has clearly more support by the same margin rule. The published pose stays
 * published, but its track should end: a lock on the other controller's ring otherwise keeps
 * that controller's blobs forever. */
bool
constellation_constrained_contests_published(const struct constellation_constrained_result *candidate,
                                             const struct constellation_constrained_result *published,
                                             unsigned minimum_support_margin);

/* One accepted optical pose with the IMU attitude at the same capture time, both in world. Read
 * both points' IMU attitudes from the same, current attitude history: a world-yaw correction
 * applied in between then cancels instead of looking like an optical jump. */
struct constellation_track_point
{
	struct xrt_pose P_world_model;
	struct xrt_quat imu_world;
	int64_t capture_ns;
};

/* Monado policy for publishing a pose after an earlier one. A hand can move fast, but an optical
 * identity error moves the estimate by about one LED spacing or flips it onto the other ring
 * without the controller's gyro seeing the rotation. */
struct constellation_motion_limits
{
	double max_speed_mps;
	double position_slack_m;
	/* Bound on the difference between the optical and the IMU rotation over the same interval,
	 * both expressed in the controller's own frame. The IMU's world yaw may be far from the
	 * optical one (the driver corrects it gradually); a world-frame comparison would then see
	 * every wrist rotation about a non-vertical axis as an error. */
	double max_rotation_error_rad;
	//! Older reference poses do not constrain the new one.
	int64_t max_gap_ns;
};

void
constellation_constrained_default_motion_limits(struct constellation_motion_limits *limits);

/* True when next is a plausible continuation of prev. Outputs the position change beyond the
 * allowed motion (<= 0 when inside) and the body-frame rotation disagreement between optics and
 * IMU. */
bool
constellation_constrained_motion_consistent(const struct constellation_track_point *prev,
                                            const struct constellation_track_point *next,
                                            const struct constellation_motion_limits *limits,
                                            double *position_excess_m,
                                            double *rotation_error_rad);

/*!
 * Publication state of one device. A pose is published only when it continues an earlier one: a
 * cold acquisition waits for a consistent second result, and while tracking every result must be a
 * plausible continuation of the last published pose. Missing a pose is preferred over publishing a
 * wrong one. This is Monado policy.
 */
enum constellation_track_state
{
	CONSTELLATION_TRACK_LOST,
	//! One unpublished result is waiting for a consistent second one.
	CONSTELLATION_TRACK_CONFIRMING,
	CONSTELLATION_TRACK_TRACKING,
	CONSTELLATION_TRACK_STATE_COUNT,
};

enum constellation_track_verdict
{
	CONSTELLATION_TRACK_PUBLISH,
	//! First result after loss; held until a consistent second result confirms it.
	CONSTELLATION_TRACK_HOLD_CONFIRMING,
	//! Inconsistent with the held result; it replaces that result as the one to confirm.
	CONSTELLATION_TRACK_HOLD_UNCONFIRMED,
	//! Inconsistent with the published track; the track is kept.
	CONSTELLATION_TRACK_HOLD_JUMP,
	//! Repeatedly inconsistent: the track ended and this result is held for confirmation.
	CONSTELLATION_TRACK_HOLD_RESTART,
	//! Not newer than the reference.
	CONSTELLATION_TRACK_HOLD_STALE,
};

struct constellation_track
{
	enum constellation_track_state state;
	//! Last published pose while tracking, or the result awaiting confirmation.
	struct constellation_track_point ref;
	//! Last published location remains a depth/position constraint during re-confirmation.
	struct constellation_track_point published;
	bool have_published;
	//! The reference orientation came from optics rather than echoing the IMU prior.
	bool ref_orientation_observed;
	unsigned consecutive_jumps;
	//! Consecutive jumps that end a track.
	unsigned max_jumps;
	struct constellation_motion_limits limits;
};

void
constellation_track_init(struct constellation_track *track);

//! End the track now; the next verified result must confirm itself again.
void
constellation_track_reset(struct constellation_track *track);

//! End a track whose reference is too old to vouch for anything. Returns true when it ended.
bool
constellation_track_expire(struct constellation_track *track, int64_t capture_ns);

/* Decide whether a verified result may be published and advance the state. next->imu_world and
 * ref_imu_world (the IMU attitude at track->ref.capture_ns) must be read from the same, current
 * attitude history; ref_imu_world may be NULL when unavailable, which makes the result
 * discontinuous. A result whose orientation was not observed is compared with the IMU's own. */
enum constellation_track_verdict
constellation_track_update(struct constellation_track *track,
                           const struct constellation_track_point *next,
                           bool orientation_observed,
                           const struct xrt_quat *ref_imu_world,
                           double *position_excess_m,
                           double *rotation_error_rad);

const char *
constellation_track_state_name(enum constellation_track_state state);

const char *
constellation_track_verdict_name(enum constellation_track_verdict verdict);

/* Select jointly verified P3P candidates with the same discrete-assignment ambiguity test as acquisition. */
bool
constellation_constrained_select_results(const struct constellation_constrained_result *results,
                                         unsigned count,
                                         const struct constellation_constrained_config *config,
                                         struct constellation_constrained_result *out);

#ifdef __cplusplus
}
#endif
