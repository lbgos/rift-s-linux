// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once
#include "rift_s_firmware.h"
#include "xrt/xrt_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

struct rift_s_world_anchor;
#define RIFT_S_WORLD_MAX_CAMERAS 5
struct rift_s_world_anchor_config
{
	// Raw sensor calibration in tracker camera order. Native camera axes are +Y down, +Z forward.
	struct rift_s_camera_calibration camera[RIFT_S_WORLD_MAX_CAMERAS];
	struct xrt_pose imu_from_camera[RIFT_S_WORLD_MAX_CAMERAS];
	uint32_t camera_count; // Zero selects the legacy two-camera rig.
	// Bind a persisted map to the committed boundary. Empty path resolves Steam's registry.
	bool require_boundary;
	char boundary_path[1024];
};
struct rift_s_world_anchor_observation
{
	struct xrt_vec3 world_point;
	struct xrt_vec2 pixel;
};
struct rift_s_world_anchor_result
{
	struct xrt_pose world_from_imu;
	uint32_t features[RIFT_S_WORLD_MAX_CAMERAS], matches[RIFT_S_WORLD_MAX_CAMERAS],
	    inliers[RIFT_S_WORLD_MAX_CAMERAS], camera_points[RIFT_S_WORLD_MAX_CAMERAS], keyframes;
	uint32_t supporting_cameras;
	uint32_t stereo_matches, triangulated;
	float max_reprojection_px;
	const char *rejection; // Static diagnostic string, NULL on success.
	bool recorded, relocalized;
};

// Opt-in bench recorder. Writes at most 600 synchronized camera sets and capture-time SLAM poses.
// RIFT_S_WORLD_RECORD_DIR is honored only with MONADO_STEAMVR_BENCH enabled.
void
rift_s_world_anchor_record_frames(struct rift_s_world_anchor *anchor,
                                  const struct xrt_frame *left,
                                  const struct xrt_frame *right,
                                  const struct xrt_pose *raw_from_imu);

void
rift_s_world_anchor_record_rig_frames(struct rift_s_world_anchor *anchor,
                                      const struct xrt_frame *const frames[],
                                      uint32_t camera_count,
                                      const struct xrt_pose *raw_from_imu);
bool
rift_s_world_anchor_process_rig(struct rift_s_world_anchor *anchor,
                                const struct xrt_frame *const frames[],
                                uint32_t camera_count,
                                const struct xrt_pose *trusted_world_from_imu,
                                struct rift_s_world_anchor_result *result);
// Recovery fixes roll/pitch to the exposure-time VIO attitude and solves only yaw/translation.
bool
rift_s_world_anchor_process_rig_gravity(struct rift_s_world_anchor *anchor,
                                        const struct xrt_frame *const frames[],
                                        uint32_t camera_count,
                                        const struct xrt_pose *trusted_world_from_imu,
                                        const struct xrt_quat *raw_attitude,
                                        struct rift_s_world_anchor_result *result);

// Worker-owned: none of these functions are thread safe. No frame references are retained.
// An absent/invalid file creates an empty map, never a nominal world alignment.
struct rift_s_world_anchor *
rift_s_world_anchor_create(const struct rift_s_world_anchor_config *config, const char *path);
void
rift_s_world_anchor_destroy(struct rift_s_world_anchor *anchor);
bool
rift_s_world_anchor_has_map(const struct rift_s_world_anchor *anchor);
// A committed standing frame must not be paired with a fresh, arbitrary VIO frame, even if
// its visual map is missing or rejected. Only room setup may establish a replacement frame.
bool
rift_s_world_anchor_has_boundary(const struct rift_s_world_anchor *anchor);
// Clear an in-memory draft, without changing the saved map. Reload restores it after Cancel.
void
rift_s_world_anchor_clear(struct rift_s_world_anchor *anchor);
// Save also accepts an empty draft, atomically invalidating the old world after a new calibration.
bool
rift_s_world_anchor_save(struct rift_s_world_anchor *anchor);
bool
rift_s_world_anchor_reload(struct rift_s_world_anchor *anchor);
// Non-null trusted_world_from_imu records stereo keyframes. NULL only attempts recovery.
// Pose must correspond to the exposure timestamp, not the latest predicted head pose.
bool
rift_s_world_anchor_process(struct rift_s_world_anchor *anchor,
                            const struct xrt_frame *left,
                            const struct xrt_frame *right,
                            const struct xrt_pose *trusted_world_from_imu,
                            struct rift_s_world_anchor_result *result);

// Geometry shared by recording/recovery and focused calibration tests.
bool
rift_s_world_anchor_project(const struct rift_s_camera_calibration *camera,
                            const struct xrt_vec3 *native_point,
                            struct xrt_vec2 *pixel);
bool
rift_s_world_anchor_unproject(const struct rift_s_camera_calibration *camera,
                              const struct xrt_vec2 *pixel,
                              struct xrt_vec3 *native_ray);
bool
rift_s_world_anchor_solve(const struct rift_s_world_anchor_config *config,
                          const struct rift_s_world_anchor_observation *observations[2],
                          const uint32_t counts[2],
                          struct rift_s_world_anchor_result *result);
bool
rift_s_world_anchor_solve_rig(const struct rift_s_world_anchor_config *config,
                              const struct rift_s_world_anchor_observation *const observations[],
                              const uint32_t counts[],
                              uint32_t camera_count,
                              struct rift_s_world_anchor_result *result);
bool
rift_s_world_anchor_solve_rig_gravity(const struct rift_s_world_anchor_config *config,
                                      const struct rift_s_world_anchor_observation *const observations[],
                                      const uint32_t counts[],
                                      uint32_t camera_count,
                                      const struct xrt_quat *raw_attitude,
                                      struct rift_s_world_anchor_result *result);
#ifdef __cplusplus
}
#endif
