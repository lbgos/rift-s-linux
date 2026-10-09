// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once
#include "xrt/xrt_camera.h"
#include "rift_s_firmware.h"
#ifdef __cplusplus
extern "C" {
#endif
struct xrt_camera *
rift_s_passthrough_create(const struct rift_s_camera_calibration_block *calibration);
void
rift_s_passthrough_push(struct xrt_camera *camera, struct xrt_frame *left, struct xrt_frame *right);
void
rift_s_passthrough_destroy(struct xrt_camera *camera);
/* Project an upright, head-aligned pinhole ray into a native calibrated camera. */
bool
rift_s_passthrough_project(const struct rift_s_camera_calibration *calibration,
                           float x,
                           float y,
                           struct xrt_vec2 *pixel);
#ifdef __cplusplus
}
#endif
