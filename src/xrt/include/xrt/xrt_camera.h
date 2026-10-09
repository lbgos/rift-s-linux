// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once
#include "xrt/xrt_defines.h"
#include "xrt/xrt_frame.h"

/* Optional local-device camera provider. Frames are rectified RGB8, left/right
 * side by side, with monotonic capture timestamps. Owned by the device; callers
 * release returned frame references. No IPC transport is implied. */
struct xrt_camera
{
	uint32_t view_width, view_height;
	struct xrt_vec2 focal_length, center;
	struct xrt_pose head_from_camera[2];
	void (*set_active)(struct xrt_camera *camera, bool active);
	bool (*get_frame)(struct xrt_camera *camera, struct xrt_frame **out_frame);
};
