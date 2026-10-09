// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once

#include "xrt/xrt_device.h"

#include <algorithm>
#include <cmath>

// Match the upright eye FOV to the green-channel pixel density at the panel centre.
// Physical panel dimensions and rotation describe the output, not the source texture.
inline void
ovrd_get_recommended_render_target_size(xrt_device *xdev, float scale, uint32_t *width, uint32_t *height)
{
	double max_width = 0, max_height = 0;
	for (uint32_t eye = 0; eye < xdev->hmd->view_count; eye++) {
		const auto &display = xdev->hmd->views[eye].display;
		double view_width = display.w_pixels, view_height = display.h_pixels;
		const auto &fov = xdev->hmd->distortion.fov[eye];
		const double tan_width = std::tan(fov.angle_right) - std::tan(fov.angle_left);
		const double tan_height = std::tan(fov.angle_up) - std::tan(fov.angle_down);
		if (xdev->compute_distortion != nullptr && view_width > 0 && view_height > 0) {
			xrt_uv_triplet left{}, right{}, up{}, down{};
			const float du = 0.5f / view_width, dv = 0.5f / view_height;
			if (xdev->compute_distortion(xdev, eye, 0.5f - du, 0.5f, &left) == XRT_SUCCESS &&
			    xdev->compute_distortion(xdev, eye, 0.5f + du, 0.5f, &right) == XRT_SUCCESS &&
			    xdev->compute_distortion(xdev, eye, 0.5f, 0.5f - dv, &up) == XRT_SUCCESS &&
			    xdev->compute_distortion(xdev, eye, 0.5f, 0.5f + dv, &down) == XRT_SUCCESS) {
				// Each pair spans one upright panel pixel. UV differences become tangent differences.
				const double dx = std::hypot((right.g.x - left.g.x) * tan_width,
				                             (right.g.y - left.g.y) * tan_height);
				const double dy = std::hypot((down.g.x - up.g.x) * tan_width,
				                             (down.g.y - up.g.y) * tan_height);
				const double density = 1.0 / std::min(dx, dy);
				if (std::isfinite(density) && dx > 0 && dy > 0 && tan_width > 0 && tan_height > 0) {
					view_width = tan_width * density;
					view_height = tan_height * density;
				}
			}
		}
		max_width = std::max(max_width, view_width);
		max_height = std::max(max_height, view_height);
	}
	*width = (uint32_t)std::ceil(max_width * scale);
	*height = (uint32_t)std::ceil(max_height * scale);
}

// OpenVR top/bottom are the minimum/maximum Y-up tangents, despite their names.
// Verified against SteamVR's projection matrix and a Rift S headset A/B test.
inline void
ovrd_get_projection_raw(const xrt_fov &fov, float *left, float *right, float *top, float *bottom)
{
	*left = std::tan(fov.angle_left);
	*right = std::tan(fov.angle_right);
	*top = std::tan(fov.angle_down);
	*bottom = std::tan(fov.angle_up);
}
