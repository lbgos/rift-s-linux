// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once

#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

/*!
 * Rift S lens model fitted to the Oculus runtime's own distortion mesh.
 *
 * Each colour channel is purely radial around the lens centre:
 * panel pixel radius = odd polynomial of the tangent-space radius.
 */
struct rift_s_optics
{
	struct xrt_fov fov;
	//! Lens centre in the upright 1280x1440 per-eye view, top-left origin, Y down.
	struct xrt_vec2 lens_center_px;
	//! Positive tangents of the published fov, used to map tangent space to UV.
	float tan_left, tan_right, tan_up, tan_down;
};

/*!
 * @param eye 0 left, 1 right
 * @param ipd_m Configured eye separation in metres; shifts the lens centre like the Oculus runtime does
 * @param fov_scale Scales all projection tangents (changes the fov only, not the lens)
 * @param fov_scale_y Additionally scales the vertical tangents
 */
bool
rift_s_optics_init(struct rift_s_optics *optics, uint32_t eye, float ipd_m, float fov_scale, float fov_scale_y);

/*!
 * @param u,v Position in the upright per-eye view, top-left origin, Y down, in 0..1
 * @param result Per-channel UV into the eye's rendered image
 */
void
rift_s_optics_compute(const struct rift_s_optics *optics, float u, float v, struct xrt_uv_triplet *result);
