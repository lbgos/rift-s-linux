// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0

#include "rift_s_optics.h"

#include <math.h>

#define VIEW_W 1280.0
#define VIEW_H 1440.0

/* Oculus runtime 1.82.0, hmdq 2.1.5, Rift S native 80 Hz, IPD 63 mm:
 * https://risa2000.github.io/hmdgdb/hmd_cfgs/OculusRiftS_Native_R80.html
 * Left eye FOV tangents; the right eye is mirrored.
 */
static const double left_extent = 0.965689;
static const double right_extent = 1.035530;
static const double up_extent = 1.000000;
static const double down_extent = 1.150368;

/* Fitted to the runtime's distortion mesh cache (Meta Horizon runtime 1.208, firmware 2.2), identical for both
 * eyes: panel pixel radius = k0*t + k1*t^3 + k2*t^5 + k3*t^7 + k4*t^9 for tangent radius t. Max fit error is
 * below 0.45 px inside the visible FOV. Monotonic on [0, TAN_MAX].
 */
#define TAN_MAX 1.7
static const double channel_k[3][5] = {
    {828.9789, -281.8288, 92.5256, -18.9440, 1.7376},  // R
    {823.6001, -288.6719, 104.1311, -25.0075, 2.8234}, // G
    {820.1631, -289.5132, 107.2478, -26.7999, 3.1539}, // B
};

/* Left eye lens centre x in the per-eye view for IPDs 53.5, 55.5, ... 73.5 mm. The runtime shifts the lens
 * centre with IPD ("eye shift swim"); y is the same for all of them. The right eye is the mirror image.
 * Firmware block 0x12 centres differ from these by ~2 px and are not what the runtime renders with.
 */
#define CENTER_IPD_MIN_MM 53.5
#define CENTER_IPD_STEP_MM 2.0
#define CENTER_COUNT 11
static const double left_center_x[CENTER_COUNT] = {
    605.495, 605.366, 605.671, 606.293, 607.116, 608.039, 608.962, 609.785, 610.407, 610.712, 610.583,
};
#define CENTER_Y 719.992

static double
radius_px(const double *k, double t)
{
	const double t2 = t * t;
	return t * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * (k[3] + t2 * k[4]))));
}

static double
radius_px_slope(const double *k, double t)
{
	const double t2 = t * t;
	return k[0] + t2 * (3 * k[1] + t2 * (5 * k[2] + t2 * (7 * k[3] + t2 * 9 * k[4])));
}

/* Inverts radius_px on [0, TAN_MAX] with a bracketed Newton iteration. Beyond the range it continues linearly;
 * those points are outside the FOV anyway.
 */
static double
tangent_radius(const double *k, double r)
{
	const double r_max = radius_px(k, TAN_MAX);
	if (r >= r_max) {
		return TAN_MAX + (r - r_max) / radius_px_slope(k, TAN_MAX);
	}
	double lo = 0, hi = TAN_MAX;
	double t = r / k[0];
	for (int i = 0; i < 50; i++) {
		if (!(t > lo && t < hi)) {
			t = 0.5 * (lo + hi);
		}
		const double err = radius_px(k, t) - r;
		if (fabs(err) < 1e-9) {
			break;
		}
		if (err > 0) {
			hi = t;
		} else {
			lo = t;
		}
		t -= err / radius_px_slope(k, t);
	}
	return t;
}

/* Piecewise-linear lookup, clamped to the table's IPD range. */
static double
lens_center_x(double ipd_mm)
{
	double pos = (ipd_mm - CENTER_IPD_MIN_MM) / CENTER_IPD_STEP_MM;
	pos = fmin(fmax(pos, 0.0), CENTER_COUNT - 1);
	const int i = pos < CENTER_COUNT - 1 ? (int)pos : CENTER_COUNT - 2;
	return left_center_x[i] + (pos - i) * (left_center_x[i + 1] - left_center_x[i]);
}

bool
rift_s_optics_init(struct rift_s_optics *optics, uint32_t eye, float ipd_m, float fov_scale, float fov_scale_y)
{
	if (eye > 1 || !isfinite(ipd_m) || !isfinite(fov_scale) || fov_scale <= 0 || !isfinite(fov_scale_y) ||
	    fov_scale_y <= 0) {
		return false;
	}
	const float left = (eye == 0 ? left_extent : right_extent) * fov_scale;
	const float right = (eye == 0 ? right_extent : left_extent) * fov_scale;
	const float up = up_extent * fov_scale * fov_scale_y;
	const float down = down_extent * fov_scale * fov_scale_y;
	const struct xrt_fov fov = {
	    .angle_left = -atanf(left),
	    .angle_right = atanf(right),
	    .angle_up = atanf(up),
	    .angle_down = -atanf(down),
	};
	/* Reject scales that overflow or round a projection angle to pi/2. */
	if (!(tanf(fov.angle_left) < 0 && tanf(fov.angle_right) > 0 && tanf(fov.angle_up) > 0 &&
	      tanf(fov.angle_down) < 0)) {
		return false;
	}
	const double cx = lens_center_x(ipd_m * 1000.0);
	*optics = (struct rift_s_optics){
	    .fov = fov,
	    .lens_center_px = {eye == 0 ? cx : VIEW_W - cx, CENTER_Y},
	    .tan_left = -tanf(fov.angle_left),
	    .tan_right = tanf(fov.angle_right),
	    .tan_up = tanf(fov.angle_up),
	    .tan_down = -tanf(fov.angle_down),
	};
	return true;
}

void
rift_s_optics_compute(const struct rift_s_optics *optics, float u, float v, struct xrt_uv_triplet *result)
{
	/* UVs are in the upright per-eye view, top-left origin and Y down.
	 * driver_monado/compositor already apply the physical panel rotation.
	 * Out-of-FOV samples are normal near the corners and are not clamped.
	 */
	const double dx = u * VIEW_W - optics->lens_center_px.x;
	const double dy = v * VIEW_H - optics->lens_center_px.y;
	const double r = hypot(dx, dy);
	const double width = (double)optics->tan_left + optics->tan_right;
	const double height = (double)optics->tan_up + optics->tan_down;
	struct xrt_vec2 *rgb[3] = {&result->r, &result->g, &result->b};
	for (int i = 0; i < 3; i++) {
		const double scale = r > 0 ? tangent_radius(channel_k[i], r) / r : 0.0;
		rgb[i]->x = (float)((dx * scale + optics->tan_left) / width);
		rgb[i]->y = (float)((dy * scale + optics->tan_up) / height);
	}
}
