/*
 * Camera distortion/projection model
 * Copyright 2014-2015 Philipp Zabel
 * Copyright 2019-2023 Jan Schmidt
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
#pragma once

#include "tracking/t_camera_models.h"

struct camera_model
{
	/* Frame width and height */
	int width;
	int height;

	/* Distortion / projection parameters */
	struct t_camera_model_params calib;
	bool fisheye62_valid;
	float fisheye62_radial[6], fisheye62_p1, fisheye62_p2;
};

/* Native six-radial/two-tangential fisheye model. Work in the angular plane so
 * inversion stays finite at the wide sensor edges, unlike view-plane x/z. */
static inline void
camera_model_fisheye62(const struct camera_model *camera, double x, double y, double *u, double *v, double jacobian[4])
{
	double r2 = x * x + y * y, radial = 1, derivative = 0, power = 1;
	for (unsigned i = 0; i < 6; ++i) {
		derivative += (i + 1) * camera->fisheye62_radial[i] * power;
		power *= r2;
		radial += camera->fisheye62_radial[i] * power;
	}
	// The two tangential coefficients act on X/Y after radial distortion.
	double p1 = camera->fisheye62_p1, p2 = camera->fisheye62_p2;
	double rx = x * radial, ry = y * radial, rr2 = rx * rx + ry * ry;
	double tangent = 2 * (p1 * rx + p2 * ry);
	*u = rx * (1 + tangent) + p1 * rr2;
	*v = ry * (1 + tangent) + p2 * rr2;
	if (jacobian) {
		double rxx = radial + 2 * x * x * derivative;
		double rxy = 2 * x * y * derivative;
		double ryy = radial + 2 * y * y * derivative;
		double txx = 1 + 6 * p1 * rx + 2 * p2 * ry;
		double txy = 2 * p1 * ry + 2 * p2 * rx;
		double tyy = 1 + 2 * p1 * rx + 6 * p2 * ry;
		// Row-major J_tangential * J_radial is generally not symmetric.
		jacobian[0] = txx * rxx + txy * rxy;
		jacobian[1] = txx * rxy + txy * ryy;
		jacobian[2] = txy * rxx + tyy * rxy;
		jacobian[3] = txy * rxy + tyy * ryy;
	}
}

static inline bool
camera_model_project(const struct camera_model *camera, float x, float y, float z, float *u, float *v)
{
	*u = *v = NAN;
	if (!camera->fisheye62_valid)
		return t_camera_models_project(&camera->calib, x, y, z, u, v) && isfinite(*u) && isfinite(*v);
	if (!isfinite(x) || !isfinite(y) || !isfinite(z) || z <= 0)
		return false;
	double r = hypot(x, y), scale = r > 1e-12 ? atan2(r, z) / r : 1 / z;
	double a, b;
	camera_model_fisheye62(camera, x * scale, y * scale, &a, &b, NULL);
	*u = camera->calib.fx * a + camera->calib.cx;
	*v = camera->calib.fy * b + camera->calib.cy;
	return isfinite(*u) && isfinite(*v);
}

static inline bool
camera_model_unproject(const struct camera_model *camera, float u, float v, float *x, float *y, float *z)
{
	*x = *y = *z = NAN;
	if (!isfinite(u) || !isfinite(v) || !(camera->calib.fx > 0) || !(camera->calib.fy > 0))
		return false;
	if (!camera->fisheye62_valid)
		return t_camera_models_unproject(&camera->calib, u, v, x, y, z) && isfinite(*x) && isfinite(*y) &&
		       isfinite(*z);
	double target_x = (u - camera->calib.cx) / camera->calib.fx;
	double target_y = (v - camera->calib.cy) / camera->calib.fy;
	double a = target_x, b = target_y;
	for (unsigned iteration = 0; iteration < 20; ++iteration) {
		double px, py, j[4];
		camera_model_fisheye62(camera, a, b, &px, &py, j);
		double ex = px - target_x, ey = py - target_y;
		if (hypot(ex, ey) < 1e-9)
			break;
		double det = j[0] * j[3] - j[1] * j[2];
		if (!isfinite(det) || det <= 1e-12)
			return false;
		a -= (j[3] * ex - j[1] * ey) / det;
		b -= (j[0] * ey - j[2] * ex) / det;
	}
	double px, py;
	camera_model_fisheye62(camera, a, b, &px, &py, NULL);
	double theta = hypot(a, b);
	if (!isfinite(theta) || theta >= M_PI || hypot(px - target_x, py - target_y) > 1e-7)
		return false;
	double scale = theta > 1e-12 ? sin(theta) / theta : 1;
	*x = a * scale;
	*y = b * scale;
	*z = cos(theta);
	return true;
}

static inline bool
camera_model_undistort(const struct camera_model *camera, float u, float v, float *x, float *y)
{
	float xp, yp, z;
	*x = *y = NAN;
	if (!camera_model_unproject(camera, u, v, &xp, &yp, &z) || z <= 1e-6f)
		return false;
	*x = xp / z;
	*y = yp / z;
	return isfinite(*x) && isfinite(*y);
}
