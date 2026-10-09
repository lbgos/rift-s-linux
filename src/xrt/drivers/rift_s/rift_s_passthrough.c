// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "rift_s_passthrough.h"
#include "os/os_threading.h"
#include "os/os_time.h"
#include "util/u_frame.h"
#include "util/u_logging.h"
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>

#define VIEW_SIZE 320
#define MAX_AGE_NS (250 * U_TIME_1MS_IN_NS)

struct rift_s_passthrough
{
	struct xrt_camera base;
	struct os_mutex mutex;
	struct os_mutex convert_mutex;
	bool active;
	struct xrt_frame *raw[2];
	struct xrt_frame *converted;
	uint64_t rejected_pairs;
	uint64_t last_diag_ns;
	struct xrt_vec2 map[2][VIEW_SIZE * VIEW_SIZE];
};

static bool
valid_payload(const struct xrt_frame *frame)
{
	return frame && frame->data && frame->format == XRT_FORMAT_L8 && frame->width >= 2 &&
	       frame->height >= 2 && frame->stride >= frame->width && frame->size >= frame->width &&
	       frame->height - 1 <= (frame->size - frame->width) / frame->stride;
}

struct sensor_pixels
{
	uint8_t min, max;
	uint64_t sum;
};

static bool
has_pixels(const struct xrt_frame *frame)
{
	for (uint32_t y = 0; y < frame->height; ++y)
		for (uint32_t x = 0; x < frame->width; ++x)
			if (frame->data[y * frame->stride + x] != 0)
				return true;
	return false;
}

static struct sensor_pixels
measure_pixels(const struct xrt_frame *frame)
{
	struct sensor_pixels pixels = {.min = UINT8_MAX};
	// Inspect image rows, not the transport padding between sensor crops.
	for (uint32_t y = 0; y < frame->height; ++y) {
		const uint8_t *row = frame->data + y * frame->stride;
		for (uint32_t x = 0; x < frame->width; ++x) {
			uint8_t value = row[x];
			pixels.min = value < pixels.min ? value : pixels.min;
			pixels.max = value > pixels.max ? value : pixels.max;
			pixels.sum += value;
		}
	}
	return pixels;
}

bool
rift_s_passthrough_project(const struct rift_s_camera_calibration *c, float x, float y, struct xrt_vec2 *pixel)
{
	/* Firmware DeviceFromCamera maps native +Y down/+Z forward camera
	 * rays into the +Y up/-Z forward headset. Include sensor roll/pitch. */
	const float *m = c->device_from_camera.v;
	double cx = m[0] * x - m[1] * y - m[2];
	double cy = m[4] * x - m[5] * y - m[6];
	double cz = m[8] * x - m[9] * y - m[10];
	if (cz <= 0 || !isfinite(cx) || !isfinite(cy) || !isfinite(cz))
		return false;
	cx /= cz;
	cy /= cz;
	double radius = hypot(cx, cy);
	double theta = atan(radius);
	double theta2 = theta * theta;
	double scale = radius > 1e-8 ? theta / radius : 1;
	double u = cx * scale, v = cy * scale;
	double radial = c->distortion.k[5];
	for (int i = 4; i >= 0; --i)
		radial = radial * theta2 + c->distortion.k[i];
	radial = radial * theta2 + 1;
	double p1 = c->distortion.p1, p2 = c->distortion.p2;
	pixel->x = c->projection.fx * (u * radial + 2 * p1 * u * v + p2 * (theta2 + 2 * u * u)) + c->projection.cx;
	pixel->y = c->projection.fy * (v * radial + 2 * p2 * u * v + p1 * (theta2 + 2 * v * v)) + c->projection.cy;
	return isfinite(pixel->x) && isfinite(pixel->y);
}

static void
set_active(struct xrt_camera *camera, bool active)
{
	struct rift_s_passthrough *p = (struct rift_s_passthrough *)camera;
	os_mutex_lock(&p->mutex);
	p->active = active;
	if (!active)
		for (int i = 0; i < 2; ++i)
			xrt_frame_reference(&p->raw[i], NULL);
	os_mutex_unlock(&p->mutex);
}

void
rift_s_passthrough_push(struct xrt_camera *camera, struct xrt_frame *left, struct xrt_frame *right)
{
	if (!camera)
		return;
	struct rift_s_passthrough *p = (struct rift_s_passthrough *)camera;
	/* This handoff runs on camera ingestion, which also delivers controller
	 * exposures. A slow export consumer must lose a video frame, not block it. */
	if (os_mutex_trylock(&p->mutex) != 0)
		return;
	if (p->active) {
		bool valid = valid_payload(left) && valid_payload(right) && left->width == right->width &&
		             left->height == right->height && left->timestamp == right->timestamp &&
		             left->source_sequence == right->source_sequence;
		bool nonempty[2] = {false};
		if (valid) {
			nonempty[0] = has_pixels(left);
			nonempty[1] = has_pixels(right);
			// Missing sensor payloads can have valid packed-frame metadata. A dark
			// stereo scene is valid; a unilateral all-zero payload is not a pair.
			valid = nonempty[0] == nonempty[1];
		}
		if (!valid) {
			++p->rejected_pairs;
			if ((p->rejected_pairs & (p->rejected_pairs - 1)) == 0)
				U_LOG_W("Passthrough rejected stereo pair count=%" PRIu64 " left_nonzero=%d right_nonzero=%d",
				        p->rejected_pairs, nonempty[0], nonempty[1]);
			// Preserve only the previous complete pair. get_frame still expires it
			// at its original capture timestamp, never at the rejected timestamp.
			os_mutex_unlock(&p->mutex);
			return;
		}
		uint64_t now = os_monotonic_get_ns();
		if (now - p->last_diag_ns >= 5ULL * U_TIME_1S_IN_NS) {
			p->last_diag_ns = now;
			struct sensor_pixels pixels[2] = {measure_pixels(left), measure_pixels(right)};
			double count = (double)left->width * left->height;
			u_log(__FILE__, __LINE__, __func__, U_LOGGING_INFO,
			      "Passthrough stereo sequence=%" PRIu64 " timestamp=%" PRIu64
			      " left=%u,%u,%.3f right=%u,%u,%.3f rejected=%" PRIu64,
			      left->source_sequence, left->timestamp, pixels[0].min, pixels[0].max,
			      pixels[0].sum / count, pixels[1].min, pixels[1].max, pixels[1].sum / count,
			      p->rejected_pairs);
		}
		xrt_frame_reference(&p->raw[0], left);
		xrt_frame_reference(&p->raw[1], right);
	}
	os_mutex_unlock(&p->mutex);
}

static bool
get_frame(struct xrt_camera *camera, struct xrt_frame **out)
{
	struct rift_s_passthrough *p = (struct rift_s_passthrough *)camera;
	struct xrt_frame *raw[2] = {0};
	os_mutex_lock(&p->convert_mutex);
	os_mutex_lock(&p->mutex);
	int64_t now = os_monotonic_get_ns();
	bool available = p->active && p->raw[0] && p->raw[1] && now >= p->raw[0]->timestamp &&
	                 now - p->raw[0]->timestamp <= MAX_AGE_NS;
	if (available)
		for (int i = 0; i < 2; ++i)
			xrt_frame_reference(&raw[i], p->raw[i]);
	os_mutex_unlock(&p->mutex);
	if (!available) {
		os_mutex_unlock(&p->convert_mutex);
		return false;
	}
	if (!p->converted || p->converted->timestamp != raw[0]->timestamp) {
		struct xrt_frame *frame = NULL;
		u_frame_create_one_off(XRT_FORMAT_R8G8B8, VIEW_SIZE * 2, VIEW_SIZE, &frame);
		frame->timestamp = raw[0]->timestamp;
		frame->source_sequence = raw[0]->source_sequence;
		frame->stereo_format = XRT_STEREO_FORMAT_SBS;
		for (int eye = 0; eye < 2; ++eye) {
			struct xrt_frame *src = raw[eye];
			for (uint32_t y = 0; y < VIEW_SIZE; ++y)
				for (uint32_t x = 0; x < VIEW_SIZE; ++x) {
					struct xrt_vec2 uv = p->map[eye][y * VIEW_SIZE + x];
					uint8_t value = 0;
					if (uv.x >= 0 && uv.y >= 0 && uv.x < src->width - 1 && uv.y < src->height - 1) {
						uint32_t ix = (uint32_t)uv.x, iy = (uint32_t)uv.y;
						float dx = uv.x - ix, dy = uv.y - iy;
						const uint8_t *a = src->data + iy * src->stride + ix;
						value =
						    (uint8_t)((a[0] * (1 - dx) + a[1] * dx) * (1 - dy) +
						              (a[src->stride] * (1 - dx) + a[src->stride + 1] * dx) *
						                  dy);
					}
					uint8_t *dst = frame->data + y * frame->stride + (eye * VIEW_SIZE + x) * 3;
					dst[0] = dst[1] = dst[2] = value;
				}
		}
		xrt_frame_reference(&p->converted, frame);
		xrt_frame_reference(&frame, NULL);
	}
	xrt_frame_reference(out, p->converted);
	for (int i = 0; i < 2; ++i)
		xrt_frame_reference(&raw[i], NULL);
	os_mutex_unlock(&p->convert_mutex);
	return true;
}

struct xrt_camera *
rift_s_passthrough_create(const struct rift_s_camera_calibration_block *calibration)
{
	struct rift_s_passthrough *p = calloc(1, sizeof(*p));
	if (!p)
		return NULL;
	if (os_mutex_init(&p->mutex) != 0) {
		free(p);
		return NULL;
	}
	if (os_mutex_init(&p->convert_mutex) != 0) {
		os_mutex_destroy(&p->mutex);
		free(p);
		return NULL;
	}
	p->base.view_width = p->base.view_height = VIEW_SIZE;
	// Both front sensors cover this head-aligned square after firmware rectification.
	// Use the same pinhole focal for the resampling map and the exported intrinsics.
	float focal = VIEW_SIZE / (2.0f * tanf(55.0f * (float)M_PI / 180.0f));
	p->base.focal_length = (struct xrt_vec2){focal, focal};
	p->base.center = (struct xrt_vec2){(VIEW_SIZE - 1) / 2.0f, (VIEW_SIZE - 1) / 2.0f};
	p->base.set_active = set_active;
	p->base.get_frame = get_frame;
	for (int eye = 0; eye < 2; ++eye) {
		const struct rift_s_camera_calibration *c = &calibration->cameras[CAM_IDX_TO_ID[eye]];
		p->base.head_from_camera[eye] = (struct xrt_pose)XRT_POSE_IDENTITY;
		p->base.head_from_camera[eye].position = (struct xrt_vec3){
		    c->device_from_camera.v[12], c->device_from_camera.v[13], c->device_from_camera.v[14]};
		for (uint32_t y = 0; y < VIEW_SIZE; ++y)
			for (uint32_t x = 0; x < VIEW_SIZE; ++x) {
				struct xrt_vec2 *uv = &p->map[eye][y * VIEW_SIZE + x];
				float rx = (x - p->base.center.x) / p->base.focal_length.x;
				float ry = (y - p->base.center.y) / p->base.focal_length.y;
				if (!rift_s_passthrough_project(c, rx, ry, uv))
					*uv = (struct xrt_vec2){-1, -1};
			}
	}
	return &p->base;
}

void
rift_s_passthrough_destroy(struct xrt_camera *camera)
{
	if (!camera)
		return;
	struct rift_s_passthrough *p = (struct rift_s_passthrough *)camera;
	set_active(camera, false);
	xrt_frame_reference(&p->converted, NULL);
	os_mutex_destroy(&p->convert_mutex);
	os_mutex_destroy(&p->mutex);
	free(p);
}
