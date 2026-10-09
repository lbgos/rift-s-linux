/*
 * constellation tracking debug visualisation functions
 * Copyright 2020-2023, Jan Schmidt <thaytan@noraisin.net>
 * SPDX-License-Identifier: BSL-1.0
 */
/*!
 * @file
 * @brief  Debug visualisation for constellation tracking
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup constellation
 */
#include <assert.h>
#include <math.h>
#include <string.h>

#include "xrt/xrt_config_have.h"

#ifdef XRT_HAVE_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include "debug_draw.h"

/* This will draw RGB flipped on big-endian */
#define WRITE_UINT24_BE(dest, colour)                                                                                  \
	dest[2] = colour & 0xff;                                                                                       \
	dest[1] = colour >> 8 & 0xff;                                                                                  \
	dest[0] = colour >> 16 & 0xff

static void
draw_rgb_marker(uint8_t *pixels,
                int width,
                int stride,
                int height,
                int x_pos,
                int y_pos,
                int mark_width,
                int mark_height,
                uint32_t colour)
{
	int x, y;
	int min_x = MAX(0, x_pos - (mark_width + 1) / 2);
	int max_x = MIN(width, x_pos + (mark_width + 1) / 2);
	int min_y = MAX(0, y_pos - (mark_height + 1) / 2);
	int max_y = MIN(height, y_pos + (mark_height + 1) / 2);

	if (y_pos < 0 || y_pos >= height)
		return;
	if (x_pos < 0 || x_pos >= width)
		return;

	/* Horizontal line */
	uint8_t *dest = pixels + stride * y_pos + 3 * min_x;
	for (x = 0; x < max_x - min_x; x++) {
		WRITE_UINT24_BE(dest, colour);
		dest += 3;
	}

	/* Vertical line */
	dest = pixels + stride * min_y + 3 * x_pos;
	for (y = 0; y < max_y - min_y; y++) {
		WRITE_UINT24_BE(dest, colour);
		dest += stride;
	}
}

static void
clamp(int *val, int max)
{
	if (*val < 0)
		*val = 0;
	if (*val >= max)
		*val = max - 1;
}

static void
clamp_rect(int *x, int *y, int *rw, int *rh, int width, int height)
{
	clamp(x, width);
	clamp(y, height);
	clamp(rw, width - *x);
	clamp(rh, height - *y);
}

/* Draw a single-pixel rect outline */
static void
draw_rgb_rect(uint8_t *pixels,
              int width,
              int stride,
              int height,
              int start_x,
              int start_y,
              int box_width,
              int box_height,
              uint32_t colour)
{
	clamp_rect(&start_x, &start_y, &box_width, &box_height, width, height);

	int x, y;
	uint8_t *dest = pixels + stride * start_y + 3 * start_x;
	for (x = 0; x < box_width; x++) {
		WRITE_UINT24_BE(dest, colour);
		dest += 3;
	}

	for (y = 1; y < box_height - 1; y++) {
		dest = pixels + stride * (start_y + y) + 3 * start_x;

		WRITE_UINT24_BE(dest, colour);
		dest += 3 * (box_width - 1);
		WRITE_UINT24_BE(dest, colour);
	}

	dest = pixels + stride * (start_y + box_height - 1) + 3 * start_x;
	for (x = 0; x < box_width; x++) {
		WRITE_UINT24_BE(dest, colour);
		dest += 3;
	}
}

static void
draw_rgb_filled_rect(uint8_t *pixels,
                     int width,
                     int stride,
                     int height,
                     int start_x,
                     int start_y,
                     int box_width,
                     int box_height,
                     uint32_t colour)
{
	clamp_rect(&start_x, &start_y, &box_width, &box_height, width, height);

	int x, y;
	uint8_t *dest;
	for (y = 0; y < box_height; y++) {
		dest = pixels + stride * (start_y + y) + 3 * start_x;
		for (x = 0; x < box_width; x++) {
			WRITE_UINT24_BE(dest, colour);
			dest += 3;
		}
	}
}


static void
colour_rgb_rect(uint8_t *pixels,
                int width,
                int stride,
                int height,
                int start_x,
                int start_y,
                int box_width,
                int box_height,
                uint32_t mask_colour)
{
	clamp_rect(&start_x, &start_y, &box_width, &box_height, width, height);

	int x, y;
	uint8_t *dest;
	/* This will draw RGB flipped on big-endian */
	int r = mask_colour >> 16;
	int g = (mask_colour >> 8) & 0xFF;
	int b = mask_colour & 0xFF;

	/* The brightness of the target pixel is boosted to make it more visible,
	 * as most blobs are quite dim */
	for (y = 0; y < box_height; y++) {
		dest = pixels + stride * (start_y + y) + 3 * start_x;
		for (x = 0; x < box_width; x++) {
			int bright = (dest[0] * 4) + 0x10;
			dest[0] = CLAMP(((r * bright) >> 8), 0, 255);
			dest[1] = CLAMP(((g * bright) >> 8), 0, 255);
			dest[2] = CLAMP(((b * bright) >> 8), 0, 255);
			dest += 3;
		}
	}
}

static int
object_id_to_colour(int dtype)
{
	const int colours[] = {0xFF4040, 0x40FF40, 0x4040FF};

	switch (dtype) {
	case XRT_DEVICE_TYPE_HMD: return colours[0];
	case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: return colours[1];
	case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: return colours[2];
	default: break;
	}
	return 0x808080;
}

static int
object_id_to_index(int dtype)
{
	switch (dtype) {
	case XRT_DEVICE_TYPE_HMD: return 0;
	case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: return 1;
	case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: return 2;
	default: break;
	}
	return -1;
}

void
debug_draw_blobs_leds(struct xrt_frame *rgb_out,
                      struct xrt_frame *gray_in,
                      enum debug_draw_flag flags,
                      struct tracking_sample_frame *view,
                      int view_id,
                      struct camera_model *calib,
                      struct tracking_sample_device_state *devices,
                      uint8_t n_devices)
{
	int x, y;

	uint8_t *src = gray_in->data;
	int width = gray_in->width;
	int height = gray_in->height;
	int in_stride = gray_in->stride;

	uint8_t *dest = rgb_out->data;
	int out_stride = rgb_out->stride;

#ifdef XRT_HAVE_OPENCV
	cv::Mat rgbOutMat = cv::Mat(rgb_out->height, rgb_out->width, CV_8UC3, dest, out_stride);
#endif

	uint8_t equalise_map[256];

	if (flags & DEBUG_DRAW_FLAG_NORMALISE) {
		uint8_t min_pix = 255, max_pix = 0;
		src = gray_in->data;
		for (y = 0; y < height; y++) {
			for (x = 0; x < width; x++) {
				if (src[x] < min_pix)
					min_pix = src[x];
				if (src[x] > max_pix)
					max_pix = src[x];
			}
			src += in_stride;
		}
		max_pix = MIN(64, max_pix); // HACK: Scale everything in the bottom 25% up
		uint8_t delta = max_pix - min_pix;
		for (int i = 0; i < 256; i++) {
			equalise_map[i] = CLAMP(((i - min_pix) * 255 + delta / 2) / delta, 0, 255);
		}
	} else {
		/* identity map */
		for (int i = 0; i < 256; i++) {
			equalise_map[i] = i;
		}
	}

	uint8_t *dest_line = dest;
	src = gray_in->data;
	for (y = 0; y < height; y++) {
		/* Expand to RGB and copy the source to the dest, then
		 * paint blob markers */
		uint8_t *d = dest_line;
		for (x = 0; x < width; x++) {
			/* Expand GRAY8 to RGB */
			d[0] = d[1] = d[2] = equalise_map[src[x]];
			d += 3;
		}

		dest_line += out_stride;
		src += in_stride;
	}

#ifdef XRT_HAVE_OPENCV
	// Draw view's gravity vector
	cv::Point from(30 - 30 * view->cam_gravity_vector.x, 46 - 30 * view->cam_gravity_vector.y);
	cv::Point to(30 + 30 * view->cam_gravity_vector.x, 46 + 30 * view->cam_gravity_vector.y);
	cv::arrowedLine(rgbOutMat, from, to, cv::Scalar(0x80, 0x80, 0x80));
#endif

	if (view->bwobs) {
		struct blobservation *bwobs = view->bwobs;

		/* Draw the blobs in the video */
		for (int index = 0; index < bwobs->num_blobs; index++) {
			struct blob *b = bwobs->blobs + index;
			int start_x, start_y, w, h;

			start_x = b->left;
			start_y = b->top;
			w = b->width;
			h = b->height;
			clamp_rect(&start_x, &start_y, &w, &h, width, height);

			uint32_t c = 0xFF80FF;
			if (b->led_id != LED_INVALID_ID) {
				c = object_id_to_colour(LED_OBJECT_ID(b->led_id));
			}

			if (flags & DEBUG_DRAW_FLAG_BLOB_TINT) {
				/* Tint known blobs by their device ID in the image. Purple for unknown */
				colour_rgb_rect(dest, width, out_stride, height, start_x, start_y, b->width, b->height,
				                c);
			}

			if (b->led_id != LED_INVALID_ID) {
				/* Draw a dot at the weighted center for known blobs */
				draw_rgb_marker(dest, width, out_stride, height, round(b->x), round(b->y), 0, 0, c);
			}

#ifdef XRT_HAVE_OPENCV
			cv::Scalar cvCol = cv::Scalar((c >> 24) & 0xff, (c >> 16) & 0xff, c & 0xff);

			if (flags & DEBUG_DRAW_FLAG_BLOB_CIRCLE) {
				cv::RotatedRect rect =
				    cv::RotatedRect(cv::Point2f(start_x - 1 + (w + 1) / 2, start_y - 1 + (h + 1) / 2),
				                    cv::Size2f(w + 2, h + 2), 0);
				cv::ellipse(rgbOutMat, rect, cvCol);
			}

			if ((flags & DEBUG_DRAW_FLAG_BLOB_IDS) && (LED_LOCAL_ID(b->led_id) != LED_INVALID_ID)) {
				cv::Point2f loc = cv::Point2f(start_x + w + 1, start_y);
				char text[64];
				sprintf(text, "%d", LED_LOCAL_ID(b->led_id));
				cv::putText(rgbOutMat, text, loc, cv::FONT_HERSHEY_PLAIN, 1.0, cvCol);
			}

			if ((flags & DEBUG_DRAW_FLAG_BLOB_UNIQUE_IDS)) {
				cv::Point2f loc = cv::Point2f(start_x + w + 1, start_y + 10 + 1);
				char text[64];
				sprintf(text, "%u", b->blob_id);
				cv::putText(rgbOutMat, text, loc, cv::FONT_HERSHEY_PLAIN, 1.0, cvCol);
			}
#endif
		}
	}

	for (int d = 0; d < n_devices; d++) {
		struct tracking_sample_device_state *dev_state = devices + d;
		uint8_t object_id = dev_state->led_model->id;
		int dev_id = object_id_to_index(object_id);

		if (dev_id < 0) {
			continue;
		}

		uint32_t dev_colour = object_id_to_colour(object_id);

		struct xrt_pose P_cam_obj_prior;
		math_pose_transform(&view->P_cam_world, &dev_state->P_world_obj_prior, &P_cam_obj_prior);

#ifdef XRT_HAVE_OPENCV
		// Draw prior orientation arrow by calculating gravity in the real world, then bringing it into
		// the camera frame.
		const struct xrt_vec3 gravity_vector = {0.0, 1.0, 0.0};
		struct xrt_vec3 dev_gravity_vector, dev_cam_gravity_vector;
		math_quat_rotate_vec3(&dev_state->P_world_obj_prior.orientation, &gravity_vector, &dev_gravity_vector);
		math_quat_rotate_vec3(&view->P_world_cam.orientation, &dev_gravity_vector, &dev_cam_gravity_vector);

		cv::Scalar cvCol = cv::Scalar((dev_colour >> 24) & 0xff, (dev_colour >> 16) & 0xff, dev_colour & 0xff);
		cv::Point from(30 - 30 * dev_cam_gravity_vector.x, 46 - 30 * dev_cam_gravity_vector.y);
		cv::Point to(30 + 30 * dev_cam_gravity_vector.x, 46 + 30 * dev_cam_gravity_vector.y);
		cv::arrowedLine(rgbOutMat, from, to, cvCol);
#endif

		if (dev_state->found_device_pose) {
			/* Draw a marker in the top-left of the frame for which devices we found a pose for */
			draw_rgb_filled_rect(dest, width, out_stride, height, 16 * dev_id, 0, 16, 16, dev_colour);

			if (flags & DEBUG_DRAW_FLAG_DEVICE_BOUNDS) {
				struct xrt_pose P_cam_obj;
				math_pose_transform(&view->P_cam_world, &dev_state->final_pose, &P_cam_obj);

				struct xrt_vec2 object_bound_points[MAX_OBJECT_LEDS];
				size_t num_object_bound_points;

				struct pose_rect device_bounds;
				pose_metrics_get_device_bounds(&P_cam_obj, dev_state->led_model, calib, &device_bounds,
				                               object_bound_points, &num_object_bound_points);

				draw_rgb_rect(dest, width, out_stride, height, device_bounds.left, device_bounds.top,
				              device_bounds.right - device_bounds.left,
				              device_bounds.bottom - device_bounds.top, dev_colour);

				for (size_t i = 0; i < num_object_bound_points; i++) {
					draw_rgb_marker(dest, width, out_stride, height, object_bound_points[i].x,
					                object_bound_points[i].y, 1, 1, dev_colour);
				}
			}

			if (flags & (DEBUG_DRAW_FLAG_LEDS | DEBUG_DRAW_FLAG_POSE_BOUNDS)) {
				struct xrt_pose P_cam_obj;
				math_pose_transform(&view->P_cam_world, &dev_state->final_pose, &P_cam_obj);

				struct pose_metrics_blob_match_info blob_match_info;

				pose_metrics_match_pose_to_blobs(&P_cam_obj, view->bwobs->blobs, view->bwobs->num_blobs,
				                                 dev_state->led_model, calib, &blob_match_info);

				if (flags & DEBUG_DRAW_FLAG_POSE_BOUNDS) {
					draw_rgb_rect(dest, width, out_stride, height, blob_match_info.bounds.left,
					              blob_match_info.bounds.top,
					              blob_match_info.bounds.right - blob_match_info.bounds.left,
					              blob_match_info.bounds.bottom - blob_match_info.bounds.top,
					              dev_colour);
				}

				if (flags & DEBUG_DRAW_FLAG_LEDS) {
					for (int l = 0; l < blob_match_info.num_visible_leds; l++) {
						struct pose_metrics_visible_led_info *led_info =
						    blob_match_info.visible_leds + l;
						uint32_t c = dev_colour;

						if (led_info->matched_blob == NULL) {
							c = 0x808080; /* Draw LEDs with no matched blob in grey */
						}

						draw_rgb_marker(dest, width, out_stride, height, led_info->pos_px.x,
						                led_info->pos_px.y, ceil(led_info->led_radius_px),
						                ceil(led_info->led_radius_px), c);
					}
				}
			}
		}
		if (flags & DEBUG_DRAW_FLAG_PRIOR_LEDS) {
			const uint32_t c = dev_colour | 0x400000; // Tint with red
			struct pose_metrics_blob_match_info blob_match_info;

			// Draw cross at pose center
			struct xrt_vec2 pose_center;
			if (camera_model_project(calib, P_cam_obj_prior.position.x, P_cam_obj_prior.position.y,
			                         P_cam_obj_prior.position.z, &pose_center.x, &pose_center.y))
				draw_rgb_marker(dest, width, out_stride, height, pose_center.x, pose_center.y, 5, 5, c);

			pose_metrics_match_pose_to_blobs(&P_cam_obj_prior, NULL, 0, dev_state->led_model, calib,
			                                 &blob_match_info);
			for (int l = 0; l < blob_match_info.num_visible_leds; l++) {
				struct pose_metrics_visible_led_info *led_info = blob_match_info.visible_leds + l;

				draw_rgb_marker(dest, width, out_stride, height, led_info->pos_px.x, led_info->pos_px.y,
				                ceil(led_info->led_radius_px), ceil(led_info->led_radius_px), c);
			}
		}
		if (dev_state->have_last_seen_pose && (flags & DEBUG_DRAW_FLAG_LAST_SEEN_LEDS)) {
			const uint32_t c = dev_colour | 0x404040; // Tint with some yellow

			struct xrt_pose P_cam_obj;
			math_pose_transform(&view->P_cam_world, &dev_state->last_seen_pose, &P_cam_obj);

			// Draw cross at pose center
			struct xrt_vec2 pose_center;
			if (camera_model_project(calib, P_cam_obj.position.x, P_cam_obj.position.y, P_cam_obj.position.z,
			                         &pose_center.x, &pose_center.y))
				draw_rgb_marker(dest, width, out_stride, height, pose_center.x, pose_center.y, 5, 5, c);

			struct pose_metrics_blob_match_info blob_match_info;

			pose_metrics_match_pose_to_blobs(&P_cam_obj, NULL, 0, dev_state->led_model, calib,
			                                 &blob_match_info);
			for (int l = 0; l < blob_match_info.num_visible_leds; l++) {
				struct pose_metrics_visible_led_info *led_info = blob_match_info.visible_leds + l;

				draw_rgb_marker(dest, width, out_stride, height, led_info->pos_px.x, led_info->pos_px.y,
				                ceil(led_info->led_radius_px), ceil(led_info->led_radius_px), c);
			}
		}
	}
}
