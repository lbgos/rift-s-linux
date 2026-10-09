/*
 * Copyright 2020 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
/*!
 * @file
 * @brief  Oculus Rift S Touch Controller driver
 *
 * Handles communication and calibration information for the Touch Controllers
 *
 * Ported from OpenHMD
 *
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup drv_rift_s
 */


#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "math/m_api.h"
#include "math/m_clock_tracking.h"
#include "math/m_predict.h"
#include "math/m_space.h"
#include "math/m_vec3.h"

#include "os/os_hid.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"

#include "rift_s.h"
#include "rift_s_hmd.h"
#include "rift_s_radio.h"
#include "rift_s_protocol.h"
#include "rift_s_controller.h"
#include "rift_s_model_check.h"
#include "rift_s_util.h"

/* LED period written to both controllers. Monado streams controller exposures at 30 Hz
 * (every second frame of the 60 Hz camera stream), so the Windows rule period = 1000000 / rate
 * gives 33333 us. 0 only reads and logs the register. */
// Position filter process noise overrides for field tuning; <= 0 keeps the defaults.
DEBUG_GET_ONCE_FLOAT_OPTION(rift_s_position_accel_noise, "RIFT_S_POSITION_ACCEL_NOISE", 0)
DEBUG_GET_ONCE_FLOAT_OPTION(rift_s_position_bias_walk, "RIFT_S_POSITION_BIAS_WALK", 0)
DEBUG_GET_ONCE_NUM_OPTION(rift_s_controller_led_period_us, "RIFT_S_CONTROLLER_LED_PERIOD_US", 1000000 / 30)

/* Set to 1 to print controller states continuously */
#define DUMP_CONTROLLER_STATE 0

static struct xrt_binding_input_pair simple_inputs_rift_s[4] = {
    {XRT_INPUT_SIMPLE_SELECT_CLICK, XRT_INPUT_TOUCH_TRIGGER_VALUE},
    {XRT_INPUT_SIMPLE_MENU_CLICK, XRT_INPUT_TOUCH_MENU_CLICK},
    {XRT_INPUT_SIMPLE_GRIP_POSE, XRT_INPUT_TOUCH_GRIP_POSE},
    {XRT_INPUT_SIMPLE_AIM_POSE, XRT_INPUT_TOUCH_AIM_POSE},
};

static struct xrt_binding_output_pair simple_outputs_rift_s[1] = {
    {XRT_OUTPUT_NAME_SIMPLE_VIBRATION, XRT_OUTPUT_NAME_TOUCH_HAPTIC},
};

static struct xrt_binding_profile binding_profiles_rift_s[1] = {
    {
        .name = XRT_DEVICE_SIMPLE_CONTROLLER,
        .inputs = simple_inputs_rift_s,
        .input_count = ARRAY_SIZE(simple_inputs_rift_s),
        .outputs = simple_outputs_rift_s,
        .output_count = ARRAY_SIZE(simple_outputs_rift_s),
    },
};

enum touch_controller_input_index
{
	/* Left controller */
	OCULUS_TOUCH_X_CLICK = 0,
	OCULUS_TOUCH_X_TOUCH,
	OCULUS_TOUCH_Y_CLICK,
	OCULUS_TOUCH_Y_TOUCH,
	OCULUS_TOUCH_MENU_CLICK,

	/* Right controller */
	OCULUS_TOUCH_A_CLICK = 0,
	OCULUS_TOUCH_A_TOUCH,
	OCULUS_TOUCH_B_CLICK,
	OCULUS_TOUCH_B_TOUCH,
	OCULUS_TOUCH_SYSTEM_CLICK,

	/* Common */
	OCULUS_TOUCH_SQUEEZE_VALUE,
	OCULUS_TOUCH_TRIGGER_TOUCH,
	OCULUS_TOUCH_TRIGGER_VALUE,
	OCULUS_TOUCH_THUMBSTICK_CLICK,
	OCULUS_TOUCH_THUMBSTICK_TOUCH,
	OCULUS_TOUCH_THUMBSTICK,
	OCULUS_TOUCH_THUMBREST_TOUCH,
	OCULUS_TOUCH_GRIP_POSE,
	OCULUS_TOUCH_AIM_POSE,

	INPUT_INDICES_LAST
};
#define SET_TOUCH_INPUT(d, NAME) ((d)->base.inputs[OCULUS_TOUCH_##NAME].name = XRT_INPUT_TOUCH_##NAME)
#define DEBUG_TOUCH_INPUT_BOOL(d, NAME, label)                                                                         \
	u_var_add_bool((d), &(d)->base.inputs[OCULUS_TOUCH_##NAME].value.boolean, label)
#define DEBUG_TOUCH_INPUT_F32(d, NAME, label)                                                                          \
	u_var_add_f32((d), &(d)->base.inputs[OCULUS_TOUCH_##NAME].value.vec1.x, label)
#define DEBUG_TOUCH_INPUT_VEC2(d, NAME, label1, label2)                                                                \
	u_var_add_f32((d), &(d)->base.inputs[OCULUS_TOUCH_##NAME].value.vec2.x, label1);                               \
	u_var_add_f32((d), &(d)->base.inputs[OCULUS_TOUCH_##NAME].value.vec2.y, label2)

#if DUMP_CONTROLLER_STATE
static void
print_controller_state(struct rift_s_controller *ctrl)
{
	if (rift_s_log_level > U_LOGGING_TRACE)
		return; // Only log at TRACE log_level

	/* Dump the controller state if we see something unexpected / unknown, otherwise be quiet */
	if (ctrl->extra_bytes_len == 0 && ctrl->mask08 == 0x50 && ctrl->mask0e == 0)
		return;

	char buf[16384] = "";
	int bufsize = sizeof(buf) - 2;
	int printed = 0;

	printed += snprintf(buf + printed, bufsize - printed,
	                    "Controller %16lx type 0x%08x IMU ts %8u v2 %x accel %6d %6d %6d gyro %6d %6d %6d | ",
	                    ctrl->device_id, ctrl->device_type, ctrl->imu_timestamp32, ctrl->imu_unknown_varying2,
	                    ctrl->raw_accel[0], ctrl->raw_accel[1], ctrl->raw_accel[2], ctrl->raw_gyro[0],
	                    ctrl->raw_gyro[1], ctrl->raw_gyro[2]);

	printed += snprintf(buf + printed, bufsize - printed, "unk %02x %02x buttons %02x fingers %02x | ",
	                    ctrl->mask08, ctrl->mask0e, ctrl->buttons, ctrl->fingers);
	printed += snprintf(buf + printed, bufsize - printed, "trigger %5d grip %5d |", ctrl->trigger, ctrl->grip);
	printed +=
	    snprintf(buf + printed, bufsize - printed, "joystick x %5d y %5d |", ctrl->joystick_x, ctrl->joystick_y);

	if (ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER) {
		printed +=
		    snprintf(buf + printed, bufsize - printed, "capsense x %u y %u joy %u trig %u | ",
		             ctrl->capsense_a_x, ctrl->capsense_b_y, ctrl->capsense_joystick, ctrl->capsense_trigger);
	} else if (ctrl->device_type == RIFT_S_DEVICE_RIGHT_CONTROLLER) {
		printed +=
		    snprintf(buf + printed, bufsize - printed, "capsense a %u b %u joy %u trig %u | ",
		             ctrl->capsense_a_x, ctrl->capsense_b_y, ctrl->capsense_joystick, ctrl->capsense_trigger);
	} else {
		printed +=
		    snprintf(buf + printed, bufsize - printed, "capsense ?? %u ?? %u ?? %u ?? %u | ",
		             ctrl->capsense_a_x, ctrl->capsense_b_y, ctrl->capsense_joystick, ctrl->capsense_trigger);
	}

	if (ctrl->extra_bytes_len) {
		printed += snprintf(buf + printed, bufsize - printed, " | extra ");
		printed += rift_s_snprintf_hexdump_buffer(buf + printed, bufsize - printed, NULL, ctrl->extra_bytes,
		                                          ctrl->extra_bytes_len);
	}

	RIFT_S_TRACE("%s", buf);
}
#endif

static bool
finite_vec3(const struct xrt_vec3 *v)
{
	return isfinite(v->x) && isfinite(v->y) && isfinite(v->z);
}

static bool
valid_quat(const struct xrt_quat *q)
{
	float norm = q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w;
	return isfinite(norm) && norm > 1e-6f;
}

static const char *
hand_alias(const struct rift_s_controller *ctrl)
{
	return ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER ? "left" : "right";
}

static struct rift_s_attitude_sample *
attitude_sample(struct rift_s_controller *ctrl, unsigned int index)
{
	return &ctrl->attitude_history[(ctrl->attitude_history_start + index) % RIFT_S_ATTITUDE_HISTORY_CAPACITY];
}

/* Forget the device-to-host clock mapping and everything captured against it. The next sample
 * starts a new mapping. Caller holds the mutex. */
static void
reset_clock_mapping(struct rift_s_controller *ctrl)
{
	if (ctrl->imu_clock != NULL) {
		m_clock_windowed_skew_tracker_destroy(ctrl->imu_clock);
		ctrl->imu_clock = NULL;
	}
	ctrl->imu_to_host_ns = 0;
	ctrl->attitude_history_count = ctrl->attitude_history_start = 0;
	ctrl->clock_stable_samples = 0;
	ctrl->optical_heading_valid = false;
	// Retained IMU times no longer match the optical capture times: hold the position.
	rift_s_position_filter_freeze(&ctrl->position_filter);
}

bool
rift_s_controller_record_attitude(struct rift_s_controller *ctrl,
                                  timepoint_ns device_capture_ns,
                                  timepoint_ns arrival_ns)
{
	if (!valid_quat(&ctrl->fusion.rot) || !finite_vec3(&ctrl->fusion.last.gyro)) {
		ctrl->diagnostics.reject_nonfinite++;
		return false;
	}
	if (device_capture_ns <= 0 || arrival_ns <= 0 ||
	    (ctrl->attitude_history_count != 0 &&
	     device_capture_ns <= attitude_sample(ctrl, ctrl->attitude_history_count - 1)->device_capture_ns)) {
		ctrl->diagnostics.reject_clock++;
		ctrl->diagnostics.reject_clock_by[RIFT_S_CLOCK_HISTORY_ORDER]++;
		return false;
	}
	/* Transport delay is milliseconds. A sample a second or more off the mapping means the device
	 * clock itself jumped: its 32-bit microsecond counter wraps every 71.6 minutes, so a controller
	 * that sleeps longer loses whole wraps. The minimum-skew window would follow such a step only
	 * by 1/64 per sample, resetting the history on each of about 660 samples (r18 wake log).
	 * Start a new mapping at once. */
	if (ctrl->imu_clock != NULL && ctrl->imu_to_host_ns != 0) {
		time_duration_ns error = arrival_ns - device_capture_ns - ctrl->imu_to_host_ns;
		if (error > U_TIME_1S_IN_NS || error < -U_TIME_1S_IN_NS) {
			RIFT_S_INFO("Controller IMU hand=%s clock_reanchor=1 mapping_error_ms=%.3f", hand_alias(ctrl),
			            (double)error / U_TIME_1MS_IN_NS);
			reset_clock_mapping(ctrl);
			ctrl->diagnostics.clock_resyncs++;
		}
	}
	/* Radio reports arrive in bursts. A mean-delay filter follows every burst, moving capture
	 * times by milliseconds and occasionally past the discontinuity threshold below. Like the
	 * CV1 Touch driver, track the minimum transport delay over a window instead: it is stable
	 * under jitter and closest to the true capture time. */
	if (ctrl->imu_clock == NULL) {
		ctrl->imu_clock = m_clock_windowed_skew_tracker_alloc(64);
		if (ctrl->imu_clock == NULL) {
			return false;
		}
	}
	m_clock_windowed_skew_tracker_push(ctrl->imu_clock, arrival_ns, device_capture_ns);
	timepoint_ns capture_ns;
	if (!m_clock_windowed_skew_tracker_to_local(ctrl->imu_clock, device_capture_ns, &capture_ns)) {
		ctrl->diagnostics.reject_clock++;
		ctrl->diagnostics.reject_clock_by[RIFT_S_CLOCK_UNMAPPED]++;
		return false;
	}
	time_duration_ns old_offset = ctrl->imu_to_host_ns;
	ctrl->imu_to_host_ns = capture_ns - device_capture_ns;
	time_duration_ns change = ctrl->imu_to_host_ns - old_offset;
	if (old_offset != 0 && (change > 2 * U_TIME_1MS_IN_NS || change < -2 * U_TIME_1MS_IN_NS)) {
		// A real discontinuity (the estimator restarted): earlier capture times no longer line up.
		RIFT_S_INFO("Controller IMU hand=%s clock_discontinuity_ms=%.3f history_reset=1", hand_alias(ctrl),
		             (double)change / U_TIME_1MS_IN_NS);
		ctrl->attitude_history_count = ctrl->clock_stable_samples = 0;
		ctrl->optical_heading_valid = false;
		// Retained IMU times no longer match the optical capture times: hold the position.
		rift_s_position_filter_freeze(&ctrl->position_filter);
		ctrl->diagnostics.reject_clock++;
		ctrl->diagnostics.reject_clock_by[RIFT_S_CLOCK_DISCONTINUITY]++;
	}
	if (change <= U_TIME_HALF_MS_IN_NS && change >= -U_TIME_HALF_MS_IN_NS) {
		if (ctrl->clock_stable_samples < 32) {
			ctrl->clock_stable_samples++;
		}
	} else {
		ctrl->clock_stable_samples = 0;
	}
	if (capture_ns <= 0 || (ctrl->attitude_history_count != 0 &&
	                        capture_ns <= attitude_sample(ctrl, ctrl->attitude_history_count - 1)->capture_ns)) {
		ctrl->diagnostics.reject_clock++;
		ctrl->diagnostics.reject_clock_by[RIFT_S_CLOCK_MAPPED_ORDER]++;
		return false;
	}
	if (ctrl->attitude_history_count == RIFT_S_ATTITUDE_HISTORY_CAPACITY) {
		ctrl->attitude_history_start = (ctrl->attitude_history_start + 1) % RIFT_S_ATTITUDE_HISTORY_CAPACITY;
		ctrl->attitude_history_count--;
	}
	struct rift_s_attitude_sample *sample = attitude_sample(ctrl, ctrl->attitude_history_count++);
	*sample = (struct rift_s_attitude_sample){
	    .device_capture_ns = device_capture_ns,
	    .capture_ns = capture_ns,
	    .arrival_ns = arrival_ns,
	    .orientation = ctrl->fusion.rot,
	    .gyro = ctrl->fusion.last.gyro,
	    .tilt_valid = ctrl->gravity_initialized && ctrl->clock_stable_samples >= 32,
	};
	math_quat_normalize(&sample->orientation);
	return true;
}

bool
rift_s_controller_get_attitude(struct rift_s_controller *ctrl,
                               timepoint_ns capture_ns,
                               struct rift_s_attitude_sample *out)
{
	enum rift_s_attitude_rejection reject = RIFT_S_ATTITUDE_EMPTY;
	if (capture_ns <= 0) {
		reject = RIFT_S_ATTITUDE_INVALID_TIME;
		goto rejected;
	}
	if (ctrl->attitude_history_count == 0) {
		goto rejected;
	}
	struct rift_s_attitude_sample *first = attitude_sample(ctrl, 0);
	struct rift_s_attitude_sample *last = attitude_sample(ctrl, ctrl->attitude_history_count - 1);
	/* The camera thread can outrun the next 500 Hz radio report. A bounded gyro
	 * prediction bridges that scheduling gap; it never bridges a lost radio burst. */
	if (capture_ns > last->capture_ns && capture_ns - last->capture_ns <= 20 * U_TIME_1MS_IN_NS &&
	    last->tilt_valid && ctrl->attitude_history_count >= 2) {
		struct rift_s_attitude_sample *previous = attitude_sample(ctrl, ctrl->attitude_history_count - 2);
		if (previous->tilt_valid && last->capture_ns - previous->capture_ns <= 10 * U_TIME_1MS_IN_NS) {
			*out = *last;
			out->capture_ns = capture_ns;
			out->device_capture_ns += capture_ns - last->capture_ns;
			math_quat_integrate_velocity(&last->orientation, &last->gyro,
			                            (float)(capture_ns - last->capture_ns) / U_TIME_1S_IN_NS,
			                            &out->orientation);
			math_quat_normalize(&out->orientation);
			return true;
		}
	}
	if (capture_ns < first->capture_ns || capture_ns > last->capture_ns) {
		reject = capture_ns < first->capture_ns ? RIFT_S_ATTITUDE_TOO_OLD : RIFT_S_ATTITUDE_TOO_NEW;
		goto rejected;
	}
	for (unsigned int i = 0; i < ctrl->attitude_history_count; i++) {
		struct rift_s_attitude_sample *right = attitude_sample(ctrl, i);
		if (capture_ns > right->capture_ns) {
			continue;
		}
		if (capture_ns == right->capture_ns) {
			*out = *right;
			if (right->tilt_valid) return true;
			reject = RIFT_S_ATTITUDE_TILT_UNREADY;
			goto rejected;
		}
		struct rift_s_attitude_sample *left = attitude_sample(ctrl, i - 1);
		time_duration_ns dt = right->capture_ns - left->capture_ns;
		// Never interpolate over dropped radio packets or a startup tilt reset.
		if (!left->tilt_valid || !right->tilt_valid || dt > 10 * U_TIME_1MS_IN_NS) {
			reject = !left->tilt_valid || !right->tilt_valid ? RIFT_S_ATTITUDE_TILT_UNREADY
			                                                : RIFT_S_ATTITUDE_RADIO_GAP;
			goto rejected;
		}
		float t = (float)(capture_ns - left->capture_ns) / (float)dt;
		*out = *left;
		out->capture_ns = capture_ns;
		out->device_capture_ns += capture_ns - left->capture_ns;
		math_quat_slerp(&left->orientation, &right->orientation, t, &out->orientation);
		out->gyro = m_vec3_add(m_vec3_mul_scalar(left->gyro, 1 - t), m_vec3_mul_scalar(right->gyro, t));
		return true;
	}
rejected:
	ctrl->diagnostics.attitude_rejected[reject]++;
	ctrl->diagnostics.last_attitude_reject = reject;
	ctrl->diagnostics.last_attitude_reject_ns = capture_ns;
	return false;
}

static const char *const attitude_rejection_names[] = {
    "empty", "invalid_time", "before_history", "after_history", "tilt_unready", "radio_gap",
};

// Called with ctrl->mutex held. Keep failed capture queries visible without per-frame logging.
static void
log_tracking_health(struct rift_s_controller *ctrl, timepoint_ns now, bool transition)
{
	struct rift_s_fusion_diagnostics *d = &ctrl->diagnostics;
	if (!transition && now - d->last_health_log_ns < 5 * (time_duration_ns)U_TIME_1S_IN_NS) return;
	d->last_health_log_ns = now;
	timepoint_ns oldest = ctrl->attitude_history_count ? attitude_sample(ctrl, 0)->capture_ns : 0;
	timepoint_ns newest = ctrl->attitude_history_count
	                          ? attitude_sample(ctrl, ctrl->attitude_history_count - 1)->capture_ns : 0;
	RIFT_S_INFO("Controller health hand=%s transition=%d source=%s imu_age_ms=%.1f optical_age_ms=%.1f"
	            " calibrated=%d configured=%d gravity=%d accel_offset=%d accel_mps2=%.3f bias_valid=%d bias_mps2=%.3f"
	            " optical_tilt_updates=%u clock_stable=%u offset_ns=%" PRIi64
	            " history=%u oldest_ns=%" PRIi64 " newest_ns=%" PRIi64
	            " history_reject=%s rejected_capture_ns=%" PRIi64
	            " attitude_empty=%" PRIu64 " attitude_invalid=%" PRIu64 " attitude_old=%" PRIu64
	            " attitude_new=%" PRIu64 " attitude_tilt=%" PRIu64 " attitude_gap=%" PRIu64
	            " optical_result=%s processing_ns=%" PRIi64 " accepted=%" PRIu64
	            " reject_stale=%" PRIu64 " reject_nonfinite=%" PRIu64 " reject_clock=%" PRIu64
	            " clock_resyncs=%u history_resets=%" PRIu64,
	            hand_alias(ctrl), transition, ctrl->position_source ? ctrl->position_source : "none",
	            ctrl->last_imu_local_time_ns ? (double)(now - ctrl->last_imu_local_time_ns) / U_TIME_1MS_IN_NS : -1,
	            ctrl->last_fix_ns ? (double)(now - ctrl->last_fix_ns) / U_TIME_1MS_IN_NS : -1,
	            ctrl->have_calibration, ctrl->have_config, ctrl->gravity_initialized, ctrl->accel_offset,
	            m_vec3_len(ctrl->accel), ctrl->attitude_bias.valid, m_vec3_len(ctrl->attitude_bias.bias),
	            ctrl->optical_tilt_updates, ctrl->clock_stable_samples, ctrl->imu_to_host_ns,
	            ctrl->attitude_history_count, oldest, newest,
	            d->last_attitude_reject_ns ? attitude_rejection_names[d->last_attitude_reject] : "none",
	            d->last_attitude_reject_ns, d->attitude_rejected[0], d->attitude_rejected[1],
	            d->attitude_rejected[2], d->attitude_rejected[3], d->attitude_rejected[4], d->attitude_rejected[5],
	            d->last_optical_result ? d->last_optical_result : "none", d->last_optical_processing_ns,
	            d->optical_accepted, d->reject_stale, d->reject_nonfinite, d->reject_clock, d->clock_resyncs,
	            d->reject_clock_by[RIFT_S_CLOCK_DISCONTINUITY]);
}

bool
rift_s_controller_has_heading_prior(struct rift_s_controller *ctrl, timepoint_ns capture_ns)
{
	struct rift_s_attitude_sample sample;
	return ctrl->optical_heading_valid && ctrl->last_tracked_pose_ts > 0 &&
	       capture_ns >= ctrl->last_tracked_pose_ts &&
	       capture_ns - ctrl->last_tracked_pose_ts <= 500 * U_TIME_1MS_IN_NS &&
	       capture_ns >= ctrl->last_optical_yaw_ts &&
	       capture_ns - ctrl->last_optical_yaw_ts <= RIFT_S_HEADING_PRIOR_MAX_AGE_NS &&
	       rift_s_controller_get_attitude(ctrl, capture_ns, &sample);
}

bool
rift_s_rest_stats_push(struct rift_s_rest_stats *s,
                       timepoint_ns timestamp_ns,
                       const struct xrt_vec3 *accel,
                       const struct xrt_vec3 *gyro)
{
	float length = m_vec3_len(*accel);
	bool usable = isfinite(length) && length > 7.0f && length < 12.5f && m_vec3_len(*gyro) < 0.1f;
	bool continues = usable && s->samples != 0 && timestamp_ns - s->last_ns <= 10 * U_TIME_1MS_IN_NS;
	if (continues) {
		struct xrt_vec3 mean = m_vec3_mul_scalar(s->sum, 1.0f / s->samples);
		continues = m_vec3_len(m_vec3_sub(*accel, mean)) < 0.3f;
	}
	s->last_ns = timestamp_ns;
	if (!continues) {
		s->samples = 0;
		s->sum = (struct xrt_vec3){0};
		s->start_ns = timestamp_ns;
	}
	if (!usable) {
		return false;
	}
	s->sum = m_vec3_add(s->sum, *accel);
	s->samples++;
	if (timestamp_ns - s->start_ns < 250 * U_TIME_1MS_IN_NS) {
		return false;
	}
	struct xrt_vec3 mean = m_vec3_mul_scalar(s->sum, 1.0f / s->samples);
	s->last_mean = mean;
	float g = m_vec3_len(mean);
	math_vec3_normalize(&mean);
	if (s->windows == 0 || g < s->min_mps2) {
		s->min_mps2 = g;
		s->min_dir = mean;
	}
	if (s->windows == 0 || g > s->max_mps2) {
		s->max_mps2 = g;
		s->max_dir = mean;
	}
	s->windows++;
	s->samples = 0;
	s->sum = (struct xrt_vec3){0};
	s->start_ns = timestamp_ns;
	return true;
}

static float
accel_bias_rms(const struct rift_s_accel_bias *b, const struct xrt_vec3 *bias)
{
	float sum = 0;
	for (unsigned int i = 0; i < b->count; i++) {
		float r = m_vec3_len(m_vec3_sub(b->means[i], *bias)) - (float)MATH_GRAVITY_M_S2;
		sum += r * r;
	}
	return b->count ? sqrtf(sum / b->count) : 0;
}

void
rift_s_accel_bias_add_window(struct rift_s_accel_bias *b, const struct xrt_vec3 *mean)
{
	float length = m_vec3_len(*mean);
	if (!isfinite(length) || length < 1.0f) {
		return;
	}
	struct xrt_vec3 dir = m_vec3_mul_scalar(*mean, 1.0f / length);
	// Keep the newest window per direction; when full, replace the window nearest in direction so
	// the stored set keeps its spread.
	unsigned int slot = b->count;
	float nearest = -2;
	for (unsigned int i = 0; i < b->count; i++) {
		float c = m_vec3_dot(dir, m_vec3_normalize(b->means[i]));
		if (c > nearest) {
			nearest = c;
			slot = i;
		}
	}
	if (b->count < RIFT_S_ACCEL_BIAS_WINDOWS && nearest < cosf(DEG_TO_RAD(15.0f))) {
		slot = b->count++;
	}
	b->means[slot] = *mean;

	float spread = 1;
	for (unsigned int i = 0; i < b->count; i++) {
		for (unsigned int j = i + 1; j < b->count; j++) {
			spread =
			    fminf(spread, m_vec3_dot(m_vec3_normalize(b->means[i]), m_vec3_normalize(b->means[j])));
		}
	}
	b->spread_deg = RAD_TO_DEG(acosf(fmaxf(-1.0f, fminf(1.0f, spread))));

	/* Gauss-Newton on sum((|m - b| - g) / 0.05)^2 + |b / 1.0|^2: window means are good to a few
	 * hundredths of m/s^2, and a bias beyond 1 m/s^2 must be observed to be believed. */
	const double inv_window_var = 1.0 / (0.05 * 0.05), inv_prior_var = 1.0;
	double x[3] = {0, 0, 0};
	for (int iteration = 0; iteration < 10; iteration++) {
		double h[3][3] = {{inv_prior_var, 0, 0}, {0, inv_prior_var, 0}, {0, 0, inv_prior_var}};
		double g[3] = {inv_prior_var * x[0], inv_prior_var * x[1], inv_prior_var * x[2]};
		for (unsigned int i = 0; i < b->count; i++) {
			double d[3] = {b->means[i].x - x[0], b->means[i].y - x[1], b->means[i].z - x[2]};
			double n = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
			if (n < 1e-6) {
				continue;
			}
			double r = n - MATH_GRAVITY_M_S2;
			double j[3] = {-d[0] / n, -d[1] / n, -d[2] / n};
			for (int k = 0; k < 3; k++) {
				g[k] += inv_window_var * j[k] * r;
				for (int l = 0; l < 3; l++) {
					h[k][l] += inv_window_var * j[k] * j[l];
				}
			}
		}
		struct xrt_matrix_3x3 m = {{(float)h[0][0], (float)h[0][1], (float)h[0][2], (float)h[1][0],
		                            (float)h[1][1], (float)h[1][2], (float)h[2][0], (float)h[2][1],
		                            (float)h[2][2]}};
		struct xrt_matrix_3x3 inverse;
		math_matrix_3x3_inverse(&m, &inverse);
		struct xrt_vec3 gradient = {(float)g[0], (float)g[1], (float)g[2]}, step;
		math_matrix_3x3_transform_vec3(&inverse, &gradient, &step);
		if (!finite_vec3(&step)) {
			return;
		}
		x[0] -= step.x;
		x[1] -= step.y;
		x[2] -= step.z;
		if (m_vec3_len(step) < 1e-4f) {
			break;
		}
	}
	struct xrt_vec3 bias = {(float)x[0], (float)x[1], (float)x[2]}, zero = {0};
	b->rms_before_mps2 = accel_bias_rms(b, &zero);
	b->rms_after_mps2 = accel_bias_rms(b, &bias);
	// Three directions at least 45 degrees apart, a fit to within 0.1 m/s^2, a plausible size.
	b->valid = b->count >= 3 && b->spread_deg >= 45 && b->rms_after_mps2 < 0.1f && m_vec3_len(bias) < 3.0f;
	if (b->valid) {
		b->bias = bias;
	}
}

// Startup gravity magnitude error above which the accelerometer is treated as offset.
#define RIFT_S_ACCEL_OFFSET_MPS2 0.4f
// Share of the optical tilt difference applied per reliable optical fix while the accelerometer
// is offset: about one second to level, while smoothing the 2-3 degree tilt noise of single fits.
#define RIFT_S_OPTICAL_TILT_FRACTION 0.05f
// Optical levelling counts as settled after this many fixes with a smoothed residual below this.
#define RIFT_S_OPTICAL_TILT_SETTLED_UPDATES 30
#define RIFT_S_OPTICAL_TILT_SETTLED_DEG 3.0f
#define RIFT_S_OPTICAL_TILT_MAX_AGE_NS (300 * U_TIME_1MS_IN_NS)
// Attitude bias samples must agree to this RMS, and the result must be plausible.
#define RIFT_S_ATTITUDE_BIAS_MAX_RMS_MPS2 0.6f
#define RIFT_S_ATTITUDE_BIAS_MAX_MPS2 3.0f

void
rift_s_attitude_bias_add(struct rift_s_attitude_bias *b,
                         const struct xrt_vec3 *rest_mean,
                         const struct xrt_quat *attitude)
{
	// Gravity's reaction in the IMU frame: world up rotated by the inverse attitude.
	struct xrt_quat inverse;
	math_quat_invert(attitude, &inverse);
	const struct xrt_vec3 up = {0, (float)MATH_GRAVITY_M_S2, 0};
	struct xrt_vec3 expected;
	math_quat_rotate_vec3(&inverse, &up, &expected);
	b->samples[b->count % RIFT_S_ATTITUDE_BIAS_SAMPLES] = m_vec3_sub(*rest_mean, expected);
	b->count++;
	unsigned int n = b->count < RIFT_S_ATTITUDE_BIAS_SAMPLES ? b->count : RIFT_S_ATTITUDE_BIAS_SAMPLES;
	struct xrt_vec3 mean = {0};
	for (unsigned int i = 0; i < n; i++) {
		mean = m_vec3_add(mean, m_vec3_mul_scalar(b->samples[i], 1.0f / n));
	}
	float sq = 0;
	for (unsigned int i = 0; i < n; i++) {
		struct xrt_vec3 d = m_vec3_sub(b->samples[i], mean);
		sq += m_vec3_dot(d, d) / n;
	}
	b->bias = mean;
	b->rms_mps2 = sqrtf(sq);
	b->valid = n >= RIFT_S_ATTITUDE_BIAS_MIN_SAMPLES && b->rms_mps2 < RIFT_S_ATTITUDE_BIAS_MAX_RMS_MPS2 &&
	           m_vec3_len(mean) < RIFT_S_ATTITUDE_BIAS_MAX_MPS2;
}

float
rift_s_optical_tilt_correction(const struct xrt_quat *imu,
                               const struct xrt_quat *optical,
                               float fraction,
                               struct xrt_quat *out_correction)
{
	// The device's up direction according to the optical fit, seen through the IMU attitude.
	const struct xrt_vec3 up = {0, 1, 0};
	struct xrt_quat optical_inverse;
	math_quat_invert(optical, &optical_inverse);
	struct xrt_vec3 device_up, v;
	math_quat_rotate_vec3(&optical_inverse, &up, &device_up);
	math_quat_rotate_vec3(imu, &device_up, &v);
	// Rotate v towards up about a horizontal axis: tilt only, heading unchanged.
	struct xrt_vec3 axis;
	math_vec3_cross(&v, &up, &axis);
	float sin_angle = m_vec3_len(axis);
	float angle = atan2f(sin_angle, m_vec3_dot(v, up));
	*out_correction = (struct xrt_quat)XRT_QUAT_IDENTITY;
	if (sin_angle > 1e-6f) {
		axis = m_vec3_mul_scalar(axis, 1.0f / sin_angle);
		math_quat_from_angle_vector(fraction * angle, &axis, out_correction);
	}
	return angle;
}

static void
set_fusion_gravity(struct rift_s_controller *ctrl, bool enabled)
{
	if (enabled) {
		ctrl->fusion.flags |= M_IMU_3DOF_USE_GRAVITY_DUR_20MS;
	} else {
		ctrl->fusion.flags &= ~M_IMU_3DOF_USE_GRAVITY_DUR_20MS;
		// Drop any tilt correction already measured from the offset reading.
		ctrl->fusion.grav.error_angle = 0;
	}
}

/* A radio gap has no measured angular velocity. Preserve the last finite attitude,
 * but rebuild capture-time support and require a fresh optical heading. */
static void
reset_fusion_interval(struct rift_s_controller *ctrl, const struct xrt_quat *orientation)
{
	m_imu_3dof_reset(&ctrl->fusion);
	ctrl->fusion.rot = *orientation;
	ctrl->pose.orientation = *orientation;
	ctrl->attitude_history_count = ctrl->attitude_history_start = 0;
	ctrl->clock_stable_samples = 0;
	ctrl->optical_heading_valid = false;
	ctrl->world_accel_valid = false;
	ctrl->gravity_samples = 0;
	ctrl->gravity_accel_sum = ctrl->gravity_gyro_sum = (struct xrt_vec3){0};
	ctrl->gravity_moving = false;
	ctrl->rest = (struct rift_s_rest_stats){0};
	rift_s_position_filter_freeze(&ctrl->position_filter);
}

static bool
bounded_imu_vector(const struct xrt_vec3 *v, double max_length)
{
	return finite_vec3(v) &&
	       (double)v->x * v->x + (double)v->y * v->y + (double)v->z * v->z <= max_length * max_length;
}

void
rift_s_controller_update_fusion(struct rift_s_controller *ctrl, timepoint_ns timestamp_ns)
{
	// These limits exceed normal hand motion and avoid overflow from corrupt calibration.
	if (!bounded_imu_vector(&ctrl->accel, 2000.0) || !bounded_imu_vector(&ctrl->gyro, 100.0)) {
		ctrl->diagnostics.reject_nonfinite++;
		ctrl->world_accel_valid = false;
		return;
	}
	if (!valid_quat(&ctrl->fusion.rot)) {
		const struct xrt_quat identity = {0, 0, 0, 1};
		reset_fusion_interval(ctrl, &identity);
		ctrl->gravity_initialized = false;
	}
	if (ctrl->fusion.last.timestamp_ns != 0 && (timestamp_ns < (timepoint_ns)ctrl->fusion.last.timestamp_ns ||
	                                            timestamp_ns - ctrl->fusion.last.timestamp_ns > U_TIME_1S_IN_NS)) {
		struct xrt_quat orientation = ctrl->fusion.rot;
		RIFT_S_INFO("Controller IMU hand=%s fusion_gap_ms=%.1f; discarding missing motion", hand_alias(ctrl),
		            (double)(timestamp_ns - ctrl->fusion.last.timestamp_ns) / U_TIME_1MS_IN_NS);
		reset_fusion_interval(ctrl, &orientation);
	}
	if (!ctrl->gravity_initialized) {
		set_fusion_gravity(ctrl, false);
		/* Average specific force in the gyro-integrated frame. Averaging sensor axes
		 * while the hand rotates cancels gravity; requiring 250 ms without motion
		 * prevents acquisition altogether. Outliers have bounded influence. */
		float length = m_vec3_len(ctrl->accel);
		if (timestamp_ns - ctrl->fusion.last.timestamp_ns > 50 * U_TIME_1MS_IN_NS) {
			ctrl->gravity_samples = 0;
			ctrl->gravity_accel_sum = ctrl->gravity_gyro_sum = (struct xrt_vec3){0};
			ctrl->gravity_moving = false;
		}
		if (length > 7.5f && length < 12.0f) {
			struct xrt_vec3 world;
			math_quat_rotate_vec3(&ctrl->fusion.rot, &ctrl->accel, &world);
			if (ctrl->gravity_samples == 0) {
				ctrl->gravity_start_ns = timestamp_ns;
			} else {
				struct xrt_vec3 mean = m_vec3_mul_scalar(ctrl->gravity_accel_sum, 1.f / ctrl->gravity_samples);
				struct xrt_vec3 residual = m_vec3_sub(world, mean);
				float error = m_vec3_len(residual);
				ctrl->gravity_moving |= error > .3f;
				if (error > 1.5f)
					world = m_vec3_add(mean, m_vec3_mul_scalar(residual, 1.5f / error));
			}
			ctrl->gravity_moving |= m_vec3_len(ctrl->gyro) >= .1f;
			ctrl->gravity_accel_sum = m_vec3_add(ctrl->gravity_accel_sum, world);
			ctrl->gravity_gyro_sum = m_vec3_add(ctrl->gravity_gyro_sum, ctrl->gyro);
			ctrl->gravity_samples++;
			if (ctrl->gravity_samples >= 64 && timestamp_ns - ctrl->gravity_start_ns >= 250 * U_TIME_1MS_IN_NS) {
				struct xrt_vec3 mean = m_vec3_mul_scalar(ctrl->gravity_accel_sum, 1.f / ctrl->gravity_samples);
				ctrl->gravity_accel_scale = MATH_GRAVITY_M_S2 / m_vec3_len(mean);
				ctrl->accel_offset = ctrl->gravity_moving ||
				    fabsf(m_vec3_len(mean) - (float)MATH_GRAVITY_M_S2) > RIFT_S_ACCEL_OFFSET_MPS2;
				math_vec3_normalize(&mean);
				const struct xrt_vec3 up = {0, 1, 0};
				struct xrt_quat correction;
				math_quat_from_vec_a_to_vec_b(&mean, &up, &correction);
				math_quat_rotate(&correction, &ctrl->fusion.rot, &ctrl->fusion.rot);
				math_quat_normalize(&ctrl->fusion.rot);
				for (unsigned i = 0; i < ctrl->attitude_history_count; i++) {
					struct rift_s_attitude_sample *sample = attitude_sample(ctrl, i);
					math_quat_rotate(&correction, &sample->orientation, &sample->orientation);
					math_quat_normalize(&sample->orientation);
					sample->tilt_valid = ctrl->clock_stable_samples >= 32;
				}
				set_fusion_gravity(ctrl, !ctrl->accel_offset);
				ctrl->fusion.grav.error_angle = 0;
				ctrl->fusion.grav.level_timestamp_ns = timestamp_ns;
				if (!ctrl->gravity_moving)
					ctrl->fusion.gyro_bias.value = m_vec3_mul_scalar(ctrl->gravity_gyro_sum, 1.f / ctrl->gravity_samples);
				ctrl->gravity_initialized = true;
				RIFT_S_INFO("Controller gravity initialized hand=%s samples=%u moving=%d accel_mps2=%.3f",
				            hand_alias(ctrl), ctrl->gravity_samples, ctrl->gravity_moving,
				            MATH_GRAVITY_M_S2 / ctrl->gravity_accel_scale);
			}
		}
	}
	if (rift_s_rest_stats_push(&ctrl->rest, timestamp_ns, &ctrl->accel, &ctrl->gyro)) {
		bool was_valid = ctrl->accel_bias.valid;
		rift_s_accel_bias_add_window(&ctrl->accel_bias, &ctrl->rest.last_mean);
		// With the attitude levelled by optical tilt, each window gives the whole offset.
		bool settled = ctrl->accel_offset &&
		               ctrl->optical_tilt_updates >= RIFT_S_OPTICAL_TILT_SETTLED_UPDATES &&
		               ctrl->optical_tilt_residual_deg < RIFT_S_OPTICAL_TILT_SETTLED_DEG &&
		               ctrl->last_imu_capture_ns - ctrl->last_optical_tilt_ns <= RIFT_S_OPTICAL_TILT_MAX_AGE_NS;
		if (settled) {
			rift_s_attitude_bias_add(&ctrl->attitude_bias, &ctrl->rest.last_mean, &ctrl->fusion.rot);
		}
		if (ctrl->accel_bias.valid &&
		    (!was_valid || timestamp_ns - ctrl->last_bias_log_ns >= 10 * (timepoint_ns)U_TIME_1S_IN_NS)) {
			ctrl->last_bias_log_ns = timestamp_ns;
			const struct rift_s_accel_bias *b = &ctrl->accel_bias;
			RIFT_S_DEBUG(
			    "Controller accel bias hand=%s bias_mps2=%.3f,%.3f,%.3f windows=%u spread_deg=%.0f"
			    " rest_rms_mps2=%.3f->%.3f",
			    hand_alias(ctrl), b->bias.x, b->bias.y, b->bias.z, b->count, b->spread_deg,
			    b->rms_before_mps2, b->rms_after_mps2);
		}
	}
	/* Keep the acceleration gate active during motion. With an estimated bias, the corrected
	 * reading is gravity at rest in every orientation. Before that, the startup average
	 * estimates a scale; normalizing every packet would hide acceleration. */
	const struct xrt_vec3 *bias = ctrl->accel_bias.valid      ? &ctrl->accel_bias.bias
	                              : ctrl->attitude_bias.valid ? &ctrl->attitude_bias.bias
	                                                          : NULL;
	if (ctrl->accel_offset && bias != NULL) {
		// The offset is known: the corrected reading is gravity again, level to it.
		ctrl->accel_offset = false;
		set_fusion_gravity(ctrl, true);
		RIFT_S_DEBUG(
		    "Controller accel hand=%s state=corrected source=%s bias_mps2=%.3f,%.3f,%.3f"
		    " attitude_samples=%u attitude_rms_mps2=%.3f",
		    hand_alias(ctrl), ctrl->accel_bias.valid ? "rest_spread" : "optical_attitude", bias->x, bias->y,
		    bias->z, ctrl->attitude_bias.count, ctrl->attitude_bias.rms_mps2);
	}
	struct xrt_vec3 accel =
	    bias != NULL ? m_vec3_sub(ctrl->accel, *bias)
	                 : m_vec3_mul_scalar(ctrl->accel, ctrl->gravity_initialized ? ctrl->gravity_accel_scale : 1.0f);
	struct xrt_quat previous = ctrl->fusion.rot;
	m_imu_3dof_update(&ctrl->fusion, timestamp_ns, &accel, &ctrl->gyro);
	if (!valid_quat(&ctrl->fusion.rot)) {
		ctrl->diagnostics.reject_nonfinite++;
		reset_fusion_interval(ctrl, &previous);
		return;
	}
	ctrl->pose.orientation = ctrl->fusion.rot;

	// Specific force in the world frame minus gravity drives the position filter.
	ctrl->world_accel_valid = ctrl->gravity_initialized && valid_quat(&ctrl->fusion.rot);
	if (ctrl->world_accel_valid) {
		math_quat_rotate_vec3(&ctrl->fusion.rot, &accel, &ctrl->world_accel);
		ctrl->world_accel.y -= MATH_GRAVITY_M_S2;
	}
}

// Stop waiting for the IMU descriptor and run with the factory gyro calibration. A descriptor
// that arrives later is still applied. Caller holds the mutex.
static void
imu_descriptor_give_up(struct rift_s_controller *ctrl, const char *reason)
{
	if (ctrl->imu_descriptor_done)
		return;
	ctrl->imu_descriptor_done = true;
	RIFT_S_WARN("No controller IMU descriptor hand=%s device=%016" PRIx64 " reason=%s; keeping factory calibration",
	            hand_alias(ctrl), ctrl->device_id, reason);
}

static void
reset_position_filter(struct rift_s_controller *ctrl)
{
	rift_s_position_filter_reset(&ctrl->position_filter);
	ctrl->reacquire_offset = (struct xrt_vec3){0};
	ctrl->reacquire_started_ns = 0;
	float accel_noise = debug_get_float_option_rift_s_position_accel_noise();
	float bias_walk = debug_get_float_option_rift_s_position_bias_walk();
	if (accel_noise > 0) {
		ctrl->position_filter.accel_noise_density = accel_noise;
	}
	if (bias_walk > 0) {
		ctrl->position_filter.bias_random_walk = bias_walk;
	}
}

// Fixes are captured every 33 ms and delivered 40-60 ms later: without one for 120 ms (two
// missed exposures plus delivery latency) the filter coasts on the IMU alone.
#define RIFT_S_COAST_AFTER_NS (120 * U_TIME_1MS_IN_NS)
// Innovation that re-initializes the filter instead of fusing: the 0.15 m broad association
// gate the Windows tracker uses by default.
#define RIFT_S_POSITION_RESET_DISTANCE_M 0.15f
// Longest render-time extrapolation of aim/grip.
#define RIFT_S_MAX_PREDICTION_NS (100 * U_TIME_1MS_IN_NS)
// Resume the optical filter immediately, but remove the held-pose display offset over 100 ms.
#define RIFT_S_REACQUIRE_BLEND_NS (100 * U_TIME_1MS_IN_NS)

enum position_tracking
{
	POSITION_NONE,     // never fixed: no position estimate
	POSITION_TRACKED,  // fresh optical fix
	POSITION_COASTING, // IMU-only, within the position duration of the loss ladder
	POSITION_LOST,     // held at the last estimate, waiting for an optical reset
};

static enum position_tracking
position_tracking_at(const struct rift_s_controller *ctrl, timepoint_ns t, bool *velocity_tracked)
{
	*velocity_tracked = false;
	if (!ctrl->position_filter.initialized) {
		return POSITION_NONE;
	}
	if (ctrl->position_filter.frozen) {
		return POSITION_LOST;
	}
	time_duration_ns position_duration, velocity_duration;
	rift_s_tracking_loss_durations(ctrl->track_vision_updates, ctrl->track_reliable_updates, &position_duration,
	                               &velocity_duration);
	time_duration_ns age = t > ctrl->last_fix_ns ? t - ctrl->last_fix_ns : 0;
	if (position_duration > RIFT_S_POSITION_FILTER_BRIDGE_NS)
		position_duration = RIFT_S_POSITION_FILTER_BRIDGE_NS;
	if (velocity_duration > position_duration)
		velocity_duration = position_duration;
	*velocity_tracked = age <= velocity_duration;
	if (age > position_duration) {
		return POSITION_LOST;
	}
	return age > RIFT_S_COAST_AFTER_NS ? POSITION_COASTING : POSITION_TRACKED;
}

// Apply the loss ladder at IMU time: coast, then hold the position once it is lost.
static void
update_position_tracking(struct rift_s_controller *ctrl, timepoint_ns capture_ns)
{
	bool velocity_tracked;
	enum position_tracking state = position_tracking_at(ctrl, capture_ns, &velocity_tracked);
	struct rift_s_position_filter *f = &ctrl->position_filter;
	f->coasting = state == POSITION_COASTING;
	if (state == POSITION_LOST && !f->frozen) {
		rift_s_position_filter_freeze(f);
		RIFT_S_DEBUG("Controller position hand=%s state=lost age_ms=%.1f vision_updates=%u reliable_updates=%u"
		             " held=%.3f,%.3f,%.3f",
		             hand_alias(ctrl), (double)(capture_ns - ctrl->last_fix_ns) / U_TIME_1MS_IN_NS,
		             ctrl->track_vision_updates, ctrl->track_reliable_updates, f->state.x[0][0],
		             f->state.x[1][0], f->state.x[2][0]);
		ctrl->track_vision_updates = ctrl->track_reliable_updates = 0;
	}
}

bool
rift_s_imu_clock_needs_resync(uint32_t backwards_us, uint32_t backwards_run)
{
	// A late resend steps back a few ms once. A run of rejects, or two in a row more than a
	// second back, means the accepted anchor was wrong (e.g. a stale timestamp after a host
	// reset without a headset power cycle) and every later sample would be rejected.
	return backwards_run >= RIFT_S_IMU_RESYNC_RUN ||
	       (backwards_run >= 2 && backwards_us > RIFT_S_IMU_RESYNC_BACKWARDS_US);
}

/* Re-anchor the 32-bit device clock on @p imu_timestamp. The 64-bit device time continues from
 * the last accepted sample by the host arrival gap, so fusion time keeps moving forward; the
 * device-to-host mapping and everything captured against it are rebuilt as after a clock
 * discontinuity. Caller holds the mutex. */
static void
resync_imu_clock(struct rift_s_controller *ctrl, uint32_t imu_timestamp, timepoint_ns local_ts)
{
	struct rift_s_fusion_diagnostics *d = &ctrl->diagnostics;
	time_duration_ns gap = local_ts - ctrl->last_imu_local_time_ns;
	if (gap < (time_duration_ns)OS_NS_PER_USEC) {
		gap = OS_NS_PER_USEC;
	}
	RIFT_S_INFO("Controller IMU hand=%s clock_resync=1 backwards_us=%u run=%u previous_us=%u new_us=%u",
	             hand_alias(ctrl), ctrl->imu_timestamp32 - imu_timestamp, d->backwards_run, ctrl->imu_timestamp32,
	             imu_timestamp);
	ctrl->last_imu_device_time_ns += gap;
	d->backwards_run = 0;
	d->clock_resyncs++;
	reset_clock_mapping(ctrl);
}

static void
handle_imu_update(struct rift_s_controller *ctrl,
                  timepoint_ns local_ts,
                  uint32_t imu_timestamp,
                  const int16_t raw_accel[3],
                  const int16_t raw_gyro[3])
{
	/* Logic to update 64-bit ns timestamp from
	 * 32-bit µS device timestamp that wraps every 71.5 minutes */
	uint32_t dt = 0;

	if (ctrl->imu_time_valid) {
		dt = imu_timestamp - ctrl->imu_timestamp32;

		/* Sometimes we see 1-2 repeated IMU updates from a controller,
		 * that must be ignored or else time jumps wildly */
		if (dt == 0 || dt > 2147483648) {
			struct rift_s_fusion_diagnostics *d = &ctrl->diagnostics;
			if (dt == 0) {
				d->reject_duplicate++;
			} else {
				d->reject_clock++;
				d->reject_clock_by[RIFT_S_CLOCK_BACKWARDS]++;
				d->backwards_run++;
				/* An old sample arriving late steps back by a few ms after normal steps; a
				 * corrupted timestamp shows as one large accepted step followed by a run of
				 * backwards samples. */
				if (local_ts - d->last_clock_log_ns >= U_TIME_1S_IN_NS) {
					d->last_clock_log_ns = local_ts;
					RIFT_S_DEBUG(
					    "Controller IMU hand=%s reject=clock_backwards backwards_us=%u run=%u"
					    " last_accepted_step_us=%u arrival_since_accepted_ms=%.1f",
					    hand_alias(ctrl), (uint32_t)(0u - dt), d->backwards_run, d->last_step_us,
					    (double)(local_ts - ctrl->last_imu_local_time_ns) / U_TIME_1MS_IN_NS);
				}
			}
			if (dt != 0 && rift_s_imu_clock_needs_resync(0u - dt, ctrl->diagnostics.backwards_run)) {
				resync_imu_clock(ctrl, imu_timestamp, local_ts);
				dt = 0;
			} else {
				RIFT_S_TRACE("Controller IMU hand=%s reject=%s capture_us=%u arrival_ns=%" PRIi64,
				             hand_alias(ctrl), dt == 0 ? "duplicate" : "clock", imu_timestamp,
				             local_ts);
				return;
			}
		} else {
			ctrl->diagnostics.last_step_us = dt;
			ctrl->diagnostics.backwards_run = 0;
			ctrl->last_imu_device_time_ns += (timepoint_ns)dt * OS_NS_PER_USEC;
		}
	} else {
		ctrl->last_imu_device_time_ns = (timepoint_ns)imu_timestamp * OS_NS_PER_USEC;
		ctrl->imu_time_valid = true;
	}
	bool log_imu = local_ts / U_TIME_1S_IN_NS != ctrl->last_imu_local_time_ns / U_TIME_1S_IN_NS;
	ctrl->imu_timestamp32 = imu_timestamp;
	ctrl->last_imu_local_time_ns = local_ts;

	if (!ctrl->have_calibration || !ctrl->have_config)
		return; /* We need to finish reading the calibration or config blocks first */

	if (!ctrl->imu_descriptor_done) {
		if (local_ts < ctrl->imu_descriptor_deadline_ns)
			return; /* Wait briefly for the IMU descriptor: it selects the gyro correction */
		imu_descriptor_give_up(ctrl, "timeout");
	}

	const float gyro_scale = ctrl->config.gyro_scale;
	const float accel_scale = MATH_GRAVITY_M_S2 * ctrl->config.accel_scale;

	struct xrt_vec3 gyro, accel;

	gyro.x = DEG_TO_RAD(gyro_scale * raw_gyro[0]);
	gyro.y = DEG_TO_RAD(gyro_scale * raw_gyro[1]);
	gyro.z = DEG_TO_RAD(gyro_scale * raw_gyro[2]);

	accel.x = accel_scale * raw_accel[0];
	accel.y = accel_scale * raw_accel[1];
	accel.z = accel_scale * raw_accel[2];

	/* Factory TrackedObject matrices map sensor samples into the LED model frame.
	 * Their biases are in sensor units and are subtracted before the row-major matrix. */
	accel = m_vec3_sub(accel, ctrl->calibration.accel_calibration.offset);
	gyro = m_vec3_sub(gyro, ctrl->gyro_calibration.offset);

	math_matrix_3x3_transform_vec3(&ctrl->calibration.accel_calibration.matrix, &accel, &ctrl->accel);
	math_matrix_3x3_transform_vec3(&ctrl->gyro_calibration.matrix, &gyro, &ctrl->gyro);

	if (!finite_vec3(&ctrl->accel) || !finite_vec3(&ctrl->gyro)) {
		ctrl->diagnostics.reject_nonfinite++;
		return;
	}
	rift_s_controller_update_fusion(ctrl, ctrl->last_imu_device_time_ns);
	if (ctrl->fusion.last.timestamp_ns != (uint64_t)ctrl->last_imu_device_time_ns)
		return; // Rejected samples must not acquire capture-time history.
	if (rift_s_controller_record_attitude(ctrl, ctrl->last_imu_device_time_ns, local_ts)) {
		timepoint_ns capture_ns = attitude_sample(ctrl, ctrl->attitude_history_count - 1)->capture_ns;
		ctrl->last_imu_capture_ns = capture_ns;
		if (ctrl->world_accel_valid) {
			update_position_tracking(ctrl, capture_ns);
			rift_s_position_filter_imu(&ctrl->position_filter, capture_ns, &ctrl->world_accel);
		}
	}
	if (log_imu) {
		RIFT_S_DEBUG("Controller IMU sample hand=%s capture_us=%u raw_accel=%d,%d,%d raw_gyro=%d,%d,%d"
		             " accel_mps2=%.6f,%.6f,%.6f gyro_rps=%.6f,%.6f,%.6f",
		             hand_alias(ctrl), imu_timestamp, raw_accel[0], raw_accel[1], raw_accel[2], raw_gyro[0],
		             raw_gyro[1], raw_gyro[2], ctrl->accel.x, ctrl->accel.y, ctrl->accel.z, ctrl->gyro.x,
		             ctrl->gyro.y, ctrl->gyro.z);
		RIFT_S_TRACE("Controller IMU device=%d ts=%u local=%" PRIu64 " mapped_ns=%" PRIi64 " offset_ns=%" PRIi64
		             " history=%u dt_us=%u accel=%f,%f,%f gyro=%f,%f,%f fusion=%f,%f,%f,%f",
		             ctrl->base.device_type, imu_timestamp, local_ts,
		             ctrl->last_imu_device_time_ns + ctrl->imu_to_host_ns, ctrl->imu_to_host_ns,
		             ctrl->attitude_history_count, dt, ctrl->accel.x, ctrl->accel.y, ctrl->accel.z,
		             ctrl->gyro.x, ctrl->gyro.y, ctrl->gyro.z, ctrl->fusion.rot.x, ctrl->fusion.rot.y,
		             ctrl->fusion.rot.z, ctrl->fusion.rot.w);
	}

#if 0
	RIFT_S_DEBUG("%" PRIx64 " dt %u device time %u ns %" PRIu64
	             " raw accel %d %d %d gyro %d %d %d -> accel %f %f %f  gyro %f %f %f\n",
	             ctrl->device_id, dt, imu_timestamp, ctrl->last_imu_device_time_ns, raw_accel[0], raw_accel[1],
	             raw_accel[2], raw_gyro[0], raw_gyro[1], raw_gyro[2], ctrl->accel.x, ctrl->accel.y, ctrl->accel.z,
	             ctrl->gyro.x, ctrl->gyro.y, ctrl->gyro.z);
#endif
}

bool
rift_s_controller_handle_report(struct rift_s_controller *ctrl,
                                timepoint_ns local_ts,
                                rift_s_controller_report_t *report)
{
#if DUMP_CONTROLLER_STATE
	bool saw_imu_update = false;
#endif
	bool saw_controls_update = false;

	os_mutex_lock(&ctrl->mutex);

	/* Collect state updates */
	ctrl->extra_bytes_len = 0;

	for (int i = 0; i < report->num_info; i++) {
		rift_s_controller_info_block_t *info = report->info + i;

		switch (info->block_id) {
		case RIFT_S_CTRL_MASK08:
			saw_controls_update = true;
			ctrl->mask08 = info->maskbyte.val;
			break;
		case RIFT_S_CTRL_BUTTONS:
			saw_controls_update = true;
			ctrl->buttons = info->maskbyte.val;
			break;
		case RIFT_S_CTRL_FINGERS:
			saw_controls_update = true;
			ctrl->fingers = info->maskbyte.val;
			break;
		case RIFT_S_CTRL_MASK0e:
			saw_controls_update = true;
			ctrl->mask0e = info->maskbyte.val;
			break;
		case RIFT_S_CTRL_TRIGGRIP: {
			saw_controls_update = true;
			ctrl->trigger = (uint16_t)(info->triggrip.vals[1] & 0x0f) << 8 | info->triggrip.vals[0];
			ctrl->grip =
			    (uint16_t)(info->triggrip.vals[1] & 0xf0) >> 4 | ((uint16_t)(info->triggrip.vals[2]) << 4);
			break;
		}
		case RIFT_S_CTRL_JOYSTICK:
			saw_controls_update = true;
			ctrl->joystick_x = info->joystick.val;
			ctrl->joystick_y = info->joystick.val >> 16;
			break;
		case RIFT_S_CTRL_CAPSENSE:
			saw_controls_update = true;
			ctrl->capsense_a_x = info->capsense.a_x;
			ctrl->capsense_b_y = info->capsense.b_y;
			ctrl->capsense_joystick = info->capsense.joystick;
			ctrl->capsense_trigger = info->capsense.trigger;
			break;
		case RIFT_S_CTRL_IMU: {
			int j;

#if DUMP_CONTROLLER_STATE
			/* print the state before updating the IMU timestamp a 2nd time */
			if (saw_imu_update)
				print_controller_state(ctrl);
			saw_imu_update = true;
#endif

			ctrl->imu_unknown_varying2 = info->imu.unknown_varying2;

			for (j = 0; j < 3; j++) {
				ctrl->raw_accel[j] = info->imu.accel[j];
				ctrl->raw_gyro[j] = info->imu.gyro[j];
			}
			handle_imu_update(ctrl, local_ts, info->imu.timestamp, ctrl->raw_accel, ctrl->raw_gyro);
			break;
		}
		default:
			RIFT_S_WARN("Invalid controller info block with ID %02x from device %08" PRIx64
			            ". Please report it.\n",
			            info->block_id, ctrl->device_id);
		}
	}

	if (saw_controls_update)
		ctrl->last_controls_local_time_ns = local_ts;

	if (report->extra_bytes_len > 0) {
		if (report->extra_bytes_len > sizeof(ctrl->extra_bytes)) {
			RIFT_S_WARN("Controller report from %16" PRIx64 " had too many extra bytes - %u (max %u)\n",
			            ctrl->device_id, report->extra_bytes_len,
			            (unsigned int)(sizeof(ctrl->extra_bytes)));
			report->extra_bytes_len = sizeof(ctrl->extra_bytes);
		}
		memcpy(ctrl->extra_bytes, report->extra_bytes, report->extra_bytes_len);
	}
	ctrl->extra_bytes_len = report->extra_bytes_len;

#if DUMP_CONTROLLER_STATE
	print_controller_state(ctrl);
#endif

	/* Finally, update and output the log */
	if (report->flags & 0x04) {
		/* New log line is starting, reset the counter */
		ctrl->log_bytes = 0;
	}

	if (ctrl->log_flags & 0x04 || (ctrl->log_flags & 0x02) != (report->flags & 0x02)) {
		/* New log bytes in this report, collect them */
		for (int i = 0; i < 3; i++) {
			uint8_t c = report->log[i];
			if (c != '\0') {
				if (ctrl->log_bytes == (MAX_LOG_SIZE - 1)) {
					/* Log line got too long... output it */
					ctrl->log[MAX_LOG_SIZE - 1] = '\0';
					RIFT_S_DEBUG("Controller: %s", ctrl->log);
					ctrl->log_bytes = 0;
				}
				ctrl->log[ctrl->log_bytes++] = c;
			} else if (ctrl->log_bytes > 0) {
				/* Found the end of the string */
				ctrl->log[ctrl->log_bytes] = '\0';
				rift_s_hexdump_buffer("Controller debug", ctrl->log, ctrl->log_bytes);
				ctrl->log_bytes = 0;
			}
		}
	}
	ctrl->log_flags = report->flags;

	os_mutex_unlock(&ctrl->mutex);
	return true;
}

struct controller_imu_descriptor_read
{
	struct rift_s_controller *ctrl;
	struct rift_s_system *sys;
	const char *hand;
	uint64_t device_id;
};

static void
ctrl_imu_descriptor_cb(bool success, uint8_t *response_bytes, int response_bytes_len, void *cb_data)
{
	struct controller_imu_descriptor_read *read = cb_data;
	if (!success) {
		// Shutdown can cancel this request after the controller itself was freed.
		RIFT_S_DEBUG("Cancelled controller IMU descriptor read hand=%s device=%016" PRIx64, read->hand,
		             read->device_id);
		free(read);
		return;
	}
	os_mutex_lock(&read->sys->dev_mutex);
	if (read->sys->controllers[0] != read->ctrl && read->sys->controllers[1] != read->ctrl) {
		os_mutex_unlock(&read->sys->dev_mutex);
		free(read);
		return;
	}
	struct rift_s_controller *ctrl = read->ctrl;
	os_mutex_lock(&ctrl->mutex);
	if (read->device_id == ctrl->device_id) {
		if (response_bytes_len >= 0 && rift_s_decode_controller_imu_descriptor(response_bytes, (size_t)response_bytes_len,
		                                            ctrl->imu_descriptor)) {
			// Applies even after the wait timed out; the correction is derived from factory data.
			ctrl->imu_descriptor_done = true;
			ctrl->gyro_calibration = rift_s_controller_gyro_calibration_for_imu(
			    &ctrl->calibration.gyro_calibration, ctrl->imu_descriptor);
			RIFT_S_INFO("Controller IMU hand=%s device=%016" PRIx64 " descriptor=%s gyro_correction=%s",
			            hand_alias(ctrl), ctrl->device_id, ctrl->imu_descriptor,
			            strcmp(ctrl->imu_descriptor, "LSM6DSL") == 0 ? "0.8714285714285714" : "none");
		} else {
			imu_descriptor_give_up(ctrl, "bad reply");
		}
	}
	os_mutex_unlock(&ctrl->mutex);
	os_mutex_unlock(&read->sys->dev_mutex);
	free(read);
}

enum controller_irled_step
{
	IRLED_READ,
	IRLED_WRITE,
	IRLED_VERIFY,
};

struct controller_irled_op
{
	struct rift_s_system *sys;
	const char *hand;
	uint64_t device_id;
	enum controller_irled_step step;
	rift_s_controller_irled_config cfg;
};

static void
ctrl_irled_cb(bool success, uint8_t *response_bytes, int response_bytes_len, void *cb_data);

static void
queue_irled_op(struct controller_irled_op *op)
{
	uint8_t req[12];
	size_t len;
	uint8_t report = RIFT_S_RADIO_REPORT_READ;
	if (op->step == IRLED_WRITE) {
		len = rift_s_encode_controller_irled_write(&op->cfg, req);
		report = RIFT_S_RADIO_REPORT_WRITE;
	} else {
		len = rift_s_encode_controller_irled_read(req);
	}
	rift_s_radio_queue_report(rift_s_system_radio(op->sys), report, op->device_id, req, (int)len, ctrl_irled_cb,
	                          op);
}

/* Same sequence as the Windows driver: read register 0x28, keep the LED on-time, set the
 * period to the controller exposure period, then read it back. Called from the radio thread. */
static void
ctrl_irled_cb(bool success, uint8_t *response_bytes, int response_bytes_len, void *cb_data)
{
	struct controller_irled_op *op = cb_data;
	if (!success) {
		RIFT_S_DEBUG("Cancelled controller IRLED request hand=%s device=%016" PRIx64, op->hand, op->device_id);
		free(op);
		return;
	}
	size_t size = response_bytes_len > 0 ? (size_t)response_bytes_len : 0;
	char hex[3 * 16 + 1] = {0};
	for (size_t i = 0; i < size && i < 16; i++)
		snprintf(hex + 3 * i, 4, "%02x ", response_bytes[i]);

	rift_s_controller_irled_config cfg;
	switch (op->step) {
	case IRLED_READ: {
		bool ok = rift_s_decode_controller_irled_config(response_bytes, size, &cfg);
		RIFT_S_INFO("Controller IRLED hand=%s device=%016" PRIx64 " read=%s period_us=%u ontime_us=%u reply=%s",
		            op->hand, op->device_id, ok ? "ok" : "failed", ok ? cfg.period_us : 0,
		            ok ? cfg.ontime_us : 0, hex);
		uint32_t period = (uint32_t)debug_get_num_option_rift_s_controller_led_period_us();
		if (period == 0) {
			free(op);
			return;
		}
		op->cfg.period_us = period;
		// Windows keeps the controller's on-time and falls back to 19 us when the read fails.
		op->cfg.ontime_us = ok ? cfg.ontime_us : RIFT_S_CONTROLLER_IRLED_DEFAULT_ONTIME_US;
		op->step = IRLED_WRITE;
		queue_irled_op(op);
		return;
	}
	case IRLED_WRITE: {
		uint32_t status = 0;
		bool ok = rift_s_controller_write_succeeded(response_bytes, size, &status);
		RIFT_S_INFO("Controller IRLED hand=%s device=%016" PRIx64
		            " write=%s period_us=%u ontime_us=%u status=0x%x reply=%s",
		            op->hand, op->device_id, ok ? "ok" : "failed", op->cfg.period_us, op->cfg.ontime_us, status,
		            hex);
		op->step = IRLED_VERIFY;
		queue_irled_op(op);
		return;
	}
	case IRLED_VERIFY: {
		bool ok = rift_s_decode_controller_irled_config(response_bytes, size, &cfg);
		bool match = ok && cfg.period_us == op->cfg.period_us && cfg.ontime_us == op->cfg.ontime_us;
		RIFT_S_INFO("Controller IRLED hand=%s device=%016" PRIx64
		            " verify=%s period_us=%u ontime_us=%u reply=%s",
		            op->hand, op->device_id, match ? "ok" : ok ? "mismatch" : "failed", ok ? cfg.period_us : 0,
		            ok ? cfg.ontime_us : 0, hex);
		free(op);
		return;
	}
	}
	free(op);
}

static void
ctrl_config_cb(bool success, uint8_t *response_bytes, int response_bytes_len, struct rift_s_controller *ctrl)
{
	if (!success) {
		RIFT_S_WARN("Failed to read controller config");
		return;
	}
	os_mutex_lock(&ctrl->mutex);
	ctrl->reading_config = false;
	if (response_bytes_len < 0 ||
	    !rift_s_decode_controller_config(response_bytes, (size_t)response_bytes_len, &ctrl->config)) {
		RIFT_S_WARN("Failed to read controller config hand=%s", hand_alias(ctrl));
		os_mutex_unlock(&ctrl->mutex);
		return;
	}
	ctrl->have_config = true;
	if (!ctrl->imu_descriptor_requested) {
		struct controller_imu_descriptor_read *read = U_TYPED_CALLOC(struct controller_imu_descriptor_read);
		ctrl->imu_descriptor_requested = true;
		ctrl->imu_descriptor_deadline_ns = os_monotonic_get_ns() + 2 * U_TIME_1S_IN_NS;
		if (read != NULL) {
			read->ctrl = ctrl;
			read->sys = ctrl->sys;
			read->hand = hand_alias(ctrl);
			read->device_id = ctrl->device_id;
			// The Windows driver uses the 0x32 reply length, then copies 16 descriptor bytes.
			const uint8_t descriptor_req[] = {0x31, response_bytes[4], 0xe8, 0x03};
			rift_s_radio_queue_command(rift_s_system_radio(ctrl->sys), ctrl->device_id, descriptor_req,
			                           sizeof(descriptor_req), ctrl_imu_descriptor_cb, read);
		} else {
			imu_descriptor_give_up(ctrl, "alloc failure");
		}
	}

	if (!ctrl->irled_requested) {
		struct controller_irled_op *op = U_TYPED_CALLOC(struct controller_irled_op);
		ctrl->irled_requested = true;
		if (op != NULL) {
			op->sys = ctrl->sys;
			op->hand = hand_alias(ctrl);
			op->device_id = ctrl->device_id;
			op->step = IRLED_READ;
			queue_irled_op(op);
		}
	}

	RIFT_S_INFO("Read config for controller 0x%16" PRIx64
	            " type %08x. "
	            "limit/scale/hz Accel %u %f %u Gyro %u %f %u",
	            ctrl->device_id, ctrl->device_type, ctrl->config.accel_limit, ctrl->config.accel_scale,
	            ctrl->config.accel_hz, ctrl->config.gyro_limit, ctrl->config.gyro_scale, ctrl->config.gyro_hz);
	os_mutex_unlock(&ctrl->mutex);
}

// One-time firmware model summary per hand, for the left/right audit. Caller holds the mutex.
static void
log_model_check(struct rift_s_controller *ctrl)
{
	if (rift_s_log_level > U_LOGGING_INFO) {
		return;
	}
	struct rift_s_controller_model_report r;
	rift_s_controller_model_check(&ctrl->calibration, &r);
	RIFT_S_INFO(
	    "Controller model hand=%s leds=%u lensing_models=%u normal_len=%.3f..%.3f outward=%u/%u"
	    " centroid=%.4f,%.4f,%.4f imu=%.4f,%.4f,%.4f imu_to_centroid_m=%.4f angles=%.0f..%.0f,%.0f..%.0f"
	    " acc_m det=%.4f rot_deg=%.2f sv=%.4f..%.4f gyro_m det=%.4f rot_deg=%.2f sv=%.4f..%.4f"
	    " acc_gyro_axes_deg=%.2f AccCalibration values=%u det=%.4f rot_deg=%.2f sv=%.4f..%.4f"
	    " offset=%.4f vs_acc_m_deg=%.2f GyroCalibration values=%u det=%.4f rot_deg=%.2f sv=%.4f..%.4f"
	    " offset=%.4f vs_gyro_m_deg=%.2f",
	    hand_alias(ctrl), r.num_leds, r.num_lensing_models, r.normal_length_min, r.normal_length_max,
	    r.outward_normals, r.num_leds, r.centroid.x, r.centroid.y, r.centroid.z, r.imu_position.x, r.imu_position.y,
	    r.imu_position.z, r.imu_to_centroid_m, r.angle_x_min, r.angle_x_max, r.angle_y_min, r.angle_y_max,
	    r.accel_rectification.determinant, r.accel_rectification.rotation_deg, r.accel_rectification.singular_min,
	    r.accel_rectification.singular_max, r.gyro_rectification.determinant, r.gyro_rectification.rotation_deg,
	    r.gyro_rectification.singular_min, r.gyro_rectification.singular_max, r.accel_gyro_axes_deg,
	    r.accel_calibration_values, r.accel_calibration.determinant, r.accel_calibration.rotation_deg,
	    r.accel_calibration.singular_min, r.accel_calibration.singular_max, r.accel_calibration_offset_norm,
	    r.accel_calibration_vs_rectification_deg, r.gyro_calibration_values, r.gyro_calibration.determinant,
	    r.gyro_calibration.rotation_deg, r.gyro_calibration.singular_min, r.gyro_calibration.singular_max,
	    r.gyro_calibration_offset_norm, r.gyro_calibration_vs_rectification_deg);
	// The raw numbers, so the frames can be compared offline without another capture.
	const struct rift_s_controller_imu_calibration *c = &ctrl->calibration;
	const float *a = c->accel.rectification.v, *g = c->gyro.rectification.v;
	const float *ta = c->accel_calibration.matrix.v, *tg = c->gyro_calibration.matrix.v;
	RIFT_S_DEBUG(
	    "Controller IMU calibration hand=%s acc_m=%g,%g,%g;%g,%g,%g;%g,%g,%g acc_b=%g,%g,%g"
	    " gyro_m=%g,%g,%g;%g,%g,%g;%g,%g,%g gyro_b=%g,%g,%g AccCalibration=%g,%g,%g;%g,%g,%g;%g,%g,%g"
	    " bias=%g,%g,%g GyroCalibration=%g,%g,%g;%g,%g,%g;%g,%g,%g bias=%g,%g,%g",
	    hand_alias(ctrl), a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], c->accel.offset.x,
	    c->accel.offset.y, c->accel.offset.z, g[0], g[1], g[2], g[3], g[4], g[5], g[6], g[7], g[8],
	    c->gyro.offset.x, c->gyro.offset.y, c->gyro.offset.z, ta[0], ta[1], ta[2], ta[3], ta[4], ta[5], ta[6],
	    ta[7], ta[8], c->accel_calibration.offset.x, c->accel_calibration.offset.y, c->accel_calibration.offset.z,
	    tg[0], tg[1], tg[2], tg[3], tg[4], tg[5], tg[6], tg[7], tg[8], c->gyro_calibration.offset.x,
	    c->gyro_calibration.offset.y, c->gyro_calibration.offset.z);
	if (r.accel_rectification.reflection || r.gyro_rectification.reflection) {
		RIFT_S_WARN("Controller %s IMU rectification contains a reflection: IMU handedness differs from the"
		            " LED model frame",
		            hand_alias(ctrl));
	}
	if (r.accel_calibration_values != 12 || r.gyro_calibration_values != 12) {
		RIFT_S_DEBUG("Controller model hand=%s suspicious=tracked_object_layout values=%u/%u (expected 12)",
		             hand_alias(ctrl), r.accel_calibration_values, r.gyro_calibration_values);
	}
	if (r.accel_gyro_axes_deg > 5) {
		RIFT_S_DEBUG("Controller model hand=%s suspicious=imu_axes accel_gyro_deg=%.1f", hand_alias(ctrl),
		             r.accel_gyro_axes_deg);
	}
}

static void
ctrl_json_cb(bool success, uint8_t *response_bytes, int response_bytes_len, struct rift_s_controller *ctrl)
{
	if (!success) {
		// Only radio shutdown completes commands unsuccessfully; the controller may be gone.
		RIFT_S_DEBUG("Failed to read controller calibration block");
		return;
	}

	os_mutex_lock(&ctrl->mutex);
	ctrl->reading_calibration = false;

	RIFT_S_TRACE("Got Controller calibration device=%d:\n%s", ctrl->base.device_type, response_bytes);

	bool parsed = rift_s_controller_parse_imu_calibration((char *)response_bytes, &ctrl->calibration) == 0;
	if (parsed) {
		ctrl->gyro_calibration = rift_s_controller_gyro_calibration_for_imu(
		    &ctrl->calibration.gyro_calibration, ctrl->imu_descriptor);
		ctrl->P_device_imu.position = ctrl->calibration.imu_position;
		math_pose_invert(&ctrl->P_device_imu, &ctrl->P_imu_device);
		ctrl->have_calibration = true;
		log_model_check(ctrl);
	} else {
		// A parse failure (including an unexpected FlsVersion) retries after a delay and is
		// counted, so a hand stuck without a model shows up in the summary.
		ctrl->calibration_retry_ns = os_monotonic_get_ns() + U_TIME_1S_IN_NS;
		ctrl->diagnostics.calibration_parse_failures++;
		RIFT_S_ERROR("Failed to parse controller calibration hand=%s failures=%" PRIu64, hand_alias(ctrl),
		             ctrl->diagnostics.calibration_parse_failures);
	}
	os_mutex_unlock(&ctrl->mutex);

	// Calibration callbacks arrive one at a time on the radio thread, so taking the other
	// controller's mutex after releasing ours cannot deadlock.
	struct rift_s_controller *left = ctrl->sys ? ctrl->sys->controllers[0] : NULL;
	struct rift_s_controller *right = ctrl->sys ? ctrl->sys->controllers[1] : NULL;
	if (parsed && left != NULL && right != NULL && rift_s_log_level <= U_LOGGING_DEBUG) {
		os_mutex_lock(&left->mutex);
		os_mutex_lock(&right->mutex);
		if (left->have_calibration && right->have_calibration) {
			struct rift_s_controller_hand_comparison cmp;
			rift_s_controller_compare_hands(&left->calibration, &right->calibration, &cmp);
			RIFT_S_DEBUG("Controller model hands best=%s led_rms_m identical=%.4f mirror_x=%.4f mirror_y=%.4f"
			             " mirror_z=%.4f normal_deg identical=%.1f mirror_x=%.1f mirror_y=%.1f mirror_z=%.1f"
			             " imu_m identical=%.4f mirror_x=%.4f mirror_y=%.4f mirror_z=%.4f",
			             rift_s_hand_relation_name(cmp.best), cmp.position_rms_m[0], cmp.position_rms_m[1],
			             cmp.position_rms_m[2], cmp.position_rms_m[3], cmp.normal_mean_deg[0],
			             cmp.normal_mean_deg[1], cmp.normal_mean_deg[2], cmp.normal_mean_deg[3],
			             cmp.imu_difference_m[0], cmp.imu_difference_m[1], cmp.imu_difference_m[2],
			             cmp.imu_difference_m[3]);
			if (cmp.best != RIFT_S_HANDS_MIRROR_X) {
				RIFT_S_DEBUG("Controller model suspicious=hands_not_mirror_x best=%s: check the right-hand"
				             " model and IMU frame handling",
				             rift_s_hand_relation_name(cmp.best));
			}
		}
		os_mutex_unlock(&right->mutex);
		os_mutex_unlock(&left->mutex);
	}
}

static void
rift_s_update_input_bool(struct rift_s_controller *ctrl, int index, int64_t when_ns, int val)
{
	ctrl->base.inputs[index].timestamp = when_ns;
	ctrl->base.inputs[index].value.boolean = (val != 0);
}

static void
rift_s_update_input_analog(struct rift_s_controller *ctrl, int index, int64_t when_ns, float val)
{
	ctrl->base.inputs[index].timestamp = when_ns;
	ctrl->base.inputs[index].value.vec1.x = val;
}

static void
rift_s_update_input_vec2(struct rift_s_controller *ctrl, int index, int64_t when_ns, float x, float y)
{
	ctrl->base.inputs[index].timestamp = when_ns;
	ctrl->base.inputs[index].value.vec2.x = x;
	ctrl->base.inputs[index].value.vec2.y = y;
}

static xrt_result_t
rift_s_controller_update_inputs(struct xrt_device *xdev)
{
	struct rift_s_controller *ctrl = (struct rift_s_controller *)(xdev);

	os_mutex_lock(&ctrl->mutex);

	uint64_t last_ns = ctrl->last_controls_local_time_ns;

	if (ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER) {
		rift_s_update_input_bool(ctrl, OCULUS_TOUCH_X_CLICK, last_ns, ctrl->buttons & RIFT_S_BUTTON_A_X);
		rift_s_update_input_bool(ctrl, OCULUS_TOUCH_Y_CLICK, last_ns, ctrl->buttons & RIFT_S_BUTTON_B_Y);
		rift_s_update_input_bool(ctrl, OCULUS_TOUCH_MENU_CLICK, last_ns,
		                         ctrl->buttons & RIFT_S_BUTTON_MENU_OCULUS);
		rift_s_update_input_bool(
		    ctrl, OCULUS_TOUCH_X_TOUCH, last_ns,
		    !!((ctrl->fingers & RIFT_S_FINGER_A_X_STRONG) ||
		       ((ctrl->fingers & RIFT_S_FINGER_A_X_WEAK) &&
		        !(ctrl->fingers & (RIFT_S_FINGER_B_Y_STRONG | RIFT_S_FINGER_STICK_STRONG)))));
		rift_s_update_input_bool(
		    ctrl, OCULUS_TOUCH_Y_TOUCH, last_ns,
		    !!((ctrl->fingers & RIFT_S_FINGER_B_Y_STRONG) ||
		       ((ctrl->fingers & RIFT_S_FINGER_B_Y_WEAK) &&
		        !(ctrl->fingers & (RIFT_S_FINGER_A_X_STRONG | RIFT_S_FINGER_STICK_STRONG)))));
	} else {
		rift_s_update_input_bool(ctrl, OCULUS_TOUCH_A_CLICK, last_ns, ctrl->buttons & RIFT_S_BUTTON_A_X);
		rift_s_update_input_bool(ctrl, OCULUS_TOUCH_B_CLICK, last_ns, ctrl->buttons & RIFT_S_BUTTON_B_Y);
		rift_s_update_input_bool(ctrl, OCULUS_TOUCH_SYSTEM_CLICK, last_ns,
		                         ctrl->buttons & RIFT_S_BUTTON_MENU_OCULUS);
		rift_s_update_input_bool(
		    ctrl, OCULUS_TOUCH_A_TOUCH, last_ns,
		    !!((ctrl->fingers & RIFT_S_FINGER_A_X_STRONG) ||
		       ((ctrl->fingers & RIFT_S_FINGER_A_X_WEAK) &&
		        !(ctrl->fingers & (RIFT_S_FINGER_B_Y_STRONG | RIFT_S_FINGER_STICK_STRONG)))));
		rift_s_update_input_bool(
		    ctrl, OCULUS_TOUCH_B_TOUCH, last_ns,
		    !!((ctrl->fingers & RIFT_S_FINGER_B_Y_STRONG) ||
		       ((ctrl->fingers & RIFT_S_FINGER_B_Y_WEAK) &&
		        !(ctrl->fingers & (RIFT_S_FINGER_A_X_STRONG | RIFT_S_FINGER_STICK_STRONG)))));
	}

	rift_s_update_input_analog(ctrl, OCULUS_TOUCH_SQUEEZE_VALUE, last_ns, 1.0 - (float)(ctrl->grip) / 4096.0);
	rift_s_update_input_analog(ctrl, OCULUS_TOUCH_TRIGGER_VALUE, last_ns, 1.0 - (float)(ctrl->trigger) / 4096.0);

	rift_s_update_input_bool(ctrl, OCULUS_TOUCH_TRIGGER_TOUCH, last_ns,
	                         !!(ctrl->fingers & (RIFT_S_FINGER_TRIGGER_WEAK | RIFT_S_FINGER_TRIGGER_STRONG)));

	rift_s_update_input_bool(ctrl, OCULUS_TOUCH_THUMBSTICK_CLICK, last_ns, ctrl->buttons & RIFT_S_BUTTON_STICK);

	rift_s_update_input_bool(ctrl, OCULUS_TOUCH_THUMBSTICK_TOUCH, last_ns,
	                         !!((ctrl->fingers & RIFT_S_FINGER_STICK_STRONG) ||
	                            ((ctrl->fingers & RIFT_S_FINGER_STICK_WEAK) &&
	                             !(ctrl->fingers & (RIFT_S_FINGER_A_X_STRONG | RIFT_S_FINGER_B_Y_STRONG)))));

	rift_s_update_input_vec2(ctrl, OCULUS_TOUCH_THUMBSTICK, last_ns,
	                         (float)(ctrl->joystick_x) / 32768.0, /* FIXME: Scale this properly */
	                         (float)(ctrl->joystick_y) / 32768.0  /* FIXME: Scale this properly */
	);

	/* FIXME: Output touch detections:
	      OCULUS_TOUCH_THUMBREST_TOUCH, - does Rift S have a thumbrest?
	*/
	os_mutex_unlock(&ctrl->mutex);

	return XRT_SUCCESS;
}

static bool
rift_s_controller_get_fusion_pose(struct rift_s_controller *ctrl,
                                  enum xrt_input_name name,
                                  int64_t at_timestamp_ns,
                                  struct xrt_space_relation *out_relation)
{
	out_relation->pose = ctrl->pose;
	struct xrt_vec3 gyro = ctrl->fusion.last.gyro;
	if (name == XRT_INPUT_GENERIC_TRACKER_POSE) {
		struct rift_s_attitude_sample sample;
		if (!rift_s_controller_get_attitude(ctrl, at_timestamp_ns, &sample)) {
			ctrl->diagnostics.reject_history++;
			timepoint_ns now = os_monotonic_get_ns();
			if (now - ctrl->diagnostics.last_history_log_ns >= U_TIME_1S_IN_NS) {
				ctrl->diagnostics.last_history_log_ns = now;
				RIFT_S_INFO("Controller capture hand=%s reject=%s capture_ns=%" PRIi64
				            " processing_ns=%" PRIi64 " latency_ms=%.1f newest_gap_ms=%.3f", hand_alias(ctrl),
				            attitude_rejection_names[ctrl->diagnostics.last_attitude_reject],
				            at_timestamp_ns, now, (double)(now - at_timestamp_ns) / U_TIME_1MS_IN_NS,
				            ctrl->attitude_history_count == 0
				                ? -1.0
				                : (double)(at_timestamp_ns -
				                           attitude_sample(ctrl, ctrl->attitude_history_count - 1)->capture_ns) /
				                      U_TIME_1MS_IN_NS);
			}
			RIFT_S_TRACE("Controller fusion hand=%s reject=history capture_ns=%" PRIi64
			             " processing_ns=%" PRIu64,
			             ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER ? "left" : "right",
			             at_timestamp_ns, os_monotonic_get_ns());
			return false;
		}
		out_relation->pose.orientation = sample.orientation;
		gyro = sample.gyro;
	}
	out_relation->linear_velocity.x = 0.0f;
	out_relation->linear_velocity.y = 0.0f;
	out_relation->linear_velocity.z = 0.0f;

	/*!
	 * @todo This is hack, fusion reports angvel relative to the device but
	 * it needs to be in relation to the base space. Rotating it with the
	 * device orientation is enough to get it into the right space, angular
	 * velocity is a derivative so needs a special rotation.
	 */
	math_quat_rotate_derivative(&out_relation->pose.orientation, &gyro, &out_relation->angular_velocity);

	out_relation->relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                               XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                                                               XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
	return true;
}

// Translation smoothing is only for presentation, never the capture-time association prior.
static bool
apply_reacquire_offset(const struct rift_s_controller *ctrl, timepoint_ns t, struct xrt_vec3 *position)
{
	if (ctrl->reacquire_started_ns == 0 || t >= ctrl->reacquire_started_ns + RIFT_S_REACQUIRE_BLEND_NS) {
		return false;
	}
	double u =
	    t > ctrl->reacquire_started_ns ? (double)(t - ctrl->reacquire_started_ns) / RIFT_S_REACQUIRE_BLEND_NS : 0;
	float weight = (float)(1 - u * u * (3 - 2 * u));
	*position = m_vec3_add(*position, m_vec3_mul_scalar(ctrl->reacquire_offset, weight));
	return true;
}

// Aim/grip: gyro orientation and the unchanged IMU/optical position filter. After the bounded
// bridge, hold a valid estimated position indefinitely with optical confidence cleared.
static bool
predict_aim_grip(struct rift_s_controller *ctrl, int64_t at_timestamp_ns, struct xrt_space_relation *rel)
{
	time_duration_ns ahead = at_timestamp_ns - ctrl->last_imu_capture_ns;
	if (ctrl->last_imu_capture_ns > 0 && ahead > 0) {
		if (ahead > RIFT_S_MAX_PREDICTION_NS) {
			ahead = RIFT_S_MAX_PREDICTION_NS;
		}
		struct xrt_space_relation predicted;
		m_predict_relation(rel, (double)ahead / U_TIME_1S_IN_NS, &predicted);
		rel->pose.orientation = predicted.pose.orientation;
	}

	bool velocity_tracked;
	enum position_tracking state = position_tracking_at(ctrl, at_timestamp_ns, &velocity_tracked);
	const char *source = "unacquired";
	if (state != POSITION_NONE) {
		struct xrt_vec3 velocity;
		rift_s_position_filter_predict(&ctrl->position_filter, at_timestamp_ns, RIFT_S_MAX_PREDICTION_NS,
		                               &rel->pose.position, &velocity);
		rel->relation_flags |= XRT_SPACE_RELATION_POSITION_VALID_BIT;
		if (state != POSITION_LOST) {
			rel->relation_flags |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
		}
		if (state != POSITION_LOST && velocity_tracked) {
			rel->linear_velocity = velocity;
			rel->relation_flags |= XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;
		}
		if (apply_reacquire_offset(ctrl, at_timestamp_ns, &rel->pose.position)) {
			// SteamVR must not extrapolate the filter velocity across this presentation correction.
			rel->linear_velocity = (struct xrt_vec3){0};
			rel->relation_flags &= ~XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT;
		}
		source = state == POSITION_TRACKED ? "optical" : state == POSITION_COASTING ? "coast" : "held";
	}
	if (source != ctrl->position_source) {
		bool fallback = state == POSITION_NONE;
		ctrl->diagnostics.fallback_entries += fallback && !ctrl->fallback_active;
		ctrl->fallback_active = fallback;
		RIFT_S_DEBUG("Controller hand=%s position_source=%s query_ns=%" PRIi64 " last_fix_ns=%" PRIi64
		             " vision_updates=%u reliable_updates=%u",
		             hand_alias(ctrl), source, at_timestamp_ns, ctrl->last_fix_ns, ctrl->track_vision_updates,
		             ctrl->track_reliable_updates);
		ctrl->position_source = source;
		log_tracking_health(ctrl, os_monotonic_get_ns(), true);
	} else {
		log_tracking_health(ctrl, os_monotonic_get_ns(), false);
	}
	return state != POSITION_NONE;
}

xrt_result_t
rift_s_controller_get_tracked_pose(struct xrt_device *xdev,
                                   enum xrt_input_name name,
                                   int64_t at_timestamp_ns,
                                   struct xrt_space_relation *out_relation)
{
	struct rift_s_controller *ctrl = (struct rift_s_controller *)(xdev);

	if (name != XRT_INPUT_TOUCH_AIM_POSE && name != XRT_INPUT_TOUCH_GRIP_POSE &&
	    name != XRT_INPUT_GENERIC_TRACKER_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&ctrl->base, rift_s_log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	struct xrt_relation_chain xrc = {0};

	os_mutex_lock(&ctrl->mutex);
	// Constellation needs the bounded gyro-compensated gravity bootstrap. Aim/grip
	// can expose gyro orientation during its first 250 ms.
	if (name == XRT_INPUT_GENERIC_TRACKER_POSE && !ctrl->gravity_initialized) {
		*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
		os_mutex_unlock(&ctrl->mutex);
		return XRT_ERROR_POSE_NOT_ACTIVE;
	}

	// Innermost first: published pose -> hand/model -> IMU -> world.
	// Constellation queries only the IMU pose and must bypass these offsets.
	if (name != XRT_INPUT_GENERIC_TRACKER_POSE) {
		const struct xrt_pose *offset =
		    name == XRT_INPUT_TOUCH_AIM_POSE ? &ctrl->P_device_aim : &ctrl->P_device_grip;
		m_relation_chain_push_pose(&xrc, offset);
		m_relation_chain_push_pose(&xrc, &ctrl->P_imu_device);
	}

	/* Apply the fusion rotation */
	struct xrt_space_relation *rel = m_relation_chain_reserve(&xrc);

	if (!rift_s_controller_get_fusion_pose(ctrl, name, at_timestamp_ns, rel)) {
		*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
		os_mutex_unlock(&ctrl->mutex);
		return XRT_ERROR_POSE_NOT_ACTIVE;
	}
	bool have_position = true;
	if (name == XRT_INPUT_GENERIC_TRACKER_POSE) {
		// Constellation prior: the last optical position, unchanged by the IMU filter.
		if (ctrl->last_tracked_pose_ts != 0 && at_timestamp_ns >= ctrl->last_tracked_pose_ts) {
			rel->pose.position = ctrl->last_tracked_pose.position;
			rel->relation_flags |= XRT_SPACE_RELATION_POSITION_VALID_BIT;
			if (at_timestamp_ns <= ctrl->last_tracked_pose_ts + 500 * U_TIME_1MS_IN_NS) {
				rel->relation_flags |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
			}
		}
	} else {
		have_position = predict_aim_grip(ctrl, at_timestamp_ns, rel);
	}
	bool position_valid = name == XRT_INPUT_GENERIC_TRACKER_POSE ||
	                      (rel->relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0;
	os_mutex_unlock(&ctrl->mutex);

	m_relation_chain_resolve(&xrc, out_relation);

	if (!have_position || !position_valid) {
		out_relation->relation_flags &=
		    ~(XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
		      XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT);
	}

	return XRT_SUCCESS;
}

static void
rift_s_controller_destroy(struct xrt_device *xdev)
{
	struct rift_s_controller *ctrl = (struct rift_s_controller *)(xdev);

	DRV_TRACE_MARKER();

	RIFT_S_DEBUG("Destroying %s controller", ctrl->device_type == RIFT_S_DEVICE_LEFT_CONTROLLER ? "left" : "right");

	// Tell the tracker that we're going away
	if (ctrl->tracking_connection) {
		t_constellation_tracked_device_connection_disconnect(ctrl->tracking_connection);
		ctrl->tracking_connection = NULL;
	}

	/* Tell the system this controller is going away */
	rift_s_system_remove_controller(ctrl->sys, ctrl);

	/* Release the HMD reference */
	rift_s_system_reference(&ctrl->sys, NULL);

	u_var_remove_root(ctrl);

	m_imu_3dof_close(&ctrl->fusion);
	if (ctrl->imu_clock != NULL) {
		m_clock_windowed_skew_tracker_destroy(ctrl->imu_clock);
	}

	os_mutex_destroy(&ctrl->mutex);

	u_device_free(&ctrl->base);
}

bool
rift_s_controller_get_led_model(struct xrt_device *xdev, struct t_constellation_led_model *led_model)
{
	struct rift_s_controller *ctrl = (struct rift_s_controller *)(xdev);

	os_mutex_lock(&ctrl->mutex);
	if (!ctrl->have_calibration) {
		os_mutex_unlock(&ctrl->mutex);
		return false;
	}
	os_mutex_unlock(&ctrl->mutex);

	t_constellation_led_model_init((int)ctrl->base.device_type, NULL, led_model, ctrl->calibration.num_leds, 0);

	// A Rift S ring can expose only five LEDs on the table or at oblique angles.
	led_model->min_acquisition_leds = 5;

	// Firmware LEDs are device-relative in OpenXR coordinates. Move them into
	// the IMU frame used by GENERIC_TRACKER_POSE, then flip to OpenCV.
	for (int i = 0; i < ctrl->calibration.num_leds; i++) {
		struct t_constellation_led *led = led_model->leds + i;
		struct xrt_vec3 pos, dir;

		math_pose_transform_point(&ctrl->P_imu_device, &ctrl->calibration.leds[i].pos, &pos);
		math_quat_rotate_vec3(&ctrl->P_imu_device.orientation, &ctrl->calibration.leds[i].dir, &dir);

		led->id = i;
		led->pos.x = pos.x;
		led->pos.y = -pos.y;
		led->pos.z = -pos.z;
		math_vec3_normalize(&dir);
		led->dir.x = dir.x;
		led->dir.y = -dir.y;
		led->dir.z = -dir.z;

		led->radius_mm = 3.5;
	}

	return true;
}

static void
log_fusion(struct rift_s_controller *ctrl,
           timepoint_ns capture_ns,
           timepoint_ns processing_ns,
           const char *result,
           const struct t_constellation_pose_observation *obs)
{
	ctrl->diagnostics.last_optical_result = result;
	ctrl->diagnostics.last_optical_processing_ns = processing_ns;
	const float *c = ctrl->last_position_covariance;
	RIFT_S_TRACE("Controller fusion hand=%s capture_ns=%" PRIi64 " processing_ns=%" PRIi64
	             " result=%s hypothesis=%s inliers=%u unique=%u yaw_observed=%d accepted=%" PRIu64
	             " reliable=%" PRIu64 " position_only=%" PRIu64
	             " position_var_m2=%g,%g,%g"
	             " velocity_covariance=unavailable reject_stale=%" PRIu64 " reject_nonfinite=%" PRIu64
	             " reject_clock=%" PRIu64 " reject_duplicate=%" PRIu64 " reject_history=%" PRIu64,
	             hand_alias(ctrl), capture_ns, processing_ns, result,
	             obs && obs->hypothesis ? obs->hypothesis : "unknown", obs ? obs->inliers : 0,
	             obs ? obs->distinct_leds : 0, obs ? obs->orientation_observed : 0,
	             ctrl->diagnostics.optical_accepted, ctrl->diagnostics.optical_reliable,
	             ctrl->diagnostics.optical_position_only, c[0], c[4], c[8], ctrl->diagnostics.reject_stale,
	             ctrl->diagnostics.reject_nonfinite, ctrl->diagnostics.reject_clock,
	             ctrl->diagnostics.reject_duplicate, ctrl->diagnostics.reject_history);
	if (rift_s_log_level <= U_LOGGING_DEBUG &&
	    processing_ns - ctrl->diagnostics.last_summary_ns >= 5 * (timepoint_ns)U_TIME_1S_IN_NS) {
		ctrl->diagnostics.last_summary_ns = processing_ns;
		const struct rift_s_position_filter *pf = &ctrl->position_filter;
		RIFT_S_DEBUG("Controller position summary hand=%s source=%s updates=%" PRIu64 " resets=%" PRIu64
		             " gated=%" PRIu64 " rejected=%" PRIu64 " stale=%" PRIu64
		             " innovation_rms_mm=%.2f (n=%u)"
		             " velocity_mps=%.3f,%.3f,%.3f accel_bias_mps2=%.3f,%.3f,%.3f position_std_mm=%.2f",
		             hand_alias(ctrl), ctrl->position_source ? ctrl->position_source : "none", pf->updates,
		             pf->resets, pf->gated, pf->rejected, pf->stale,
		             pf->innovation_count ? 1000.0 * sqrt(pf->innovation_sq_sum / pf->innovation_count) : 0.0,
		             pf->innovation_count, pf->state.x[0][1], pf->state.x[1][1], pf->state.x[2][1],
		             pf->state.x[0][2], pf->state.x[1][2], pf->state.x[2][2],
		             1000.0 * sqrt(fmax(pf->state.P[0][0][0] + pf->state.P[1][0][0] +
		                                    pf->state.P[2][0][0],
		                                0)));
		ctrl->position_filter.innovation_sq_sum = 0;
		ctrl->position_filter.innovation_count = 0;
		RIFT_S_DEBUG(
		    "Controller fusion summary hand=%s accepted=%" PRIu64 " reliable=%" PRIu64 " position_only=%" PRIu64
		    " fallback_entries=%" PRIu64 " calibration_parse_failures=%" PRIu64
		    " heading_prior=%d position_std_m=%.4f velocity_covariance=unavailable reject_stale=%" PRIu64
		    " reject_nonfinite=%" PRIu64 " reject_clock=%" PRIu64 " (backwards=%" PRIu64
		    " history_order=%" PRIu64 " unmapped=%" PRIu64 " discontinuity=%" PRIu64 " mapped_order=%" PRIu64
		    " resyncs=%u"
		    ") reject_duplicate=%" PRIu64 " reject_history=%" PRIu64
		    " rest_windows=%u rest_mps2=%.3f..%.3f rest_min_dir=%.2f,%.2f,%.2f"
		    " rest_max_dir=%.2f,%.2f,%.2f accel_bias=%s%.3f,%.3f,%.3f bias_windows=%u bias_spread_deg=%.0f"
		    " bias_rms_mps2=%.3f->%.3f accel_offset=%d optical_tilt_updates=%u optical_tilt_residual_deg=%.1f"
		    " attitude_bias=%s%.3f,%.3f,%.3f attitude_samples=%u attitude_rms_mps2=%.3f",
		    hand_alias(ctrl), ctrl->diagnostics.optical_accepted, ctrl->diagnostics.optical_reliable,
		    ctrl->diagnostics.optical_position_only, ctrl->diagnostics.fallback_entries,
		    ctrl->diagnostics.calibration_parse_failures, ctrl->optical_heading_valid,
		    sqrtf(fmaxf(c[0] + c[4] + c[8], 0)), ctrl->diagnostics.reject_stale,
		    ctrl->diagnostics.reject_nonfinite, ctrl->diagnostics.reject_clock,
		    ctrl->diagnostics.reject_clock_by[RIFT_S_CLOCK_BACKWARDS],
		    ctrl->diagnostics.reject_clock_by[RIFT_S_CLOCK_HISTORY_ORDER],
		    ctrl->diagnostics.reject_clock_by[RIFT_S_CLOCK_UNMAPPED],
		    ctrl->diagnostics.reject_clock_by[RIFT_S_CLOCK_DISCONTINUITY],
		    ctrl->diagnostics.reject_clock_by[RIFT_S_CLOCK_MAPPED_ORDER], ctrl->diagnostics.clock_resyncs,
		    ctrl->diagnostics.reject_duplicate, ctrl->diagnostics.reject_history, ctrl->rest.windows,
		    ctrl->rest.min_mps2, ctrl->rest.max_mps2, ctrl->rest.min_dir.x, ctrl->rest.min_dir.y,
		    ctrl->rest.min_dir.z, ctrl->rest.max_dir.x, ctrl->rest.max_dir.y, ctrl->rest.max_dir.z,
		    ctrl->accel_bias.valid ? "" : "pending:", ctrl->accel_bias.bias.x, ctrl->accel_bias.bias.y,
		    ctrl->accel_bias.bias.z, ctrl->accel_bias.count, ctrl->accel_bias.spread_deg,
		    ctrl->accel_bias.rms_before_mps2, ctrl->accel_bias.rms_after_mps2, ctrl->accel_offset,
		    ctrl->optical_tilt_updates, ctrl->optical_tilt_residual_deg,
		    ctrl->attitude_bias.valid ? "" : "pending:", ctrl->attitude_bias.bias.x, ctrl->attitude_bias.bias.y,
		    ctrl->attitude_bias.bias.z, ctrl->attitude_bias.count, ctrl->attitude_bias.rms_mps2);
	}
}

// Per-axis variance of an optical position: the fit covariance diagonal, else 3 mm.
static struct xrt_vec3
optical_position_variance(const float covariance[9])
{
	float v[3];
	for (int i = 0; i < 3; i++) {
		float c = covariance[4 * i];
		v[i] = isfinite(c) && c > 0 ? c : 3e-3f * 3e-3f;
		v[i] = fminf(fmaxf(v[i], 1e-3f * 1e-3f), 0.05f * 0.05f);
	}
	return (struct xrt_vec3){v[0], v[1], v[2]};
}

static enum rift_s_position_fix_result
fuse_optical_position(struct rift_s_controller *ctrl,
                      const struct t_constellation_pose_observation *obs,
                      timepoint_ns processing_ns)
{
	struct xrt_vec3 variance = optical_position_variance(obs->position_covariance);
	bool was_held = ctrl->position_filter.initialized && ctrl->position_filter.frozen;
	struct xrt_vec3 held_position = {0}, velocity;
	if (was_held) {
		rift_s_position_filter_predict(&ctrl->position_filter, processing_ns, RIFT_S_MAX_PREDICTION_NS,
		                               &held_position, &velocity);
		apply_reacquire_offset(ctrl, processing_ns, &held_position);
	}
	bool was_lost = !ctrl->position_filter.initialized || ctrl->position_filter.frozen;
	enum rift_s_position_fix_result result =
	    rift_s_position_filter_fix(&ctrl->position_filter, obs->capture_ns, &obs->pose.position, &variance,
	                               RIFT_S_POSITION_RESET_DISTANCE_M);
	if (result == RIFT_S_POSITION_FIX_STALE) {
		return result;
	}
	if (result == RIFT_S_POSITION_FIX_PENDING) {
		// Not part of a track until confirmed: the held or tracked estimate stays.
		if (ctrl->position_filter.candidate_count == 1 &&
		    obs->capture_ns - ctrl->last_candidate_log_ns >= U_TIME_1S_IN_NS) {
			ctrl->last_candidate_log_ns = obs->capture_ns;
			RIFT_S_DEBUG("Controller position hand=%s state=candidate capture_ns=%" PRIi64
			             " position=%.3f,%.3f,%.3f held=%.3f,%.3f,%.3f rejected=%" PRIu64,
			             hand_alias(ctrl), obs->capture_ns, obs->pose.position.x, obs->pose.position.y,
			             obs->pose.position.z, ctrl->position_filter.state.x[0][0],
			             ctrl->position_filter.state.x[1][0], ctrl->position_filter.state.x[2][0],
			             ctrl->position_filter.rejected);
		}
		return result;
	}
	if (result == RIFT_S_POSITION_FIX_RESET) {
		if (was_held) {
			struct xrt_vec3 resumed_position;
			rift_s_position_filter_predict(&ctrl->position_filter, processing_ns, RIFT_S_MAX_PREDICTION_NS,
			                               &resumed_position, &velocity);
			ctrl->reacquire_offset = m_vec3_sub(held_position, resumed_position);
			ctrl->reacquire_started_ns = processing_ns;
		}
		// A new track: the loss ladder counts from here.
		ctrl->track_vision_updates = ctrl->track_reliable_updates = 0;
		RIFT_S_DEBUG("Controller position hand=%s state=reset reason=%s capture_ns=%" PRIi64
		             " position=%.3f,%.3f,%.3f resets=%" PRIu64 " gated=%" PRIu64,
		             hand_alias(ctrl), was_lost ? "acquire" : "innovation", obs->capture_ns,
		             obs->pose.position.x, obs->pose.position.y, obs->pose.position.z,
		             ctrl->position_filter.resets, ctrl->position_filter.gated);
	}
	ctrl->track_vision_updates++;
	if (obs->orientation_observed && obs->distinct_leds >= 3) {
		ctrl->track_reliable_updates++;
	}
	if (obs->capture_ns > ctrl->last_fix_ns) {
		ctrl->last_fix_ns = obs->capture_ns;
	}
	return result;
}

/* Refine startup and accelerometer tilt with a geometry-verified optical fit. Like optical yaw,
 * the world-frame correction is applied to the current
 * fusion state and the retained capture attitudes. Caller holds the mutex. */
static void
apply_optical_tilt(struct rift_s_controller *ctrl, timepoint_ns capture_ns, const struct xrt_quat *optical)
{
	struct rift_s_attitude_sample at_capture;
	if (!rift_s_controller_get_attitude(ctrl, capture_ns, &at_capture)) {
		return;
	}
	struct xrt_quat correction;
	float tilt =
	    rift_s_optical_tilt_correction(&at_capture.orientation, optical, RIFT_S_OPTICAL_TILT_FRACTION, &correction);
	math_quat_rotate(&correction, &ctrl->fusion.rot, &ctrl->fusion.rot);
	math_quat_normalize(&ctrl->fusion.rot);
	for (unsigned int i = 0; i < ctrl->attitude_history_count; i++) {
		struct rift_s_attitude_sample *sample = attitude_sample(ctrl, i);
		math_quat_rotate(&correction, &sample->orientation, &sample->orientation);
		math_quat_normalize(&sample->orientation);
	}
	ctrl->pose.orientation = ctrl->fusion.rot;
	float tilt_deg = RAD_TO_DEG(tilt);
	ctrl->optical_tilt_residual_deg =
	    ctrl->optical_tilt_updates == 0 ? tilt_deg : 0.9f * ctrl->optical_tilt_residual_deg + 0.1f * tilt_deg;
	ctrl->optical_tilt_updates++;
	ctrl->last_optical_tilt_ns = capture_ns;
}

void
rift_s_controller_push_pose_observation(struct xrt_device *xdev, const struct t_constellation_pose_observation *obs)
{
	struct rift_s_controller *ctrl = (struct rift_s_controller *)(xdev);
	timepoint_ns processing_ns = os_monotonic_get_ns();
	timepoint_ns frame_mono_ns = obs->capture_ns;
	const struct xrt_pose *pose = &obs->pose;
	os_mutex_lock(&ctrl->mutex);
	const char *reject = NULL;
	struct rift_s_attitude_sample historical;
	if (frame_mono_ns <= 0 || frame_mono_ns <= ctrl->last_tracked_pose_ts) {
		ctrl->diagnostics.reject_stale++;
		reject = "reject_stale";
	} else if (!finite_vec3(&pose->position) || !valid_quat(&pose->orientation)) {
		ctrl->diagnostics.reject_nonfinite++;
		reject = "reject_nonfinite";
	} else if (!rift_s_controller_get_attitude(ctrl, frame_mono_ns, &historical)) {
		ctrl->diagnostics.reject_history++;
		reject = "reject_history";
	}
	if (reject != NULL) {
		log_fusion(ctrl, frame_mono_ns, processing_ns, reject, obs);
		os_mutex_unlock(&ctrl->mutex);
		return;
	}
	// Candidates must pass position confirmation before changing the IMU heading or
	// the association prior. Otherwise a rejected fit can steer the next search.
	enum rift_s_position_fix_result position_result = fuse_optical_position(ctrl, obs, processing_ns);
	if (position_result == RIFT_S_POSITION_FIX_PENDING || position_result == RIFT_S_POSITION_FIX_STALE) {
		log_fusion(ctrl, frame_mono_ns, processing_ns, "reject_position", obs);
		os_mutex_unlock(&ctrl->mutex);
		return;
	}
	struct xrt_quat optical = pose->orientation;
	math_quat_normalize(&optical);
	// A position-only fit returns the IMU prior as its orientation. Treating that echo as an
	// optical yaw measurement would keep a drifting heading "verified" indefinitely.
	if (ctrl->update_yaw_from_optical && obs->orientation_observed) {
		struct xrt_quat corrected = historical.orientation;
		float yaw = rift_s_apply_optical_yaw(&corrected, &optical);
		struct xrt_quat correction = {0, sinf(yaw / 2), 0, cosf(yaw / 2)};
		// A world-yaw change commutes with gravity correction. Apply it to the
		// capture state, retained IMU states and current fusion consistently.
		math_quat_rotate(&correction, &ctrl->fusion.rot, &ctrl->fusion.rot);
		math_quat_normalize(&ctrl->fusion.rot);
		for (unsigned int i = 0; i < ctrl->attitude_history_count; i++) {
			struct rift_s_attitude_sample *sample = attitude_sample(ctrl, i);
			math_quat_rotate(&correction, &sample->orientation, &sample->orientation);
			math_quat_normalize(&sample->orientation);
		}
		ctrl->pose.orientation = ctrl->fusion.rot;
		struct xrt_quat inverse, residual;
		math_quat_invert(&corrected, &inverse);
		math_quat_rotate(&optical, &inverse, &residual);
		ctrl->optical_heading_valid = fabsf(residual.y) + fabsf(residual.w) >= 1e-6f &&
		                              2 * atan2f(fabsf(residual.y), fabsf(residual.w)) < DEG_TO_RAD(5.0f);
		ctrl->last_optical_yaw_ts = frame_mono_ns;
		RIFT_S_TRACE("Optical hand=%s capture_ns=%" PRIi64 " yaw_correction_deg=%f heading_prior=%d",
		             hand_alias(ctrl), frame_mono_ns, RAD_TO_DEG(yaw), ctrl->optical_heading_valid);
	} else if (!obs->orientation_observed) {
		ctrl->diagnostics.optical_position_only++;
	}
	if (obs->orientation_observed && obs->distinct_leds >= 3) {
		ctrl->diagnostics.optical_reliable++;
		if (historical.tilt_valid) {
			apply_optical_tilt(ctrl, frame_mono_ns, &optical);
		}
	}
	ctrl->last_tracked_pose_ts = frame_mono_ns;
	ctrl->last_tracked_pose = *pose;
	ctrl->pose.position = pose->position;
	memcpy(ctrl->last_position_covariance, obs->position_covariance, sizeof(ctrl->last_position_covariance));
	ctrl->diagnostics.optical_accepted++;
	log_fusion(ctrl, frame_mono_ns, processing_ns, "accepted", obs);
	os_mutex_unlock(&ctrl->mutex);
}

void
rift_s_controller_push_observed_pose(struct xrt_device *xdev, timepoint_ns frame_mono_ns, const struct xrt_pose *pose)
{
	// Legacy full-pose path: the caller vouches for the orientation.
	struct t_constellation_pose_observation obs = {
	    .capture_ns = frame_mono_ns,
	    .pose = *pose,
	    .orientation_observed = true,
	    .information_rank = 4,
	    .yaw_variance = NAN,
	    .hypothesis = "legacy",
	};
	for (unsigned i = 0; i < 9; i++) {
		obs.position_covariance[i] = NAN;
	}
	rift_s_controller_push_pose_observation(xdev, &obs);
}

static bool
has_heading_prior(struct xrt_device *xdev, timepoint_ns capture_ns)
{
	struct rift_s_controller *ctrl = (struct rift_s_controller *)xdev;
	os_mutex_lock(&ctrl->mutex);
	bool valid = rift_s_controller_has_heading_prior(ctrl, capture_ns);
	os_mutex_unlock(&ctrl->mutex);
	return valid;
}

static struct t_constellation_tracked_device_callbacks tracking_callbacks = {
    .get_led_model = rift_s_controller_get_led_model,
    .notify_frame_received = NULL,
    .push_observed_pose = rift_s_controller_push_observed_pose,
    .has_heading_prior = has_heading_prior,
    .push_pose_observation = rift_s_controller_push_pose_observation,
};

static struct xrt_pose
pose_about_x(float deg, float x, float y, float z)
{
	struct xrt_pose p = XRT_POSE_IDENTITY;
	const struct xrt_vec3 axis = {1, 0, 0};
	math_quat_from_angle_vector(DEG_TO_RAD(deg), &axis, &p.orientation);
	p.position = (struct xrt_vec3){x, y, z};
	return p;
}

void
rift_s_controller_model_frames(bool left, struct xrt_pose *out_model_device, struct xrt_pose *out_model_grip)
{
	const float sx = left ? 1.0f : -1.0f;
	// Oculus publishes the firmware Device frame as HandPose with the default identity hand
	// correction. Its SteamVR driver supplies this HandPoseFromModel transform. Normalize
	// the rounded vendor quaternion before using it in XRT's rigid-pose math.
	struct xrt_pose device_model = {
	    .orientation = {0.3371f, 0.0f, 0.0f, 0.94147f},
	    .position = {-0.007f * sx, 0.036124f, -0.037617f},
	};
	math_quat_normalize(&device_model.orientation);
	math_pose_invert(&device_model, out_model_device);
	// OpenXR grip of the render model: in the handle, 10 cm behind the model origin.
	*out_model_grip = pose_about_x(20.6f, 0.007f * sx, -0.00182941f, 0.1019482f);
}

void
rift_s_controller_pose_frames(bool left, struct xrt_pose *device_aim, struct xrt_pose *device_grip)
{
	// The render-model frame is X right, Y up, Z back: the LED ring is at the front (Z 0.026 to
	// 0.05 m) and the handle runs back along +Z to 0.14 m. Aim relative to grip is the Windows
	// LCON offset: grip = aim * (+60 degrees about X, (0, -0.03, 0.095) m).
	const float sx = left ? 1.0f : -1.0f;
	struct xrt_pose model_device, model_grip, device_model;
	rift_s_controller_model_frames(left, &model_device, &model_grip);
	math_pose_invert(&model_device, &device_model);
	const struct xrt_pose model_aim = pose_about_x(-39.4f, 0.007f * sx, -0.03894766f, 0.00949694f);
	math_pose_transform(&device_model, &model_grip, device_grip);
	math_pose_transform(&device_model, &model_aim, device_aim);
}

struct rift_s_controller *
rift_s_controller_create(struct rift_s_system *sys, enum xrt_device_type device_type)
{
	DRV_TRACE_MARKER();

	enum u_device_alloc_flags flags = (enum u_device_alloc_flags)(U_DEVICE_ALLOC_TRACKING_NONE);

	struct rift_s_controller *ctrl = U_DEVICE_ALLOCATE(struct rift_s_controller, flags, INPUT_INDICES_LAST, 1);
	if (ctrl == NULL) {
		return NULL;
	}

	/* Store a ref to the parent hmd, released in destroy */
	rift_s_system_reference(&ctrl->sys, sys);

	os_mutex_init(&ctrl->mutex);

	rift_s_controller_pose_frames(device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER, &ctrl->P_device_aim,
	                              &ctrl->P_device_grip);

	u_device_populate_function_pointers(&ctrl->base, rift_s_controller_get_tracked_pose, rift_s_controller_destroy);
	ctrl->base.update_inputs = rift_s_controller_update_inputs;
	ctrl->base.get_view_poses = u_device_get_view_poses;
	ctrl->base.name = XRT_DEVICE_TOUCH_CONTROLLER;
	ctrl->base.device_type = device_type;

	ctrl->base.supported.orientation_tracking = true;
	ctrl->base.supported.position_tracking = true;


	if (device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) {
		ctrl->device_type = RIFT_S_DEVICE_LEFT_CONTROLLER;
	} else {
		ctrl->device_type = RIFT_S_DEVICE_RIGHT_CONTROLLER;
	}

	ctrl->pose.orientation.w = 1.0f; // All other values set to zero by U_DEVICE_ALLOCATE (which calls U_CALLOC)
	m_imu_3dof_init(&ctrl->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);

	// Real offset will be updated from the calibration once available
	ctrl->P_imu_device = ctrl->P_device_imu = (struct xrt_pose)XRT_POSE_IDENTITY;

	// Setup inputs and outputs
	if (device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) {
		snprintf(ctrl->base.str, XRT_DEVICE_NAME_LEN, "Oculus Rift S Left Touch Controller");
		snprintf(ctrl->base.serial, XRT_DEVICE_NAME_LEN, "Left Controller");
		SET_TOUCH_INPUT(ctrl, X_CLICK);
		SET_TOUCH_INPUT(ctrl, X_TOUCH);
		SET_TOUCH_INPUT(ctrl, Y_CLICK);
		SET_TOUCH_INPUT(ctrl, Y_TOUCH);
		SET_TOUCH_INPUT(ctrl, MENU_CLICK);
	} else {
		snprintf(ctrl->base.str, XRT_DEVICE_NAME_LEN, "Oculus Rift S Right Touch Controller");
		snprintf(ctrl->base.serial, XRT_DEVICE_NAME_LEN, "Right Controller");
		SET_TOUCH_INPUT(ctrl, A_CLICK);
		SET_TOUCH_INPUT(ctrl, A_TOUCH);
		SET_TOUCH_INPUT(ctrl, B_CLICK);
		SET_TOUCH_INPUT(ctrl, B_TOUCH);
		SET_TOUCH_INPUT(ctrl, SYSTEM_CLICK);
	}

	SET_TOUCH_INPUT(ctrl, SQUEEZE_VALUE);
	SET_TOUCH_INPUT(ctrl, TRIGGER_TOUCH);
	SET_TOUCH_INPUT(ctrl, TRIGGER_VALUE);
	SET_TOUCH_INPUT(ctrl, THUMBSTICK_CLICK);
	SET_TOUCH_INPUT(ctrl, THUMBSTICK_TOUCH);
	SET_TOUCH_INPUT(ctrl, THUMBSTICK);
	SET_TOUCH_INPUT(ctrl, THUMBREST_TOUCH);
	SET_TOUCH_INPUT(ctrl, GRIP_POSE);
	SET_TOUCH_INPUT(ctrl, AIM_POSE);

	ctrl->base.outputs[0].name = XRT_OUTPUT_NAME_TOUCH_HAPTIC;

	ctrl->base.binding_profiles = binding_profiles_rift_s;
	ctrl->base.binding_profile_count = ARRAY_SIZE(binding_profiles_rift_s);

	u_var_add_root(ctrl, ctrl->base.str, true);
	u_var_add_gui_header(ctrl, NULL, "Tracking");
	u_var_add_pose(ctrl, &ctrl->pose, "Tracked Pose");

	u_var_add_pose(ctrl, &ctrl->P_device_aim, "Aim pose in device frame");
	u_var_add_pose(ctrl, &ctrl->P_device_grip, "Grip pose in device frame");

	u_var_add_gui_header(ctrl, NULL, "3DoF Tracking");
	m_imu_3dof_add_vars(&ctrl->fusion, ctrl, "");

	u_var_add_gui_header(ctrl, NULL, "Controls");
	if (device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER) {
		DEBUG_TOUCH_INPUT_BOOL(ctrl, X_CLICK, "X button");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, X_TOUCH, "X button touch");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, Y_CLICK, "Y button");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, Y_TOUCH, "Y button touch");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, MENU_CLICK, "Menu button");
	} else {
		DEBUG_TOUCH_INPUT_BOOL(ctrl, A_CLICK, "A button");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, A_TOUCH, "A button touch");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, B_CLICK, "B button");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, B_TOUCH, "B button touch");
		DEBUG_TOUCH_INPUT_BOOL(ctrl, SYSTEM_CLICK, "Oculus button");
	}

	DEBUG_TOUCH_INPUT_F32(ctrl, SQUEEZE_VALUE, "Grip value");

	DEBUG_TOUCH_INPUT_BOOL(ctrl, TRIGGER_TOUCH, "Trigger touch");
	DEBUG_TOUCH_INPUT_F32(ctrl, TRIGGER_VALUE, "Trigger");
	DEBUG_TOUCH_INPUT_BOOL(ctrl, THUMBSTICK_CLICK, "Thumbstick click");
	DEBUG_TOUCH_INPUT_BOOL(ctrl, THUMBSTICK_TOUCH, "Thumbstick touch");
	DEBUG_TOUCH_INPUT_VEC2(ctrl, THUMBSTICK, "Thumbstick X", "Thumbstick Y");
	DEBUG_TOUCH_INPUT_BOOL(ctrl, THUMBREST_TOUCH, "Thumbrest touch");

	struct rift_s_tracker *tracker = rift_s_system_get_tracker(sys);
	ctrl->update_yaw_from_optical = true;
	reset_position_filter(ctrl);
	ctrl->tracking_connection = rift_s_tracker_add_controller(tracker, &ctrl->base, &tracking_callbacks);
	return ctrl;
}

void
rift_s_controller_update_configuration(struct rift_s_controller *ctrl, uint64_t device_id)
{
	rift_s_radio_state *radio = rift_s_system_radio(ctrl->sys);
	os_mutex_lock(&ctrl->mutex);

	if (ctrl->device_id != device_id) {
		ctrl->device_id = device_id;
		snprintf(ctrl->base.serial, XRT_DEVICE_NAME_LEN, "%016" PRIx64, device_id);
		// If the device ID changed somehow, re-read the JSON blocks
		ctrl->have_config = ctrl->have_calibration = false;
		ctrl->imu_descriptor_requested = ctrl->imu_descriptor_done = false;
		ctrl->irled_requested = false;
		reset_position_filter(ctrl);
		ctrl->track_vision_updates = ctrl->track_reliable_updates = 0;
		ctrl->last_fix_ns = 0;
		ctrl->imu_descriptor[0] = '\0';
		ctrl->gravity_initialized = false;
		ctrl->gravity_moving = false;
		// A bias belongs to one physical accelerometer.
		ctrl->accel_bias = (struct rift_s_accel_bias){0};
		ctrl->attitude_bias = (struct rift_s_attitude_bias){0};
		ctrl->accel_offset = false;
		ctrl->optical_tilt_updates = 0;
		ctrl->optical_tilt_residual_deg = 0;
		set_fusion_gravity(ctrl, true);
		ctrl->imu_time_valid = false;
		ctrl->imu_to_host_ns = 0;
		if (ctrl->imu_clock != NULL) {
			// Re-allocated on the next sample; reset() would keep a stale window position.
			m_clock_windowed_skew_tracker_destroy(ctrl->imu_clock);
			ctrl->imu_clock = NULL;
		}
		ctrl->clock_stable_samples = 0;
		ctrl->attitude_history_count = ctrl->attitude_history_start = 0;
		ctrl->optical_heading_valid = false;
		ctrl->last_tracked_pose_ts = 0;
		ctrl->gravity_samples = 0;
		ctrl->gravity_accel_sum = ctrl->gravity_gyro_sum = (struct xrt_vec3){0};
		m_imu_3dof_reset(&ctrl->fusion);
	}

	timepoint_ns now = os_monotonic_get_ns();
	if (!ctrl->have_config && !ctrl->reading_config) {
		const uint8_t config_req[] = {0x32, 0x20, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		rift_s_radio_queue_command(radio, ctrl->device_id, config_req, sizeof(config_req),
		                           (rift_s_radio_completion_fn)ctrl_config_cb, ctrl);
		ctrl->reading_config = true;
	}

	if (!ctrl->have_calibration && !ctrl->reading_calibration && now >= ctrl->calibration_retry_ns) {
		rift_s_radio_get_json_block(radio, ctrl->device_id, (rift_s_radio_completion_fn)ctrl_json_cb, ctrl);
		ctrl->reading_calibration = true;
	}
	os_mutex_unlock(&ctrl->mutex);
}
