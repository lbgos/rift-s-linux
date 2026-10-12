// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "rift_s_haptics.h"
#include <math.h>
#include "util/u_time.h"

size_t
rift_s_encode_haptic_raw(float amplitude, uint8_t out[5])
{
	// Radio write register 0x97: one amplitude byte, 8 ms transaction timeout.
	out[0] = 0x97;
	out[1] = 1;
	out[2] = 8;
	out[3] = 0;
	float bounded = isfinite(amplitude) ? fminf(fmaxf(amplitude, 0.0f), 1.0f) : 0.0f;
	out[4] = (uint8_t)lroundf(bounded * 255.0f);
	return 5;
}

time_duration_ns
rift_s_haptic_duration(const struct xrt_output_value_vibration *value)
{
	if (!isfinite(value->amplitude) || value->amplitude <= 0.0f)
		return 0;
	time_duration_ns duration = value->duration_ns;
	if (duration < 5 * U_TIME_1MS_IN_NS)
		duration = 5 * U_TIME_1MS_IN_NS;
	if (duration > 60LL * U_TIME_1S_IN_NS)
		duration = 60LL * U_TIME_1S_IN_NS;
	return duration;
}


bool
rift_s_haptic_next(const struct rift_s_haptic_radio *h, timepoint_ns now, uint64_t *id, uint8_t *amplitude)
{
	if (h->in_flight || now < h->retry_ns)
		return false;
	*id = h->device_id;
	*amplitude = now < h->end_ns ? h->amplitude : 0;
	if (h->applied_amplitude != 0 && h->applied_device_id != *id) {
		*id = h->applied_device_id;
		*amplitude = 0;
		return true;
	}
	return *amplitude != h->applied_amplitude ||
	       (*amplitude != 0 && (*id != h->applied_device_id || now >= h->refresh_ns));
}
