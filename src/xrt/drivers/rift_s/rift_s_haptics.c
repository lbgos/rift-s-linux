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
	if (duration > 60 * U_TIME_1S_IN_NS)
		duration = 60 * U_TIME_1S_IN_NS;
	return duration;
}
