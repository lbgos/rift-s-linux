// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once
#include "xrt/xrt_defines.h"
#include "util/u_time.h"
#ifdef __cplusplus
extern "C" {
#endif

//! The HAL raw-motor register has amplitude only; it uses the motor's native frequency.
size_t
rift_s_encode_haptic_raw(float amplitude, uint8_t out[5]);

//! Zero amplitude stops. Other requests are bounded, including OpenXR's minimum/infinite durations.
time_duration_ns
rift_s_haptic_duration(const struct xrt_output_value_vibration *value);

#ifdef __cplusplus
}
#endif
