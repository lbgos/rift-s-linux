// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once
#include "xrt/xrt_defines.h"
#include "util/u_time.h"
#ifdef __cplusplus
extern "C" {
#endif

struct rift_s_haptic_radio
{
	struct rift_s_system *sys;
	unsigned hand;
	uint64_t device_id, sent_device_id, applied_device_id;
	timepoint_ns end_ns, refresh_ns, retry_ns;
	uint8_t amplitude, sent_amplitude, applied_amplitude;
	bool in_flight;
};

//! Discard requests while retaining the identity that may still have a latched motor.
void
rift_s_haptic_transport_lost(struct rift_s_haptic_radio *h);

//! Select the next serialized write, stopping an old identity before its replacement.
bool
rift_s_haptic_next(const struct rift_s_haptic_radio *h, timepoint_ns now, uint64_t *id, uint8_t *amplitude);

//! The HAL raw-motor register has amplitude only; it uses the motor's native frequency.
size_t
rift_s_encode_haptic_raw(float amplitude, uint8_t out[5]);

//! Zero amplitude stops. Other requests are bounded, including OpenXR's minimum/infinite durations.
time_duration_ns
rift_s_haptic_duration(const struct xrt_output_value_vibration *value);

#ifdef __cplusplus
}
#endif
