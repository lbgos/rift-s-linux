// Copyright 2026
// SPDX-License-Identifier: BSL-1.0
#pragma once

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>

enum rift_s_usb_result
{
	RIFT_S_USB_OK,
	RIFT_S_USB_RETRY,
	RIFT_S_USB_LOST,
};

static inline enum rift_s_usb_result
rift_s_usb_read_error(int error, uint32_t consecutive)
{
	if (error == ENODEV || error == ENXIO || error == EBADF)
		return RIFT_S_USB_LOST;
	if (error == EINTR || error == EAGAIN || error == EINPROGRESS || error == ETIMEDOUT)
		return RIFT_S_USB_RETRY;
	// Unknown persistent failures need a fresh handle, not an infinite hot loop.
	return consecutive >= 8 ? RIFT_S_USB_LOST : RIFT_S_USB_RETRY;
}

static inline uint64_t
rift_s_usb_backoff_ns(uint32_t consecutive)
{
	uint32_t shift = consecutive < 7 ? consecutive : 7;
	return ((uint64_t)1 << shift) * 1000000;
}
