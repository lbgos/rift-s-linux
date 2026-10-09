// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once

#include <stdint.h>

#ifndef RIFTS_PASSTHROUGH_SHM_PATH
#define RIFTS_PASSTHROUGH_SHM_PATH "/dev/shm/rifts_passthrough"
#endif
#define RIFTS_PASSTHROUGH_MAGIC 0x54535052 /* "RPST" */
#define RIFTS_PASSTHROUGH_VERSION 2
#define RIFTS_PASSTHROUGH_WIDTH 640
#define RIFTS_PASSTHROUGH_HEIGHT 320

#ifdef __cplusplus
extern "C" {
#endif

struct rifts_passthrough_shm
{
	uint32_t magic;         /* RIFTS_PASSTHROUGH_MAGIC */
	uint32_t version;       /* RIFTS_PASSTHROUGH_VERSION */
	uint32_t width;         /* 640 */
	uint32_t height;        /* 320 */
	uint32_t stride;        /* 640 * 4 = 2560 */
	uint32_t format;        /* 0 = RGBA32 */
	uint32_t client_active; /* Consumer monotonic-ms heartbeat; 0 = idle */
	uint32_t toggle_count;  /* Incremented on controller button toggle */
	uint64_t sequence;      /* Seqlock: odd during write, even when complete */
	uint64_t timestamp_ns;  /* Monotonic capture timestamp */
	uint8_t pixels[RIFTS_PASSTHROUGH_WIDTH * RIFTS_PASSTHROUGH_HEIGHT * 4];
};

#ifdef __cplusplus
}
#endif
