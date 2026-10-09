// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RIFT_S_METADATA_SIZE 50
#define RIFT_S_METADATA_WIDTH (RIFT_S_METADATA_SIZE * 8 * 8)

struct rift_s_frame_metadata
{
	uint8_t frame_type;
	uint16_t frame_ctr;
	uint64_t frame_ts_us;
	uint32_t frame_ctr2;
	uint16_t exposure[5];
	uint8_t gain[5];
};

enum rift_s_metadata_status
{
	RIFT_S_METADATA_OK,
	RIFT_S_METADATA_BAD_LAYOUT,
	RIFT_S_METADATA_BAD_MAGIC,
	RIFT_S_METADATA_BAD_CRC,
};

static inline uint16_t
rift_s_metadata_crc(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xffff;
	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (int bit = 0; bit < 8; bit++) {
			crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
		}
	}
	return crc;
}

static inline uint64_t
rift_s_metadata_le(const uint8_t *data, size_t len)
{
	uint64_t result = 0;
	for (size_t i = 0; i < len; i++) {
		result |= (uint64_t)data[i] << (8 * i);
	}
	return result;
}

static inline enum rift_s_metadata_status
rift_s_metadata_decode(const uint8_t raw[RIFT_S_METADATA_SIZE], struct rift_s_frame_metadata *out)
{
	if (rift_s_metadata_le(raw + 1, 2) != 0xabcd || rift_s_metadata_le(raw + 48, 2) != 0xface) {
		return RIFT_S_METADATA_BAD_MAGIC;
	}
	if (rift_s_metadata_crc(raw + 16, 30) != rift_s_metadata_le(raw + 46, 2)) {
		return RIFT_S_METADATA_BAD_CRC;
	}

	out->frame_type = raw[0];
	out->frame_ctr = (uint16_t)rift_s_metadata_le(raw + 3, 2);
	out->frame_ts_us = rift_s_metadata_le(raw + 16, 8);
	out->frame_ctr2 = (uint32_t)rift_s_metadata_le(raw + 24, 4);
	for (int i = 0; i < 5; i++) {
		out->exposure[i] = (uint16_t)rift_s_metadata_le(raw + 28 + 2 * i, 2);
		out->gain[i] = raw[40 + i];
	}
	return RIFT_S_METADATA_OK;
}

// Metadata bits occupy 8x8 blocks. Sample their centers using the transport stride.
static inline enum rift_s_metadata_status
rift_s_metadata_extract(
    const uint8_t *data, uint32_t width, uint32_t height, size_t stride, size_t size, struct rift_s_frame_metadata *out)
{
	if (data == NULL || width != RIFT_S_METADATA_WIDTH || height < 8 || stride < width || stride > SIZE_MAX / 4 ||
	    size < width || 4 * stride > size - width) {
		return RIFT_S_METADATA_BAD_LAYOUT;
	}
	uint8_t raw[RIFT_S_METADATA_SIZE] = {0};
	const uint8_t *row = data + 4 * stride;
	for (size_t i = 0; i < RIFT_S_METADATA_SIZE * 8; i++) {
		if (row[4 + 8 * i] > 128) {
			raw[i / 8] |= 1u << (7 - i % 8);
		}
	}
	return rift_s_metadata_decode(raw, out);
}

// Unsigned subtraction accepts 65535 -> 0. Half-range jumps indicate a reset/backwards counter.
static inline bool
rift_s_metadata_counter_delta(uint16_t previous, uint16_t current, uint16_t *missing)
{
	uint16_t delta = (uint16_t)(current - previous);
	*missing = 0;
	if (delta == 0 || delta >= 0x8000) {
		return false;
	}
	*missing = delta - 1;
	return true;
}
