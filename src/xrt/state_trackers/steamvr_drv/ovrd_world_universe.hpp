// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once

#include <cstdint>
#include <string_view>

// Universe 2 contains the saved room. Universe 3 has only an unbounded session origin.
// A session frame must never use universe 2's floor, heading or collision bounds.
static inline uint64_t
rift_s_world_universe(std::string_view status, int64_t stamp, int64_t now, int64_t started = 0)
{
	bool fresh = stamp > 0 && stamp >= started && stamp <= now && now - stamp <= 2000000000LL;
	return fresh && (status == "localized" || status == "setup") ? 2 : 3;
}
