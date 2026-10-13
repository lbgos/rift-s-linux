// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once

#include <cstdint>
#include <string_view>
#include <charconv>

// Universe 2 contains the saved room. Universe 3 has only an unbounded session origin.
// A session frame must never use universe 2's floor, heading or collision bounds.
static inline uint64_t
rift_s_world_universe(std::string_view status, int64_t stamp, int64_t now, int64_t started = 0)
{
	bool fresh = stamp > 0 && stamp >= started && stamp <= now && now - stamp <= 2000000000LL;
	return fresh && (status == "localized" || status == "setup") ? 2 : 3;
}

// Keep an admitted room through visual-matching misses in the same tracked frame.
// Coordinate-frame replacement is immediate; continuous tracking/status loss gets five seconds.
class RiftSWorldUniverse
{
	int64_t started = 0;
	int64_t lost_since = 0;
	uint64_t generation = 0;
	bool have_generation = false;

	static int64_t field(std::string_view details, std::string_view key)
	{
		while (!details.empty()) {
			auto end = details.find(' ');
			auto token = details.substr(0, end);
			if (token.substr(0, key.size()) == key && token.size() > key.size() && token[key.size()] == '=') {
				int64_t value = -1;
				auto number = token.substr(key.size() + 1);
				auto result = std::from_chars(number.data(), number.data() + number.size(), value);
				return result.ec == std::errc{} && result.ptr == number.data() + number.size() ? value : -1;
			}
			if (end == std::string_view::npos) break;
			details.remove_prefix(end + 1);
		}
		return -1;
	}

public:
	uint64_t update(std::string_view status, int64_t stamp, int64_t now,
	                std::string_view details, uint64_t current)
	{
		if (!started) started = now;
		current = current == 2 ? 2 : 3;
		bool fresh = stamp > 0 && stamp >= started && stamp <= now && now - stamp <= 2000000000LL;
		if (fresh) {
			auto next_generation = field(details, "generation");
			bool replaced = next_generation >= 0 && have_generation &&
			                generation != static_cast<uint64_t>(next_generation);
			if (next_generation >= 0) {
				generation = static_cast<uint64_t>(next_generation);
				have_generation = true;
			}
			if (replaced || field(details, "standby") == 1 || field(details, "restart") == 1 ||
			    field(details, "world_valid") == 0 || field(details, "alignment_changed") == 1 ||
			    status == "unanchored" || status == "save_failed") {
				lost_since = 0;
				return 3;
			}
			if ((status == "setup" || status == "localized") && field(details, "tracking") == 1 && field(details, "world_valid") == 1 &&
			    field(details, "paused") == 0) {
				lost_since = 0;
				return 2;
			}
			if (status == "relocalizing" && field(details, "tracking") == 1 && field(details, "world_valid") == 1 &&
			    field(details, "paused") == 0) {
				lost_since = 0;
				return current;
			}
		}
		if (!lost_since) lost_since = now;
		return now - lost_since >= 5000000000LL ? 3 : current;
	}
};
