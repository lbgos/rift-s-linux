// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once
#include "openvr.h"
#include "ovrd_roomsetup_frame.hpp"
#include <cmath>
#include <cstdio>
#include <optional>

namespace roomsetup {
// OpenVR mailbox and path interfaces use the same client connection as
// IVRSystem.
class Mailbox
{
public:
	virtual int
	Register(const char *name, uint64_t *handle) = 0;
	virtual int
	Unregister(uint64_t handle) = 0;
	virtual int
	Send(uint64_t handle, const char *destination, const char *json) = 0;
	virtual int
	Read(uint64_t handle, char *buffer, uint32_t bytes, uint32_t *required) = 0;
};
struct PathRead
{
	uint64_t path;
	void *buffer;
	uint32_t size, tag, required;
	int error;
	const char *name;
};
class Paths
{
public:
	virtual int
	Read(uint64_t root, PathRead *batch, uint32_t count) = 0;
};

inline std::optional<float>
saved_floor(const vr::HmdMatrix34_t &standing, const vr::HmdMatrix34_t &setup)
{
	float raw_floor[3]{};
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			raw_floor[i] -= standing.m[j][i] * standing.m[j][3];
	float y = setup.m[1][3];
	for (int i = 0; i < 3; ++i)
		y += setup.m[1][i] * raw_floor[i];
	return std::isfinite(y) ? std::optional<float>{y} : std::nullopt;
}

class SavedFloor
{
	Mailbox *mailbox;
	Paths *paths;
	uint64_t handle = 0;
	SharedFrame frame{true};

public:
	SavedFloor(Mailbox *messages, Paths *properties) : mailbox(messages), paths(properties)
	{
		if (mailbox && mailbox->Register("monado/roomsetup-floor", &handle) != 0)
			handle = 0;
	}
	~SavedFloor()
	{
		if (handle)
			mailbox->Unregister(handle);
	}
	bool
	transforms(vr::HmdMatrix34_t &standing, vr::HmdMatrix34_t &setup)
	{
		if (!paths)
			return false;
		PathRead read[] = {{0, &standing, sizeof(standing), 0, 0, 0, "/chaperone/standing/transform"},
		                   {0, &setup, sizeof(setup), 0, 0, 0, "/chaperone/room_setup/transform"}};
		return paths->Read(0x600000003, read, 2) == 0 && !read[0].error && !read[1].error &&
		       read[0].tag == 20 && read[1].tag == 20;
	}
	void
	update_frame(bool active)
	{
		frame.write(active);
	}
	bool
	publish(bool initialize_manual = false)
	{
		if (!handle || !paths)
			return false;
		vr::HmdMatrix34_t standing{}, setup{};
		if (!transforms(standing, setup))
			return false;
		auto y = saved_floor(standing, setup);
		if (!y)
			return false;
		char message[128];
		snprintf(message, sizeof(message), "{\"type\":\"floor_detection_result\",\"pos\":%.9g}", *y);
		bool sent = mailbox->Send(handle, "vrcompositor_systemlayer", message) == 0;
		// The native manual step starts at tracking-origin Y=0. Seed it from the
		// saved plane once; the first stick/laser adjustment selects Manual again.
		if (sent && initialize_manual)
			sent = mailbox->Send(handle, "vrcompositor_systemlayer",
			                     "{\"type\":\"reset_playspace_floorheight\",\"value_"
			                     "source\":2}") == 0;
		fprintf(stderr, "monado-roomsetup-floor: saved_floor_setup_y=%.6f sent=%d\n", *y, sent);
		return sent;
	}
};
} // namespace roomsetup
