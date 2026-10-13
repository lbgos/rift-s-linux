// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
// Minimal OpenVR client loader. It finds the registered SteamVR runtime and loads
// its vrclient.so directly, so the package needs no separate libopenvr_api.
#pragma once

#include <openvr.h>

#include <string>

namespace rd {

struct Vr
{
	vr::IVRSystem *system = nullptr;
	vr::IVROverlay *overlay = nullptr;
	vr::IVRApplications *apps = nullptr;
	vr::IVRCompositor *compositor = nullptr; // Vulkan requirements only; never submits.
};

//! Connect to the running SteamVR server. Never starts SteamVR: returns false if
//! no vrserver process of this user exists.
bool
vr_connect(vr::EVRApplicationType type, Vr &out, std::string &error);

void
vr_disconnect();

//! PID of this user's vrserver, or 0.
int
vrserver_pid();

} // namespace rd
