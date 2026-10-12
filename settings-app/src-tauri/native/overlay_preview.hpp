// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once
#include "openvr.h"
#include <cstddef>
#include <cstdint>
// Calls are serialized with the background OpenVR client. Success returns owned RGBA bytes.
const char *rifts_overlay_frame(vr::IVRSystem *, vr::IVRCompositor *, vr::IVROverlay *, vr::IVROverlayView *,
                               const uint8_t **, size_t *, uint32_t *, uint32_t *);
void rifts_overlay_shutdown();
