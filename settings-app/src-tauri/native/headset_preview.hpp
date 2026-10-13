// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once
#include "openvr.h"
#include <cstddef>
#include <cstdint>

// Zero-copy headset preview, see headset_preview.cpp.

// GTK main thread. `webview` is the WebKitWebView widget; the rectangle is the
// preview box in CSS pixels relative to the webview.
extern "C" void rifts_surface_place(void *webview, int x, int y, int width,
                                    int height, bool visible);
// Preview thread, OpenVR client lock held. Presents at most one frame and
// returns microseconds until the next call, or -1 while hidden.
int64_t rifts_headset_preview_step(vr::IVRSystem *, vr::IVROverlay *,
                                   vr::IVROverlayView *, vr::IVRCompositor *);
// OpenVR client lock held. Frees all GPU and OpenVR resources.
void rifts_headset_preview_shutdown();
// Any thread. Copies the current error (empty while frames flow) into
// `error` and returns its length.
extern "C" size_t rifts_preview_status(uint64_t *presented, uint32_t *width,
                                       uint32_t *height, char *error,
                                       size_t capacity);
