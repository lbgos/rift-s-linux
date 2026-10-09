#!/usr/bin/env python3
# Copyright 2026, lbgos
# SPDX-License-Identifier: BSL-1.0
"""Check driver_monado's tracked camera through an existing SteamVR session.

Requires Python openvr. Opens no V4L2/HID devices and writes no camera images.
"""
import ctypes
import json
import statistics
import time
import openvr

openvr.init(openvr.VRApplication_Background)
handle = None
try:
    camera = openvr.VRTrackedCamera()
    if not camera.hasCamera(0):
        raise SystemExit("The active SteamVR driver does not advertise a camera")
    kind = openvr.VRTrackedCameraFrameType_Undistorted
    width, height, size = camera.getCameraFrameSize(0, kind)
    pixels = (ctypes.c_uint8 * size)()
    handle = camera.acquireVideoStreamingService(0)
    seen = set()
    arrivals = []
    black = 0
    last_error = None
    end = time.monotonic() + 5
    while time.monotonic() < end:
        try:
            header = camera.getVideoStreamFrameBuffer(handle, kind, pixels, size)
            if header.nFrameSequence not in seen:
                seen.add(header.nFrameSequence)
                arrivals.append(time.monotonic())
                # Ignore alpha when SteamVR converts RGBX32 to public RGBA buffers.
                channels = header.nBytesPerPixel
                if channels not in (3, 4) or header.nWidth * header.nHeight * channels > size:
                    raise SystemExit("Unexpected camera buffer layout")
                stride = max(1, header.nWidth * header.nHeight // 1000) * channels
                black += not any(pixels[offset + c] for offset in range(0, size - channels + 1, stride)
                                 for c in range(3))
        except openvr.error_code.TrackedCameraError as error:
            last_error = type(error).__name__
        time.sleep(0.01)
    intervals = [b - a for a, b in zip(arrivals, arrivals[1:])]
    print(json.dumps({"width": width, "height": height, "bytes": size,
                      "unique_frames": len(seen), "black_frames": black,
                      "median_delivery_hz": 1 / statistics.median(intervals) if intervals else None,
                      "last_error": last_error}))
    if len(seen) < 2:
        raise SystemExit("No advancing video stream; inspect vrserver camera logs")
finally:
    if handle is not None:
        camera.releaseVideoStreamingService(handle)
    openvr.shutdown()
