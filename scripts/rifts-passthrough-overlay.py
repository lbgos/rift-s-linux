#!/usr/bin/env python3
# Copyright 2026, lbgos
# SPDX-License-Identifier: BSL-1.0
"""Optional head-relative stereo overlay; native Room View is preferred."""
import argparse
import ctypes
import json
import signal
import statistics
import time

import openvr
from rifts_passthrough import Stream


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--distance", type=float, default=1.0)
    parser.add_argument("--width", type=float, default=2.0)
    parser.add_argument("--start-hidden", action="store_true")
    parser.add_argument("--test-frames", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=0)
    args = parser.parse_args()
    if args.distance <= 0 or args.width <= 0:
        parser.error("distance and width must be positive")
    stream = Stream()
    initialized = False
    handle = None
    visible = not args.start_hidden
    running = True
    delivered = 0
    previous = 0
    toggles = stream.toggles
    arrivals, ages, upload_ms = [], [], []

    def toggle(*_):
        nonlocal visible
        visible = not visible

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGUSR1, toggle)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    start = time.monotonic()
    try:
        openvr.init(openvr.VRApplication_Overlay)
        initialized = True
        overlay = openvr.VROverlay()
        handle = overlay.createOverlay("rifts_passthrough", "Rift S Passthrough")
        overlay.setOverlayFlag(handle, openvr.VROverlayFlags_SideBySide_Parallel, True)
        overlay.setOverlayWidthInMeters(handle, args.width)
        # The packed texture is twice as wide as one eye. Keep the quad square.
        overlay.setOverlayTexelAspect(handle, 0.5)
        overlay.setOverlaySortOrder(handle, 1000)
        matrix = openvr.HmdMatrix34_t()
        for i in range(3):
            matrix.m[i][i] = 1
        matrix.m[2][3] = -args.distance
        overlay.setOverlayTransformTrackedDeviceRelative(handle, openvr.k_unTrackedDeviceIndex_Hmd, matrix)
        vr_system = openvr.VRSystem()
        event = openvr.VREvent_t()
        shown = False
        buffer = (ctypes.c_uint8 * stream.size)()
        print("Ready: right Oculus button or left menu double-click", flush=True)
        while running:
            while vr_system.pollNextEvent(event):
                if event.eventType == openvr.VREvent_Quit:
                    running = False
            current = stream.toggles
            if (current - toggles) & 1:
                toggle()
            toggles = current
            stream.activate(visible)
            frame = stream.frame(previous) if visible else None
            if frame:
                previous, timestamp, pixels, age = frame
                # A stable private copy remains unchanged throughout SetOverlayRaw.
                ctypes.memmove(buffer, pixels, stream.size)
                before = time.monotonic()
                overlay.setOverlayRaw(handle, ctypes.cast(buffer, ctypes.c_void_p), stream.width, stream.height, 4)
                upload_ms.append((time.monotonic() - before) * 1000)
                arrivals.append(time.monotonic())
                ages.append(age)
                if len(arrivals) > 600:
                    arrivals.pop(0)
                    ages.pop(0)
                    upload_ms.pop(0)
                delivered += 1
                if not shown:
                    overlay.showOverlay(handle)
                    shown = True
            elif shown and (not visible or time.monotonic() - arrivals[-1] > 0.25):
                overlay.hideOverlay(handle)
                shown = False
            if args.test_frames and delivered >= args.test_frames:
                break
            if args.timeout and time.monotonic() - start >= args.timeout:
                break
            time.sleep(0.005 if visible else 0.025)
    finally:
        stream.close()
        try:
            if handle is not None:
                overlay.hideOverlay(handle)
                overlay.destroyOverlay(handle)
        finally:
            if initialized:
                openvr.shutdown()
        intervals = [(b - a) * 1000 for a, b in zip(arrivals, arrivals[1:])]
        print(json.dumps({"submitted_frames": delivered, "last_sequence": previous,
                          "median_interval_ms": statistics.median(intervals) if intervals else None,
                          "max_interval_ms": max(intervals) if intervals else None,
                          "median_capture_age_ms": statistics.median(ages) if ages else None,
                          "median_upload_ms": statistics.median(upload_ms) if upload_ms else None}), flush=True)
    if args.test_frames and delivered < args.test_frames:
        raise SystemExit("Camera did not deliver the requested frames")


if __name__ == "__main__":
    main()
