#!/usr/bin/env python3
# Copyright 2026, lbgos
# SPDX-License-Identifier: BSL-1.0
"""Check advancing, non-black camera export frames and capture-to-reader age."""
import argparse
import hashlib
import json
import statistics
import time
from rifts_passthrough import Stream


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--min-frames", type=int, default=5)
    args = parser.parse_args()
    stream = Stream()
    frames, ages, hashes, sequences = [], [], set(), []
    black = 0
    previous = 0
    try:
        end = time.monotonic() + args.timeout
        while time.monotonic() < end:
            stream.activate(True)
            frame = stream.frame(previous)
            if frame:
                previous, timestamp, pixels, age = frame
                frames.append(time.monotonic())
                ages.append(age)
                sequences.append(previous)
                hashes.add(hashlib.sha256(pixels).digest())
                black += not any(pixels[c::4].strip(b"\0") for c in range(3))
            time.sleep(0.002)
    finally:
        stream.close()
    intervals = [(b - a) * 1000 for a, b in zip(frames, frames[1:])]
    success = len(frames) >= args.min_frames and black < len(frames)
    print(json.dumps({"width": stream.width, "height": stream.height, "advancing_frames": len(frames),
                      "first_sequence": sequences[0] if sequences else None,
                      "last_sequence": previous, "distinct_images": len(hashes), "black_frames": black,
                      "median_delivery_hz": 1000 / statistics.median(intervals) if intervals else None,
                      "median_capture_age_ms": statistics.median(ages) if ages else None,
                      "max_capture_age_ms": max(ages) if ages else None, "success": success}, indent=2))
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
