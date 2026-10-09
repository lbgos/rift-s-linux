# Copyright 2026, lbgos
# SPDX-License-Identifier: BSL-1.0
"""Versioned camera export reader. One consumer owns the activation flag."""
import fcntl
import mmap
import os
import struct
import time

PATH = "/dev/shm/rifts_passthrough"
HEADER = struct.Struct("<IIIIIIIIQQ")


class Stream:
    def __init__(self):
        self.fd = os.open(PATH, os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.mm = mmap.mmap(self.fd, 0)
            magic, version, self.width, self.height, stride, fmt, *_ = HEADER.unpack_from(self.mm)
            if magic != 0x54535052 or version != 2 or fmt != 0:
                raise ValueError("Incompatible camera export; install matching driver and scripts")
            self.size = stride * self.height
            if not (0 < self.width <= 640 and 0 < self.height <= 320 and stride == self.width * 4
                    and len(self.mm) >= HEADER.size + self.size):
                raise ValueError("Invalid camera export dimensions")
        except BaseException:
            if hasattr(self, "mm"):
                self.mm.close()
            os.close(self.fd)
            raise

    def activate(self, visible):
        # Lease expires if the consumer crashes, so rectification cannot stay on.
        heartbeat = int(time.monotonic() * 1000) & 0xffffffff if visible else 0
        struct.pack_into("<I", self.mm, 24, heartbeat)

    @property
    def toggles(self):
        return struct.unpack_from("<I", self.mm, 28)[0]

    def frame(self, previous):
        for _ in range(3):
            sequence = struct.unpack_from("<Q", self.mm, 32)[0]
            if sequence == previous or sequence == 0 or sequence & 1:
                return None
            timestamp = struct.unpack_from("<Q", self.mm, 40)[0]
            pixels = self.mm[HEADER.size:HEADER.size + self.size]
            if sequence == struct.unpack_from("<Q", self.mm, 32)[0]:
                age_ms = (time.monotonic_ns() - timestamp) / 1e6
                if 0 <= age_ms <= 250:
                    return sequence, timestamp, pixels, age_ms
        return None

    def close(self):
        self.activate(False)
        self.mm.close()
        os.close(self.fd)
