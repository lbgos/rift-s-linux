// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// Same-user native setup guard and tracker exchange only command IDs/status. Maps live in config.
static inline void
rift_s_world_control_path(const char *name, char *out, size_t size)
{
	const char *runtime = getenv("XDG_RUNTIME_DIR");
	if (runtime && runtime[0]) snprintf(out, size, "%s/monado-rifts-world-%s", runtime, name);
	else snprintf(out, size, "/run/user/%u/monado-rifts-world-%s", (unsigned)getuid(), name);
}
