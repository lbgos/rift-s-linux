// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Keep SteamVR's inherited descriptors out of spawned guard processes.
 *
 * The spawner calls roomsetup_spawn_close_from() on its spawn actions. glibc before
 * 2.34 (the Steam Runtime) has no close-from spawn action, so the guard also calls
 * roomsetup_close_inherited() first thing in main().
 */
#pragma once

#include <spawn.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

inline void
roomsetup_spawn_close_from(posix_spawn_file_actions_t *actions, int first)
{
#if defined(__GLIBC__) && (__GLIBC__ > 2 || (__GLIBC__ == 2 && __GLIBC_MINOR__ >= 34))
	posix_spawn_file_actions_addclosefrom_np(actions, first);
#else
	(void)actions;
	(void)first;
#endif
}

inline void
roomsetup_close_inherited(int first)
{
#ifdef SYS_close_range
	if (syscall(SYS_close_range, first, ~0U, 0) == 0) return;
#endif
	// Kernels before 5.9.
	rlimit limit{};
	int last = getrlimit(RLIMIT_NOFILE, &limit) == 0 && limit.rlim_cur != RLIM_INFINITY ? int(limit.rlim_cur) : 65536;
	for (int descriptor = first; descriptor < last; ++descriptor) close(descriptor);
}
