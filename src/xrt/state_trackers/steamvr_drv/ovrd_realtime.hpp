// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once
#ifdef __linux__
#include <endian.h>
#include <linux/capability.h>
#include <sys/resource.h>
#include <sys/xattr.h>
#include <sys/types.h>

/*!
 * vrcompositor inherits vrserver's RLIMIT_RTTIME. When this process has a zero limit and the
 * launcher at @p launcher carries CAP_SYS_NICE, the compositor's realtime threads are SIGKILLed
 * by the kernel at the first scheduler tick. SteamVR then only reports a compositor crash.
 */
static inline bool
ovrd_realtime_compositor_killed(const char *launcher)
{
	struct rlimit rttime;
	if (getrlimit(RLIMIT_RTTIME, &rttime) != 0 || (rttime.rlim_cur != 0 && rttime.rlim_max != 0))
		return false;
	struct vfs_ns_cap_data caps = {};
	ssize_t size = getxattr(launcher, "security.capability", &caps, sizeof(caps));
	if (size < (ssize_t)(sizeof(caps.magic_etc) + sizeof(caps.data[0])))
		return false;
	return (le32toh(caps.data[0].permitted) & (1u << CAP_SYS_NICE)) != 0;
}
#endif
