// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Logger code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup st_ovrd
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>

#include "xrt/xrt_compiler.h"
#include "util/u_logging.h"

#ifdef __cplusplus
}
#endif

#include "openvr_driver.h"

static vr::IVRDriverLog *s_pLogFile = NULL;

static inline void
ovrd_log_sink(const char *, int, const char *func, enum u_logging_level level,
              const char *format, va_list args, void *)
{
	if (level < U_LOGGING_WARN || level == U_LOGGING_RAW || !s_pLogFile) {
		return;
	}
	char message[1024], line[1152];
	vsnprintf(message, sizeof(message), format, args);
	snprintf(line, sizeof(line), "%s [%s] %s", level == U_LOGGING_ERROR ? "ERROR" : "WARN",
	         func ? func : "monado", message);
	s_pLogFile->Log(line);
}

static inline void
ovrd_log_init(vr::IVRDriverLog *pDriverLog)
{
	s_pLogFile = pDriverLog;
	// Register before device workers start; remove after they have joined in Cleanup.
	u_log_set_sink(ovrd_log_sink, nullptr);
}

// Cannot use the XRT_PRINTF_FORMAT macro on a function definition.
static inline void
ovrd_log(const char *fmt, ...) XRT_PRINTF_FORMAT(1, 2);

static inline void
ovrd_log(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	char buf[1024];
#if defined(WIN32)
	vsprintf_s(buf, fmt, args);
#else
	vsnprintf(buf, sizeof(buf), fmt, args);
#endif

	if (s_pLogFile)
		s_pLogFile->Log(buf);

	va_end(args);
}

static inline void
ovrd_log_cleanup()
{
	u_log_set_sink(nullptr, nullptr);
	s_pLogFile = nullptr;
}
