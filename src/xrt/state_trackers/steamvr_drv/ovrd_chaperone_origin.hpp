// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <string>

// Add a parseable, unbounded session origin before publishing universe 3.
// The saved universe and its geometry remain unchanged. Existing entries are preserved.
bool rift_s_prepare_session_origin(const std::string &path);
bool rift_s_prepare_session_origin();
