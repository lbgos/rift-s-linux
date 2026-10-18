// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <string>

// Write a parseable, unbounded session origin before publishing universe 3, replacing any
// offsets left in universe 3 by an earlier session. A missing universe 2 gets the same origin
// so a first Room Setup has a calibrated frame; an existing universe 2 remains unchanged.
bool rift_s_prepare_session_origin(const std::string &path);
bool rift_s_prepare_session_origin();
