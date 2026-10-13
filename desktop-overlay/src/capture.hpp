// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
// One PipeWire video stream of a desktop output. Prefers DMA-BUF (imported into
// Vulkan without CPU copies) and falls back to SHM if import is impossible.
#pragma once

#include "gpu.hpp"

#include <pipewire/pipewire.h>

#include <functional>
#include <string>

namespace rd {

class Capture
{
public:
	//! Called on the PipeWire loop for the newest complete frame. The image is
	//! only valid during the call.
	using FrameFn = std::function<void(GpuImage &image)>;

	Capture(pw_core *core, Gpu &gpu, const std::string &target_object, uint32_t max_fps, FrameFn on_frame);
	~Capture();
	Capture(const Capture &) = delete;
	Capture &
	operator=(const Capture &) = delete;

	bool
	failed() const
	{
		return failed_;
	}
	bool
	dmabuf() const
	{
		return dmabuf_;
	}

private:
	static void
	on_state(void *data, pw_stream_state old, pw_stream_state state, const char *error);
	static void
	on_param(void *data, uint32_t id, const spa_pod *param);
	static void
	on_add_buffer(void *data, pw_buffer *buffer);
	static void
	on_remove_buffer(void *data, pw_buffer *buffer);
	static void
	on_process(void *data);

	void
	offer_formats(bool with_dmabuf);

	Gpu &gpu_;
	FrameFn on_frame_;
	pw_stream *stream_ = nullptr;
	spa_hook listener_ = {};
	uint32_t max_fps_;
	uint32_t width_ = 0, height_ = 0;
	uint64_t modifier_ = 0;
	bool dmabuf_ = false;
	bool dmabuf_failed_ = false;
	bool failed_ = false;
	GpuImage shm_image_;
};

} // namespace rd
