// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

namespace roomsetup {
using Matrix = std::array<float, 16>;
inline Matrix
identity()
{
	return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}
inline Matrix
multiply(const Matrix &a, const Matrix &b)
{
	Matrix out{};
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			for (int k = 0; k < 4; ++k)
				out[4 * i + j] += a[4 * i + k] * b[4 * k + j];
	return out;
}
inline Matrix
inverse_rigid(const Matrix &a)
{
	auto out = identity();
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j)
			out[4 * i + j] = a[4 * j + i];
		for (int j = 0; j < 3; ++j)
			out[4 * i + 3] -= out[4 * i + j] * a[4 * j + 3];
	}
	return out;
}
inline uint64_t
monotonic_ns()
{
	timespec now{};
	clock_gettime(CLOCK_MONOTONIC, &now);
	return uint64_t(now.tv_sec) * 1000000000 + now.tv_nsec;
}
struct FrameState
{
	uint64_t sequence, timestamp;
	uint32_t active, reserved;
};
class SharedFrame
{
	FrameState *state = nullptr;
	std::string path = "/monado-roomsetup-frame-" + std::to_string(getuid());
	bool writer;

public:
	explicit SharedFrame(bool write) : writer(write) {}
	~SharedFrame()
	{
		if (state)
			munmap(state, sizeof(FrameState));
		if (writer && state)
			shm_unlink(path.c_str());
	}
	bool
	open()
	{
		if (state)
			return true;
		int fd = shm_open(path.c_str(), (writer ? O_CREAT | O_RDWR : O_RDONLY) | O_CLOEXEC, 0600);
		if (fd < 0)
			return false;
		if (writer && ftruncate(fd, sizeof(FrameState)) != 0) {
			close(fd);
			return false;
		}
		struct stat info{};
		if (fstat(fd, &info) != 0 || info.st_size != sizeof(FrameState) || info.st_uid != getuid()) {
			close(fd);
			return false;
		}
		void *mapped =
		    mmap(nullptr, sizeof(FrameState), writer ? PROT_READ | PROT_WRITE : PROT_READ, MAP_SHARED, fd, 0);
		close(fd);
		if (mapped == MAP_FAILED)
			return false;
		state = static_cast<FrameState *>(mapped);
		return true;
	}
	void
	write(bool active)
	{
		if (!writer || !open())
			return;
		auto sequence = __atomic_load_n(&state->sequence, __ATOMIC_RELAXED);
		__atomic_store_n(&state->sequence, sequence | 1, __ATOMIC_SEQ_CST);
		state->timestamp = monotonic_ns();
		state->active = active;
		__atomic_store_n(&state->sequence, (sequence | 1) + 1, __ATOMIC_RELEASE);
	}
	bool
	read()
	{
		if (writer || !open())
			return false;
		auto sequence = __atomic_load_n(&state->sequence, __ATOMIC_ACQUIRE);
		if (sequence & 1)
			return false;
		FrameState value;
		memcpy(&value, state, sizeof(value));
		if (sequence != __atomic_load_n(&state->sequence, __ATOMIC_ACQUIRE) || !value.active ||
		    monotonic_ns() - value.timestamp > 200000000)
			return false;
		return true;
	}
};

// RGB supplies no measured depth. In every Room View mode, retain capture-to-eye
// rotation but remove translation against the assumed dome and floor surface.
// The same eye ray then samples the same camera pixel at every mesh depth.
inline bool
correct_camera(void *bytes, size_t size)
{
	if (size != 400)
		return false;
	std::array<float, 98> values;
	memcpy(values.data(), bytes, sizeof(values));
	if (!std::all_of(values.begin(), values.end(), [](float v) { return std::isfinite(v); }))
		return false;
	auto model = identity();
	for (int i = 0; i < 16; ++i)
		if (std::abs(values[i] - model[i]) > 1e-5f)
			return false;
	// Exclude UI uniforms and the depth path, which has a nonidentity model.
	if (std::abs(values[46]) < .5f || std::abs(values[47]) > .01f)
		return false;
	Matrix view, camera;
	memcpy(view.data(), values.data() + 16, sizeof(view));
	memcpy(camera.data(), values.data() + 48, sizeof(camera));
	for (int i = 12; i < 16; ++i)
		if (std::abs(view[i] - model[i]) > 1e-5f)
			return false;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			float dot = 0;
			for (int k = 0; k < 3; ++k)
				dot += view[4 * i + k] * view[4 * j + k];
			if (std::abs(dot - (i == j ? 1.f : 0.f)) > .001f)
				return false;
		}
	}
	float dot = 0, norm = 0;
	for (int i = 0; i < 3; ++i) {
		dot += camera[8 + i] * camera[12 + i];
		norm += camera[12 + i] * camera[12 + i];
	}
	if (!std::isfinite(norm) || norm < .25f)
		return false;
	float depth_scale = dot / norm;
	if (!std::isfinite(depth_scale))
		return false;
	for (int i = 0; i < 3; ++i)
		if (std::abs(camera[8 + i] - depth_scale * camera[12 + i]) > .001f)
			return false;
	auto relative = multiply(camera, inverse_rigid(view));
	// Preserve the projection's near/far depth term while dropping translation.
	relative[11] -= depth_scale * relative[15];
	relative[3] = relative[7] = relative[15] = 0;
	auto fixed = multiply(relative, view);
	if (!std::all_of(fixed.begin(), fixed.end(), [](float v) { return std::isfinite(v); }))
		return false;
	memcpy(static_cast<unsigned char *>(bytes) + 192, fixed.data(), sizeof(fixed));
	return true;
}
} // namespace roomsetup
