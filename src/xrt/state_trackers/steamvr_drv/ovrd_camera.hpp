// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once
#include "openvr_driver.h"
#include "ovrd_camera_queue.hpp"
#include "rifts_passthrough_shm.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_camera.h"
#include "xrt/xrt_tracking.h"
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

class MonadoCamera final : public vr::IVRCameraComponent
{
public:
	explicit MonadoCamera(xrt_device *device, vr::IVRBlockQueue *queue = nullptr, vr::IVRPaths *paths = nullptr);
	bool
	native_ready() const
	{
		return queue_handle != 0;
	}
	~MonadoCamera();
	bool
	GetCameraFrameDimensions(vr::ECameraVideoStreamFormat, uint32_t *, uint32_t *) override;
	bool
	GetCameraFrameBufferingRequirements(int *, uint32_t *) override;
	bool
	SetCameraFrameBuffering(int, void **, uint32_t) override;
	bool
	SetCameraVideoStreamFormat(vr::ECameraVideoStreamFormat format) override;
	vr::ECameraVideoStreamFormat
	GetCameraVideoStreamFormat() override
	{
		return vr::CVS_FORMAT_RGBX32;
	}
	bool
	StartVideoStream() override;
	void
	StopVideoStream() override;
	bool
	IsVideoStreamActive(bool *, float *) override;
	const vr::CameraVideoStreamFrame_t *
	GetVideoStreamFrame() override;
	void
	ReleaseVideoStreamFrame(const vr::CameraVideoStreamFrame_t *) override;
	// Exposure belongs to tracking. Accept its existing automatic mode only.
	bool
	SetAutoExposure(bool enable) override
	{
		return enable;
	}
	bool
	PauseVideoStream() override;
	bool
	ResumeVideoStream() override;
	bool
	GetCameraDistortion(uint32_t, float, float, float *, float *) override;
	bool
	GetCameraProjection(uint32_t, vr::EVRTrackedCameraFrameType, float, float, vr::HmdMatrix44_t *) override;
	bool
	SetFrameRate(int isp, int sensor) override
	{
		return isp == 30 && (sensor == 30 || sensor == 60);
	}
	bool
	SetCameraVideoSinkCallback(vr::ICameraVideoSinkCallback *) override;
	bool
	GetCameraCompatibilityMode(vr::ECameraCompatibilityMode *) override;
	bool
	SetCameraCompatibilityMode(vr::ECameraCompatibilityMode mode) override
	{
		return mode == vr::CAMERA_COMPAT_MODE_BULK_DEFAULT;
	}
	bool
	GetCameraFrameBounds(vr::EVRTrackedCameraFrameType, uint32_t *, uint32_t *, uint32_t *, uint32_t *) override;
	bool
	GetCameraIntrinsics(uint32_t,
	                    vr::EVRTrackedCameraFrameType,
	                    vr::HmdVector2_t *,
	                    vr::HmdVector2_t *,
	                    vr::EVRDistortionFunctionType *,
	                    double *) override;

	static void
	NotifyButton(enum xrt_input_name name, bool pressed);

private:
	struct Buffer
	{
		vr::CameraVideoStreamFrame_t frame{};
		void *data = nullptr;
		bool ready = false, acquired = false;
	};
	xrt_device *device;
	xrt_camera *camera;
	std::mutex mutex;
	std::recursive_mutex callback_mutex;
	vr::ICameraVideoSinkCallback *callback = nullptr;
	std::vector<Buffer> buffers;
	std::vector<std::vector<uint8_t>> storage;
	bool active = false, paused = false; // Protected by mutex.
	std::thread worker;
	int64_t start_ns = 0;
	uint64_t stream_generation = 0, native_sequence = 0;
	uint32_t frame_sequence = 0;

	int shm_fd = -1;
	struct rifts_passthrough_shm *shm = nullptr;
	std::atomic<bool> worker_running{false};
	uint64_t export_sequence = 0;
	int64_t last_menu_press_ns = 0;
	bool system_pressed = false, menu_pressed = false;
	vr::IVRBlockQueue *queue;
	vr::IVRPaths *paths;
	vr::PropertyContainerHandle_t queue_handle = 0;
	vr::PathHandle_t metadata_paths[6]{};
	bool
	initialize_queue();
	void
	publish_queue(const uint8_t *pixels, int64_t timestamp, uint64_t sequence, double interval);
	static std::atomic<MonadoCamera *> s_instance;

	void
	run();
	uint32_t
	bytes() const
	{
		return camera->view_width * 2 * camera->view_height * 4;
	}
};
