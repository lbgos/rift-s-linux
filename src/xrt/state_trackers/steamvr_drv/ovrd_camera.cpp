// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "ovrd_camera.hpp"
#include "math/m_api.h"
#include "math/m_space.h"
#include "os/os_time.h"
#include "util/u_debug.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

DEBUG_GET_ONCE_BOOL_OPTION(steamvr_bench, "MONADO_STEAMVR_BENCH", false)
DEBUG_GET_ONCE_BOOL_OPTION(bench_camera, "MONADO_STEAMVR_BENCH_CAMERA", false)

std::atomic<MonadoCamera *> MonadoCamera::s_instance{nullptr};

MonadoCamera::MonadoCamera(xrt_device *device, vr::IVRBlockQueue *queue, vr::IVRPaths *paths)
    : device(device), camera(device->camera), queue(queue), paths(paths)
{
	s_instance.store(this);
	shm_fd = open(RIFTS_PASSTHROUGH_SHM_PATH, O_CREAT | O_RDWR | O_NOFOLLOW | O_CLOEXEC, 0600);
	if (shm_fd >= 0) {
		// Hold the export ownership lock for the camera's lifetime. Consumers can
		// distinguish this producer from a stale export left by an earlier session.
		struct flock owner{};
		owner.l_type = F_WRLCK;
		owner.l_whence = SEEK_SET;
		if (fcntl(shm_fd, F_SETLK, &owner) < 0) {
			close(shm_fd);
			shm_fd = -1;
		}
	}
	if (shm_fd >= 0) {
		fchmod(shm_fd, 0600);
		if (ftruncate(shm_fd, sizeof(struct rifts_passthrough_shm)) == 0) {
			shm = static_cast<struct rifts_passthrough_shm *>(
			    mmap(nullptr, sizeof(struct rifts_passthrough_shm), PROT_READ | PROT_WRITE, MAP_SHARED,
			         shm_fd, 0));
			if (shm == MAP_FAILED) {
				shm = nullptr;
			} else {
				memset(shm, 0, sizeof(*shm));
				shm->magic = RIFTS_PASSTHROUGH_MAGIC;
				shm->version = RIFTS_PASSTHROUGH_VERSION;
				shm->width = camera ? camera->view_width * 2 : RIFTS_PASSTHROUGH_WIDTH;
				shm->height = camera ? camera->view_height : RIFTS_PASSTHROUGH_HEIGHT;
				shm->stride = shm->width * 4;
				shm->format = 0; // RGBA32
				shm->client_active = 0;
				shm->toggle_count = 0;
				shm->sequence = 0;
				shm->timestamp_ns = 0;
			}
		}
	}
	if (queue && paths && !initialize_queue()) {
		if (queue_handle)
			queue->Destroy(queue_handle);
		queue_handle = 0;
	}
	worker_running = true;
	worker = std::thread(&MonadoCamera::run, this);
}

MonadoCamera::~MonadoCamera()
{
	s_instance.store(nullptr);
	StopVideoStream();
	worker_running = false;
	if (worker.joinable())
		worker.join();
	if (queue_handle)
		queue->Destroy(queue_handle);
	if (shm) {
		shm->client_active = 0;
		munmap(shm, sizeof(struct rifts_passthrough_shm));
		shm = nullptr;
	}
	if (shm_fd >= 0) {
		close(shm_fd);
		shm_fd = -1;
	}
}

void
MonadoCamera::NotifyButton(enum xrt_input_name name, bool pressed)
{
	MonadoCamera *self = s_instance.load();
	if (!self || !self->shm || (name != XRT_INPUT_TOUCH_SYSTEM_CLICK && name != XRT_INPUT_TOUCH_MENU_CLICK))
		return;
	// RunFrame reports the held state repeatedly. Toggle only on rising edges.
	bool &previous = name == XRT_INPUT_TOUCH_SYSTEM_CLICK ? self->system_pressed : self->menu_pressed;
	if (pressed == previous)
		return;
	previous = pressed;
	if (!pressed)
		return;

	int64_t now = os_monotonic_get_ns();

	bool toggle = false;
	if (name == XRT_INPUT_TOUCH_SYSTEM_CLICK) {
		toggle = true;
	} else if (name == XRT_INPUT_TOUCH_MENU_CLICK) {
		if (self->last_menu_press_ns != 0 && now - self->last_menu_press_ns < 400 * 1000 * 1000) {
			toggle = true;
			self->last_menu_press_ns = 0;
		} else {
			self->last_menu_press_ns = now;
		}
	}

	if (toggle) {
		__atomic_fetch_add(&self->shm->toggle_count, 1, __ATOMIC_SEQ_CST);
	}
}

bool
MonadoCamera::GetCameraFrameDimensions(vr::ECameraVideoStreamFormat format, uint32_t *width, uint32_t *height)
{
	if (format != vr::CVS_FORMAT_RGBX32 || !width || !height)
		return false;
	*width = camera->view_width * 2;
	*height = camera->view_height;
	return true;
}

bool
MonadoCamera::GetCameraFrameBufferingRequirements(int *count, uint32_t *size)
{
	if (!count || !size)
		return false;
	*count = 3;
	*size = bytes();
	return true;
}

bool
MonadoCamera::SetCameraFrameBuffering(int count, void **data, uint32_t size)
{
	std::lock_guard lock(mutex);
	if (active || count < 2 || count > 32 || !data || size < bytes())
		return false;
	for (const auto &b : buffers)
		if (b.acquired)
			return false;
	for (int i = 0; i < count; ++i)
		if (!data[i])
			return false;
	buffers.clear();
	storage.clear();
	buffers.resize(count);
	for (int i = 0; i < count; ++i)
		buffers[i].data = data[i];
	return true;
}

bool
MonadoCamera::SetCameraVideoStreamFormat(vr::ECameraVideoStreamFormat format)
{
	return format == vr::CVS_FORMAT_RGBX32;
}

bool
MonadoCamera::StartVideoStream()
{
	std::lock_guard callback_lock(callback_mutex);
	std::lock_guard lock(mutex);
	if (active)
		return true;
	for (const auto &b : buffers)
		if (b.acquired)
			return false;
	if (buffers.empty()) {
		storage.resize(3, std::vector<uint8_t>(bytes()));
		buffers.resize(3);
		for (size_t i = 0; i < buffers.size(); ++i)
			buffers[i].data = storage[i].data();
	}
	for (auto &b : buffers)
		b.ready = false;
	start_ns = os_monotonic_get_ns();
	++stream_generation;
	frame_sequence = 0;
	native_sequence = 0;
	paused = false;
	active = true;
	return true;
}

void
MonadoCamera::StopVideoStream()
{
	// Serialize completion with publication and callbacks, but never wait for get_frame.
	std::lock_guard callback_lock(callback_mutex);
	std::lock_guard lock(mutex);
	active = false;
	++stream_generation;
	for (auto &b : buffers)
		b.ready = false;
}

bool
MonadoCamera::IsVideoStreamActive(bool *is_paused, float *elapsed)
{
	std::lock_guard lock(mutex);
	if (is_paused)
		*is_paused = paused;
	if (elapsed)
		*elapsed = active ? (os_monotonic_get_ns() - start_ns) / 1e9f : 0;
	return active;
}

bool
MonadoCamera::PauseVideoStream()
{
	std::lock_guard callback_lock(callback_mutex);
	std::lock_guard lock(mutex);
	paused = true;
	return active;
}

bool
MonadoCamera::ResumeVideoStream()
{
	std::lock_guard callback_lock(callback_mutex);
	std::lock_guard lock(mutex);
	if (!active)
		return false;
	paused = false;
	return true;
}

const vr::CameraVideoStreamFrame_t *
MonadoCamera::GetVideoStreamFrame()
{
	std::lock_guard lock(mutex);
	if (!active || paused)
		return nullptr;
	Buffer *latest = nullptr;
	uint64_t now = os_monotonic_get_ns();
	for (auto &b : buffers) {
		if (b.ready && !b.acquired && now >= b.frame.m_nFrameCaptureTicks_ServerAbsolute &&
		    now - b.frame.m_nFrameCaptureTicks_ServerAbsolute <= 250 * U_TIME_1MS_IN_NS &&
		    (!latest || b.frame.m_nFrameSequence > latest->frame.m_nFrameSequence))
			latest = &b;
	}
	if (!latest)
		return nullptr;
	for (auto &b : buffers)
		if (!b.acquired)
			b.ready = false;
	latest->acquired = true;
	return &latest->frame;
}

void
MonadoCamera::ReleaseVideoStreamFrame(const vr::CameraVideoStreamFrame_t *frame)
{
	std::lock_guard lock(mutex);
	for (auto &b : buffers)
		if (&b.frame == frame)
			b.acquired = false;
}

bool
MonadoCamera::SetCameraVideoSinkCallback(vr::ICameraVideoSinkCallback *sink)
{
	std::lock_guard lock(callback_mutex);
	callback = sink;
	return true;
}

void
MonadoCamera::run()
{
	// Export must not inherit a tracking thread's realtime policy or priority.
	pthread_setname_np(pthread_self(), "rift-s-export");
	sched_param scheduling{};
	pthread_setschedparam(pthread_self(), SCHED_OTHER, &scheduling);
	setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 10);
	int64_t last_timestamp = 0;
	std::vector<uint8_t> rgba(bytes());
	while (worker_running) {
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
		bool stream_active;
		uint64_t generation;
		int64_t stream_start_ns;
		{
			std::lock_guard lock(mutex);
			stream_active = active && !paused;
			generation = stream_generation;
			stream_start_ns = start_ns;
		}
		uint32_t heartbeat = shm ? __atomic_load_n(&shm->client_active, __ATOMIC_ACQUIRE) : 0;
		uint32_t now_ms = os_monotonic_get_ns() / U_TIME_1MS_IN_NS;
		const bool export_active = heartbeat != 0 && static_cast<uint32_t>(now_ms - heartbeat) < 1500;
		bool want_frames = stream_active || export_active;
		if (!want_frames || !camera) {
			if (camera)
				camera->set_active(camera, false);
			std::this_thread::sleep_for(std::chrono::milliseconds(8));
			continue;
		}

		camera->set_active(camera, true);
		xrt_frame *frame = nullptr;
		if (!camera->get_frame(camera, &frame))
			continue;
		if (frame->format != XRT_FORMAT_R8G8B8 || frame->width != camera->view_width * 2 ||
		    frame->height != camera->view_height || frame->stride < frame->width * 3 ||
		    frame->size < frame->stride * frame->height) {
			xrt_frame_reference(&frame, nullptr);
			continue;
		}
		if (frame->timestamp <= last_timestamp) {
			xrt_frame_reference(&frame, nullptr);
			continue;
		}

		// Convert once for both the compositor queue and the overlay export.
		for (uint32_t y = 0; y < frame->height; ++y) {
			const uint8_t *src = frame->data + y * frame->stride;
			uint8_t *dst = rgba.data() + y * frame->width * 4;
			for (uint32_t x = 0; x < frame->width; ++x) {
				memcpy(dst + x * 4, src + x * 3, 3);
				dst[x * 4 + 3] = 255;
			}
		}
		if (debug_get_bool_option_steamvr_bench() && debug_get_bool_option_bench_camera()) {
			// Distinct diagnostic cameras identify eye selection and packed UVs.
			for (uint32_t y = 0; y < frame->height; ++y) {
				for (uint32_t x = 0; x < frame->width; ++x) {
					bool right = x >= camera->view_width;
					bool grid = x % 40 < 2 || y % 40 < 2;
					auto *pixel = rgba.data() + (y * frame->width + x) * 4;
					pixel[0] = right ? (grid ? 255 : 0) : 255;
					pixel[1] = grid ? 255 : 0;
					pixel[2] = right ? 255 : (grid ? 255 : 0);
				}
			}
		}
		if (export_active) {
			// Odd while writing; readers accept only equal, even sequence values.
			__atomic_store_n(&shm->sequence, ++export_sequence, __ATOMIC_SEQ_CST);
			memcpy(shm->pixels, rgba.data(), rgba.size());
			shm->timestamp_ns = frame->timestamp;
			__atomic_store_n(&shm->sequence, ++export_sequence, __ATOMIC_RELEASE);
		}
		// A frame fetched outside the stream lock may belong to a stopped generation.
		if (stream_active && frame->timestamp >= stream_start_ns) {
			xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
			xrt_device_get_tracked_pose(device, XRT_INPUT_GENERIC_HEAD_POSE, frame->timestamp, &rel);
			xrt_relation_chain chain{};
			m_relation_chain_push_relation(&chain, &rel);
			m_relation_chain_push_pose_if_not_identity(&chain, &device->tracking_origin->initial_offset);
			m_relation_chain_resolve(&chain, &rel);
			bool delivered = false;
			std::lock_guard callback_lock(callback_mutex);
			{
				std::lock_guard lock(mutex);
				if (!active || paused || generation != stream_generation) {
					xrt_frame_reference(&frame, nullptr);
					continue;
				}
				if (queue_handle) {
					double interval =
					    last_timestamp ? (frame->timestamp - last_timestamp) / 1e9 : 1.0 / 30;
					publish_queue(rgba.data(), frame->timestamp, native_sequence++, interval);
				}
				for (size_t i = 0; i < buffers.size(); ++i) {
					auto &b = buffers[i];
					if (b.acquired)
						continue;
					memcpy(b.data, rgba.data(), rgba.size());
					b.frame = {};
					b.frame.m_nStreamFormat = vr::CVS_FORMAT_RGBX32;
					b.frame.m_nWidth = frame->width;
					b.frame.m_nHeight = frame->height;
					b.frame.m_nImageDataSize = bytes();
					b.frame.m_nFrameSequence = frame_sequence++;
					b.frame.m_nBufferIndex = i;
					b.frame.m_nBufferCount = buffers.size();
					b.frame.m_flFrameElapsedTime = (frame->timestamp - start_ns) / 1e9;
					b.frame.m_flFrameDeliveryRate = 30;
					b.frame.m_flFrameCaptureTime_DriverAbsolute = frame->timestamp / 1e9;
					b.frame.m_flFrameCaptureTime_ServerRelative =
					    (static_cast<int64_t>(frame->timestamp) -
					     static_cast<int64_t>(os_monotonic_get_ns())) /
					    1e9;
					b.frame.m_nFrameCaptureTicks_ServerAbsolute = frame->timestamp;
					xrt_matrix_4x4 matrix;
					math_matrix_4x4_isometry_from_pose(&rel.pose, &matrix);
					for (int row = 0; row < 3; ++row)
						for (int col = 0; col < 4; ++col)
							b.frame.m_RawTrackedDevicePose.mDeviceToAbsoluteTracking
							    .m[row][col] = matrix.v[col * 4 + row];
					auto required = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
					                XRT_SPACE_RELATION_POSITION_VALID_BIT;
					b.frame.m_RawTrackedDevicePose.bPoseIsValid =
					    (rel.relation_flags & required) == required;
					b.frame.m_RawTrackedDevicePose.bDeviceIsConnected = true;
					b.frame.m_RawTrackedDevicePose.eTrackingResult =
					    b.frame.m_RawTrackedDevicePose.bPoseIsValid
					        ? vr::TrackingResult_Running_OK
					        : vr::TrackingResult_Running_OutOfRange;
					b.frame.m_pImageData = reinterpret_cast<uintptr_t>(b.data);
					b.ready = true;
					delivered = true;
					break;
				}
			}
			if (delivered && callback)
				callback->OnCameraVideoSinkCallback();
		}

		last_timestamp = frame->timestamp;
		xrt_frame_reference(&frame, nullptr);
	}

	if (camera)
		camera->set_active(camera, false);
}

bool
MonadoCamera::GetCameraDistortion(uint32_t eye, float u, float v, float *out_u, float *out_v)
{
	if (eye >= 2 || !out_u || !out_v)
		return false;
	*out_u = u;
	*out_v = v;
	return true;
}

bool
MonadoCamera::GetCameraProjection(
    uint32_t eye, vr::EVRTrackedCameraFrameType type, float near, float far, vr::HmdMatrix44_t *out)
{
	if (eye >= 2 || type < 0 || type >= vr::MAX_CAMERA_FRAME_TYPES || !std::isfinite(near) || !std::isfinite(far) ||
	    near <= 0 || far <= near || !out)
		return false;
	*out = {};
	// SteamVR 2D selects the raw stereo half before projection. Each eye
	// then uses the same camera-local region of a packed-sized texture.
	float width = camera->view_width * 2.0f;
	float center_x = camera->center.x;
	out->m[0][0] = 2 * camera->focal_length.x / width;
	out->m[1][1] = 2 * camera->focal_length.y / camera->view_height;
	out->m[0][2] = 1 - 2 * center_x / width;
	out->m[1][2] = 2 * camera->center.y / camera->view_height - 1;
	// OpenVR camera projections use a zero-to-one depth range.
	out->m[2][2] = -double(far) / (double(far) - near);
	out->m[2][3] = -double(far) * near / (double(far) - near);
	if (!std::isfinite(out->m[2][2]) || !std::isfinite(out->m[2][3])) {
		*out = {};
		return false;
	}
	out->m[3][2] = -1;
	return true;
}

bool
MonadoCamera::GetCameraCompatibilityMode(vr::ECameraCompatibilityMode *out)
{
	if (!out)
		return false;
	*out = vr::CAMERA_COMPAT_MODE_BULK_DEFAULT;
	return true;
}

bool
MonadoCamera::GetCameraFrameBounds(
    vr::EVRTrackedCameraFrameType type, uint32_t *left, uint32_t *top, uint32_t *width, uint32_t *height)
{
	if (type < 0 || type >= vr::MAX_CAMERA_FRAME_TYPES || !left || !top || !width || !height)
		return false;
	*left = *top = 0;
	*width = camera->view_width * 2;
	*height = camera->view_height;
	return true;
}

bool
MonadoCamera::GetCameraIntrinsics(uint32_t eye,
                                  vr::EVRTrackedCameraFrameType type,
                                  vr::HmdVector2_t *focal,
                                  vr::HmdVector2_t *center,
                                  vr::EVRDistortionFunctionType *distortion,
                                  double *coefficients)
{
	if (eye >= 2 || type < 0 || type >= vr::MAX_CAMERA_FRAME_TYPES || !focal || !center || !distortion ||
	    !coefficients)
		return false;
	*focal = {{camera->focal_length.x, camera->focal_length.y}};
	*center = {{camera->center.x, camera->center.y}};
	*distortion = vr::VRDistortionFunctionType_None;
	std::fill_n(coefficients, vr::k_unMaxDistortionFunctionParameters, 0);
	return true;
}

// Queue storage and synchronization belong to SteamVR. Metadata is typed path
// data in its 512-byte header, not a CameraVideoStreamFrame_t memory dump.
bool
MonadoCamera::initialize_queue()
{
	vr::PropertyContainerHandle_t created = 0;
	if (queue->Create(&created, "/lighthouse/camera/raw_frames", bytes(), 512, 4, 0) != 0)
		return false;
	queue_handle = created;
	const char *names[] = {"/format", "/width", "/height"};
	int32_t values[] = {vr::CVS_FORMAT_RGBX32, static_cast<int32_t>(camera->view_width * 2),
	                    static_cast<int32_t>(camera->view_height)};
	for (int i = 0; i < 3; ++i) {
		vr::PathWrite_t write{};
		if (paths->StringToHandle(&write.ulPath, names[i]) != vr::TrackedProp_Success)
			return false;
		write.writeType = vr::PropertyWrite_Set;
		write.pvBuffer = &values[i];
		write.unBufferSize = sizeof(values[i]);
		write.unTag = vr::k_unInt32PropertyTag;
		if (paths->WritePathBatch(queue_handle, &write, 1) != vr::TrackedProp_Success ||
		    write.eError != vr::TrackedProp_Success)
			return false;
	}
	const char *metadata[] = {"/frame_size",        "/frame_sequence", "/frame_time_monotonic",
	                          "/server_time_ticks", "/delivery_rate",  "/elapsed_time"};
	for (int i = 0; i < 6; ++i)
		if (paths->StringToHandle(&metadata_paths[i], metadata[i]) != vr::TrackedProp_Success)
			return false;
	return true;
}

void
MonadoCamera::publish_queue(const uint8_t *pixels, int64_t timestamp, uint64_t sequence, double interval)
{
	vr::PropertyContainerHandle_t block = 0;
	void *data = nullptr;
	if (queue->AcquireWriteOnlyBlock(queue_handle, &block, &data) != 0)
		return;
	if (!data) {
		queue->ReleaseWriteOnlyBlock(queue_handle, block);
		return;
	}
	memcpy(data, pixels, bytes());
	int32_t size = bytes();
	double capture = timestamp / 1e9;
	uint64_t ticks = timestamp;
	double elapsed = (timestamp - start_ns) / 1e9;
	void *values[] = {&size, &sequence, &capture, &ticks, &interval, &elapsed};
	uint32_t sizes[] = {sizeof(size),  sizeof(sequence), sizeof(capture),
	                    sizeof(ticks), sizeof(interval), sizeof(elapsed)};
	vr::PropertyTypeTag_t tags[] = {vr::k_unInt32PropertyTag,  vr::k_unUint64PropertyTag,
	                                vr::k_unDoublePropertyTag, vr::k_unUint64PropertyTag,
	                                vr::k_unDoublePropertyTag, vr::k_unDoublePropertyTag};
	vr::PathWrite_t writes[6]{};
	for (int i = 0; i < 6; ++i) {
		writes[i].ulPath = metadata_paths[i];
		writes[i].writeType = vr::PropertyWrite_Set;
		writes[i].pvBuffer = values[i];
		writes[i].unBufferSize = sizes[i];
		writes[i].unTag = tags[i];
	}
	paths->WritePathBatch(block, writes, 6);
	queue->ReleaseWriteOnlyBlock(queue_handle, block);
}
