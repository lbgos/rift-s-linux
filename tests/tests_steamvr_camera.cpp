// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#include "ovrd_camera.hpp"
#include "ovrd_roomsetup_frame.hpp"
#include "os/os_time.h"
#include "util/u_frame.h"
#include <cstring>
#include <condition_variable>
#include <limits>

struct CameraFixture : xrt_camera
{
	xrt_frame *frame = nullptr;
	std::mutex lock;
	CameraFixture() : xrt_camera{}
	{
		view_width = view_height = 4;
		focal_length = {2, 2};
		center = {1.5f, 1.5f};
		set_active = [](xrt_camera *, bool) {};
		get_frame = [](xrt_camera *base, xrt_frame **out) {
			auto &c = *static_cast<CameraFixture *>(base);
			std::lock_guard guard(c.lock);
			if (!c.frame)
				return false;
			xrt_frame_reference(out, c.frame);
			return true;
		};
	}
	~CameraFixture()
	{
		xrt_frame_reference(&frame, nullptr);
	}
	void
	publish(uint8_t value)
	{
		std::lock_guard guard(lock);
		xrt_frame *next = nullptr;
		u_frame_create_one_off(XRT_FORMAT_R8G8B8, 8, 4, &next);
		memset(next->data, value, next->size);
		next->timestamp = os_monotonic_get_ns();
		xrt_frame_reference(&frame, next);
		xrt_frame_reference(&next, nullptr);
	}
};

static const vr::CameraVideoStreamFrame_t *
wait_frame(MonadoCamera &camera)
{
	for (int i = 0; i < 100; ++i) {
		if (auto *f = camera.GetVideoStreamFrame())
			return f;
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}
	return nullptr;
}

TEST_CASE("SteamVR camera buffers stay immutable until release and reject stale restart frames")
{
	CameraFixture provider;
	xrt_tracking_origin origin{};
	origin.initial_offset = XRT_POSE_IDENTITY;
	xrt_device device{};
	device.camera = &provider;
	device.tracking_origin = &origin;
	device.get_tracked_pose = [](xrt_device *, xrt_input_name, int64_t, xrt_space_relation *out) {
		*out = XRT_SPACE_RELATION_ZERO;
		out->pose = XRT_POSE_IDENTITY;
		out->pose.position = {1, 2, 3};
		out->relation_flags = XRT_SPACE_RELATION_BITMASK_ALL;
		return XRT_SUCCESS;
	};
	MonadoCamera camera(&device);
	CHECK_FALSE(camera.SetCameraFrameBuffering(2, nullptr, 96));
	CHECK_FALSE(camera.SetCameraVideoStreamFormat(vr::CVS_FORMAT_NV12));
	REQUIRE(camera.StartVideoStream());
	provider.publish(70);
	auto *held = wait_frame(camera);
	REQUIRE(held);
	CHECK(held->m_nFrameSequence == 0);
	CHECK(held->m_nWidth == 8);
	CHECK(held->m_nImageDataSize == 128);
	CHECK(held->m_RawTrackedDevicePose.bPoseIsValid);
	CHECK(held->m_RawTrackedDevicePose.mDeviceToAbsoluteTracking.m[1][3] == 2);
	auto *data = reinterpret_cast<const uint8_t *>(held->m_pImageData);
	CHECK(data[0] == 70);
	provider.publish(180);
	auto *newer = wait_frame(camera);
	REQUIRE(newer);
	CHECK(newer != held);
	CHECK(data[0] == 70);
	CHECK(reinterpret_cast<const uint8_t *>(newer->m_pImageData)[0] == 180);
	camera.ReleaseVideoStreamFrame(newer);
	camera.StopVideoStream();
	CHECK(data[0] == 70);
	CHECK_FALSE(camera.StartVideoStream());
	camera.ReleaseVideoStreamFrame(held);
	REQUIRE(camera.StartVideoStream());
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	CHECK(camera.GetVideoStreamFrame() == nullptr);
	provider.publish(210);
	auto *restarted = wait_frame(camera);
	REQUIRE(restarted);
	CHECK(restarted->m_nFrameSequence == 0);
	camera.ReleaseVideoStreamFrame(restarted);
	camera.StopVideoStream();
}

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

TEST_CASE("MonadoCamera exports rectified frames to shared memory and tracks controller buttons")
{
	CameraFixture provider;
	xrt_tracking_origin origin{};
	origin.initial_offset = XRT_POSE_IDENTITY;
	xrt_device device{};
	device.camera = &provider;
	device.tracking_origin = &origin;
	device.get_tracked_pose = [](xrt_device *, xrt_input_name, int64_t, xrt_space_relation *out) {
		*out = XRT_SPACE_RELATION_ZERO;
		out->pose = XRT_POSE_IDENTITY;
		out->relation_flags = XRT_SPACE_RELATION_BITMASK_ALL;
		return XRT_SUCCESS;
	};

	MonadoCamera camera(&device);

	int fd = open(RIFTS_PASSTHROUGH_SHM_PATH, O_RDWR);
	REQUIRE(fd >= 0);
	auto *shm = static_cast<struct rifts_passthrough_shm *>(
	    mmap(nullptr, sizeof(struct rifts_passthrough_shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
	REQUIRE(shm != MAP_FAILED);

	CHECK(shm->magic == RIFTS_PASSTHROUGH_MAGIC);
	CHECK(shm->version == RIFTS_PASSTHROUGH_VERSION);
	CHECK(shm->width == 8);
	CHECK(shm->height == 4);

	uint64_t seq0 = shm->sequence;
	provider.publish(100);
	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	CHECK(shm->sequence == seq0);

	shm->client_active = os_monotonic_get_ns() / U_TIME_1MS_IN_NS;
	provider.publish(123);
	for (int i = 0; i < 50 && (__atomic_load_n(&shm->sequence, __ATOMIC_ACQUIRE) <= seq0 ||
	                           (__atomic_load_n(&shm->sequence, __ATOMIC_ACQUIRE) & 1));
	     ++i)
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
	CHECK(shm->sequence > seq0);
	CHECK(shm->pixels[0] == 123);
	CHECK(shm->pixels[1] == 123);
	CHECK(shm->pixels[2] == 123);
	CHECK(shm->pixels[3] == 255);

	uint32_t count0 = shm->toggle_count;
	MonadoCamera::NotifyButton(XRT_INPUT_TOUCH_SYSTEM_CLICK, true);
	CHECK(shm->toggle_count == count0 + 1);
	MonadoCamera::NotifyButton(XRT_INPUT_TOUCH_SYSTEM_CLICK, true);
	CHECK(shm->toggle_count == count0 + 1);
	MonadoCamera::NotifyButton(XRT_INPUT_TOUCH_SYSTEM_CLICK, false);

	MonadoCamera::NotifyButton(XRT_INPUT_TOUCH_B_CLICK, true);
	CHECK(shm->toggle_count == count0 + 1);
	MonadoCamera::NotifyButton(XRT_INPUT_TOUCH_Y_CLICK, true);
	CHECK(shm->toggle_count == count0 + 1);
	MonadoCamera::NotifyButton(XRT_INPUT_TOUCH_MENU_CLICK, true);
	CHECK(shm->toggle_count == count0 + 1);
	MonadoCamera::NotifyButton(XRT_INPUT_TOUCH_MENU_CLICK, false);
	MonadoCamera::NotifyButton(XRT_INPUT_TOUCH_MENU_CLICK, true);
	CHECK(shm->toggle_count == count0 + 2);

	shm->client_active = 0;
	munmap(shm, sizeof(struct rifts_passthrough_shm));
	close(fd);
}

#include <map>
#include <string>

struct NativeQueueFixture : vr::IVRBlockQueue, vr::IVRPaths
{
	std::vector<uint8_t> pixels;
	std::map<std::string, vr::PathHandle_t> names;
	std::map<vr::PathHandle_t, std::vector<uint8_t>> values;
	std::mutex mutex;
	std::atomic<int> released{0};
	std::atomic<bool> busy{false};
	bool destroyed = false;
	uint32_t header_size = 0, block_count = 0;
	std::string queue_path;
	vr::EBlockQueueError
	Create(uint64_t *out, const char *path, uint32_t size, uint32_t header, uint32_t count, uint32_t) override
	{
		queue_path = path;
		header_size = header;
		block_count = count;
		pixels.resize(size);
		*out = 100;
		return vr::EBlockQueueError_BlockQueueError_None;
	}
	vr::EBlockQueueError
	Connect(uint64_t *, const char *) override
	{
		return vr::EBlockQueueError_BlockQueueError_InvalidParam;
	}
	vr::EBlockQueueError
	Destroy(uint64_t) override
	{
		destroyed = true;
		return vr::EBlockQueueError_BlockQueueError_None;
	}
	vr::EBlockQueueError
	AcquireWriteOnlyBlock(uint64_t, uint64_t *out, void **data) override
	{
		if (busy)
			return vr::EBlockQueueError_BlockQueueError_BlockNotAvailable;
		mutex.lock();
		*out = 101;
		*data = pixels.data();
		return vr::EBlockQueueError_BlockQueueError_None;
	}
	vr::EBlockQueueError
	ReleaseWriteOnlyBlock(uint64_t, uint64_t) override
	{
		mutex.unlock();
		++released;
		return vr::EBlockQueueError_BlockQueueError_None;
	}
	vr::EBlockQueueError
	WaitAndAcquireReadOnlyBlock(uint64_t, uint64_t *, void **, vr::EBlockQueueReadType, uint32_t) override
	{
		return vr::EBlockQueueError_BlockQueueError_InvalidParam;
	}
	vr::EBlockQueueError
	AcquireReadOnlyBlock(uint64_t, uint64_t *, void **, vr::EBlockQueueReadType) override
	{
		return vr::EBlockQueueError_BlockQueueError_InvalidParam;
	}
	vr::EBlockQueueError
	ReleaseReadOnlyBlock(uint64_t, uint64_t) override
	{
		return vr::EBlockQueueError_BlockQueueError_InvalidParam;
	}
	vr::EBlockQueueError
	QueueHasReader(uint64_t, bool *out) override
	{
		*out = true;
		return vr::EBlockQueueError_BlockQueueError_None;
	}
	vr::ETrackedPropertyError
	ReadPathBatch(uint64_t, vr::PathRead_t *, uint32_t) override
	{
		return vr::TrackedProp_InvalidOperation;
	}
	vr::ETrackedPropertyError
	WritePathBatch(uint64_t, vr::PathWrite_t *writes, uint32_t count) override
	{
		for (uint32_t i = 0; i < count; ++i) {
			auto *first = static_cast<uint8_t *>(writes[i].pvBuffer);
			values[writes[i].ulPath] = {first, first + writes[i].unBufferSize};
			writes[i].eError = vr::TrackedProp_Success;
		}
		return vr::TrackedProp_Success;
	}
	vr::ETrackedPropertyError
	StringToHandle(uint64_t *out, const char *path) override
	{
		*out = names.emplace(path, names.size() + 1).first->second;
		return vr::TrackedProp_Success;
	}
	vr::ETrackedPropertyError
	HandleToString(uint64_t, const char *, uint32_t, uint32_t *) override
	{
		return vr::TrackedProp_InvalidOperation;
	}
	template <typename T>
	T
	value(const char *path)
	{
		T out{};
		auto &data = values.at(names.at(path));
		REQUIRE(data.size() == sizeof(T));
		memcpy(&out, data.data(), sizeof(out));
		return out;
	}
};

TEST_CASE("Native camera queue publishes stable frames and recovers from a busy compositor")
{
	CameraFixture provider;
	xrt_device device{};
	xrt_tracking_origin origin{};
	origin.initial_offset = XRT_POSE_IDENTITY;
	device.camera = &provider;
	device.tracking_origin = &origin;
	device.get_tracked_pose = [](xrt_device *, xrt_input_name, int64_t, xrt_space_relation *out) {
		*out = XRT_SPACE_RELATION_ZERO;
		out->pose = XRT_POSE_IDENTITY;
		return XRT_SUCCESS;
	};
	NativeQueueFixture queue;
	{
		MonadoCamera camera(&device, &queue, &queue);
		REQUIRE(camera.native_ready());
		CHECK(queue.queue_path == "/lighthouse/camera/raw_frames");
		CHECK(queue.header_size == 512);
		CHECK(queue.block_count == 4);
		CHECK(queue.value<int32_t>("/format") == vr::CVS_FORMAT_RGBX32);
		CHECK(queue.value<int32_t>("/width") == 8);
		CHECK(queue.value<int32_t>("/height") == 4);
		queue.busy = true;
		REQUIRE(camera.StartVideoStream());
		provider.publish(90);
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
		CHECK(queue.released == 0);
		queue.busy = false;
		provider.publish(145);
		for (int i = 0; i < 100 && queue.released == 0; ++i)
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
		REQUIRE(queue.released == 1);
		{
			std::lock_guard lock(queue.mutex);
			CHECK(queue.pixels[0] == 145);
			CHECK(queue.pixels[3] == 255);
			CHECK(queue.value<int32_t>("/frame_size") == 128);
			CHECK(queue.value<uint64_t>("/server_time_ticks") > 0);
			CHECK(queue.value<double>("/frame_time_monotonic") > 0);
			CHECK(queue.value<double>("/delivery_rate") > 0);
		}
		camera.StopVideoStream();
	}
	CHECK(queue.destroyed);
}

TEST_CASE("2D camera projection rays use the same local region for both eye textures")
{
	CameraFixture provider;
	xrt_device device{};
	device.camera = &provider;
	MonadoCamera camera(&device);
	for (uint32_t eye = 0; eye < 2; ++eye) {
		for (int type = 0; type < vr::MAX_CAMERA_FRAME_TYPES; ++type) {
			vr::HmdMatrix44_t projection{};
			REQUIRE(camera.GetCameraProjection(eye, static_cast<vr::EVRTrackedCameraFrameType>(type),
			                                   0.1f, 10.0f, &projection));
			// Camera-local center and a ray one pixel to the right.
			float center = (1 - projection.m[0][2]) * 0.5f;
			CHECK(center == Catch::Approx(provider.center.x / 8));
			float next = (projection.m[0][0] / provider.focal_length.x - projection.m[0][2] + 1) * 0.5f;
			CHECK(next - center == Catch::Approx(1.0f / 8));
			float near_depth = (-0.1f * projection.m[2][2] + projection.m[2][3]) / 0.1f;
			float far_depth = (-10.0f * projection.m[2][2] + projection.m[2][3]) / 10.0f;
			CHECK(near_depth == Catch::Approx(0).margin(1e-6));
			CHECK(far_depth == Catch::Approx(1));
		}
	}
}

TEST_CASE("Room View exported stereo rays stay at the correct pixels through downward pitch and yaw")
{
	using namespace roomsetup;
	CameraFixture provider;
	provider.view_width = provider.view_height = 320;
	provider.focal_length = {112.033206f, 112.033206f};
	provider.center = {159.5f, 159.5f};
	xrt_device device{};
	device.camera = &provider;
	MonadoCamera camera(&device);
	auto rotation = [](float pitch, float yaw) {
		auto p = identity(), y = identity();
		p[5] = p[10] = std::cos(pitch); p[6] = -std::sin(pitch); p[9] = std::sin(pitch);
		y[0] = y[10] = std::cos(yaw); y[2] = std::sin(yaw); y[8] = -std::sin(yaw);
		return multiply(y, p);
	};
	auto apply = [](const Matrix &m, const std::array<float, 4> &p) {
		std::array<float, 4> out{};
		for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) out[i] += m[4*i+j] * p[j];
		return out;
	};
	float worst_pixel_error = 0;
	for (float pitch_deg : {0.f, -45.f, -90.f}) for (float yaw_deg : {-40.f, 0.f, 40.f}) {
		auto head = rotation(pitch_deg * float(M_PI) / 180, yaw_deg * float(M_PI) / 180);
		auto capture = rotation((pitch_deg + 2) * float(M_PI) / 180, (yaw_deg - 4) * float(M_PI) / 180);
		capture[3] = .2f; capture[7] = 1.7f; capture[11] = .1f;
		for (uint32_t eye_index = 0; eye_index < 2; ++eye_index) {
			vr::HmdMatrix44_t exported{};
			REQUIRE(camera.GetCameraProjection(eye_index, vr::VRTrackedCameraFrameType_Undistorted,
			                                   .05f, 15.f, &exported));
			Matrix projection{};
			memcpy(projection.data(), exported.m, sizeof(exported));
			for (float height : {.9f, 1.7f}) {
				auto eye = head; eye[3] = eye_index ? .032f : -.032f; eye[7] = height;
				auto view = inverse_rigid(eye), old = multiply(projection, inverse_rigid(capture));
				std::array<float, 100> uniform{};
				auto model = identity();
				memcpy(uniform.data(), model.data(), 64); memcpy(uniform.data()+16, view.data(), 64);
				memcpy(uniform.data()+32, projection.data(), 64); memcpy(uniform.data()+48, old.data(), 64);
				REQUIRE(correct_camera(uniform.data(), sizeof(uniform)));
				Matrix fixed{}; memcpy(fixed.data(), uniform.data()+48, 64);
				for (float x : {-.7f, 0.f, .7f}) for (float y : {-.7f, 0.f, .7f}) {
					// The depth-free reference is P * R_capture^-1 * R_eye * eye_ray.
					auto direction = apply(eye, {x, y, -1, 0});
					auto reference = apply(old, direction);
					for (float depth : {.25f, 1.f, 15.f}) {
						auto world = apply(eye, {x*depth, y*depth, -depth, 1});
						auto clip = apply(fixed, world);
						// Packed X normalization and eye-half selection together yield 320 local pixels.
						float error = std::max(std::abs(clip[0]/clip[3] - reference[0]/reference[3]) * 320,
						                       std::abs(clip[1]/clip[3] - reference[1]/reference[3]) * 160);
						worst_pixel_error = std::max(worst_pixel_error, error);
						CHECK(error < .002f);
						// Independently project the known point back into the current eye.
						auto eye_point = apply(view, world);
						CHECK(eye_point[0]/-eye_point[2] == Catch::Approx(x).margin(1e-5));
						CHECK(eye_point[1]/-eye_point[2] == Catch::Approx(y).margin(1e-5));
					}
				}
			}
		}
	}
	printf("Room View pitch/yaw/stereo worst pixel error: %.9g\n", worst_pixel_error);
}

struct BlockedCameraFixture : CameraFixture
{
	std::condition_variable cv;
	bool entered = false, release = false, consumed = false;
	BlockedCameraFixture()
	{
		get_frame = [](xrt_camera *base, xrt_frame **out) {
			auto &c = *static_cast<BlockedCameraFixture *>(base);
			std::unique_lock guard(c.lock);
			if (!c.frame)
				return false;
			if (!c.entered) {
				c.entered = true;
				c.cv.notify_all();
				c.cv.wait_for(guard, std::chrono::milliseconds(500), [&] { return c.release; });
				// A newly captured image from an old request still belongs to that generation.
				c.frame->timestamp = os_monotonic_get_ns();
			}
			xrt_frame_reference(out, c.frame);
			xrt_frame_reference(&c.frame, nullptr);
			c.consumed = true;
			c.cv.notify_all();
			return true;
		};
	}
};

TEST_CASE("SteamVR camera stop and restart discard an in-flight provider request")
{
	BlockedCameraFixture provider;
	xrt_tracking_origin origin{};
	origin.initial_offset = XRT_POSE_IDENTITY;
	xrt_device device{};
	device.camera = &provider;
	device.tracking_origin = &origin;
	device.get_tracked_pose = [](xrt_device *, xrt_input_name, int64_t, xrt_space_relation *out) {
		*out = XRT_SPACE_RELATION_ZERO;
		out->pose = XRT_POSE_IDENTITY;
		return XRT_SUCCESS;
	};
	struct Callback : vr::ICameraVideoSinkCallback
	{
		std::atomic<int> count{0};
		void
		OnCameraVideoSinkCallback() override
		{
			++count;
		}
	} callback;
	NativeQueueFixture queue;
	MonadoCamera camera(&device, &queue, &queue);
	camera.SetCameraVideoSinkCallback(&callback);
	REQUIRE(camera.StartVideoStream());
	provider.publish(80);
	{
		std::unique_lock lock(provider.lock);
		REQUIRE(provider.cv.wait_for(lock, std::chrono::milliseconds(200), [&] { return provider.entered; }));
	}
	camera.StopVideoStream();
	REQUIRE(camera.StartVideoStream());
	{
		std::unique_lock lock(provider.lock);
		provider.release = true;
		provider.cv.notify_all();
		REQUIRE(provider.cv.wait_for(lock, std::chrono::milliseconds(200), [&] { return provider.consumed; }));
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	CHECK(queue.released == 0);
	CHECK(callback.count == 0);
	CHECK(camera.GetVideoStreamFrame() == nullptr);
	provider.publish(190);
	auto *frame = wait_frame(camera);
	REQUIRE(frame);
	CHECK(frame->m_nFrameSequence == 0);
	CHECK(reinterpret_cast<const uint8_t *>(frame->m_pImageData)[0] == 190);
	camera.ReleaseVideoStreamFrame(frame);
	camera.StopVideoStream();
	CHECK(queue.released == 1);
	CHECK(callback.count == 1);
	{
		std::lock_guard lock(queue.mutex);
		CHECK(queue.value<uint64_t>("/frame_sequence") == 0);
	}
	camera.SetCameraVideoSinkCallback(nullptr);
}

TEST_CASE("SteamVR camera projection rejects nonfinite clipping planes")
{
	CameraFixture provider;
	xrt_device device{};
	device.camera = &provider;
	MonadoCamera camera(&device);
	vr::HmdMatrix44_t projection{};
	for (float invalid : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
		CHECK_FALSE(
		    camera.GetCameraProjection(0, vr::VRTrackedCameraFrameType_Undistorted, invalid, 100, &projection));
		CHECK_FALSE(
		    camera.GetCameraProjection(0, vr::VRTrackedCameraFrameType_Undistorted, .1f, invalid, &projection));
	}
	CHECK(camera.GetCameraProjection(0, vr::VRTrackedCameraFrameType_Undistorted, 1e30f, 2e30f, &projection));
	CHECK(std::isfinite(projection.m[2][3]));
}
