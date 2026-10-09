// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#define main roomsetup_guard_entry
#include "../src/xrt/state_trackers/steamvr_drv/ovrd_roomsetup_guard.cpp"
#undef main

namespace {
struct LogFixture
{
	fs::path path = fs::temp_directory_path() / ("monado-roomsetup-guard-test-" + std::to_string(getpid()));
	OverflowLog log{path};
	void write(const std::string &text, bool append = false)
	{
		std::ofstream stream(path, append ? std::ios::app : std::ios::trunc);
		stream << text;
		REQUIRE(stream.good());
	}
	~LogFixture() { std::error_code error; fs::remove(path, error); }
};
}

TEST_CASE("Native setup guard ignores old failures and recognizes a new split assertion")
{
	LogFixture f;
	f.write("Vulkan transfer ring buffer overflow; transfer size 99400!\n");
	f.log.begin();
	CHECK_FALSE(f.log.overflowed());
	f.write("ordinary texture upload\nVulkan transfer ring ", true);
	CHECK_FALSE(f.log.overflowed());
	f.write("buffer overflow; transfer size 144760!\n", true);
	CHECK(f.log.overflowed());
	CHECK_FALSE(f.log.overflowed());
	f.log.begin();
	CHECK_FALSE(f.log.overflowed());
}

TEST_CASE("Native setup guard handles truncated logs and bounds each read")
{
	LogFixture f;
	f.write(std::string(70000, 'x'));
	f.log.begin();
	f.write("ordinary restart\n");
	CHECK_FALSE(f.log.overflowed());
	f.write(std::string(70000, 'x') + "Vulkan transfer ring buffer overflow\n", true);
	CHECK_FALSE(f.log.overflowed());
	CHECK(f.log.overflowed());
}

TEST_CASE("Saved floor converts from raw into setup without HMD height")
{
	vr::HmdMatrix34_t standing{{{0, 0, -1, .8f}, {0, 1, 0, -1.8f}, {1, 0, 0, .4f}}};
	vr::HmdMatrix34_t setup{{{1, 0, 0, -.2f}, {0, 1, 0, -.7f}, {0, 0, 1, .3f}}};
	REQUIRE(roomsetup::saved_floor(standing, setup));
	CHECK(*roomsetup::saved_floor(standing, setup) == Catch::Approx(1.1f));
	standing.m[1][3] += 1.6f;
	setup.m[1][3] += 1.6f;
	CHECK(*roomsetup::saved_floor(standing, setup) == Catch::Approx(1.1f));
}

TEST_CASE("Setup RGB projection keeps eye rays independent of surface depth")
{
	using namespace roomsetup;
	auto projection = Matrix{1.2f, 0, .1f, 0, 0, 1.4f, -.2f, 0, 0, 0, -1.01f, -.1f, 0, 0, -1, 0};
	auto capture = identity();
	capture[3] = .2f;
	capture[7] = 1.7f;
	capture[11] = .1f;
	auto old = multiply(projection, inverse_rigid(capture));
	for (float height : {1.7f, .9f, 1.7f}) {
		for (float pitch : {0.f, -.9f}) {
			auto eye = identity();
			eye[5] = eye[10] = std::cos(pitch);
			eye[6] = -std::sin(pitch);
			eye[9] = std::sin(pitch);
			eye[3] = -.03f;
			eye[7] = height;
			auto view = inverse_rigid(eye);
			std::array<float, 100> uniform{};
			auto model = identity();
			memcpy(uniform.data(), model.data(), 64);
			memcpy(uniform.data() + 16, view.data(), 64);
			memcpy(uniform.data() + 32, projection.data(), 64);
			memcpy(uniform.data() + 48, old.data(), 64);
			REQUIRE(correct_camera(uniform.data(), sizeof(uniform)));
			// An eye-space ray (.15, -.1, -1) rotates into capture space.
			float direction[] = {.15f, -.1f * std::cos(pitch) + std::sin(pitch),
			                     -.1f * std::sin(pitch) - std::cos(pitch)};
			float expected_w = -direction[2];
			float expected_u = (1.2f * direction[0] + .1f * direction[2]) / expected_w;
			float expected_v = (1.4f * direction[1] - .2f * direction[2]) / expected_w;
			for (float depth : {.25f, 1.f, 15.f}) {
				float point[4] = {eye[3] + depth * direction[0], eye[7] + depth * direction[1],
				                  depth * direction[2], 1};
				float clip[4]{};
				for (int i = 0; i < 4; ++i)
					for (int j = 0; j < 4; ++j)
						clip[i] += uniform[48 + 4 * i + j] * point[j];
				CHECK(clip[0] / clip[3] == Catch::Approx(expected_u).margin(1e-5));
				CHECK(clip[1] / clip[3] == Catch::Approx(expected_v).margin(1e-5));
				CHECK(clip[2] == Catch::Approx(-1.01f * depth * direction[2] - .1f).margin(1e-5));
			}
			CHECK_FALSE(correct_camera(uniform.data(), sizeof(uniform) - 4));
			uniform[0] = 2;
			CHECK_FALSE(correct_camera(uniform.data(), sizeof(uniform)));
		}
	}
}

TEST_CASE("Saved floor sends a numeric detection and only seeds manual entry once")
{
	struct Messages : roomsetup::Mailbox
	{
		std::vector<std::string> sent;
		int
		Register(const char *, uint64_t *handle) override
		{
			*handle = 1;
			return 0;
		}
		int
		Unregister(uint64_t) override
		{
			return 0;
		}
		int
		Send(uint64_t, const char *destination, const char *json) override
		{
			CHECK(std::string(destination) == "vrcompositor_systemlayer");
			sent.emplace_back(json);
			return 0;
		}
		int
		Read(uint64_t, char *, uint32_t, uint32_t *) override
		{
			return 0;
		}
	} messages;
	struct Properties : roomsetup::Paths
	{
		int
		Read(uint64_t, roomsetup::PathRead *batch, uint32_t count) override
		{
			REQUIRE(count == 2);
			vr::HmdMatrix34_t standing{{{1, 0, 0, 0}, {0, 1, 0, -.2f}, {0, 0, 1, 0}}};
			vr::HmdMatrix34_t setup{{{1, 0, 0, 0}, {0, 1, 0, .7f}, {0, 0, 1, 0}}};
			memcpy(batch[0].buffer, &standing, sizeof(standing));
			memcpy(batch[1].buffer, &setup, sizeof(setup));
			for (uint32_t i = 0; i < count; ++i)
				batch[i].tag = 20;
			return 0;
		}
	} paths;
	roomsetup::SavedFloor floor{&messages, &paths};
	REQUIRE(floor.publish());
	REQUIRE(messages.sent.size() == 1);
	auto *result = cJSON_Parse(messages.sent[0].c_str());
	REQUIRE(result);
	CHECK(cJSON_IsNumber(cJSON_GetObjectItem(result, "pos")));
	CHECK(cJSON_GetNumberValue(cJSON_GetObjectItem(result, "pos")) == Catch::Approx(.9));
	CHECK(cJSON_GetObjectItem(result, "position") == nullptr);
	cJSON_Delete(result);
	REQUIRE(floor.publish(true));
	REQUIRE(messages.sent.size() == 3);
	auto *reset = cJSON_Parse(messages.sent[2].c_str());
	REQUIRE(reset);
	CHECK(cJSON_GetNumberValue(cJSON_GetObjectItem(reset, "value_source")) == 2);
	cJSON_Delete(reset);
}

TEST_CASE("Boundary warning retries failed delivery and removal")
{
	struct Notifications : vr::IVRNotifications {
		unsigned creates = 0, removes = 0;
		vr::EVRNotificationError CreateNotification(vr::VROverlayHandle_t, uint64_t,
		    vr::EVRNotificationType, const char *, vr::EVRNotificationStyle,
		    const vr::NotificationBitmap_t *, vr::VRNotificationId *id) override {
			if (++creates == 1) return vr::VRNotificationError_InvalidOverlayHandle;
			*id = 7;
			return vr::VRNotificationError_OK;
		}
		vr::EVRNotificationError RemoveNotification(vr::VRNotificationId) override {
			return ++removes == 1 ? vr::VRNotificationError_InvalidOverlayHandle : vr::VRNotificationError_OK;
		}
	} api;
	std::string delivered, warning = "Boundary not confirmed, look around.";
	vr::VRNotificationId id = 0;
	CHECK_FALSE(update_world_notification(api, 1, warning, delivered, id));
	CHECK(delivered.empty());
	REQUIRE(update_world_notification(api, 1, warning, delivered, id));
	CHECK(delivered == warning);
	CHECK(id == 7);
	REQUIRE(update_world_notification(api, 1, warning, delivered, id));
	CHECK(api.creates == 2);
	CHECK_FALSE(update_world_notification(api, 1, "", delivered, id));
	CHECK(delivered == warning);
	CHECK(id == 7);
	REQUIRE(update_world_notification(api, 1, "", delivered, id));
	CHECK(delivered.empty());
	CHECK(id == 0);
}
