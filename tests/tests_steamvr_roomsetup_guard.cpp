// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "catch_amalgamated.hpp"
#define main roomsetup_guard_entry
#include "../src/xrt/state_trackers/steamvr_drv/ovrd_roomsetup_guard.cpp"
#undef main
#include "../src/xrt/state_trackers/steamvr_drv/ovrd_chaperone_origin.hpp"

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
	// An unlocalized room must not inject either the old plane or a manual-floor reset.
	CHECK_FALSE(floor.publish(true, false));
	CHECK_FALSE(floor.publish(false, false));
	CHECK(messages.sent.empty());
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
	CHECK_FALSE(floor.publish(true, false));
	CHECK(messages.sent.size() == 3);
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

TEST_CASE("Native session origin has no geometry and preserves the saved room")
{
    LogFixture fixture;
    const std::string saved = R"({"universeID":"2","standing":{"translation":[1,2,3],"yaw":0.7},"play_area":[2,3],"collision_bounds":[[1,2,3]]})";
    fixture.write(R"({"jsonid":"chaperone_info","version":5,"universes":[)" + saved + "]}");
    REQUIRE(rift_s_prepare_session_origin(fixture.path.string()));
    auto bytes = read_file(fixture.path);
    REQUIRE(bytes.find(saved) != std::string::npos);
    std::unique_ptr<cJSON, decltype(&cJSON_Delete)> document(cJSON_Parse(bytes.c_str()), cJSON_Delete);
    auto universes = cJSON_GetObjectItemCaseSensitive(document.get(), "universes");
    REQUIRE(cJSON_GetArraySize(universes) == 2);
    auto origin = cJSON_GetArrayItem(universes, 1);
    CHECK(cJSON_IsObject(cJSON_GetObjectItemCaseSensitive(origin, "standing")));
    CHECK(cJSON_IsObject(cJSON_GetObjectItemCaseSensitive(origin, "seated")));
    CHECK(cJSON_GetObjectItemCaseSensitive(origin, "play_area") == nullptr);
    CHECK(cJSON_GetObjectItemCaseSensitive(origin, "collision_bounds") == nullptr);
    REQUIRE(rift_s_prepare_session_origin(fixture.path.string()));
    CHECK(read_file(fixture.path) == bytes);
    fixture.write("broken json");
    CHECK_FALSE(rift_s_prepare_session_origin(fixture.path.string()));
    CHECK(read_file(fixture.path) == "broken json");
}

TEST_CASE("Display recovery requires one exact runtime process and the expected server PID")
{
	auto executable = fs::read_symlink("/proc/self/exe");
	int process = unique_runtime_process(executable, getpid());
	REQUIRE(process >= 0);
	CHECK_FALSE(exited(process));
	close(process);
	CHECK(unique_runtime_process(executable, getpid() + 1) == -1);
	CHECK(unique_runtime_process(executable.parent_path() / "missing-vr-runtime") == -1);

	// A second session with the same executable must make recovery refuse it.
	int hold[2];
	REQUIRE(pipe2(hold, O_CLOEXEC) == 0);
	pid_t child = fork();
	REQUIRE(child >= 0);
	if (child == 0) {
		close(hold[1]);
		char value;
		while (read(hold[0], &value, 1) < 0 && errno == EINTR) {}
		_exit(0);
	}
	close(hold[0]);
	CHECK(unique_runtime_process(executable) == -1);
	CHECK(unique_runtime_process(executable, getpid()) == -1);
	close(hold[1]);
	while (waitpid(child, nullptr, 0) < 0 && errno == EINTR) {}
	process = unique_runtime_process(executable, getpid());
	REQUIRE(process >= 0);
	close(process);
}

TEST_CASE("Late display watchdog remains armed until native vsync really resumes")
{
	using namespace std::chrono;
	DisplayHealth health;
	auto start = steady_clock::time_point{seconds(100)};
	CHECK_FALSE(health.update(false, .01f, 1000, start));
	// USB/poses and the process can return while native presentation is frozen.
	for (int second = 1; second <= 30; ++second)
		CHECK_FALSE(health.update(true, float(second), 1000, start + seconds(second)));
	CHECK_FALSE(health.update(true, .01f, 1001, start + seconds(31)));
	CHECK_FALSE(health.update(true, .01f, 1002, start + seconds(32)));
	CHECK(health.update(true, .01f, 1003, start + seconds(33)));
	// A fresh freeze/disconnection breaks that run rather than retaining health.
	CHECK_FALSE(health.update(true, .01f, 1003, start + seconds(34)));
	CHECK_FALSE(health.update(true, .01f, 1004, start + seconds(35)));
	CHECK_FALSE(health.update(false, .01f, 1005, start + seconds(36)));
	CHECK_FALSE(health.update(true, -1, 1006, start + seconds(37)));
	CHECK_FALSE(health.update(true, .01f, 0, start + seconds(38)));
}

namespace {
// A private runtime whose daemons are uniquely identifiable by executable path,
// so DisplayRecovery can pin and kill them without touching a real SteamVR session.
struct FakeRuntime
{
	fs::path dir;
	fs::path library;
	pid_t compositor = -1, server = -1, monitor = -1;
	int server_fd = -1;

	static fs::path copy_sleeper(const fs::path &dir, const char *name)
	{
		const char *candidates[] = {"/bin/sleep", "/usr/bin/sleep"};
		std::string bytes;
		for (auto candidate : candidates) {
			bytes = read_file(candidate);
			if (!bytes.empty()) break;
		}
		REQUIRE_FALSE(bytes.empty());
		auto dest = dir / name;
		{
			std::ofstream out(dest, std::ios::binary | std::ios::trunc);
			REQUIRE(out.good());
			out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
			REQUIRE(out.good());
		}
		REQUIRE(chmod(dest.c_str(), 0700) == 0);
		return dest;
	}
	static pid_t spawn(const fs::path &binary)
	{
		pid_t pid = fork();
		REQUIRE(pid >= 0);
		if (pid == 0) {
			execl(binary.c_str(), binary.filename().c_str(), "60", nullptr);
			_exit(127);
		}
		return pid;
	}
	static void stop(pid_t pid)
	{
		if (pid <= 0) return;
		kill(pid, SIGKILL);
		while (waitpid(pid, nullptr, 0) < 0 && errno == EINTR) {}
	}
	explicit FakeRuntime(const char *tag)
	{
		dir = fs::temp_directory_path() / (std::string("monado-display-recovery-") + tag +
		                                   "-" + std::to_string(getpid()));
		std::error_code error;
		fs::create_directories(dir, error);
		REQUIRE_FALSE(error);
		library = (dir / "libopenvr_api.so").string();
		compositor = spawn(copy_sleeper(dir, "vrcompositor"));
		server = spawn(copy_sleeper(dir, "vrserver"));
		monitor = spawn(copy_sleeper(dir, "vrmonitor"));
		server_fd = static_cast<int>(syscall(SYS_pidfd_open, server, 0));
		REQUIRE(server_fd >= 0);
	}
	void stop_compositor()
	{
		stop(compositor);
		compositor = -1;
	}
	void restart_compositor()
	{
		compositor = spawn(dir / "vrcompositor");
	}
	// The failed spawn below execs this test binary with an unknown flag, so
	// the helper child exits without signalling anything. Only the parent's
	// "automatically restarting" line marks a fired recovery.
	template <typename Poll> static std::string fired(Poll poll)
	{
		int saved = dup(STDERR_FILENO);
		REQUIRE(saved >= 0);
		int pipefd[2];
		REQUIRE(pipe2(pipefd, O_CLOEXEC) == 0);
		REQUIRE(dup2(pipefd[1], STDERR_FILENO) >= 0);
		poll();
		fflush(stderr);
		REQUIRE(dup2(saved, STDERR_FILENO) >= 0);
		close(saved);
		close(pipefd[1]);
		std::string out;
		char buffer[4096];
		ssize_t count = 0;
		while ((count = read(pipefd[0], buffer, sizeof(buffer))) > 0)
			out.append(buffer, static_cast<size_t>(count));
		close(pipefd[0]);
		return out;
	}
	~FakeRuntime()
	{
		stop(compositor);
		stop(server);
		stop(monitor);
		if (server_fd >= 0) close(server_fd);
		std::error_code error;
		fs::remove_all(dir, error);
	}
};
}

TEST_CASE("Display recovery adopts a respawned compositor instead of restarting the session")
{
	FakeRuntime runtime("healed");
	DisplayRecovery recovery;
	const char *library = runtime.library.c_str();
	// Pin the first compositor generation, then arm the loss latch.
	recovery.update(true, .01f, 100, runtime.server_fd, runtime.server, library);
	recovery.update(false, .01f, 101, runtime.server_fd, runtime.server, library);
	// SteamVR respawns the compositor on its own before the next check.
	runtime.stop_compositor();
	runtime.restart_compositor();
	std::string output = FakeRuntime::fired([&] {
		recovery.update(true, .01f, 101, runtime.server_fd, runtime.server, library);
	});
	CHECK(output.find("automatically restarting") == std::string::npos);
	// The rig above can fire: with no replacement left, the dead handle triggers.
	runtime.stop_compositor();
	output = FakeRuntime::fired([&] {
		recovery.update(true, .01f, 101, runtime.server_fd, runtime.server, library);
	});
	CHECK(output.find("automatically restarting") != std::string::npos);
	// Let the helper child exit so the destructor does not leave a zombie.
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
	recovery.update(false, -1, 101, runtime.server_fd, runtime.server, library);
}
