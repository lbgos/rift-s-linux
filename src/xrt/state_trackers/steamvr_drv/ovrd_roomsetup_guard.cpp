// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
// Keep the pre-setup calibration if SteamVR's compositor dies during native drawing.
#include "openvr.h"
#include "ovrd_roomsetup_floor.hpp"
#include "../../drivers/rift_s/rift_s_world_control.h"
#include <cjson/cJSON.h>
#include <chrono>
#include <cerrno>
#include <fcntl.h>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <filesystem>
#include <fstream>
#include <optional>
#include <memory>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/syscall.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;
extern char **environ;

static std::string read_file(const fs::path &path)
{
	std::ifstream stream(path, std::ios::binary);
	if (!stream) return {};
	stream.seekg(0, std::ios::end);
	auto size = stream.tellg();
	if (size <= 0 || size > 4 * 1024 * 1024) return {};
	std::string bytes(static_cast<size_t>(size), '\0');
	stream.seekg(0);
	stream.read(bytes.data(), bytes.size());
	return stream ? bytes : std::string{};
}

static fs::path calibration_path(fs::path &log)
{
	const char *home = getenv("HOME");
	if (!home) return {};
	const char *config = getenv("XDG_CONFIG_HOME");
	fs::path registry = (config ? fs::path(config) : fs::path(home) / ".config") / "openvr/openvrpaths.vrpath";
	auto bytes = read_file(registry);
	cJSON *json = cJSON_Parse(bytes.c_str());
	if (!json) return {};
	auto *entry = cJSON_GetArrayItem(cJSON_GetObjectItemCaseSensitive(json, "config"), 0);
	fs::path path;
	if (cJSON_IsString(entry) && entry->valuestring) path = fs::path(entry->valuestring) / "chaperone_info.vrchap";
	auto *log_entry = cJSON_GetArrayItem(cJSON_GetObjectItemCaseSensitive(json, "log"), 0);
	if (cJSON_IsString(log_entry) && log_entry->valuestring) log = fs::path(log_entry->valuestring) / "vrcompositor.txt";
	cJSON_Delete(json);
	return path;
}

// Room setup is the only action that may establish a new physical world.
static void world_command(const char *command)
{
	char name[1024];
	rift_s_world_control_path("command", name, sizeof(name));
	fs::path path(name), temp = path.string() + ".tmp";
	std::ofstream stream(temp, std::ios::trunc);
	if (!stream) return;
	auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
	    std::chrono::steady_clock::now().time_since_epoch()).count();
	stream << ns << " " << command << "\n";
	stream.close();
	if (stream) { std::error_code error; fs::rename(temp, path, error); }
}

static std::string world_status()
{
	char path[1024];
	rift_s_world_control_path("status", path, sizeof(path));
	std::ifstream stream(path);
	int64_t stamp = 0;
	std::string status;
	stream >> stamp >> status;
	auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
	    std::chrono::steady_clock::now().time_since_epoch()).count();
	if (!stream || stamp > ns || ns - stamp > 2000000000LL) return {};
	return status;
}

// Retain the delivered state only after OpenVR accepts it, so a transient failure retries.
static bool update_world_notification(vr::IVRNotifications &notifications,
                                     vr::VROverlayHandle_t overlay,
                                     const std::string &warning,
                                     std::string &delivered,
                                     vr::VRNotificationId &id)
{
	if (warning == delivered) return true;
	if (id) {
		auto error = notifications.RemoveNotification(id);
		if (error != vr::VRNotificationError_OK && error != vr::VRNotificationError_InvalidNotificationId)
			return false;
		id = 0;
		delivered.clear();
	}
	if (!warning.empty()) {
		vr::VRNotificationId created = 0;
		if (notifications.CreateNotification(overlay, 0, vr::EVRNotificationType_Persistent,
		    warning.c_str(), vr::EVRNotificationStyle_Application, nullptr, &created) !=
		    vr::VRNotificationError_OK) return false;
		id = created;
		fprintf(stderr, "monado-roomsetup-guard: Guardian notification %u: %s\n", created, warning.c_str());
	}
	delivered = warning;
	return true;
}

static int compositor_fd()
{
	std::error_code error;
	for (const auto &entry : fs::directory_iterator("/proc", error)) {
		auto name = entry.path().filename().string();
		if (name.empty() || name.find_first_not_of("0123456789") != std::string::npos) continue;
		std::ifstream comm(entry.path() / "comm");
		std::string process;
		std::getline(comm, process);
		if (process != "vrcompositor") continue;
		// Ignore another user's compositor and retain this process generation, not its reusable PID.
		struct stat state{};
		if (stat(entry.path().c_str(), &state) != 0 || state.st_uid != getuid()) continue;
		return static_cast<int>(syscall(SYS_pidfd_open, std::stoi(name), 0));
	}
	return -1;
}

static bool exited(int fd)
{
	if (fd < 0) return true;
	pollfd process{fd, POLLIN, 0};
	return poll(&process, 1, 0) > 0;
}

// SteamVR daemons detach from their parent/session. Its per-user runtime lock
// normally permits one session; require exactly one matching executable anyway.
static int unique_runtime_process(const fs::path &executable, pid_t expected = 0)
{
	int result = -1;
	unsigned matches = 0;
	std::error_code error;
	for (const auto &entry : fs::directory_iterator("/proc", error)) {
		auto name = entry.path().filename().string();
		if (name.empty() || name.find_first_not_of("0123456789") != std::string::npos) continue;
		pid_t pid = std::stoi(name);
		int candidate = static_cast<int>(syscall(SYS_pidfd_open, pid, 0));
		if (candidate < 0) continue;
		struct stat state{};
		auto path = fs::read_symlink(entry.path() / "exe", error);
		if (!error && path == executable && stat(entry.path().c_str(), &state) == 0 &&
		    state.st_uid == getuid() && !exited(candidate)) {
			++matches;
			if (result < 0 && (!expected || pid == expected)) { result = candidate; continue; }
		}
		close(candidate);
	}
	if (matches != 1 && result >= 0) { close(result); result = -1; }
	return result;
}

// Read only new compositor diagnostics for this setup attempt. The assertion is
// visible before the process finishes crash handling and its pidfd becomes ready.
class OverflowLog
{
	fs::path path;
	std::string needle;
	uint64_t cursor = 0;
	std::string suffix;
public:
	explicit OverflowLog(fs::path value, std::string match = "Vulkan transfer ring buffer overflow")
	    : path(std::move(value)), needle(std::move(match)) {}
	void begin()
	{
		std::error_code error;
		cursor = fs::file_size(path, error);
		if (error) cursor = 0;
		suffix.clear();
	}
	bool overflowed()
	{
		std::error_code error;
		uint64_t size = fs::file_size(path, error);
		if (error) return false;
		if (size < cursor) { cursor = 0; suffix.clear(); }
		if (size == cursor) return false;
		std::ifstream stream(path, std::ios::binary);
		stream.seekg(cursor);
		std::string bytes(static_cast<size_t>(std::min<uint64_t>(size - cursor, 65536)), '\0');
		stream.read(bytes.data(), bytes.size());
		bytes.resize(static_cast<size_t>(stream.gcount()));
		cursor += bytes.size();
		suffix += bytes;
		bool found = suffix.find(needle) != std::string::npos;
		if (suffix.size() > 128) suffix.erase(0, suffix.size() - 128);
		return found;
	}
};


// Replacement compositors can retain a broken direct-display session and grow
// NVIDIA/SteamVR descriptors. Drain the crashed session after transport returns.
// This bounded helper deliberately outlives the old server during shutdown.
static int recover_display()
{
	// Ask the whole pinned session to stop up front. Signalling only the
	// monitor leaves a live server/compositor to burn the entire first wait.
	bool server_signalled = syscall(SYS_pidfd_send_signal, 3, SIGTERM, nullptr, 0) == 0;
	syscall(SYS_pidfd_send_signal, 4, SIGTERM, nullptr, 0);
	syscall(SYS_pidfd_send_signal, 5, SIGTERM, nullptr, 0);
	if (!server_signalled) return 1;
	auto wait_session = [](std::chrono::seconds duration) {
		auto deadline = std::chrono::steady_clock::now() + duration;
		while (!exited(3) || !exited(4) || !exited(5)) {
			if (std::chrono::steady_clock::now() >= deadline) return false;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		return true;
	};
	if (!wait_session(std::chrono::seconds(10))) {
		// A wedged old server must not retain the failed display resources.
		// Signal only the pinned old session generations.
		for (int fd : {3, 4, 5})
			if (!exited(fd)) syscall(SYS_pidfd_send_signal, fd, SIGTERM, nullptr, 0);
		if (!wait_session(std::chrono::seconds(5))) {
			for (int fd : {3, 4, 5})
				if (!exited(fd)) syscall(SYS_pidfd_send_signal, fd, SIGKILL, nullptr, 0);
			if (!wait_session(std::chrono::seconds(5))) return 1;
		}
	}
	close(3); close(4); close(5);
	// The guard is gone now, so it cannot observe a later user Quit. Never launch
	// Steam from this detached helper; the user owns starting the next session.
	fprintf(stderr, "monado-roomsetup-guard: display-loss session stopped; start SteamVR again\n");
	return 0;
}

class DisplayHealth
{
	std::chrono::steady_clock::time_point healthy{};
	uint64_t last_vsync = 0;
public:
	bool update(bool available, float age, uint64_t vsync, std::chrono::steady_clock::time_point now)
	{
		bool advancing = available && age >= 0 && age < .1f && vsync > last_vsync;
		last_vsync = vsync;
		if (!advancing) { healthy = {}; return false; }
		if (healthy == std::chrono::steady_clock::time_point{}) healthy = now;
		return now - healthy >= std::chrono::seconds(2);
	}
};

class DisplayRecovery
{
	int compositor = -1;
	pid_t requester = -1;
	bool connected_once = false, lost = false, crash_seen = false, quit_requested = false;
	unsigned loss_streak = 0, attempts = 0;
	OverflowLog crash_log;
	std::chrono::steady_clock::time_point next_request{};
	DisplayHealth display_health;
public:
	explicit DisplayRecovery(fs::path log = {})
	    : crash_log(std::move(log), "Failed Watchdog timeout in thread Render in WaitForPresent")
	{ crash_log.begin(); }
	void note_quit() { quit_requested = true; }
	~DisplayRecovery()
	{
		if (compositor >= 0) close(compositor);
		// The cleanup helper must survive this guard/server's shutdown.
		if (requester > 0) waitpid(requester, nullptr, WNOHANG);
	}
	void update(bool connected, float vsync_age, uint64_t vsync, int server, pid_t server_pid, const char *library)
	{
		auto now = std::chrono::steady_clock::now();
		if (quit_requested) return;
		if (requester > 0) {
			int status = 0;
			pid_t result = waitpid(requester, &status, WNOHANG);
			if (result == requester || (result < 0 && errno == ECHILD)) {
				requester = -1;
				next_request = now + std::chrono::seconds(5);
				fprintf(stderr, "monado-roomsetup-guard: display-loss cleanup helper exited; retries capped at three\n");
			}
		}
		if (loss_streak || lost) crash_seen |= crash_log.overflowed();
		fs::path runtime = fs::path(library).parent_path();
		bool stable = display_health.update(connected && !exited(compositor), vsync_age, vsync, now);
		if (connected) {
			connected_once = true;
			loss_streak = 0;
			if (exited(compositor)) {
				// Adopt a replacement compositor if SteamVR already respawned
				// one: killing the session then would destroy a self-healed
				// display. Keep the dead handle as evidence while no
				// replacement exists so the trigger below still fires.
				int replacement = unique_runtime_process(runtime / "vrcompositor");
				if (replacement >= 0) {
					if (compositor >= 0) close(compositor);
					compositor = replacement;
				}
			}
			// A watchdog can exit well after USB returns. Disarm only after native
			// vsync advances continuously, never merely because the process lives.
			if (lost && stable) {
				lost = crash_seen = false;
				attempts = 0;
				crash_log.begin();
			}
		} else if (connected_once) {
			if (loss_streak == 0 && !lost) { crash_log.begin(); crash_seen = false; }
			// Five consecutive 100 ms polls reject one transient connection miss.
			if (loss_streak < 5) ++loss_streak;
			if (loss_streak >= 5) lost = true;
		}
		if (!connected || !lost || !crash_seen || attempts >= 3 || compositor < 0 || !exited(compositor) || requester > 0 ||
		    now < next_request) return;
		next_request = now + std::chrono::seconds(5);
		int current_server = unique_runtime_process(runtime / "vrserver", server_pid);
		if (current_server < 0) return;
		close(current_server);
		int monitor = unique_runtime_process(runtime / "vrmonitor");
		if (monitor < 0) {
			// There is no monitor generation left to signal. /dev/null is already
			// readable, so exited() treats this slot as complete.
			monitor = open("/dev/null", O_RDONLY | O_CLOEXEC);
			if (monitor < 0) return;
		}
		char executable[4096];
		ssize_t length = readlink("/proc/self/exe", executable, sizeof(executable) - 1);
		if (length <= 0) { close(monitor); return; }
		executable[length] = 0;
		char mode[] = "--recover-display";
		char *arguments[] = {executable, mode, nullptr};
		// Keep sources above the fixed child slots, even if a caller's fd is 3/4/5.
		int server_child = fcntl(server, F_DUPFD_CLOEXEC, 6);
		int monitor_child = fcntl(monitor, F_DUPFD_CLOEXEC, 6);
		int compositor_child = fcntl(compositor, F_DUPFD_CLOEXEC, 6);
		close(monitor);
		if (server_child < 0 || monitor_child < 0 || compositor_child < 0) {
			if (server_child >= 0) close(server_child);
			if (monitor_child >= 0) close(monitor_child);
			if (compositor_child >= 0) close(compositor_child);
			return;
		}
		posix_spawn_file_actions_t actions;
		posix_spawn_file_actions_init(&actions);
		posix_spawn_file_actions_adddup2(&actions, server_child, 3);
		posix_spawn_file_actions_adddup2(&actions, monitor_child, 4);
		posix_spawn_file_actions_adddup2(&actions, compositor_child, 5);
		posix_spawn_file_actions_addclosefrom_np(&actions, 6);
		int spawned = posix_spawn(&requester, executable, &actions, nullptr, arguments, environ);
		posix_spawn_file_actions_destroy(&actions);
		close(server_child); close(monitor_child); close(compositor_child);
		if (spawned) requester = -1;
		else {
			++attempts;
			fprintf(stderr, "monado-roomsetup-guard: draining crashed display-loss session, attempt %u/3; restart SteamVR manually\n", attempts);
		}
	}
};


struct Snapshot
{
	std::string file;
	std::vector<char> exported;
};

static std::optional<Snapshot> snapshot(vr::IVRChaperoneSetup &setup, const fs::path &path)
{
	Snapshot saved{read_file(path), {}};
	uint32_t size = 0;
	setup.ExportLiveToBuffer(nullptr, &size);
	if (saved.file.empty() || size == 0 || size > 4 * 1024 * 1024) return std::nullopt;
	saved.exported.resize(size + 1, 0);
	if (!setup.ExportLiveToBuffer(saved.exported.data(), &size)) return std::nullopt;
	return saved;
}

static bool restore(vr::IVRChaperoneSetup &setup, const fs::path &path, const Snapshot &saved)
{
	setup.RevertWorkingCopy();
	if (!setup.ImportFromBufferToWorking(saved.exported.data(), 0) ||
	    !setup.CommitWorkingCopy(vr::EChaperoneConfigFile_Live)) return false;
	// Commit updates the server's live calibration. Restore the complete file too,
	// including other universes and formatting that OpenVR's export does not retain.
	auto temporary = path;
	temporary += ".monado-guard-" + std::to_string(getpid());
	{
		std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
		stream.write(saved.file.data(), saved.file.size());
		stream.flush();
		if (!stream) return false;
	}
	std::error_code error;
	fs::rename(temporary, path, error);
	if (error) { fs::remove(temporary, error); return false; }
	setup.ReloadFromDisk(vr::EChaperoneConfigFile_Live);
	return read_file(path) == saved.file;
}

// Coordinate dump issues several public queries per call. Keep it behind an
// explicit opt-in so default runs only pay for meaningful state changes/errors.
static bool
coords_debug_enabled()
{
	static const bool enabled = [] {
		const char *value = getenv("MONADO_ROOMSETUP_COORDS_DEBUG");
		return value && value[0] != '\0' && !(value[0] == '0' && value[1] == '\0');
	}();
	return enabled;
}

// Capture public coordinate state on transitions without changing calibration.
static void log_coordinates(vr::IVRSystem &system, vr::IVRChaperoneSetup &setup, uint32_t event, uint32_t step)
{
	if (!coords_debug_enabled()) return;
	auto raw_to_standing = system.GetRawZeroPoseToStandingAbsoluteTrackingPose();
	auto seated_to_standing = system.GetSeatedZeroPoseToStandingAbsoluteTrackingPose();
	vr::HmdMatrix34_t standing_to_raw{};
	bool working = setup.GetWorkingStandingZeroPoseToRawTrackingPose(&standing_to_raw);
	vr::TrackedDevicePose_t head{};
	system.GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0, &head, 1);
	auto universe = system.GetUint64TrackedDeviceProperty(0, vr::Prop_CurrentUniverseId_Uint64);
	auto ticks = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	fprintf(stderr, "monado-coordinates: event=%u step=%u monotonic_ns=%lld universe=%llu head_valid=%d tracking=%d raw_head=(%.6f,%.6f,%.6f) floor_raw_y=%.6f working=%d\n",
	        event, step, (long long)ticks, (unsigned long long)universe, head.bPoseIsValid, head.eTrackingResult,
	        head.mDeviceToAbsoluteTracking.m[0][3], head.mDeviceToAbsoluteTracking.m[1][3],
	        head.mDeviceToAbsoluteTracking.m[2][3], standing_to_raw.m[1][3], working);
	auto log_matrix = [](const char *name, const vr::HmdMatrix34_t &m) {
		fprintf(stderr, "monado-coordinates: %s=[%.6f %.6f %.6f %.6f; %.6f %.6f %.6f %.6f; %.6f %.6f %.6f %.6f]\n",
		        name, m.m[0][0], m.m[0][1], m.m[0][2], m.m[0][3], m.m[1][0], m.m[1][1], m.m[1][2], m.m[1][3],
		        m.m[2][0], m.m[2][1], m.m[2][2], m.m[2][3]);
	};
	log_matrix("raw_to_standing", raw_to_standing);
	log_matrix("seated_to_standing", seated_to_standing);
	if (working) log_matrix("working_standing_to_raw", standing_to_raw);
}

int main(int argc, char **argv)
{
	if (argc == 2 && strcmp(argv[1], "--recover-display") == 0) return recover_display();
	if (argc != 4) return 1;
	int parent = static_cast<int>(syscall(SYS_pidfd_open, std::atoi(argv[2]), 0));
	int ready = std::atoi(argv[3]);
	void *library = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
	if (!library || parent < 0) return 1;
	auto init = reinterpret_cast<uint32_t (*)(vr::EVRInitError *, vr::EVRApplicationType, const char *)>(dlsym(library, "VR_InitInternal2"));
	auto interface = reinterpret_cast<void *(*)(const char *, vr::EVRInitError *)>(dlsym(library, "VR_GetGenericInterface"));
	auto shutdown = reinterpret_cast<void (*)()>(dlsym(library, "VR_ShutdownInternal"));
	if (!init || !interface || !shutdown) return 1;
	vr::EVRInitError error = vr::VRInitError_Unknown;
	for (unsigned attempt = 0; attempt < 100 && !exited(parent); ++attempt) {
		init(&error, vr::VRApplication_Background, nullptr);
		if (error == vr::VRInitError_None) break;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	if (error != vr::VRInitError_None) return 1;
	auto *system = static_cast<vr::IVRSystem *>(interface(vr::IVRSystem_Version, &error));
	auto *setup = static_cast<vr::IVRChaperoneSetup *>(interface(vr::IVRChaperoneSetup_Version, &error));
	auto *overlays = static_cast<vr::IVROverlay *>(interface(vr::IVROverlay_Version, &error));
	auto *notifications = static_cast<vr::IVRNotifications *>(interface(vr::IVRNotifications_Version, &error));
	fs::path log_path;
	fs::path path = calibration_path(log_path);
	OverflowLog diagnostics(log_path);
	if (!system) { shutdown(); return 1; }
	const bool setup_available = setup && !path.empty() && snapshot(*setup, path).has_value();
	char model[128] = {};
	system->GetStringTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_ModelNumber_String,
	                                     model, sizeof(model));
	const bool is_rift_s = std::strstr(model, "Oculus Rift S") != nullptr;
	vr::VROverlayHandle_t overlay = vr::k_ulOverlayHandleInvalid;
	if (overlays) overlays->CreateOverlay("monado.roomsetup.guard", "Monado playspace setup", &overlay);
	if (write(ready, setup_available ? "1" : "0", 1) != 1) { shutdown(); return 1; }
	fprintf(stderr, "monado-roomsetup-guard: ready\n");
	std::optional<Snapshot> saved;
	int compositor = -1;
	uint32_t step = 0;
	bool failed = false, failure_handled = false, native_commit = false;
	bool overflow_detected = false;
	auto commit_deadline = std::chrono::steady_clock::time_point{};
	auto exit_deadline = std::chrono::steady_clock::time_point{};
	auto compositor_failed = [&] {
		overflow_detected |= diagnostics.overflowed();
		return overflow_detected || exited(compositor);
	};
	auto drawing = [](uint32_t value) { return value == 5 || value == 7; };
	auto repair = [&] {
		failure_handled = true;
		// Hide setup until a fresh SteamVR session, while retaining late-Accept protection.
		if (write(ready, "0", 1) != 1) return false;
		bool ok = saved && restore(*setup, path, *saved);
		fprintf(stderr, "monado-roomsetup-guard: previous calibration restore=%s\n", ok ? "ok" : "FAILED");
		if (notifications && overlay != vr::k_ulOverlayHandleInvalid) {
			vr::VRNotificationId id;
			notifications->CreateNotification(overlay, 0, vr::EVRNotificationType_Persistent,
			    ok ? "Playspace setup failed. Previous boundary restored. Restart SteamVR before drawing again."
			       : "Playspace setup failed. Calibration restore failed. Stop using VR and restore your saved boundary.",
			    vr::EVRNotificationStyle_Application, nullptr, &id);
		}
		return ok;
	};
	auto *mailbox = static_cast<roomsetup::Mailbox *>(interface("IVRMailbox_001", &error));
	auto *paths = static_cast<roomsetup::Paths *>(interface("IVRPaths_001", &error));
	auto floor = std::make_unique<roomsetup::SavedFloor>(mailbox, paths);
	if (setup_available) {
		floor->publish();
		log_coordinates(*system, *setup, 0, step);
	}
	bool floor_pending = false;
	auto floor_deadline = std::chrono::steady_clock::time_point{};
	bool running = true;
	bool world_setup = false;
	bool saved_plane_trusted = !is_rift_s;
	auto world_cancel_deadline = std::chrono::steady_clock::time_point{};
	std::string world_warning;
	vr::VRNotificationId world_notification = 0;
	auto next_notification_attempt = std::chrono::steady_clock::time_point{};
	DisplayRecovery display_recovery(log_path);
	auto next_display_check = std::chrono::steady_clock::time_point{};
	while (running && !exited(parent)) {
		// Test before consuming Accept's events, including a compositor that exits immediately before Accept.
		bool recent_exit = std::chrono::steady_clock::now() <= exit_deadline;
		if ((drawing(step) || recent_exit) && saved && !failure_handled &&
		    compositor_failed()) {
			failed = true;
			fprintf(stderr, "monado-roomsetup-guard: native setup failure=%s\n",
			        overflow_detected ? "transfer_ring_overflow" : "compositor_exit");
			if (!repair()) break;
		}
		vr::VREvent_t event{};
		while (system->PollNextEvent(&event, sizeof(event))) {
			if (setup_available && event.eventType == 809) {
				// SteamVR 2.17's native setup step event is reserved in the public header.
				uint32_t next = 0;
				memcpy(&next, &event.data, sizeof(next));
				if (drawing(next) && !drawing(step) && !failed) {
					saved = snapshot(*setup, path);
					diagnostics.begin();
					overflow_detected = false;
					exit_deadline = {};
					if (compositor >= 0) close(compositor);
					compositor = compositor_fd();
					if (!saved || exited(compositor)) { running = false; break; }
					fprintf(stderr, "monado-roomsetup-guard: snapshot step=%u\n", next);
				}
				if (next == 0 && drawing(step)) {
					native_commit = true;
					exit_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
					commit_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
					if (saved && compositor_failed()) failed = true;
				}
				if ((next == 2 || next == 3 || drawing(next)) && !world_setup && !failed) {
					// Capture trust before begin switches the tracker into a new setup frame.
					saved_plane_trusted = !is_rift_s || (world_status() == "localized" &&
					    system->GetUint64TrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd,
					        vr::Prop_CurrentUniverseId_Uint64) == 2);
					world_setup = true;
					world_cancel_deadline = {};
					world_command("begin");
				}
				if (next == 0 && world_setup)
					world_cancel_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
				step = next;
				// Native Manual mode retains stick adjustments even when a result arrives.
				if (next == 2 || next == 3 || next == 5 || next == 7) {
					floor_pending = true;
					floor_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
				}
			}
			if (event.eventType == vr::VREvent_ChaperoneRoomSetupFinished && native_commit) {
				// Cancel has the same step-zero event, but no immediate commit.
				// A later unrelated calibration edit must not inherit that attempt.
				bool immediate = std::chrono::steady_clock::now() <= commit_deadline;
				native_commit = false; // Our restore also emits this event.
				if (immediate && saved && compositor_failed()) failed = true;
				// A handled failure already restored its snapshot. Later finishes
				// must not restore that stale snapshot over a newer setup.
				if (immediate && failed && !failure_handled && !repair()) { running = false; break; }
				if (immediate && world_setup && !failed) {
					world_command("save");
					world_setup = false;
					world_cancel_deadline = {};
				}
			}
			if (setup_available && (event.eventType == 809 || event.eventType == vr::VREvent_ChaperoneUniverseHasChanged ||
			    event.eventType == vr::VREvent_ChaperoneDataHasChanged || event.eventType == vr::VREvent_ChaperoneRoomSetupFinished ||
			    event.eventType == vr::VREvent_StandingZeroPoseReset || event.eventType == vr::VREvent_SeatedZeroPoseReset ||
			    event.eventType == vr::VREvent_RoomViewShown || event.eventType == vr::VREvent_RoomViewHidden ||
			    event.eventType == vr::VREvent_DashboardActivated || event.eventType == vr::VREvent_DashboardDeactivated))
				log_coordinates(*system, *setup, event.eventType, step);
			if (event.eventType == vr::VREvent_Quit) { display_recovery.note_quit(); running = false; }
		}
		if (!running) break;
		if (world_setup && world_cancel_deadline != std::chrono::steady_clock::time_point{} &&
		    std::chrono::steady_clock::now() >= world_cancel_deadline) {
			world_command("cancel");
			world_setup = false;
			world_cancel_deadline = {};
		}
		auto status = world_status();
		std::string warning;
		if (status == "save_failed")
			warning = "Visual boundary map could not be saved. Check Monado config write access before restarting VR.";
		else if (status == "unanchored")
			warning = "Guardian not found: no usable saved room map. Run Room Setup to anchor the boundary.";
		else if (status == "relocalizing")
			warning = "Guardian not found. Uncover headset cameras and look around. If the room stays unavailable, run Room Setup.";
		auto now = std::chrono::steady_clock::now();
		if (warning != world_warning && notifications && overlays && now >= next_notification_attempt) {
			next_notification_attempt = now + std::chrono::seconds(1);
			if (overlay == vr::k_ulOverlayHandleInvalid)
				overlays->CreateOverlay("monado.roomsetup.guard", "Monado playspace setup", &overlay);
			if (overlay != vr::k_ulOverlayHandleInvalid)
				update_world_notification(*notifications, overlay, warning, world_warning, world_notification);
		}
		if (floor_pending) {
			if (step == 0 || floor->publish(step == 2, saved_plane_trusted) ||
			    std::chrono::steady_clock::now() > floor_deadline)
				floor_pending = false;
		}
		if (is_rift_s && now >= next_display_check) {
			next_display_check = now + std::chrono::milliseconds(100);
			vr::TrackedDevicePose_t head{};
			system->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0, &head, 1);
			float vsync_age = -1;
			uint64_t vsync = 0;
			if (!system->GetTimeSinceLastVsync(&vsync_age, &vsync)) vsync_age = -1;
			display_recovery.update(head.bDeviceIsConnected, vsync_age, vsync, parent, std::atoi(argv[2]), argv[1]);
		}
		floor->update_frame(step == 2 || step == 3 || step == 4 || step == 5 || step == 7);
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
	if (world_setup) world_command("cancel");
	if (world_notification && notifications) notifications->RemoveNotification(world_notification);
	if (overlays && overlay != vr::k_ulOverlayHandleInvalid) overlays->DestroyOverlay(overlay);
	if (compositor >= 0) close(compositor);
	close(parent);
	close(ready);
	floor.reset();
	shutdown();
	dlclose(library);
	return running ? 1 : 0;
}
