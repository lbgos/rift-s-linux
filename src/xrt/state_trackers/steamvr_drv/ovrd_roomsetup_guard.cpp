// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
// Keep the pre-setup calibration if SteamVR's compositor dies during native drawing.
#include "openvr.h"
#include "ovrd_roomsetup_floor.hpp"
#include "../../drivers/rift_s/rift_s_world_control.h"
#include <cjson/cJSON.h>
#include <chrono>
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
#include <sys/syscall.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;

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

// Read only new compositor diagnostics for this setup attempt. The assertion is
// visible before the process finishes crash handling and its pidfd becomes ready.
class OverflowLog
{
	fs::path path;
	uint64_t cursor = 0;
	std::string suffix;
public:
	explicit OverflowLog(fs::path value) : path(std::move(value)) {}
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
		bool found = suffix.find("Vulkan transfer ring buffer overflow") != std::string::npos;
		if (suffix.size() > 128) suffix.erase(0, suffix.size() - 128);
		return found;
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

// Capture public coordinate state on transitions without changing calibration.
static void log_coordinates(vr::IVRSystem &system, vr::IVRChaperoneSetup &setup, uint32_t event, uint32_t step)
{
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
	if (!system || !setup || path.empty() || !snapshot(*setup, path)) { shutdown(); return 1; }
	vr::VROverlayHandle_t overlay = vr::k_ulOverlayHandleInvalid;
	if (overlays) overlays->CreateOverlay("monado.roomsetup.guard", "Monado playspace setup", &overlay);
	if (write(ready, "1", 1) != 1) { shutdown(); return 1; }
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
	floor->publish();
	log_coordinates(*system, *setup, 0, step);
	bool floor_pending = false;
	auto floor_deadline = std::chrono::steady_clock::time_point{};
	bool running = true;
	bool world_setup = false;
	auto world_cancel_deadline = std::chrono::steady_clock::time_point{};
	std::string world_warning;
	vr::VRNotificationId world_notification = 0;
	auto next_notification_attempt = std::chrono::steady_clock::time_point{};
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
			if (event.eventType == 809) {
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
				if (immediate && failed && !repair()) { running = false; break; }
				if (immediate && world_setup && !failed) {
					world_command("save");
					world_setup = false;
					world_cancel_deadline = {};
				}
			}
			if (event.eventType == 809 || event.eventType == vr::VREvent_ChaperoneUniverseHasChanged ||
			    event.eventType == vr::VREvent_ChaperoneDataHasChanged || event.eventType == vr::VREvent_ChaperoneRoomSetupFinished ||
			    event.eventType == vr::VREvent_StandingZeroPoseReset || event.eventType == vr::VREvent_SeatedZeroPoseReset ||
			    event.eventType == vr::VREvent_RoomViewShown || event.eventType == vr::VREvent_RoomViewHidden ||
			    event.eventType == vr::VREvent_DashboardActivated || event.eventType == vr::VREvent_DashboardDeactivated)
				log_coordinates(*system, *setup, event.eventType, step);
			if (event.eventType == vr::VREvent_Quit) running = false;
		}
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
			warning = "Boundary has no visual anchor. Complete room setup once to save its physical location.";
		else if (status == "relocalizing")
			warning = "Boundary not confirmed, look around. Temporary tracking is active.";
		auto now = std::chrono::steady_clock::now();
		if (warning != world_warning && notifications && overlays && now >= next_notification_attempt) {
			next_notification_attempt = now + std::chrono::seconds(1);
			if (overlay == vr::k_ulOverlayHandleInvalid)
				overlays->CreateOverlay("monado.roomsetup.guard", "Monado playspace setup", &overlay);
			if (overlay != vr::k_ulOverlayHandleInvalid)
				update_world_notification(*notifications, overlay, warning, world_warning, world_notification);
		}
		if (floor_pending) {
			if (step == 0 || floor->publish(step == 2) || std::chrono::steady_clock::now() > floor_deadline)
				floor_pending = false;
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
