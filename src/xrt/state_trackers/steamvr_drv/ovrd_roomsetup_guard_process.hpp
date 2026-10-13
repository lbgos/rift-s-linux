// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#pragma once
#ifdef __linux__
#include <dlfcn.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;
static void roomsetup_guard_module() {}
static pid_t roomsetup_wait_child(pid_t pid, int options)
{
	pid_t result;
	do { result = waitpid(pid, nullptr, options); } while (result < 0 && errno == EINTR);
	return result;
}

// Native setup is advertised only after the session-owned guard acknowledges readiness.
class RoomSetupGuardProcess
{
	pid_t pid = -1;
	int acknowledgement = -1;
	bool acknowledged = false;
public:
	~RoomSetupGuardProcess()
	{
		if (acknowledgement >= 0) close(acknowledgement);
		if (pid > 0 && roomsetup_wait_child(pid, WNOHANG) == 0) {
			kill(pid, SIGTERM);
			roomsetup_wait_child(pid, 0);
		}
	}
	bool start()
	{
		if (pid > 0) {
			// Activate can run again on the same driver object. Reuse the live
			// guard instead of orphaning it behind a second child.
			if (roomsetup_wait_child(pid, WNOHANG) == 0) return true;
			pid = -1;
			acknowledged = false;
			if (acknowledgement >= 0) { close(acknowledgement); acknowledgement = -1; }
		}
		Dl_info module{};
		char server[4096];
		ssize_t length = readlink("/proc/self/exe", server, sizeof(server) - 1);
		if (length <= 0 || !dladdr(reinterpret_cast<void *>(roomsetup_guard_module), &module)) return false;
		server[length] = 0;
		auto directory = std::filesystem::path(module.dli_fname).parent_path();
		std::string executable = (directory / "monado-roomsetup-guard").string();
		std::string library = (std::filesystem::path(server).parent_path() / "libopenvr_api.so").string();
		std::string parent = std::to_string(getpid());
		int descriptors[2];
		if (pipe2(descriptors, O_CLOEXEC | O_NONBLOCK) != 0) return false;
		posix_spawn_file_actions_t actions;
		posix_spawn_file_actions_init(&actions);
		constexpr int child_acknowledgement = 3;
		posix_spawn_file_actions_adddup2(&actions, descriptors[1], child_acknowledgement);
		// SteamVR also owns IPC descriptors without CLOEXEC. The guard must not
		// retain those connections or shared buffers after a compositor exits.
		posix_spawn_file_actions_addclosefrom_np(&actions, child_acknowledgement + 1);
		char destination[] = "3";
		char *arguments[] = {executable.data(), library.data(), parent.data(), destination, nullptr};
		int error = posix_spawn(&pid, executable.c_str(), &actions, nullptr, arguments, environ);
		posix_spawn_file_actions_destroy(&actions);
		close(descriptors[1]);
		if (error) { fprintf(stderr, "monado-roomsetup-guard: spawn %s: %s\n", executable.c_str(), strerror(error)); close(descriptors[0]); pid = -1; return false; }
		acknowledgement = descriptors[0];
		return true;
	}
	bool ready()
	{
		if (pid <= 0) return false;
		if (roomsetup_wait_child(pid, WNOHANG) == pid) { pid = -1; acknowledged = false; }
		if (acknowledgement >= 0) {
			char value = 0;
			ssize_t count = read(acknowledgement, &value, 1);
			if (count == 1) acknowledged = pid > 0 && value == '1';
			if (count == 0) {
				acknowledged = false;
				close(acknowledgement);
				acknowledgement = -1;
			}
		}
		return acknowledged;
	}
};
#else
class RoomSetupGuardProcess
{
public:
	bool start() { return false; }
	bool ready() { return false; }
};
#endif
