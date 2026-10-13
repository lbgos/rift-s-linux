// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "vr.hpp"

#include <dirent.h>
#include <dlfcn.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

namespace rd {

// Public client-core contract of vrclient.so, as used by Valve's open-source
// openvr_api (BSD-3, github.com/ValveSoftware/openvr, src/ivrclientcore.h).
// Implemented inside vrclient.so. Keep it out of the anonymous namespace: there
// the compiler may assume no subclass exists and call the pure stubs directly.
class IVRClientCore
{
public:
	virtual vr::EVRInitError
	Init(vr::EVRApplicationType type, const char *startup_info) = 0;
	virtual void
	Cleanup() = 0;
	virtual vr::EVRInitError
	IsInterfaceVersionValid(const char *version) = 0;
	virtual void *
	GetGenericInterface(const char *version, vr::EVRInitError *error) = 0;
	virtual bool
	BIsHmdPresent() = 0;
	virtual const char *
	GetEnglishStringForHmdError(vr::EVRInitError error) = 0;
	virtual const char *
	GetIDForVRInitError(vr::EVRInitError error) = 0;
};

namespace {

using Factory = void *(*)(const char *interface_name, int *return_code);

void *g_library = nullptr;
IVRClientCore *g_core = nullptr;

// Reads the "runtime" string array of openvrpaths.vrpath. The file is small,
// machine-written JSON; only string escapes need handling.
std::vector<std::string>
runtime_paths()
{
	std::string path;
	if (const char *o = getenv("VR_PATHREG_OVERRIDE"); o && *o) {
		path = o;
	} else if (const char *c = getenv("XDG_CONFIG_HOME"); c && *c) {
		path = std::string(c) + "/openvr/openvrpaths.vrpath";
	} else if (const char *h = getenv("HOME"); h && *h) {
		path = std::string(h) + "/.config/openvr/openvrpaths.vrpath";
	}
	std::ifstream file(path);
	std::stringstream ss;
	ss << file.rdbuf();
	const std::string json = ss.str();

	std::vector<std::string> out;
	size_t at = json.find("\"runtime\"");
	if (at == std::string::npos || (at = json.find('[', at)) == std::string::npos) {
		return out;
	}
	for (size_t i = at + 1; i < json.size() && json[i] != ']'; i++) {
		if (json[i] != '"') {
			continue;
		}
		std::string value;
		for (i++; i < json.size() && json[i] != '"'; i++) {
			if (json[i] == '\\' && i + 1 < json.size()) {
				i++;
			}
			value += json[i];
		}
		out.push_back(value);
	}
	return out;
}

} // namespace

int
vrserver_pid()
{
	DIR *proc = opendir("/proc");
	if (!proc) {
		return 0;
	}
	int found = 0;
	while (dirent *entry = readdir(proc)) {
		char *end = nullptr;
		long pid = strtol(entry->d_name, &end, 10);
		if (*end != '\0' || pid <= 0) {
			continue;
		}
		std::string base = std::string("/proc/") + entry->d_name;
		struct stat st = {};
		if (stat(base.c_str(), &st) != 0 || st.st_uid != getuid()) {
			continue;
		}
		std::ifstream comm(base + "/comm");
		std::string name;
		std::getline(comm, name);
		if (name == "vrserver") {
			found = int(pid);
			break;
		}
	}
	closedir(proc);
	return found;
}

bool
vr_connect(vr::EVRApplicationType type, Vr &out, std::string &error)
{
	if (vrserver_pid() == 0) {
		error = "SteamVR is not running";
		return false;
	}
	for (const std::string &runtime : runtime_paths()) {
		std::string lib = runtime + "/bin/linux64/vrclient.so";
		if (access(lib.c_str(), R_OK) == 0 && (g_library = dlopen(lib.c_str(), RTLD_NOW | RTLD_LOCAL))) {
			break;
		}
	}
	if (!g_library) {
		error = "no registered SteamVR runtime with vrclient.so";
		return false;
	}
	auto factory = reinterpret_cast<Factory>(dlsym(g_library, "VRClientCoreFactory"));
	int rc = 0;
	g_core = factory ? static_cast<IVRClientCore *>(factory("IVRClientCore_003", &rc)) : nullptr;
	if (!g_core) {
		error = "vrclient.so has no IVRClientCore_003";
		return false;
	}
	vr::EVRInitError init = g_core->Init(type, nullptr);
	if (init != vr::VRInitError_None) {
		error = std::string("VR init failed: ") + g_core->GetIDForVRInitError(init);
		g_core = nullptr;
		return false;
	}
	auto get = [](const char *version) {
		vr::EVRInitError e = vr::VRInitError_None;
		void *p = g_core->GetGenericInterface(version, &e);
		return e == vr::VRInitError_None ? p : nullptr;
	};
	out.system = static_cast<vr::IVRSystem *>(get(vr::IVRSystem_Version));
	out.overlay = static_cast<vr::IVROverlay *>(get(vr::IVROverlay_Version));
	out.apps = static_cast<vr::IVRApplications *>(get(vr::IVRApplications_Version));
	out.compositor = static_cast<vr::IVRCompositor *>(get(vr::IVRCompositor_Version));
	if (!out.apps || (type != vr::VRApplication_Utility && (!out.system || !out.overlay))) {
		error = "SteamVR is missing a required interface version";
		vr_disconnect();
		return false;
	}
	return true;
}

void
vr_disconnect()
{
	if (g_core) {
		g_core->Cleanup();
		g_core = nullptr;
	}
}

} // namespace rd
