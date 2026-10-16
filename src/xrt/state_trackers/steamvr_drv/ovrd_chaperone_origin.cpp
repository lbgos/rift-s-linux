// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "ovrd_chaperone_origin.hpp"
#include "util/u_json.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#ifdef __linux__
#include <unistd.h>
#include <sys/stat.h>
#endif

namespace {
using Json = std::unique_ptr<cJSON, decltype(&cJSON_Delete)>;
std::string read(const std::string &path, size_t maximum)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) return {};
    file.seekg(0, std::ios::end);
    auto size = file.tellg();
    if (size < 0 || size > std::streamoff(maximum)) return {};
    std::string bytes(size_t(size), '\0');
    file.seekg(0);
    return file.read(bytes.data(), size) ? bytes : std::string{};
}
}

bool rift_s_prepare_session_origin(const std::string &path)
{
#ifdef __linux__
    auto original = read(path, 4 * 1024 * 1024);
    Json document(nullptr, cJSON_Delete);
    if (original.empty()) {
        if (access(path.c_str(), F_OK) == 0) return false;
        document.reset(cJSON_Parse("{\"jsonid\":\"chaperone_info\",\"version\":5,\"universes\":[]}"));
    } else {
        document.reset(cJSON_Parse(original.c_str()));
    }
    auto id = cJSON_GetObjectItemCaseSensitive(document.get(), "jsonid");
    auto universes = cJSON_GetObjectItemCaseSensitive(document.get(), "universes");
    if (!cJSON_IsString(id) || std::strcmp(id->valuestring, "chaperone_info") || !cJSON_IsArray(universes))
        return false;
    Json origin(cJSON_Parse("{\"universeID\":\"3\",\"seated\":{\"translation\":[0,0,0],\"yaw\":0},\"standing\":{\"translation\":[0,0,0],\"yaw\":0}}"), cJSON_Delete);
    if (!origin) return false;
    // Each session re-bases tracking, so offsets saved in universe 3 (for example by
    // Quick Calibrate) would misplace the floor next time. Reset it to the plain origin.
    int index = 0;
    cJSON *entry;
    cJSON_ArrayForEach(entry, universes) {
        auto universe = cJSON_GetObjectItemCaseSensitive(entry, "universeID");
        if (cJSON_IsString(universe) && std::strcmp(universe->valuestring, "3") == 0) {
            if (cJSON_Compare(entry, origin.get(), true)) return true;
            if (!cJSON_ReplaceItemInArray(universes, index, origin.get())) return false;
            break;
        }
        ++index;
    }
    if (!entry && !cJSON_AddItemToArray(universes, origin.get())) return false;
    origin.release();
    std::unique_ptr<char, decltype(&cJSON_free)> bytes(cJSON_PrintUnformatted(document.get()), cJSON_free);
    if (!bytes) return false;
    std::string temporary = path + ".monado-XXXXXX";
    int fd = mkstemp(temporary.data());
    if (fd < 0) return false;
    FILE *file = fdopen(fd, "wb");
    if (!file) { close(fd); unlink(temporary.c_str()); return false; }
    size_t size = std::strlen(bytes.get());
    bool written = fwrite(bytes.get(), 1, size, file) == size;
    written = fflush(file) == 0 && written;
    written = fsync(fd) == 0 && written;
    written = fclose(file) == 0 && written;
    // Fail rather than replace a concurrent SteamVR update.
    if (!written || read(path, 4 * 1024 * 1024) != original || rename(temporary.c_str(), path.c_str()) != 0) {
        unlink(temporary.c_str());
        return false;
    }
    return true;
#else
    return false;
#endif
}

bool rift_s_prepare_session_origin()
{
    const char *home = std::getenv("HOME"), *config = std::getenv("XDG_CONFIG_HOME");
    if (!home && !config) return false;
    auto registry_path = (config ? std::string(config) : std::string(home) + "/.config") + "/openvr/openvrpaths.vrpath";
    auto bytes = read(registry_path, 8192);
    Json registry(cJSON_Parse(bytes.c_str()), cJSON_Delete);
    auto config_path = cJSON_GetArrayItem(cJSON_GetObjectItemCaseSensitive(registry.get(), "config"), 0);
    return cJSON_IsString(config_path) && config_path->valuestring[0] &&
           rift_s_prepare_session_origin(std::string(config_path->valuestring) + "/chaperone_info.vrchap");
}
