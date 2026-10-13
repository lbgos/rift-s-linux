// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
#include "desktop.hpp"

#include <systemd/sd-bus.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>

namespace rd {
namespace {
constexpr const char *kService = "org.freedesktop.portal.Desktop";
constexpr const char *kPath = "/org/freedesktop/portal/desktop";
constexpr const char *kCast = "org.freedesktop.portal.ScreenCast";
constexpr const char *kRemote = "org.freedesktop.portal.RemoteDesktop";
constexpr const char *kSession = "org.freedesktop.portal.Session";

// Requests are asynchronous: the monitor picker must never block VR input or shutdown.
class PortalDesktop final : public Desktop
{
public:
	PortalDesktop(pw_context *context, const DesktopEvents &events) : context_(context), events_(events) {}
	~PortalDesktop() override
	{
		stop("Desktop");
		sd_bus_slot_unref(response_slot_);
		sd_bus_slot_unref(closed_slot_);
		sd_bus_slot_unref(call_slot_);
		sd_bus_flush_close_unref(bus_);
	}

	bool connect(std::string &error)
	{
		if (sd_bus_open_user(&bus_) < 0) {
			error = "cannot connect to the desktop portal session bus";
			return false;
		}
		uint32_t version = 0;
		if (sd_bus_get_property_trivial(bus_, kService, kPath, kCast, "version", nullptr, 'u', &version) < 0) {
			error = "ScreenCast portal unavailable";
			return false;
		}
		cast_persistence_ = version >= 4;
		uint32_t cursor_modes = 1;
		sd_bus_get_property_trivial(bus_, kService, kPath, kCast, "AvailableCursorModes", nullptr, 'u', &cursor_modes);
		cursor_mode_ = (cursor_modes & 2) ? 2 : 1;
		version = 0;
		remote_ = sd_bus_get_property_trivial(bus_, kService, kPath, kRemote, "version", nullptr, 'u', &version) >= 0;
		remote_persistence_ = version >= 2;
		const char *unique = nullptr;
		sd_bus_get_unique_name(bus_, &unique);
		owner_ = unique ? unique + 1 : "";
		for (char &c : owner_) if (c == '.') c = '_';
		const char *state = getenv("XDG_STATE_HOME"), *home = getenv("HOME");
		if (state && *state) token_path_ = std::string(state) + "/rifts-desktop/portal-token";
		else if (home) token_path_ = std::string(home) + "/.local/state/rifts-desktop/portal-token";
		if (!token_path_.empty()) {
			std::ifstream file(token_path_);
			std::getline(file, token_); // never log the permission token
		}
		return true;
	}

	const char *name() const override { return remote_ ? "portal-remote-desktop" : "portal-screencast"; }
	std::vector<Screen> screens() const override { return {{"Desktop", width_, height_}}; }
	bool has_keyboard() const override { return remote_; }
	int fd() const override { return sd_bus_get_fd(bus_); }
	void prepare() override { sd_bus_flush(bus_); }
	bool dispatch(bool) override
	{
		int r;
		do { r = sd_bus_process(bus_, nullptr); } while (r > 0);
		return r >= 0;
	}

	void start(const std::string &) override
	{
		if (stage_ != Stage::Idle || denied_) return;
		fprintf(stderr, "rifts-desktop: portal sharing may require one approval on the monitor\n");
		request(Stage::Create);
	}
	void stop(const std::string &) override
	{
		sd_bus_slot_unref(call_slot_); call_slot_ = nullptr;
		sd_bus_slot_unref(response_slot_); response_slot_ = nullptr;
		sd_bus_slot_unref(closed_slot_); closed_slot_ = nullptr;
		if (!request_path_.empty()) close_object(request_path_, "org.freedesktop.portal.Request");
		request_path_.clear();
		if (!session_.empty()) close_object(session_, kSession);
		session_.clear();
		if (core_) { pw_core_disconnect(core_); core_ = nullptr; }
		stage_ = Stage::Idle;
		devices_ = 0;
		denied_ = false;
		target_.clear();
	}

	void pointer(const std::string &, double x, double y) override
	{
		if (!(devices_ & 2)) return;
		auto *m = input_message("NotifyPointerMotionAbsolute");
		if (m) { sd_bus_message_append(m, "udd", node_, x, y); send_input(m); }
	}
	void button(uint32_t code, bool pressed) override
	{
		if (!(devices_ & 2)) return;
		auto *m = input_message("NotifyPointerButton");
		if (m) { sd_bus_message_append(m, "iu", int32_t(code), uint32_t(pressed)); send_input(m); }
	}
	void scroll(double dx, double dy) override
	{
		if (!(devices_ & 2)) return;
		for (uint32_t axis = 0; axis < 2; ++axis) {
			int32_t steps = int32_t(std::round(axis ? dx : dy));
			if (!steps) continue;
			auto *m = input_message("NotifyPointerAxisDiscrete");
			if (m) { sd_bus_message_append(m, "ui", axis, steps); send_input(m); }
		}
	}
	void key(uint32_t keysym) override
	{
		if (!(devices_ & 1)) return;
		for (uint32_t state : {1u, 0u}) {
			auto *m = input_message("NotifyKeyboardKeysym");
			if (m) { sd_bus_message_append(m, "iu", int32_t(keysym), state); send_input(m); }
		}
	}

private:
	enum class Stage { Idle, Create, Devices, Sources, Start, Open, Streaming };
	static void option(sd_bus_message *m, const char *key, const char *type, const void *value)
	{
		sd_bus_message_open_container(m, 'e', "sv");
		sd_bus_message_append(m, "s", key);
		sd_bus_message_open_container(m, 'v', type);
		sd_bus_message_append_basic(m, *type, value);
		sd_bus_message_close_container(m);
		sd_bus_message_close_container(m);
	}
	void close_object(const std::string &path, const char *interface)
	{
		sd_bus_message *m = nullptr;
		if (sd_bus_message_new_method_call(bus_, &m, kService, path.c_str(), interface, "Close") >= 0) {
			sd_bus_message_set_expect_reply(m, 0);
			sd_bus_send(bus_, m, nullptr);
		}
		sd_bus_message_unref(m);
	}
	void fail(const char *reason)
	{
		fprintf(stderr, "rifts-desktop: portal %s\n", reason);
		if (events_.stream_closed) events_.stream_closed("Desktop");
		stop("Desktop");
		denied_ = true; // wait for the user to leave and reopen the tab before prompting again
	}
	void request(Stage stage)
	{
		sd_bus_slot_unref(call_slot_); call_slot_ = nullptr;
		sd_bus_slot_unref(response_slot_); response_slot_ = nullptr;
		stage_ = stage;
		const char *interface = stage == Stage::Sources || stage == Stage::Open || !remote_ ? kCast : kRemote;
		const char *method = stage == Stage::Create ? "CreateSession" : stage == Stage::Devices ? "SelectDevices" :
		                     stage == Stage::Sources ? "SelectSources" : stage == Stage::Start ? "Start" : "OpenPipeWireRemote";
		std::string handle = "rd" + std::to_string(++sequence_);
		request_path_ = "/org/freedesktop/portal/desktop/request/" + owner_ + "/" + handle;
		if (stage != Stage::Open && sd_bus_match_signal(bus_, &response_slot_, kService, request_path_.c_str(),
		                                              "org.freedesktop.portal.Request", "Response", response, this) < 0) {
			fail("response subscription failed"); return;
		}
		sd_bus_message *m = nullptr;
		int r = sd_bus_message_new_method_call(bus_, &m, kService, kPath, interface, method);
		if (r >= 0 && stage != Stage::Create) r = sd_bus_message_append(m, "o", session_.c_str());
		if (r >= 0 && stage == Stage::Start) r = sd_bus_message_append(m, "s", "");
		if (r >= 0) r = sd_bus_message_open_container(m, 'a', "{sv}");
		if (r >= 0) {
			option(m, "handle_token", "s", handle.c_str());
			if (stage == Stage::Create) option(m, "session_handle_token", "s", handle.c_str());
			uint32_t types = stage == Stage::Devices ? 3 : 1, mode = 2;
			if (stage == Stage::Devices || stage == Stage::Sources) option(m, "types", "u", &types);
			if (stage == Stage::Sources) {
				int multiple = 0;
				option(m, "multiple", "b", &multiple);
				// Prefer an embedded cursor, but allow view-only portals offering only hidden.
				option(m, "cursor_mode", "u", &cursor_mode_);
			}
			if ((stage == Stage::Devices && remote_persistence_) ||
			    (stage == Stage::Sources && !remote_ && cast_persistence_)) {
				option(m, "persist_mode", "u", &mode);
				if (!token_.empty()) option(m, "restore_token", "s", token_.c_str());
			}
			r = sd_bus_message_close_container(m);
		}
		if (r >= 0) r = sd_bus_call_async(bus_, &call_slot_, m, reply, this, 30000000);
		sd_bus_message_unref(m);
		if (r < 0) fail("request failed");
	}
	static int reply(sd_bus_message *m, void *data, sd_bus_error *)
	{
		auto &self = *static_cast<PortalDesktop *>(data);
		if (sd_bus_message_is_method_error(m, nullptr)) { self.fail("method failed"); return 0; }
		if (self.stage_ == Stage::Open) {
			int fd = -1;
			if (sd_bus_message_read(m, "h", &fd) < 0 || fd < 0) { self.fail("missing PipeWire fd"); return 0; }
			int owned = fcntl(fd, F_DUPFD_CLOEXEC, 0);
			self.core_ = owned < 0 ? nullptr : pw_context_connect_fd(self.context_, owned, nullptr, 0);
			if (!self.core_) { self.fail("PipeWire connection failed"); return 0; }
			self.request_path_.clear();
			self.stage_ = Stage::Streaming;
			if (self.events_.screens_changed) self.events_.screens_changed();
			if (self.events_.stream_ready) self.events_.stream_ready("Desktop", self.core_, self.target_);
		}
		return 0;
	}
	void read_streams(sd_bus_message *m)
	{
		if (sd_bus_message_enter_container(m, 'a', "(ua{sv})") <= 0) return;
		if (sd_bus_message_enter_container(m, 'r', "ua{sv}") > 0) {
			sd_bus_message_read(m, "u", &node_);
			target_ = std::to_string(node_);
			sd_bus_message_enter_container(m, 'a', "{sv}");
			while (sd_bus_message_enter_container(m, 'e', "sv") > 0) {
				const char *key = nullptr, *sig = nullptr;
				sd_bus_message_read(m, "s", &key);
				sd_bus_message_peek_type(m, nullptr, &sig);
				if (key && sig && (!strcmp(key, "size") || !strcmp(key, "logical_size")) && !strcmp(sig, "(ii)")) {
					sd_bus_message_enter_container(m, 'v', "(ii)");
					sd_bus_message_read(m, "(ii)", &width_, &height_);
					sd_bus_message_exit_container(m);
				} else if (key && sig && !strcmp(key, "pipewire-serial") && !strcmp(sig, "t")) {
					uint64_t serial;
					sd_bus_message_enter_container(m, 'v', "t"); sd_bus_message_read(m, "t", &serial);
					target_ = std::to_string(serial); sd_bus_message_exit_container(m);
				} else sd_bus_message_skip(m, "v");
				sd_bus_message_exit_container(m);
			}
			sd_bus_message_exit_container(m);
			sd_bus_message_exit_container(m);
		}
		while (sd_bus_message_skip(m, "(ua{sv})") > 0) {}
		sd_bus_message_exit_container(m);
	}
	static int response(sd_bus_message *m, void *data, sd_bus_error *)
	{
		auto &self = *static_cast<PortalDesktop *>(data);
		uint32_t result = 2;
		if (sd_bus_message_read(m, "u", &result) < 0 || result != 0) {
			self.fail("sharing denied or cancelled"); return 0;
		}
		self.request_path_.clear();
		sd_bus_message_enter_container(m, 'a', "{sv}");
		while (sd_bus_message_enter_container(m, 'e', "sv") > 0) {
			const char *key = nullptr, *sig = nullptr, *value = nullptr;
			sd_bus_message_read(m, "s", &key);
			sd_bus_message_peek_type(m, nullptr, &sig);
			if (key && sig && (!strcmp(key, "session_handle") || !strcmp(key, "restore_token")) && !strcmp(sig, "s")) {
				sd_bus_message_enter_container(m, 'v', "s"); sd_bus_message_read(m, "s", &value);
				if (!strcmp(key, "session_handle")) self.session_ = value;
				else { self.token_ = value; self.save_token(); }
				sd_bus_message_exit_container(m);
			} else if (key && sig && !strcmp(key, "devices") && !strcmp(sig, "u")) {
				sd_bus_message_enter_container(m, 'v', "u"); sd_bus_message_read(m, "u", &self.devices_);
				sd_bus_message_exit_container(m);
			} else if (key && sig && !strcmp(key, "streams") && !strcmp(sig, "a(ua{sv})")) {
				sd_bus_message_enter_container(m, 'v', "a(ua{sv})"); self.read_streams(m);
				sd_bus_message_exit_container(m);
			} else sd_bus_message_skip(m, "v");
			sd_bus_message_exit_container(m);
		}
		if (self.stage_ == Stage::Create) {
			if (self.session_.empty()) { self.fail("missing session"); return 0; }
			sd_bus_match_signal(self.bus_, &self.closed_slot_, kService, self.session_.c_str(), kSession, "Closed", closed, &self);
			self.request(self.remote_ ? Stage::Devices : Stage::Sources);
		} else if (self.stage_ == Stage::Devices) self.request(Stage::Sources);
		else if (self.stage_ == Stage::Sources) self.request(Stage::Start);
		else if (self.stage_ == Stage::Start) {
			if (self.target_.empty()) self.fail("no shared monitor");
			else self.request(Stage::Open);
		}
		return 0;
	}
	static int closed(sd_bus_message *, void *data, sd_bus_error *)
	{
		static_cast<PortalDesktop *>(data)->fail("session closed"); return 0;
	}
	void save_token()
	{
		if (token_path_.empty()) return;
		std::error_code error;
		auto parent = std::filesystem::path(token_path_).parent_path();
		std::filesystem::create_directories(parent, error);
		if (error) return;
		chmod(parent.c_str(), 0700);
		std::string temporary = token_path_ + ".tmp";
		int fd = open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC | O_NOFOLLOW, 0600);
		if (fd < 0) return;
		bool ok = write(fd, token_.data(), token_.size()) == ssize_t(token_.size());
		close(fd);
		if (ok) rename(temporary.c_str(), token_path_.c_str());
		else unlink(temporary.c_str());
	}
	sd_bus_message *input_message(const char *method)
	{
		if (stage_ != Stage::Streaming) return nullptr;
		sd_bus_message *m = nullptr;
		if (sd_bus_message_new_method_call(bus_, &m, kService, kPath, kRemote, method) < 0) return nullptr;
		sd_bus_message_append(m, "o", session_.c_str());
		sd_bus_message_open_container(m, 'a', "{sv}"); sd_bus_message_close_container(m);
		return m;
	}
	void send_input(sd_bus_message *m)
	{
		sd_bus_message_set_expect_reply(m, 0); sd_bus_send(bus_, m, nullptr); sd_bus_message_unref(m);
	}

	pw_context *context_;
	DesktopEvents events_;
	sd_bus *bus_ = nullptr;
	sd_bus_slot *response_slot_ = nullptr, *closed_slot_ = nullptr, *call_slot_ = nullptr;
	pw_core *core_ = nullptr;
	Stage stage_ = Stage::Idle;
	bool denied_ = false, remote_ = false, remote_persistence_ = false, cast_persistence_ = false;
	uint32_t cursor_mode_ = 1, node_ = 0, devices_ = 0, sequence_ = 0;
	int width_ = 1920, height_ = 1080;
	std::string owner_, session_, request_path_, target_, token_, token_path_;
};
} // namespace

std::unique_ptr<Desktop>
make_portal_desktop(pw_context *context, const DesktopEvents &events, std::string &error)
{
	auto desktop = std::make_unique<PortalDesktop>(context, events);
	return desktop->connect(error) ? std::move(desktop) : nullptr;
}
} // namespace rd
