// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
// rifts-desktop: SteamVR dashboard tabs that show and control the Linux desktop.
//
// One dashboard overlay per screen. A screen is captured only while its tab is
// visible. Laser mouse and SteamVR keyboard events are injected as desktop input.
// SteamVR starts it through its app manifest (autolaunch) and stops it with
// VREvent_Quit; a pidfd on vrserver covers a crashed server.
#include "capture.hpp"
#include "desktop.hpp"
#include "gpu.hpp"
#include "vr.hpp"

#include <fcntl.h>
#include <linux/input-event-codes.h>
#include <poll.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/resource.h>
#include <sys/signalfd.h>
#include <sys/syscall.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <set>
#include <vector>

#ifndef RIFTS_DESKTOP_MANIFEST
#define RIFTS_DESKTOP_MANIFEST "/usr/share/rifts-desktop/rifts-desktop.vrmanifest"
#endif

namespace rd {
namespace {

constexpr const char *kAppKey = "lbgos.rifts.desktop";
// Newer OpenVR flags than the bundled 1.16 header: dashboard control bar with a keyboard button.
constexpr auto kIgnoreTextureAlpha = vr::VROverlayFlags(1 << 22);
constexpr auto kEnableControlBar = vr::VROverlayFlags(1 << 23);
constexpr auto kEnableControlBarKeyboard = vr::VROverlayFlags(1 << 24);

using Clock = std::chrono::steady_clock;

struct Options
{
	bool autolaunch = false, portal = false, register_app = false, unregister_app = false, list = false, always = false, stats = false, input_test = false;
	uint32_t fps = 60;
	int seconds = 0;
	std::string output, dump;
};

void
usage()
{
	fputs("usage: rifts-desktop [--fps N] [--output NAME]\n"
	      "       rifts-desktop --register | --unregister   (SteamVR app manifest, autolaunch)\n"
	      "debug: --list, --portal, --always, --stats, --seconds N, --dump-frame FILE.ppm, --input-test NAME (stdin)\n",
	      stderr);
}

bool
parse(int argc, char **argv, Options &o)
{
	for (int i = 1; i < argc; i++) {
		std::string a = argv[i];
		auto value = [&]() -> const char * { return i + 1 < argc ? argv[++i] : nullptr; };
		const char *v = nullptr;
		if (a == "--autolaunch") {
			o.autolaunch = true;
		} else if (a == "--portal") {
			o.portal = true;
		} else if (a == "--register") {
			o.register_app = true;
		} else if (a == "--unregister") {
			o.unregister_app = true;
		} else if (a == "--list") {
			o.list = true;
		} else if (a == "--always") {
			o.always = true;
		} else if (a == "--stats") {
			o.stats = true;
		} else if (a == "--fps" && (v = value())) {
			o.fps = uint32_t(std::max(1, std::min(240, atoi(v))));
		} else if (a == "--seconds" && (v = value())) {
			o.seconds = atoi(v);
		} else if (a == "--output" && (v = value())) {
			o.output = v;
		} else if (a == "--dump-frame" && (v = value())) {
			o.dump = v;
		} else if (a == "--input-test" && (v = value())) {
			o.input_test = true;
			o.output = v;
		} else {
			return false;
		}
	}
	return true;
}

//! SteamVR keyboard text to X keysyms. Latin-1 maps 1:1; other Unicode uses the 0x01000000 range.
uint32_t
keysym_for(uint32_t cp)
{
	switch (cp) {
	case '\b': return 0xff08; // BackSpace
	case '\t': return 0xff09; // Tab
	case '\n':
	case '\r': return 0xff0d; // Return
	case 0x1b: return 0xff1b; // Escape
	case 0x7f: return 0xffff; // Delete
	}
	if ((cp >= 0x20 && cp <= 0x7e) || (cp >= 0xa0 && cp <= 0xff)) {
		return cp;
	}
	return 0x01000000 | cp;
}

std::vector<uint32_t>
utf8_codepoints(const char *s, size_t max)
{
	std::vector<uint32_t> out;
	for (size_t i = 0; i < max && s[i];) {
		auto c = uint8_t(s[i]);
		int len = c < 0x80 ? 1 : (c >> 5) == 6 ? 2 : (c >> 4) == 14 ? 3 : (c >> 3) == 30 ? 4 : 0;
		if (len == 0 || i + size_t(len) > max) {
			break;
		}
		uint32_t cp = len == 1 ? c : c & (0x7f >> len);
		for (int k = 1; k < len; k++) {
			cp = (cp << 6) | (uint8_t(s[i + k]) & 0x3f);
		}
		out.push_back(cp);
		i += size_t(len);
	}
	return out;
}

//! Original white monitor silhouette, antialiased for the dashboard's small icons.
std::vector<uint8_t>
thumbnail_icon()
{
	std::vector<uint8_t> px(64 * 64 * 4, 0);
	auto rounded = [](float x, float y, float x0, float y0, float x1, float y1, float r) {
		float dx = x - std::clamp(x, x0 + r, x1 - r);
		float dy = y - std::clamp(y, y0 + r, y1 - r);
		return dx * dx + dy * dy <= r * r;
	};
	for (int y = 0; y < 64; y++) {
		for (int x = 0; x < 64; x++) {
			unsigned coverage = 0;
			for (int sy = 0; sy < 4; sy++) {
				for (int sx = 0; sx < 4; sx++) {
					float u = x + (sx + 0.5f) / 4, v = y + (sy + 0.5f) / 4;
					bool bezel = rounded(u, v, 8, 11, 56, 43, 3) &&
					             !rounded(u, v, 11, 14, 53, 37, 1);
					bool stand = u >= 29 && u <= 35 && v >= 43 && v <= 49;
					coverage += bezel || stand || rounded(u, v, 22, 48, 42, 52, 1);
				}
			}
			uint8_t *p = &px[size_t(y * 64 + x) * 4];
			p[0] = p[1] = p[2] = 255;
			p[3] = uint8_t((coverage * 255 + 8) / 16);
		}
	}
	return px;
}

bool
write_ppm(const std::string &path, const std::vector<uint8_t> &bgra, uint32_t w, uint32_t h)
{
	FILE *f = fopen(path.c_str(), "wb");
	if (!f) {
		return false;
	}
	fprintf(f, "P6\n%u %u\n255\n", w, h);
	std::vector<uint8_t> row(size_t(w) * 3);
	for (uint32_t y = 0; y < h; y++) {
		for (uint32_t x = 0; x < w; x++) {
			const uint8_t *p = &bgra[(size_t(y) * w + x) * 4];
			row[x * 3] = p[2];
			row[x * 3 + 1] = p[1];
			row[x * 3 + 2] = p[0];
		}
		fwrite(row.data(), 1, row.size(), f);
	}
	return fclose(f) == 0;
}

bool
ensure_registered(const Vr &vr)
{
	bool installed = vr.apps->IsApplicationInstalled(kAppKey);
	auto result = vr.apps->AddApplicationManifest(RIFTS_DESKTOP_MANIFEST, false);
	if (result != vr::VRApplicationError_None) {
		fprintf(stderr, "rifts-desktop: manifest registration: %s\n", vr.apps->GetApplicationsErrorNameFromEnum(result));
		return false;
	}
	// Refresh the installed path, while keeping the user's existing autolaunch choice.
	if (!installed) {
		result = vr.apps->SetApplicationAutoLaunch(kAppKey, true);
		if (result != vr::VRApplicationError_None) return false;
		fprintf(stderr, "rifts-desktop: registered with SteamVR (autolaunch on)\n");
	}
	return true;
}

int
run_registration(bool remove)
{
	Vr vr;
	std::string error;
	if (!vr_connect(vr::VRApplication_Utility, vr, error)) {
		fprintf(stderr, "rifts-desktop: %s\n", error.c_str());
		return 2;
	}
	// Not LaunchDashboardOverlay: SteamVR identifies this process as the app by its path,
	// so the overlay would count as running. Callers start it themselves.
	if (remove) {
		vr.apps->RemoveApplicationManifest(RIFTS_DESKTOP_MANIFEST);
	} else {
		if (!ensure_registered(vr)) { vr_disconnect(); return 1; }
	}
	vr_disconnect();
	return 0;
}

struct Tab
{
	Screen screen;
	vr::VROverlayHandle_t main = vr::k_ulOverlayHandleInvalid;
	vr::VROverlayHandle_t thumb = vr::k_ulOverlayHandleInvalid;
	std::unique_ptr<Capture> capture;
	bool requested = false;
	Clock::time_point retry_at;
	uint32_t width = 0, height = 0; // current texture size
	uint64_t frames = 0, frames_at_stats = 0;
	bool texture_error_logged = false;
	std::set<uint32_t> pressed;
	bool keyboard_received = false;
};

class App
{
public:
	explicit App(const Options &o) : opt_(o) {}

	~App()
	{
		tabs_.clear(); // captures before their PipeWire core
		set_native_desktop_hidden(false);
		vr_disconnect();
		desktop_.reset();
		if (context_) {
			pw_context_destroy(context_);
		}
		if (loop_) {
			pw_loop_leave(loop_);
			pw_loop_destroy(loop_);
		}
		for (int fd : {timer_fd_, signal_fd_, server_fd_}) {
			if (fd >= 0) {
				close(fd);
			}
		}
	}

	int
	run()
	{
		std::string error;
		bool need_vr = !opt_.list && !opt_.input_test;
		sigset_t mask;
		sigemptyset(&mask);
		sigaddset(&mask, SIGINT);
		sigaddset(&mask, SIGTERM);
		if (need_vr) sigprocmask(SIG_BLOCK, &mask, nullptr);
		if (need_vr) {
			if (!vr_connect(vr::VRApplication_Overlay, vr_, error)) {
				fprintf(stderr, "rifts-desktop: %s\n", error.c_str());
				return 2;
			}
			if (!ensure_registered(vr_)) { vr_disconnect(); return 1; }
			if (opt_.autolaunch && !vr_.apps->GetApplicationAutoLaunch(kAppKey)) {
				vr_disconnect();
				return 0;
			}
			// SteamVR passes game-oriented env to launched apps. This copy-only client needs
			// neither Steam's pipeline recorder nor the driver's vrcompositor layer.
			unsetenv("ENABLE_VK_LAYER_VALVE_steam_fossilize_1");
			setenv("MONADO_DISABLE_ROOMSETUP_COMPAT", "1", 1);
			if (!gpu_.init(vr_, error)) {
				fprintf(stderr, "rifts-desktop: %s\n", error.c_str());
				return 1;
			}
			fprintf(stderr, "rifts-desktop: Vulkan ready, %zu importable DMA-BUF modifiers\n",
			        gpu_.import_modifiers().size());
		}

		loop_ = pw_loop_new(nullptr);
		context_ = loop_ ? pw_context_new(loop_, nullptr, 0) : nullptr;
		if (!context_) {
			fprintf(stderr, "rifts-desktop: PipeWire unavailable\n");
			return 1;
		}
		pw_loop_enter(loop_);

		DesktopEvents events;
		events.screens_changed = [this] { screens_dirty_ = true; };
		events.stream_ready = [this](const std::string &s, pw_core *core, const std::string &target) {
			stream_ready(s, core, target);
		};
		events.stream_closed = [this](const std::string &s) {
			if (Tab *t = tab(s)) {
				t->capture.reset();
				t->requested = false;
				t->retry_at = Clock::now() + std::chrono::seconds(2);
			}
		};
		if (!opt_.portal) {
			desktop_ = make_kwin_desktop(context_, events, error);
		}
		if (!desktop_) {
			if (!error.empty()) fprintf(stderr, "rifts-desktop: %s; using the desktop portal\n", error.c_str());
			desktop_ = make_portal_desktop(context_, events, error);
		}
		if (!desktop_) {
			fprintf(stderr, "rifts-desktop: %s\n", error.c_str());
			return 1;
		}

		if (opt_.list) {
			printf("backend %s keyboard %s\n", desktop_->name(), desktop_->has_keyboard() ? "yes" : "no");
			for (const Screen &s : desktop_->screens()) {
				printf("screen %s %dx%d\n", s.name.c_str(), s.width, s.height);
			}
			return 0;
		}
		if (opt_.input_test) {
			return input_test();
		}

		signal_fd_ = signalfd(-1, &mask, SFD_CLOEXEC);
		timer_fd_ = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
		if (signal_fd_ < 0 || timer_fd_ < 0) {
			fprintf(stderr, "rifts-desktop: cannot create event descriptors\n");
			return 1;
		}
		if (int pid = vrserver_pid()) {
			server_fd_ = int(syscall(SYS_pidfd_open, pid, 0));
		}
		update_tabs();
		set_tick(true);
		return loop();
	}

private:
	int
	loop()
	{
		const auto started = Clock::now();
		stats_at_ = started;
		getrusage(RUSAGE_SELF, &stats_usage_);
		while (running_) {
			desktop_->prepare();
			pollfd fds[] = {{desktop_->fd(), POLLIN, 0},
			                {pw_loop_get_fd(loop_), POLLIN, 0},
			                {timer_fd_, POLLIN, 0},
			                {signal_fd_, POLLIN, 0},
			                {server_fd_, POLLIN, 0}};
			int n = poll(fds, server_fd_ >= 0 ? 5 : 4, -1);
			if (!desktop_->dispatch(n > 0 && (fds[0].revents & POLLIN))) {
				fprintf(stderr, "rifts-desktop: lost the desktop connection\n");
				break;
			}
			if (n <= 0) {
				continue;
			}
			if (fds[1].revents) {
				pw_loop_iterate(loop_, 0);
			}
			if (fds[2].revents) {
				uint64_t expirations;
				if (read(timer_fd_, &expirations, sizeof(expirations)) > 0) {
					tick();
				}
			}
			if (fds[3].revents) {
				running_ = false;
			}
			if (server_fd_ >= 0 && fds[4].revents) {
				fprintf(stderr, "rifts-desktop: vrserver exited\n");
				running_ = false;
			}
			if (opt_.seconds > 0 && Clock::now() - started > std::chrono::seconds(opt_.seconds)) {
				running_ = false;
			}
		}
		for (auto &t : tabs_) {
			release_buttons(*t);
			vr_.overlay->DestroyOverlay(t->thumb);
			vr_.overlay->DestroyOverlay(t->main);
		}
		tabs_.clear();
		set_native_desktop_hidden(false);
		vr_disconnect();
		return 0;
	}

	// The timer runs at 60 Hz while a tab is visible (laser input) and at 10 Hz otherwise.
	void
	set_tick(bool fast)
	{
		if (fast == fast_tick_) {
			return;
		}
		fast_tick_ = fast;
		long ns = fast ? 16666667 : 100000000;
		itimerspec spec = {{0, ns}, {0, ns}};
		timerfd_settime(timer_fd_, 0, &spec, nullptr);
	}

	Tab *
	tab(const std::string &screen)
	{
		for (auto &t : tabs_) {
			if (t->screen.name == screen) {
				return t.get();
			}
		}
		return nullptr;
	}

	void
	update_tabs()
	{
		screens_dirty_ = false;
		std::vector<Screen> screens;
		for (const Screen &s : desktop_->screens()) {
			if (opt_.output.empty() || s.name == opt_.output) {
				screens.push_back(s);
			}
		}
		for (auto it = tabs_.begin(); it != tabs_.end();) {
			bool keep = false;
			for (const Screen &s : screens) {
				if (s.name == (*it)->screen.name) {
					(*it)->screen = s;
					keep = true;
				}
			}
			if (!keep) {
				(*it)->capture.reset();
				desktop_->stop((*it)->screen.name);
				release_buttons(**it);
				vr_.overlay->DestroyOverlay((*it)->thumb);
				vr_.overlay->DestroyOverlay((*it)->main);
				it = tabs_.erase(it);
			} else {
				++it;
			}
		}
		static const std::vector<uint8_t> icon = thumbnail_icon();
		for (const Screen &s : screens) {
			if (tab(s.name)) {
				continue;
			}
			auto t = std::make_unique<Tab>();
			t->screen = s;
			std::string key = std::string(kAppKey) + "." + s.name;
			auto e = vr_.overlay->CreateDashboardOverlay(key.c_str(), "Desktop", &t->main, &t->thumb);
			if (e != vr::VROverlayError_None) {
				fprintf(stderr, "rifts-desktop: dashboard overlay %s: %s\n", key.c_str(),
				        vr_.overlay->GetOverlayErrorNameFromEnum(e));
				continue;
			}
			vr_.overlay->SetOverlayFlag(t->main, kIgnoreTextureAlpha, true);
			vr_.overlay->SetOverlayInputMethod(t->main, vr::VROverlayInputMethod_Mouse);
			vr_.overlay->SetOverlayFlag(t->main, vr::VROverlayFlags_SendVRDiscreteScrollEvents, true);
			vr_.overlay->SetOverlayFlag(t->main, kEnableControlBar, true);
			if (desktop_->has_keyboard()) {
				vr_.overlay->SetOverlayFlag(t->main, kEnableControlBarKeyboard, true);
			}
			vr_.overlay->SetOverlayWidthInMeters(t->main, 2.5f);
			vr_.overlay->SetOverlayRaw(t->thumb, const_cast<uint8_t *>(icon.data()), 64, 64, 4);
			fprintf(stderr, "rifts-desktop: tab for %s (%dx%d)\n", s.name.c_str(), s.width, s.height);
			tabs_.push_back(std::move(t));
		}
		// Primary screen first; numbered only when there is more than one.
		for (size_t i = 0; i < tabs_.size(); i++) {
			std::string name = tabs_.size() == 1 ? "Desktop" : "Desktop " + tabs_[i]->screen.name;
			vr_.overlay->SetOverlayName(tabs_[i]->main, name.c_str());
			vr_.overlay->SetOverlayName(tabs_[i]->thumb, name.c_str());
		}
		set_native_desktop_hidden(!tabs_.empty() && !strcmp(desktop_->name(), "kwin"));
	}

	void
	set_native_desktop_hidden(bool hidden)
	{
		if (!vr_.settings || (!hidden && !native_desktop_hidden_)) return;
		// Public SteamVR advanced setting "Show Desktop Views". The SDK has no key constant.
		vr::EVRSettingsError error = vr::VRSettingsError_None;
		bool shown = vr_.settings->GetBool(vr::k_pch_Dashboard_Section, "showDesktop", &error);
		if (error != vr::VRSettingsError_None) return;
		if (shown == hidden) {
			vr_.settings->SetBool(vr::k_pch_Dashboard_Section, "showDesktop", !hidden, &error);
			if (error != vr::VRSettingsError_None) {
				fprintf(stderr, "rifts-desktop: could not change dashboard.showDesktop\n");
				return;
			}
			native_desktop_hidden_ = hidden;
		}
		// Preserve an existing disabled preference; restore only a value we changed.
		if (!hidden) native_desktop_hidden_ = false;
	}

	void
	stream_ready(const std::string &screen, pw_core *core, const std::string &target)
	{
		Tab *t = tab(screen);
		if (!t || !t->requested) {
			return;
		}
		t->capture = std::make_unique<Capture>(core, gpu_, target, opt_.fps, [this, t](GpuImage &image) {
			frame(*t, image);
		});
	}

	void
	frame(Tab &t, GpuImage &image)
	{
		if (image.width != t.width || image.height != t.height) {
			t.width = image.width;
			t.height = image.height;
			vr::HmdVector2_t scale = {{float(t.width), float(t.height)}};
			vr_.overlay->SetOverlayMouseScale(t.main, &scale);
		}
		vr::VRVulkanTextureData_t vk = gpu_.texture(image);
		vr::Texture_t texture = {&vk, vr::TextureType_Vulkan, vr::ColorSpace_Gamma};
		auto e = vr_.overlay->SetOverlayTexture(t.main, &texture);
		if (e != vr::VROverlayError_None && !t.texture_error_logged) {
			fprintf(stderr, "rifts-desktop: SetOverlayTexture: %s\n", vr_.overlay->GetOverlayErrorNameFromEnum(e));
			t.texture_error_logged = true;
		}
		if (e != vr::VROverlayError_None) return;
		t.frames++;
		if (!opt_.dump.empty() && !dumped_) {
			dumped_ = true;
			std::vector<uint8_t> bgra;
			if (gpu_.read_back(image, bgra) && write_ppm(opt_.dump, bgra, image.width, image.height)) {
				uint8_t amin = 255, amax = 0;
				for (size_t i = 3; i < bgra.size(); i += 4) {
					amin = std::min(amin, bgra[i]);
					amax = std::max(amax, bgra[i]);
				}
				fprintf(stderr, "rifts-desktop: dumped %ux%u frame to %s (alpha %u..%u)\n", image.width,
				        image.height, opt_.dump.c_str(), amin, amax);
			}
		}
	}

	void
	tick()
	{
		vr::VREvent_t ev;
		while (vr_.system->PollNextEvent(&ev, sizeof(ev))) {
			if (ev.eventType == vr::VREvent_Quit) {
				fprintf(stderr, "rifts-desktop: SteamVR requested quit\n");
				vr_.system->AcknowledgeQuit_Exiting();
				running_ = false;
			}
		}
		if (screens_dirty_) {
			update_tabs();
		}
		bool any_visible = false;
		const auto now = Clock::now();
		for (auto &tp : tabs_) {
			Tab &t = *tp;
			while (vr_.overlay->PollNextOverlayEvent(t.main, &ev, sizeof(ev))) {
				input(t, ev);
			}
			bool want = opt_.always || vr_.overlay->IsOverlayVisible(t.main);
			any_visible |= want;
			if (t.capture && t.capture->failed()) {
				t.capture.reset();
				desktop_->stop(t.screen.name);
				t.requested = false;
				t.retry_at = now + std::chrono::seconds(2);
			}
			if (want && !t.requested && now >= t.retry_at) {
				t.requested = true;
				desktop_->start(t.screen.name);
			} else if (!want && t.requested) {
				release_buttons(t);
				t.capture.reset();
				desktop_->stop(t.screen.name);
				t.requested = false;
			}
		}
		set_tick(any_visible);
		if (opt_.stats) {
			stats(now);
		}
	}

	void
	release_buttons(Tab &t)
	{
		for (uint32_t code : t.pressed) desktop_->button(code, false);
		t.pressed.clear();
	}

	void
	input(Tab &t, const vr::VREvent_t &ev)
	{
		const std::string &name = t.screen.name;
		switch (ev.eventType) {
		case vr::VREvent_MouseMove:
		case vr::VREvent_MouseButtonDown:
		case vr::VREvent_MouseButtonUp: {
			if (t.width == 0 || t.height == 0) {
				return;
			}
			// OpenVR mouse coordinates are in mouse-scale units with the origin at the bottom left.
			double x = std::clamp(double(ev.data.mouse.x / t.width), 0.0, 1.0) * (t.screen.width - 1);
			double y = std::clamp(1.0 - ev.data.mouse.y / t.height, 0.0, 1.0) * (t.screen.height - 1);
			desktop_->pointer(name, x, y);
			if (ev.eventType != vr::VREvent_MouseMove) {
				uint32_t code = ev.data.mouse.button == vr::VRMouseButton_Right    ? BTN_RIGHT
				                : ev.data.mouse.button == vr::VRMouseButton_Middle ? BTN_MIDDLE
				                                                                   : BTN_LEFT;
				bool pressed = ev.eventType == vr::VREvent_MouseButtonDown;
				if (pressed) t.pressed.insert(code);
				else t.pressed.erase(code);
				desktop_->button(code, pressed);
			}
			break;
		}
		case vr::VREvent_ScrollDiscrete:
			// OpenVR: positive y scrolls up. Desktop: positive is down.
			desktop_->scroll(ev.data.scroll.xdelta, -ev.data.scroll.ydelta);
			break;
		case vr::VREvent_FocusLeave:
		case vr::VREvent_OverlayHidden:
			release_buttons(t);
			break;
		case vr::VREvent_KeyboardCharInput:
			t.keyboard_received = true;
			for (uint32_t cp : utf8_codepoints(ev.data.keyboard.cNewInput, sizeof(ev.data.keyboard.cNewInput))) {
				desktop_->key(keysym_for(cp));
			}
			break;
		case vr::VREvent_KeyboardDone:
			if (!t.keyboard_received) {
				char text[4096] = {};
				vr_.overlay->GetKeyboardText(text, sizeof(text));
				for (uint32_t cp : utf8_codepoints(text, sizeof(text))) desktop_->key(keysym_for(cp));
			}
			t.keyboard_received = false;
			break;
		case vr::VREvent_KeyboardClosed:
			t.keyboard_received = false;
			break;
		default: break;
		}
	}

	void
	stats(Clock::time_point now)
	{
		const double seconds = std::chrono::duration<double>(now - stats_at_).count();
		if (seconds < 10) {
			return;
		}
		rusage usage;
		getrusage(RUSAGE_SELF, &usage);
		auto us = [](const rusage &r) {
			return double(r.ru_utime.tv_sec + r.ru_stime.tv_sec) * 1e6 + r.ru_utime.tv_usec + r.ru_stime.tv_usec;
		};
		double cpu = (us(usage) - us(stats_usage_)) / (seconds * 1e6) * 100.0;
		for (auto &t : tabs_) {
			fprintf(stderr, "rifts-desktop: stats %s %.1f fps %ux%u %s cpu %.1f%%\n", t->screen.name.c_str(),
			        (t->frames - t->frames_at_stats) / seconds, t->width, t->height,
			        t->capture ? (t->capture->dmabuf() ? "dmabuf" : "shm") : "idle", cpu);
			t->frames_at_stats = t->frames;
		}
		stats_at_ = now;
		stats_usage_ = usage;
	}

	//! Debug: reads "move X Y", "down|up left|right|middle", "scroll DY", "type TEXT" from stdin.
	int
	input_test()
	{
		std::string line;
		while (std::getline(std::cin, line)) {
			char word[16] = {}, arg[64] = {};
			double a = 0, b = 0;
			if (sscanf(line.c_str(), "move %lf %lf", &a, &b) == 2) {
				desktop_->pointer(opt_.output, a, b);
			} else if (sscanf(line.c_str(), "%15s %63s", word, arg) == 2 && (!strcmp(word, "down") || !strcmp(word, "up"))) {
				uint32_t code = !strcmp(arg, "right") ? BTN_RIGHT : !strcmp(arg, "middle") ? BTN_MIDDLE : BTN_LEFT;
				desktop_->button(code, !strcmp(word, "down"));
			} else if (sscanf(line.c_str(), "scroll %lf", &a) == 1) {
				desktop_->scroll(0, a);
			} else if (line.rfind("type ", 0) == 0) {
				for (uint32_t cp : utf8_codepoints(line.c_str() + 5, line.size() - 5)) {
					desktop_->key(keysym_for(cp));
				}
			}
			desktop_->prepare();
			desktop_->dispatch(false);
			usleep(50000);
		}
		return 0;
	}

	Options opt_;
	Vr vr_;
	Gpu gpu_;
	pw_loop *loop_ = nullptr;
	pw_context *context_ = nullptr;
	std::unique_ptr<Desktop> desktop_;
	std::vector<std::unique_ptr<Tab>> tabs_;
	int timer_fd_ = -1, signal_fd_ = -1, server_fd_ = -1;
	bool running_ = true, fast_tick_ = false, screens_dirty_ = false, dumped_ = false;
	bool native_desktop_hidden_ = false;
	Clock::time_point stats_at_;
	rusage stats_usage_ = {};
};

} // namespace
} // namespace rd

int
main(int argc, char **argv)
{
	rd::Options opt;
	if (!rd::parse(argc, argv, opt)) {
		rd::usage();
		return 64;
	}
	if (opt.register_app || opt.unregister_app) {
		return rd::run_registration(opt.unregister_app);
	}

	// One instance per user: SteamVR autolaunch and --register may both start us.
	std::string lock = std::string(getenv("XDG_RUNTIME_DIR") ? getenv("XDG_RUNTIME_DIR") : "/tmp") + "/rifts-desktop.lock";
	int lock_fd = open(lock.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
	if (!opt.list && !opt.input_test && (lock_fd < 0 || flock(lock_fd, LOCK_EX | LOCK_NB) != 0)) {
		fprintf(stderr, "rifts-desktop: already running\n");
		return 0;
	}
	pw_init(&argc, &argv);
	int rc = rd::App(opt).run();
	pw_deinit();
	return rc;
}
