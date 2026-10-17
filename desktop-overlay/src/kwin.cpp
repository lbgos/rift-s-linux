// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
// KDE Plasma backend. KWin offers zkde_screencast_unstable_v1, org_kde_kwin_fake_input
// and kde_output_order_v1 only to clients whose executable matches a desktop file
// declaring them in X-KDE-Wayland-Interfaces, so no picker or prompt is involved.
#include "desktop.hpp"

#include "fake-input-client-protocol.h"
#include "kde-output-order-v1-client-protocol.h"
#include "xdg-output-unstable-v1-client-protocol.h"
#include "zkde-screencast-unstable-v1-client-protocol.h"

#include <linux/input-event-codes.h>
#include <wayland-client.h>

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace rd {
namespace {

class KWinDesktop final : public Desktop
{
public:
	struct Output
	{
		KWinDesktop *self;
		uint32_t global;
		wl_output *output = nullptr;
		zxdg_output_v1 *xdg = nullptr;
		std::string name;
		int x = 0, y = 0, width = 0, height = 0;
	};
	struct Stream
	{
		KWinDesktop *self;
		std::string screen;
		zkde_screencast_stream_unstable_v1 *stream = nullptr;
		std::string serial;
	};

	KWinDesktop(pw_context *context, const DesktopEvents &events) : context_(context), events_(events) {}

	~KWinDesktop() override
	{
		for (auto &s : streams_) {
			zkde_screencast_stream_unstable_v1_close(s->stream);
		}
		for (auto &o : outputs_) {
			if (o->xdg) {
				zxdg_output_v1_destroy(o->xdg);
			}
			wl_output_release(o->output);
		}
		if (input_) {
			org_kde_kwin_fake_input_destroy(input_);
		}
		if (order_) {
			kde_output_order_v1_destroy(order_);
		}
		if (screencast_) {
			zkde_screencast_unstable_v1_destroy(screencast_);
		}
		if (xdg_manager_) {
			zxdg_output_manager_v1_destroy(xdg_manager_);
		}
		if (registry_) {
			wl_registry_destroy(registry_);
		}
		if (display_) {
			wl_display_flush(display_);
			wl_display_disconnect(display_);
		}
		if (core_) {
			pw_core_disconnect(core_);
		}
	}

	bool
	connect(std::string &error)
	{
		display_ = wl_display_connect(nullptr);
		if (!display_) {
			error = "cannot connect to the Wayland display";
			return false;
		}
		registry_ = wl_display_get_registry(display_);
		wl_registry_add_listener(registry_, &kRegistry, this);
		if (wl_display_roundtrip(display_) < 0 || wl_display_roundtrip(display_) < 0) {
			error = "Wayland initialization failed";
			return false;
		}
		if (!screencast_) {
			error = "KWin does not offer zkde_screencast_unstable_v1 to this executable";
			return false;
		}
		core_ = pw_context_connect(context_, nullptr, 0);
		if (!core_) {
			error = "cannot connect to PipeWire";
			return false;
		}
		if (!input_ || input_version_ < ORG_KDE_KWIN_FAKE_INPUT_POINTER_MOTION_ABSOLUTE_SINCE_VERSION) {
			fprintf(stderr, "rifts-desktop: KWin fake input unavailable; desktop is view-only\n");
		}
		return true;
	}

	const char *
	name() const override
	{
		return "kwin";
	}

	std::vector<Screen>
	screens() const override
	{
		std::vector<const Output *> ready;
		for (const auto &o : outputs_) {
			if (!o->name.empty() && o->width > 0 && o->height > 0) {
				ready.push_back(o.get());
			}
		}
		// kde_output_order_v1 lists the primary output first; unknown outputs keep registry order.
		auto rank = [&](const Output *o) {
			return size_t(std::find(order_names_.begin(), order_names_.end(), o->name) - order_names_.begin());
		};
		std::stable_sort(ready.begin(), ready.end(),
		                 [&](const Output *a, const Output *b) { return rank(a) < rank(b); });
		std::vector<Screen> out;
		for (const Output *o : ready) {
			out.push_back({o->name, o->width, o->height});
		}
		return out;
	}

	void
	start(const std::string &screen) override
	{
		const Output *o = find(screen);
		if (!o || stream_for(screen)) {
			return;
		}
		auto s = std::make_unique<Stream>(Stream{this, screen});
		s->stream = zkde_screencast_unstable_v1_stream_output(screencast_, o->output,
		                                                      ZKDE_SCREENCAST_UNSTABLE_V1_POINTER_EMBEDDED);
		zkde_screencast_stream_unstable_v1_add_listener(s->stream, &kStream, s.get());
		streams_.push_back(std::move(s));
	}

	void
	stop(const std::string &screen) override
	{
		auto it = std::find_if(streams_.begin(), streams_.end(), [&](auto &s) { return s->screen == screen; });
		if (it != streams_.end()) {
			zkde_screencast_stream_unstable_v1_close((*it)->stream);
			streams_.erase(it);
		}
	}

	void
	pointer(const std::string &screen, double x, double y) override
	{
		const Output *o = find(screen);
		if (o && input_ && input_version_ >= ORG_KDE_KWIN_FAKE_INPUT_POINTER_MOTION_ABSOLUTE_SINCE_VERSION) {
			org_kde_kwin_fake_input_pointer_motion_absolute(input_, wl_fixed_from_double(o->x + std::clamp(x, 0.0, double(o->width - 1))),
			                                                wl_fixed_from_double(o->y + std::clamp(y, 0.0, double(o->height - 1))));
			wl_display_flush(display_);
		}
	}

	void
	button(uint32_t code, bool pressed) override
	{
		if (input_) {
			org_kde_kwin_fake_input_button(input_, code,
			                               pressed ? WL_POINTER_BUTTON_STATE_PRESSED : WL_POINTER_BUTTON_STATE_RELEASED);
			wl_display_flush(display_);
		}
	}

	void
	scroll(double dx, double dy) override
	{
		if (!input_) {
			return;
		}
		// KWin treats axis values like libinput wheel deltas: 15 per notch.
		if (dy != 0) {
			org_kde_kwin_fake_input_axis(input_, WL_POINTER_AXIS_VERTICAL_SCROLL, wl_fixed_from_double(dy * 15));
		}
		if (dx != 0) {
			org_kde_kwin_fake_input_axis(input_, WL_POINTER_AXIS_HORIZONTAL_SCROLL, wl_fixed_from_double(dx * 15));
		}
		wl_display_flush(display_);
	}

	void
	key(uint32_t keysym) override
	{
		if (has_keyboard()) {
			org_kde_kwin_fake_input_keyboard_keysym(input_, keysym, WL_KEYBOARD_KEY_STATE_PRESSED);
			org_kde_kwin_fake_input_keyboard_keysym(input_, keysym, WL_KEYBOARD_KEY_STATE_RELEASED);
			wl_display_flush(display_);
		}
	}

	bool
	has_keyboard() const override
	{
		return input_ && input_version_ >= ORG_KDE_KWIN_FAKE_INPUT_KEYBOARD_KEYSYM_SINCE_VERSION;
	}

	int
	fd() const override
	{
		return wl_display_get_fd(display_);
	}

	void
	prepare() override
	{
		while (wl_display_prepare_read(display_) != 0) {
			if (wl_display_dispatch_pending(display_) < 0) return;
		}
		wl_display_flush(display_);
	}

	bool
	dispatch(bool readable) override
	{
		if (readable) {
			if (wl_display_read_events(display_) < 0) {
				return false;
			}
		} else {
			wl_display_cancel_read(display_);
		}
		return wl_display_dispatch_pending(display_) >= 0;
	}

private:
	const Output *
	find(const std::string &name) const
	{
		for (const auto &o : outputs_) {
			if (o->name == name) {
				return o.get();
			}
		}
		return nullptr;
	}

	Stream *
	stream_for(const std::string &screen)
	{
		for (auto &s : streams_) {
			if (s->screen == screen) {
				return s.get();
			}
		}
		return nullptr;
	}

	void
	drop_stream(Stream *stream, const char *reason)
	{
		std::string screen = stream->screen;
		fprintf(stderr, "rifts-desktop: %s stream %s\n", screen.c_str(), reason);
		zkde_screencast_stream_unstable_v1_close(stream->stream);
		streams_.erase(std::find_if(streams_.begin(), streams_.end(), [&](auto &s) { return s.get() == stream; }));
		if (events_.stream_closed) {
			events_.stream_closed(screen);
		}
	}

	void
	add_xdg(Output *o)
	{
		if (xdg_manager_ && !o->xdg) {
			o->xdg = zxdg_output_manager_v1_get_xdg_output(xdg_manager_, o->output);
			zxdg_output_v1_add_listener(o->xdg, &kXdgOutput, o);
		}
	}

	static void
	global(void *data, wl_registry *registry, uint32_t id, const char *interface, uint32_t version)
	{
		auto *self = static_cast<KWinDesktop *>(data);
		if (strcmp(interface, wl_output_interface.name) == 0 && version >= 4) {
			auto o = std::make_unique<Output>(Output{self, id});
			o->output = static_cast<wl_output *>(wl_registry_bind(registry, id, &wl_output_interface, kOutputVersion));
			wl_output_add_listener(o->output, &kOutput, o.get());
			self->add_xdg(o.get());
			self->outputs_.push_back(std::move(o));
		} else if (strcmp(interface, zxdg_output_manager_v1_interface.name) == 0) {
			self->xdg_manager_ = static_cast<zxdg_output_manager_v1 *>(
			    wl_registry_bind(registry, id, &zxdg_output_manager_v1_interface, std::min(version, 3u)));
			for (auto &o : self->outputs_) {
				self->add_xdg(o.get());
			}
		} else if (strcmp(interface, zkde_screencast_unstable_v1_interface.name) == 0) {
			self->screencast_ = static_cast<zkde_screencast_unstable_v1 *>(wl_registry_bind(
			    registry, id, &zkde_screencast_unstable_v1_interface,
			    std::min<uint32_t>(version, zkde_screencast_unstable_v1_interface.version)));
		} else if (strcmp(interface, org_kde_kwin_fake_input_interface.name) == 0) {
			self->input_version_ = std::min<uint32_t>(version, org_kde_kwin_fake_input_interface.version);
			self->input_ = static_cast<org_kde_kwin_fake_input *>(
			    wl_registry_bind(registry, id, &org_kde_kwin_fake_input_interface, self->input_version_));
			org_kde_kwin_fake_input_authenticate(self->input_, "Rift S Desktop",
			                                     "Use the desktop from the SteamVR dashboard");
		} else if (strcmp(interface, kde_output_order_v1_interface.name) == 0) {
			self->order_ = static_cast<kde_output_order_v1 *>(
			    wl_registry_bind(registry, id, &kde_output_order_v1_interface, 1));
			kde_output_order_v1_add_listener(self->order_, &kOrder, self);
		}
	}

	static void
	global_remove(void *data, wl_registry *, uint32_t id)
	{
		auto *self = static_cast<KWinDesktop *>(data);
		auto it = std::find_if(self->outputs_.begin(), self->outputs_.end(),
		                       [&](auto &o) { return o->global == id; });
		if (it == self->outputs_.end()) {
			return;
		}
		if (Stream *s = self->stream_for((*it)->name)) {
			self->drop_stream(s, "output removed");
		}
		if ((*it)->xdg) {
			zxdg_output_v1_destroy((*it)->xdg);
		}
		wl_output_release((*it)->output);
		self->outputs_.erase(it);
		if (self->events_.screens_changed) {
			self->events_.screens_changed();
		}
	}

	static void
	output_name(void *data, wl_output *, const char *name)
	{
		static_cast<Output *>(data)->name = name;
	}

	static void
	output_done(void *data, wl_output *)
	{
		auto *self = static_cast<Output *>(data)->self;
		if (self->events_.screens_changed) {
			self->events_.screens_changed();
		}
	}

	static void
	xdg_position(void *data, zxdg_output_v1 *, int32_t x, int32_t y)
	{
		auto *o = static_cast<Output *>(data);
		o->x = x;
		o->y = y;
	}

	static void
	xdg_size(void *data, zxdg_output_v1 *, int32_t width, int32_t height)
	{
		auto *o = static_cast<Output *>(data);
		o->width = width;
		o->height = height;
	}

	static void
	order_output(void *data, kde_output_order_v1 *, const char *name)
	{
		static_cast<KWinDesktop *>(data)->pending_order_.push_back(name);
	}

	static void
	order_done(void *data, kde_output_order_v1 *)
	{
		auto *self = static_cast<KWinDesktop *>(data);
		self->order_names_.swap(self->pending_order_);
		self->pending_order_.clear();
		if (self->events_.screens_changed) {
			self->events_.screens_changed();
		}
	}

	static void
	stream_closed(void *data, zkde_screencast_stream_unstable_v1 *)
	{
		auto *s = static_cast<Stream *>(data);
		s->self->drop_stream(s, "closed by KWin");
	}

	static void
	stream_created(void *data, zkde_screencast_stream_unstable_v1 *, uint32_t node)
	{
		auto *s = static_cast<Stream *>(data);
		// Prefer the object serial (v6+): node ids can be reused.
		std::string target = s->serial.empty() ? std::to_string(node) : s->serial;
		if (s->self->events_.stream_ready) {
			s->self->events_.stream_ready(s->screen, s->self->core_, target);
		}
	}

	static void
	stream_failed(void *data, zkde_screencast_stream_unstable_v1 *, const char *error)
	{
		auto *s = static_cast<Stream *>(data);
		fprintf(stderr, "rifts-desktop: KWin screencast failed: %s\n", error);
		s->self->drop_stream(s, "failed");
	}

	static void
	stream_serial(void *data, zkde_screencast_stream_unstable_v1 *, uint32_t hi, uint32_t lo)
	{
		static_cast<Stream *>(data)->serial = std::to_string((uint64_t(hi) << 32) | lo);
	}

	static constexpr wl_registry_listener kRegistry = {global, global_remove};
	static constexpr wl_output_listener kOutput = {
	    .geometry = [](void *, wl_output *, int32_t, int32_t, int32_t, int32_t, int32_t, const char *, const char *,
	                   int32_t) {},
	    .mode = [](void *, wl_output *, uint32_t, int32_t, int32_t, int32_t) {},
	    .done = output_done,
	    .scale = [](void *, wl_output *, int32_t) {},
#ifdef WL_OUTPUT_NAME_SINCE_VERSION
	    .name = output_name,
	    .description = [](void *, wl_output *, const char *) {},
#endif
	};
	// Older libwayland headers (the Steam Runtime SDK) stop at version 3; xdg-output
	// then supplies the same connector name.
#ifdef WL_OUTPUT_NAME_SINCE_VERSION
	static constexpr uint32_t kOutputVersion = 4;
#else
	static constexpr uint32_t kOutputVersion = 3;
#endif
	static constexpr zxdg_output_v1_listener kXdgOutput = {
	    .logical_position = xdg_position,
	    .logical_size = xdg_size,
	    .done = [](void *, zxdg_output_v1 *) {},
	    .name = [](void *data, zxdg_output_v1 *, const char *name) {
		    auto *o = static_cast<Output *>(data);
		    if (o->name.empty()) o->name = name;
	    },
	    .description = [](void *, zxdg_output_v1 *, const char *) {},
	};
	static constexpr kde_output_order_v1_listener kOrder = {order_output, order_done};
	static constexpr zkde_screencast_stream_unstable_v1_listener kStream = {
	    .closed = stream_closed,
	    .created = stream_created,
	    .failed = stream_failed,
	    .serial = stream_serial,
	};

	pw_context *context_;
	DesktopEvents events_;
	pw_core *core_ = nullptr;
	wl_display *display_ = nullptr;
	wl_registry *registry_ = nullptr;
	zxdg_output_manager_v1 *xdg_manager_ = nullptr;
	zkde_screencast_unstable_v1 *screencast_ = nullptr;
	org_kde_kwin_fake_input *input_ = nullptr;
	uint32_t input_version_ = 0;
	kde_output_order_v1 *order_ = nullptr;
	std::vector<std::string> order_names_, pending_order_;
	std::vector<std::unique_ptr<Output>> outputs_;
	std::vector<std::unique_ptr<Stream>> streams_;
};

} // namespace

std::unique_ptr<Desktop>
make_kwin_desktop(pw_context *context, const DesktopEvents &events, std::string &error)
{
	auto desktop = std::make_unique<KWinDesktop>(context, events);
	if (!desktop->connect(error)) {
		return nullptr;
	}
	return desktop;
}

} // namespace rd
