// Copyright 2026, lbgos
// SPDX-License-Identifier: BSL-1.0
// Desktop backends: they list screens, open PipeWire streams for them and inject
// pointer and keyboard input. KWin uses its privileged protocols without any
// prompt; other desktops use the xdg-desktop-portal RemoteDesktop session.
#pragma once

#include <pipewire/pipewire.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace rd {

struct Screen
{
	std::string name; // stable key, e.g. "HDMI-A-1" or a portal stream id
	int width = 0, height = 0; // logical size; pointer coordinates use this space
};

struct DesktopEvents
{
	std::function<void()> screens_changed;
	//! A stream for `screen` is ready: connect to `target_object` on `core`.
	std::function<void(const std::string &screen, pw_core *core, const std::string &target_object)> stream_ready;
	std::function<void(const std::string &screen)> stream_closed;
};

class Desktop
{
public:
	virtual ~Desktop() = default;

	virtual const char *
	name() const = 0;
	//! Primary screen first.
	virtual std::vector<Screen>
	screens() const = 0;
	virtual void
	start(const std::string &screen) = 0;
	virtual void
	stop(const std::string &screen) = 0;

	//! Screen-local logical coordinates.
	virtual void
	pointer(const std::string &screen, double x, double y) = 0;
	//! Linux evdev button code (BTN_LEFT...).
	virtual void
	button(uint32_t code, bool pressed) = 0;
	//! Wheel notches; positive is down / right.
	virtual void
	scroll(double dx, double dy) = 0;
	//! Press and release an X keysym.
	virtual void
	key(uint32_t keysym) = 0;
	virtual bool
	has_keyboard() const = 0;

	//! Main loop integration.
	virtual int
	fd() const = 0;
	virtual void
	prepare() = 0;
	virtual bool
	dispatch(bool readable) = 0; // false: connection lost
};

//! Null if KWin's screencast protocol is not offered to this process.
std::unique_ptr<Desktop>
make_kwin_desktop(pw_context *context, const DesktopEvents &events, std::string &error);

//! xdg-desktop-portal RemoteDesktop + ScreenCast with a persisted restore token.
std::unique_ptr<Desktop>
make_portal_desktop(pw_context *context, const DesktopEvents &events, std::string &error);

} // namespace rd
