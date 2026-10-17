#!/bin/sh
# Install or remove the universal rift-s-linux build for the current user.
#
#   ./install.sh               install into ~/.local/share/rift-s-linux
#   ./install.sh --uninstall   remove it again
#
# Everything lands in the user's home, which SteamVR can also see inside the Steam
# Linux Runtime container. Only the udev rule for headset access needs root.
set -eu

BUILD_PREFIX=/opt/rift-s-linux
here=$(cd "$(dirname "$0")" && pwd)
data=${XDG_DATA_HOME:-$HOME/.local/share}
config=${XDG_CONFIG_HOME:-$HOME/.config}
prefix=$data/rift-s-linux
units=$config/systemd/user
service=rifts-steamvr-registration.service
layer=$data/vulkan/implicit_layer.d/monado_roomsetup_vulkan.json
rule=/etc/udev/rules.d/70-monado-rifts.rules
desktop_files="rifts-steamvr.desktop rifts-desktop.desktop rifts-settings.desktop"

[ "$(id -u)" -ne 0 ] || { echo 'Run as your normal user, not root.' >&2; exit 1; }

as_root() {
	if command -v sudo >/dev/null; then sudo "$@"; else pkexec "$@"; fi
}

uninstall() {
	if [ -x "$prefix/bin/rifts-setup" ]; then
		systemctl --user disable --now "$service" 2>/dev/null || true
		"$prefix/bin/rifts-setup" uninstall || true
	fi
	rm -f "$units/$service" "$layer"
	for file in $desktop_files; do rm -f "$data/applications/$file"; done
	rm -rf "$prefix"
	systemctl --user daemon-reload 2>/dev/null || true
	if [ -e "$rule" ]; then
		as_root rm -f "$rule" && as_root udevadm control --reload-rules || true
	fi
	echo 'rift-s-linux removed.'
}

if [ "${1:-}" = --uninstall ]; then
	uninstall
	exit
fi

if [ -d /usr/share/steamvr-monado ]; then
	echo 'A system-wide rift-s-linux package is installed. Remove it first.' >&2
	exit 1
fi

# Stop the running watcher before replacing its files.
systemctl --user stop "$service" 2>/dev/null || true
rm -rf "$prefix.new"
mkdir -p "$prefix.new"
cp -a "$here/files/." "$prefix.new/"
# Text files carry the build prefix; binaries locate their files at run time.
grep -rlI --null "$BUILD_PREFIX" "$prefix.new" | xargs -0 -r sed -i "s|$BUILD_PREFIX|$prefix|g"
rm -rf "$prefix"
mv "$prefix.new" "$prefix"

mkdir -p "$units" "$(dirname "$layer")" "$data/applications"
cp "$prefix/lib/systemd/user/$service" "$units/$service"
cp "$prefix/share/vulkan/implicit_layer.d/monado_roomsetup_vulkan.json" "$layer"
for file in $desktop_files; do
	if [ -f "$prefix/share/applications/$file" ]; then cp "$prefix/share/applications/$file" "$data/applications/$file"; fi
done

if ! cmp -s "$prefix/share/udev/70-monado-rifts.rules" "$rule" 2>/dev/null; then
	echo 'Installing the udev rule for headset access (needs your password once).'
	as_root install -Dm644 "$prefix/share/udev/70-monado-rifts.rules" "$rule"
	as_root udevadm control --reload-rules
	as_root udevadm trigger --subsystem-match=usb --attr-match=idVendor=2833 || true
	as_root udevadm trigger --subsystem-match=hidraw || true
fi

systemctl --user daemon-reload
systemctl --user enable --now "$service"
echo 'rift-s-linux installed. Reconnect the Rift S, then start SteamVR from Steam.'
