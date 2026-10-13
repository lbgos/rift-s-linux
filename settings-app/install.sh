#!/usr/bin/env bash
# Install a built Settings release and refresh the current user's launcher.
set -euo pipefail

settings_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
binary=${1:-$settings_dir/src-tauri/target/release/rifts-settings}
if (( $# > 1 )) || [[ ! -x $binary ]]; then
    echo "Usage: $0 [path/to/release/rifts-settings]" >&2
    exit 1
fi
if (( EUID == 0 )); then
    echo "Run as the desktop user; sudo is used only for system files." >&2
    exit 1
fi

desktop=$settings_dir/rifts-settings.desktop
desktop-file-validate "$desktop"
sudo install -Dm755 "$binary" /usr/local/bin/rifts-settings
sudo install -Dm644 "$settings_dir/src-tauri/icons/icon.png" \
    /usr/local/share/icons/hicolor/128x128/apps/rifts-settings.png
sudo install -Dm644 "$desktop" /usr/local/share/applications/rifts-settings.desktop

# A per-user entry shadows the system entry, including older development builds.
install -Dm644 "$desktop" \
    "${XDG_DATA_HOME:-$HOME/.local/share}/applications/rifts-settings.desktop"
if command -v qdbus6 >/dev/null && qdbus6 org.kde.KWin /KWin >/dev/null 2>&1; then
    bash "$settings_dir/install-vrview-rule.sh"
fi
if command -v kbuildsycoca6 >/dev/null; then
    kbuildsycoca6 --noincremental >/dev/null
fi
