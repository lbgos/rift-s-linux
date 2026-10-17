#!/bin/sh
# Build the Settings app and add it to /work/stage. Runs in an Ubuntu 22.04 root
# (webkit2gtk 4.1, glibc 2.35) after build.sh; needs cargo, pnpm and glslc.
set -eu
app=/work/src/monado/settings-app
aur=/work/src/monado/packaging/aur/rift-s-linux-git
dest=/work/stage/opt/rift-s-linux
licenses=$dest/share/licenses/rift-s-linux
meta=/work/build/settings
mkdir -p "$meta"

cd "$app"
pnpm install --frozen-lockfile
pnpm build
# custom-protocol embeds the built frontend instead of using the dev server.
cargo build --manifest-path src-tauri/Cargo.toml --release --locked --features custom-protocol -j${J:-8}
cargo test --manifest-path src-tauri/Cargo.toml --release --locked --features custom-protocol -j${J:-8}
cargo metadata --manifest-path src-tauri/Cargo.toml --locked --features custom-protocol --format-version 1 \
	--filter-platform x86_64-unknown-linux-gnu > "$meta/rust.json"
pnpm licenses list --prod --json > "$meta/js.json"

install -Dm755 src-tauri/target/release/rifts-settings "$dest/bin/rifts-settings"
install -Dm644 rifts-settings.desktop "$dest/share/applications/rifts-settings.desktop"
sed -i -e "s|^Exec=.*|Exec=/opt/rift-s-linux/bin/rifts-settings|" \
	-e "s|^Icon=.*|Icon=/opt/rift-s-linux/share/icons/hicolor/128x128/apps/rifts-settings.png|" \
	"$dest/share/applications/rifts-settings.desktop"
install -Dm644 src-tauri/icons/icon.png "$dest/share/icons/hicolor/128x128/apps/rifts-settings.png"
install -Dm755 install-vrview-rule.sh "$dest/share/rifts-settings/install-vrview-rule.sh"
install -Dm644 vrview.kwinrule "$dest/share/rifts-settings/vrview.kwinrule"

install -Dm644 LICENSE "$licenses/Settings-MIT"
install -Dm644 src/fonts/OFL.txt "$licenses/OpenSans-OFL"
install -Dm644 src-tauri/native/OpenVR-LICENSE "$licenses/Settings-OpenVR"
python3 "$aur/settings-licenses.py" "$meta/rust.json" "$meta/js.json" "$aur/settings-license-fallbacks.json" \
	> "$licenses/Settings-third-party-notices.txt"
echo settings-ok
