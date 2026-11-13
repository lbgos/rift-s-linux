#!/bin/bash
# Prepare the Ubuntu 22.04 root of packaging/release/build.sh for packaging/universal/settings.sh and
# package.sh (packaging/universal/README.md lists the same tools). Runs as root of a user namespace that
# maps one user, so apt keeps that user instead of switching to _apt.
set -euo pipefail
cat >/etc/apt/apt.conf.d/90release <<'EOF'
APT::Sandbox::User "root";
APT::Install-Recommends "false";
EOF
cat >/etc/apt/sources.list <<'EOF'
deb http://archive.ubuntu.com/ubuntu jammy main universe
deb http://archive.ubuntu.com/ubuntu jammy-updates main universe
deb http://security.ubuntu.com/ubuntu jammy-security main universe
EOF
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y build-essential pkg-config libwebkit2gtk-4.1-dev libgtk-3-dev libsoup-3.0-dev \
	libvulkan-dev libwayland-dev glslang-tools python3 ca-certificates curl xz-utils file patchelf
curl -fsSL "https://nodejs.org/dist/v$NODE_VERSION/node-v$NODE_VERSION-linux-x64.tar.xz" | tar -xJ -C /opt
ln -sfn "node-v$NODE_VERSION-linux-x64" /opt/node
npm install -g "pnpm@$PNPM_VERSION"
curl -fsSL https://sh.rustup.rs | sh -s -- -y --profile minimal --no-modify-path
install -m755 /work/src/monado/packaging/release/glslc /usr/local/bin/glslc
