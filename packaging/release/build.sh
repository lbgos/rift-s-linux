#!/usr/bin/env bash
# Build every release asset of the committed HEAD into dist/.
#
#   packaging/release/build.sh [version]
#
# version defaults to `git describe --tags --match 'r[0-9]*'`: r38 on a release tag, r37-4-g1a2b3c4d5 after one.
#
# Assets, as published since r33:
#   basalt-monado-rifts-*.pkg.tar.zst      Arch, packaging/aur/basalt-monado-rifts
#   rift-s-linux-git-*.pkg.tar.zst         Arch, packaging/aur/rift-s-linux-git at this commit
#   rift-s-linux-<version>-x86_64.tar.zst  /usr payload: rift-s-linux-git plus Basalt's library and licenses
#   rift-s-linux-bin-*.pkg.tar.zst         Arch, packaging/aur/rift-s-linux-bin repacking the payload
#   rift-s-linux-<version>-x86_64.tar.xz   universal build, packaging/universal
#   rift-s-linux-bin-x86_64.pkg.tar.zst, rift-s-linux-x86_64.tar.xz
#                                          the same two files under the names the README links through releases/latest
#   SHA256SUMS
# Versioned names stay: rifts-setup update and the AUR recipe look them up.
#
# Compilers and libraries come from three roots downloaded per run and entered with bubblewrap:
#   arch    Arch Linux bootstrap: the Arch packages and the payload
#   sniper  Steam Runtime sniper SDK: driver, Basalt and bundled libraries of the universal build
#   jammy   Ubuntu 22.04: Settings of the universal build (WebKitGTK 4.1, glibc 2.35) and its packaging
# Package managers run as root of a user namespace that maps only the calling user, so the host needs no
# root, only: bash, git, curl, tar, gzip, xz, zstd, bwrap, sha256sum, unprivileged user namespaces, network
# access and about 20 GB free.
#
# Environment:
#   RELEASE_WORK   scratch directory, default build-release/ in the checkout; removed on exit unless KEEP_WORK=1
#   RELEASE_CACHE  checksum-pinned downloads kept between runs, default $RELEASE_WORK/cache
#   DIST           output directory, default dist/ in the checkout; emptied first
#   JOBS           parallel compile jobs, default 4; in CI it comes from the runner's environment
set -euo pipefail

SNIPER_VERSION=3.0.20261021.266025
SNIPER_SHA256=8f71947035f97925a311b2ed753f22efcf2f13d6f1631945b8e537f8903ce2cf
JAMMY_SHA256=242cd8898b33ea806ef5f13b1076ed7c76f9f989d18384452f7166692438ff1a
NODE_VERSION=22.23.5
PNPM_VERSION=10.34.9

here=$(cd "$(dirname "$0")" && pwd)
repo=$(git -C "$here" rev-parse --show-toplevel)
commit=$(git -C "$repo" rev-parse HEAD)
version=${1:-$(git -C "$repo" describe --tags --match 'r[0-9]*')}
if [[ ! $version =~ ^r[0-9]+(-[0-9]+-g[0-9a-f]+)?$ ]]; then
	echo "version must look like r38 or r37-4-g1a2b3c4d5, got: $version" >&2
	exit 2
fi
binver=${version#r}
binver=${binver//-/.}
work=${RELEASE_WORK:-$repo/build-release}
cache=${RELEASE_CACHE:-$work/cache}
dist=${DIST:-$repo/dist}
jobs=${JOBS:-4}
roots=$work/roots
universal=$work/universal

step() { printf '== %s %s\n' "$(date +%T)" "$*"; }

# Remove trees that contain read-only directories from the roots.
wipe() {
	chmod -R u+rwX "$@" 2>/dev/null || true
	rm -rf "$@"
}

# fetch <url> <file> [sha256]: download <url> to <file> unless it exists, then check the digest.
# A file with the wrong digest is deleted and fails the build.
fetch() {
	local url=$1 file=$2 sum=${3:-}
	if [[ ! -f $file ]]; then
		curl -fsSL --retry 5 --retry-all-errors -o "$file.part" "$url"
		mv "$file.part" "$file"
	fi
	if [[ -n $sum ]] && ! echo "$sum  $file" | sha256sum -c --quiet -; then
		rm -f "$file"
		return 1
	fi
}

# unpack <archive> <root> <sentinel> [tar options]: extract a root filesystem owned by the calling user.
unpack() {
	local archive=$1 root=$2 sentinel=$3
	shift 3
	mkdir -p "$root"
	# The Arch bootstrap stores files after their read-only directory (ca-certificates' cadir).
	tar -xf "$archive" -C "$root" --no-same-owner --delay-directory-restore "$@"
	test -e "$root/$sentinel"
	rm -f "$root/etc/resolv.conf"
	cp -L /etc/resolv.conf "$root/etc/resolv.conf"
}

# enter <root> <uid> <dir> [bwrap options] -- <command>: run a command in a root with <dir> at /work.
# uid 0 is the namespace root used for package managers; 1000 is the builder that makepkg requires.
enter() {
	local root=$1 uid=$2 dir=$3 opts=()
	shift 3
	while [[ $1 != -- ]]; do
		opts+=("$1")
		shift
	done
	shift
	bwrap --unshare-user --uid "$uid" --gid "$uid" --unshare-pid --die-with-parent \
		--bind "$root" / --dev /dev --proc /proc --tmpfs /tmp --bind "$dir" /work \
		--clearenv --setenv HOME /work/home --setenv LANG C.UTF-8 \
		--setenv PATH /opt/node/bin:/opt/cargo/bin:/usr/local/bin:/usr/bin:/bin \
		--setenv VERSION "$version" --setenv BINVER "$binver" \
		--setenv J "$jobs" --setenv BJ 2 --setenv MAKEFLAGS "-j$jobs" \
		"${opts[@]}" --chdir /work "$@"
}

problems=()
for tool in git curl tar gzip xz zstd bwrap sha256sum; do
	command -v "$tool" >/dev/null || problems+=("$tool is missing")
done
mkdir -p "$work"
wipe "$roots" "$work/arch" "$universal"
free_gb=$(df -Pk "$work" | awk 'NR == 2 { print int($4 / 1048576) }')
((free_gb >= 20)) || problems+=("$work has $free_gb GB free, the build needs about 20 GB")
if ((${#problems[@]})); then
	printf 'cannot build: %s\n' "${problems[@]}" >&2
	exit 1
fi
mkdir -p "$cache" "$roots" "$work/arch/home" "$universal/src" "$universal/home"
trap '[[ ${KEEP_WORK:-0} == 1 ]] || wipe "$work"' EXIT
[[ -z $(git -C "$repo" status --porcelain) ]] || echo "Uncommitted changes are not built; building $commit." >&2

step "$version at $commit"
# Every stage builds from this clone; branch release lets makepkg fetch the commit from it.
git clone -q --no-checkout "$repo" "$universal/src/monado"
git -C "$universal/src/monado" checkout -q -B release "$commit"
src=$universal/src/monado

step roots
fetch https://geo.mirror.pkgbuild.com/iso/latest/archlinux-bootstrap-x86_64.tar.zst "$roots/arch.tar.zst"
unpack "$roots/arch.tar.zst" "$roots/arch" usr/bin/pacman --zstd --strip-components=1
fetch "https://repo.steampowered.com/steamrt-images-sniper/snapshots/$SNIPER_VERSION/com.valvesoftware.SteamRuntime.Sdk-amd64,i386-sniper-sysroot.tar.gz" \
	"$cache/sniper-$SNIPER_VERSION.tar.gz" "$SNIPER_SHA256"
unpack "$cache/sniper-$SNIPER_VERSION.tar.gz" "$roots/sniper" usr/share/cmake-3.25/Modules/CMake.cmake -z
fetch https://cdimage.ubuntu.com/ubuntu-base/releases/22.04/release/ubuntu-base-22.04.5-base-amd64.tar.gz \
	"$cache/ubuntu-base-22.04.5.tar.gz" "$JAMMY_SHA256"
unpack "$cache/ubuntu-base-22.04.5.tar.gz" "$roots/jammy" usr/bin/apt-get -z

step "arch packages"
in_arch() { enter "$roots/arch" "$1" "$work/arch" --ro-bind "$src" /repo -- bash /repo/packaging/release/arch.sh "${@:2}"; }
in_arch 0 setup
in_arch 1000 package basalt-monado-rifts
in_arch 0 install_package basalt-monado-rifts
in_arch 1000 package rift-s-linux-git
in_arch 1000 payload

step "universal sources"
mv "$work/arch/hostdeps" "$universal/hostdeps"
cp -a "$work/arch/basalt-monado-rifts/src/basalt" "$universal/src/basalt"
while read -r sum name url; do
	fetch "$url" "$cache/$name" "$sum"
	tar -xzf "$cache/$name" -C "$universal/src"
done <<'EOF'
bc23066d87ab3168f27cef3e97d545fa63314f5c79df5ea444d41d56f962c6af fmt-11.2.0.tar.gz https://github.com/fmtlib/fmt/archive/refs/tags/11.2.0.tar.gz
f0f78001c8c8edb4bddc3d4c5ee7428d56ae313254158ad1eec49eced57f6a5b oneTBB-2022.2.0.tar.gz https://github.com/uxlfoundation/oneTBB/archive/refs/tags/v2022.2.0.tar.gz
44c106d5bb47efec04e531fd93008b3fcd1d27138985c5baf4eafac0e1ec9e9d opencv-4.12.0.tar.gz https://github.com/opencv/opencv/archive/refs/tags/4.12.0.tar.gz
EOF

step "universal sniper build"
enter "$roots/sniper" 1000 "$universal" -- sh /work/src/monado/packaging/universal/deps.sh
enter "$roots/sniper" 1000 "$universal" -- sh /work/src/monado/packaging/universal/build.sh

step "universal settings and tarball"
jammy=(--setenv RUSTUP_HOME /opt/rustup --setenv CARGO_HOME /opt/cargo)
enter "$roots/jammy" 0 "$universal" "${jammy[@]}" --setenv NODE_VERSION "$NODE_VERSION" \
	--setenv PNPM_VERSION "$PNPM_VERSION" -- bash /work/src/monado/packaging/release/jammy-setup.sh
enter "$roots/jammy" 1000 "$universal" "${jammy[@]}" -- sh /work/src/monado/packaging/universal/settings.sh
enter "$roots/jammy" 1000 "$universal" "${jammy[@]}" -- sh /work/src/monado/packaging/universal/package.sh /work "$version"

step "dist"
wipe "$dist"
mkdir -p "$dist"
cp "$work"/arch/out/*.pkg.tar.zst "$work/arch/out/rift-s-linux-$version-x86_64.tar.zst" \
	"$universal/dist/rift-s-linux-$version-x86_64.tar.xz" "$dist/"
cp "$dist/rift-s-linux-bin-$binver-1-x86_64.pkg.tar.zst" "$dist/rift-s-linux-bin-x86_64.pkg.tar.zst"
cp "$dist/rift-s-linux-$version-x86_64.tar.xz" "$dist/rift-s-linux-x86_64.tar.xz"
enter "$roots/arch" 1000 "$work/arch" --ro-bind "$src" /repo --ro-bind "$dist" /work/dist -- \
	python /repo/packaging/release/verify.py /work/dist "$version" "$repo" "$work"
(cd "$dist" && LC_ALL=C sha256sum -- * >"$work/SHA256SUMS" && mv "$work/SHA256SUMS" SHA256SUMS)
ls -l "$dist"
step finished
