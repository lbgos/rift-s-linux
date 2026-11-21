#!/bin/bash
# Arch Linux stage of packaging/release/build.sh, run inside the Arch bootstrap root.
#
#   arch.sh setup                     namespace root: keyring, build dependencies, header dependencies of the universal build
#   arch.sh package <recipe>          builder: makepkg packaging/aur/<recipe>
#   arch.sh install_package <name>    namespace root: install a package built by `package`
#   arch.sh payload                   builder: the /usr payload tarball and rift-s-linux-bin
#
# /repo is the clone of the commit being released (branch release), /work the scratch directory,
# /work/out receives the packages. VERSION (r38) and BINVER (38) come from build.sh.
set -euo pipefail
out=/work/out
# The namespace maps no uid 0, so a real chown to root fails with EINVAL, which fakeroot passes on
# (`cp -a` in rift-s-linux-bin). Ownership in the packages comes from fakeroot either way.
export FAKEROOTDONTTRYCHOWN=1

setup() {
	# The namespace maps one user: pacman cannot switch to its download user, and files that
	# packages assign to other users stay with root (pacman warns and continues).
	sed -i -e 's/^DownloadUser/#&/' -e 's/^CheckSpace/#&/' /etc/pacman.conf
	# shellcheck disable=SC2016 # pacman expands $repo and $arch
	echo 'Server = https://geo.mirror.pkgbuild.com/$repo/os/$arch' >/etc/pacman.d/mirrorlist
	pacman-key --init
	pacman-key --populate archlinux
	pacman -Sy --noconfirm archlinux-keyring
	local recipe want=(base-devel git)
	for recipe in basalt-monado-rifts rift-s-linux-git; do
		mapfile -t -O "${#want[@]}" want < <(
			# shellcheck source=/dev/null
			source "/repo/packaging/aur/$recipe/PKGBUILD"
			# shellcheck disable=SC2154 # the PKGBUILD defines depends and makedepends
			printf '%s\n' "${depends[@]}" "${makedepends[@]}" | grep -vx basalt-monado-rifts
		)
	done
	pacman -Su --noconfirm --needed "${want[@]}"
	echo 'builder:x:1000:1000::/work/home:/bin/bash' >>/etc/passwd
	echo 'builder:x:1000:' >>/etc/group
	mkdir -p "$out" /work/srcdest

	# Header-only dependencies of the universal build (packaging/universal/README.md).
	local file
	mkdir -p /work/hostdeps
	for file in $(pacman -Sp --print-format %f boost eigen vulkan-headers plasma-wayland-protocols); do
		bsdtar -xf "/var/cache/pacman/pkg/$file" -C /work/hostdeps usr
	done
	# Part of build.sh's cache key for the sniper prefix, which compiles Basalt against these headers.
	pacman -Q boost eigen >/work/hostdeps/versions
	# build.sh keeps the package cache between runs; keep only the installed versions in it.
	pacman -Sc --noconfirm
}

package() {
	local recipe=$1
	rm -rf "/work/$recipe"
	cp -r "/repo/packaging/aur/$recipe" "/work/$recipe"
	cd "/work/$recipe"
	if [[ $recipe == rift-s-linux-git ]]; then
		# Build the commit being released instead of main on GitHub.
		sed -i 's|git+https://github.com/lbgos/rift-s-linux.git#branch=main|git+file:///repo#branch=release|' PKGBUILD
		grep -q 'git+file:///repo#branch=release' PKGBUILD
	fi
	if [[ $recipe == basalt-monado-rifts ]]; then
		package_basalt
		return
	fi
	PKGDEST=$out SRCDEST=/work/srcdest makepkg -f --noconfirm
}

# Basalt changes only with its recipe and the Arch libraries it builds against, so /cache keeps its
# package between runs. The prepared sources are extracted every run: the universal build uses them.
package_basalt() {
	local key cached
	key=$({
		cat ./*
		pacman -Q glibc gcc opencv onetbb fmt eigen boost cmake
	} | sha256sum | cut -c1-16)
	cached=(/cache/basalt-"$key"/*.pkg.tar.zst)
	if [[ -f ${cached[0]} ]]; then
		echo "Basalt package from cache $key"
		SRCDEST=/work/srcdest makepkg -o --noconfirm
		cp "${cached[0]}" "$out/"
		return
	fi
	PKGDEST=$out SRCDEST=/work/srcdest makepkg -f --noconfirm
	rm -rf /cache/basalt-*
	mkdir -p "/cache/basalt-$key.part"
	cp "$out"/basalt-monado-rifts-[0-9]*.pkg.tar.zst "/cache/basalt-$key.part/"
	mv "/cache/basalt-$key.part" "/cache/basalt-$key"
}

install_package() {
	pacman -U --noconfirm "$out/$1"-[0-9]*.pkg.tar.zst
}

payload() {
	local stage=/work/stage name=rift-s-linux-$VERSION-x86_64.tar.zst
	rm -rf "$stage"
	mkdir -p "$stage"
	bsdtar -xf "$out"/rift-s-linux-git-[0-9]*.pkg.tar.zst -C "$stage" usr
	bsdtar -xf "$out"/basalt-monado-rifts-[0-9]*.pkg.tar.zst -C "$stage" usr/lib usr/share/licenses
	mv "$stage/usr/share/licenses/rift-s-linux-git" "$stage/usr/share/licenses/rift-s-linux"
	mv "$stage/usr/share/licenses/basalt-monado-rifts" "$stage/usr/share/licenses/rift-s-linux/Basalt"
	tar -C "$stage" --sort=name --owner=0 --group=0 --numeric-owner --zstd -cf "$out/$name" usr

	local recipe=/work/rift-s-linux-bin sum
	rm -rf "$recipe"
	cp -r /repo/packaging/aur/rift-s-linux-bin "$recipe"
	# The recipe downloads the payload from the release; makepkg takes a copy found next to it.
	cp "$out/$name" "$recipe/rift-s-linux-r$BINVER-x86_64.tar.zst"
	sum=$(sha256sum "$out/$name" | cut -d' ' -f1)
	sed -i -e "s/^pkgver=.*/pkgver=$BINVER/" -e 's/^pkgrel=.*/pkgrel=1/' \
		-e "s/^sha256sums=.*/sha256sums=('$sum')/" "$recipe/PKGBUILD"
	# The payload links opencv's minor-versioned and fmt's major-versioned sonames. Pin the builds
	# it links, so pacman refuses an upgrade that would leave the driver unloadable.
	local opencv major minor fmt
	opencv=$(pacman -Q opencv | cut -d' ' -f2)
	IFS=. read -r major minor _ <<<"$opencv"
	fmt=$(pacman -Q fmt | cut -d' ' -f2 | cut -d. -f1)
	sed -i -E -e "s/'opencv[^']*'( 'opencv[^']*')?/'opencv>=$major.$minor' 'opencv<$major.$((minor + 1))'/" \
		-e "s/'(fmt|libfmt\.so[^']*)'/'libfmt.so=$fmt-64'/" "$recipe/PKGBUILD"
	grep -F "'opencv>=$major.$minor' 'opencv<$major.$((minor + 1))'" "$recipe/PKGBUILD" | grep -qF "'libfmt.so=$fmt-64'"
	echo "rift-s-linux-bin pins opencv $major.$minor and libfmt.so=$fmt-64"
	cd "$recipe"
	# -bin only repacks the payload, so its runtime dependencies need not be installed here.
	PKGDEST=$out makepkg -d -f --noconfirm
}

"$@"
