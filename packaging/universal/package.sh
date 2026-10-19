#!/bin/sh
# Assemble the universal tarball from a finished SDK build (see build.sh).
#
#   package.sh <work dir> <version>
#
# <work dir> holds src/{monado,basalt,...}, prefix/ (bundled libraries) and
# stage/opt/rift-s-linux (the installed tree). Needs patchelf and xz on the host.
set -eu

work=$(cd "$1" && pwd)
version=$2
name=rift-s-linux-$version-x86_64
out=$work/dist/$name
files=$out/files
src=$work/src
licenses=$files/share/licenses/rift-s-linux

rm -rf "$out"
mkdir -p "$files"
cp -a "$work/stage/opt/rift-s-linux/." "$files/"
# rifts-setup triage reads the installed release from here.
echo "$version" > "$files/version"
# The universal install never registers itself as the system OpenXR runtime.
rm -rf "$files/share/openxr" "$files/lib/libopenxr_monado.so"

# Bundled libraries: Basalt and its OpenCV/oneTBB. Everything else comes from the
# host or, for the SteamVR driver, from the Steam Linux Runtime.
for lib in libbasalt.so.2 libtbb.so.12 $(cd "$work/prefix/lib" && ls libopencv_*.so.412); do
	cp -L "$work/prefix/lib/$lib" "$files/lib/$lib"
done
ln -s libbasalt.so.2 "$files/lib/libbasalt.so"

# Relocatable lookup from each install location to the bundled lib/.
set_rpath() {
	for f in "$@"; do
		if [ ! -L "$f" ] && file -b "$f" | grep -q ELF; then patchelf --set-rpath "$rpath" "$f"; fi
	done
}
rpath='$ORIGIN/../lib' set_rpath "$files"/bin/*
rpath='$ORIGIN' set_rpath "$files"/lib/*.so*
rpath='$ORIGIN/../../../../lib' set_rpath "$files"/share/steamvr-monado/bin/linux64/*

install -Dm644 "$src/monado/packaging/aur/rift-s-linux-git/70-monado-rifts.rules" \
	"$files/share/udev/70-monado-rifts.rules"

mkdir -p "$licenses"
install -m644 "$src/monado/LICENSE" "$licenses/LICENSE"
cp -a "$src/monado/LICENSES" "$licenses/Monado"
(cd "$src/monado" && find src/external -type f \( -iname LICENSE -o -iname LICENSE.txt -o -iname COPYING \
	-o -iname '*.license' \) -exec install -Dm644 {} "$licenses/{}" \;)
for item in LICENSE thirdparty/basalt-headers/LICENSE thirdparty/basalt-headers/thirdparty/Sophus/LICENSE.txt \
	thirdparty/basalt-headers/thirdparty/cereal/LICENSE thirdparty/CLI11/LICENSE thirdparty/magic_enum/LICENSE \
	thirdparty/opengv/License.txt thirdparty/vit/LICENSE; do
	install -Dm644 "$src/basalt/$item" "$licenses/Basalt/$item"
done
cp -a "$work/prefix/share/licenses/opencv4" "$licenses/OpenCV"
install -m644 "$src"/opencv-*/LICENSE "$licenses/OpenCV/LICENSE"
install -Dm644 "$src"/oneTBB-*/LICENSE.txt "$licenses/oneTBB/LICENSE.txt"
install -Dm644 "$src"/fmt-*/LICENSE "$licenses/fmt/LICENSE"

install -m755 "$src/monado/packaging/universal/install.sh" "$out/install.sh"
tar -C "$work/dist" --sort=name --owner=0 --group=0 --numeric-owner -cJf "$work/dist/$name.tar.xz" "$name"
echo "$work/dist/$name.tar.xz"
