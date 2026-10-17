# Universal x86_64 build

A tarball for non-Arch distributions. `install.sh` puts everything under
`~/.local/share/rift-s-linux`, which SteamVR also sees inside the Steam Linux
Runtime container, so the driver needs no SteamVR launch option. Only the udev
rule needs root.

The driver, Basalt and their libraries are built in the Steam Runtime sniper SDK
(Debian 11, glibc 2.31) and bundle Basalt, oneTBB and a trimmed OpenCV. The
Settings app needs webkit2gtk 4.1, which the SDK lacks, so it is built in an
Ubuntu 22.04 root (glibc 2.35).

## Layout

`work/` is bind-mounted at `/work` in both roots:

- `src/monado` this repository
- `src/basalt` Basalt prepared by `packaging/aur/basalt-monado-rifts` (`makepkg -o`)
- `src/{opencv-4.12.0,oneTBB-2022.2.0,fmt-11.2.0}` upstream release tarballs
- `hostdeps/usr` header-only build deps extracted from the Arch packages
  `boost`, `eigen`, `vulkan-headers`, `plasma-wayland-protocols`

`sysroot/` is `com.valvesoftware.SteamRuntime.Sdk-amd64,i386-sniper-sysroot.tar.gz`
from `repo.steampowered.com/steamrt-images-sniper/snapshots/`. `sdk` runs a
command in it with bwrap.

The Ubuntu root needs `build-essential pkg-config libwebkit2gtk-4.1-dev
libgtk-3-dev libsoup-3.0-dev libvulkan-dev libwayland-dev glslang-tools
python3`, rustup, Node 22 and pnpm 10. Ubuntu 22.04 has no `glslc` package; a
wrapper around `glslangValidator -V -x` that adds the surrounding braces is enough
for `settings-app/src-tauri/build.rs`.

## Build

```sh
./sdk /work/src/monado/packaging/universal/deps.sh      # once
./sdk /work/src/monado/packaging/universal/build.sh
<ubuntu root> /work/src/monado/packaging/universal/settings.sh
packaging/universal/package.sh work <version>          # host, needs patchelf
```

The result is `work/dist/rift-s-linux-<version>-x86_64.tar.xz`. Users extract it
and run `./install.sh`; `./install.sh --uninstall` removes it.
