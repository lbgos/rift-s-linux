# Rift S on Linux

Oculus Rift S support for Monado and SteamVR on Linux. It has inside-out head tracking with Basalt SLAM, Touch controller tracking from the controller LEDs, and room setup with a play-area boundary.

This is a fork of thaytan's Monado Rift S branch, `dev-constellation-controller-tracking`. The original Monado readme is in `README-monado.md`.

## Status

Early project. It already works and is usable in SteamVR with the headset and both controllers. Good changes will keep landing here.

Known rough edges:

- Controller behavior when the controllers leave the camera view is still rough.
- The boundary does not persist reliably across restarts.

## Build and install

Head tracking needs Basalt. Clone Basalt, apply the patches in `patches/basalt/` in order, then build and install it by its own readme so that `libbasalt.so` is on the library path. If it is somewhere else, set `VIT_SYSTEM_LIBRARY_PATH`.

```
git clone https://gitlab.freedesktop.org/mateosss/basalt.git
cd basalt
git submodule update --init --recursive
for p in /path/to/this/repo/patches/basalt/*.patch; do patch -p1 < "$p"; done
```

The patches were made against Basalt main at df6e970.

Then build Monado. OpenCV, Vulkan and Eigen are required on top of Monado's usual dependencies.

```
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DXRT_BUILD_DRIVER_RIFT_S=ON -DXRT_FEATURE_SLAM=ON -DXRT_FEATURE_STEAMVR_PLUGIN=ON
ninja -C build
sudo ninja -C build install
```

The install puts the SteamVR driver in `/usr/local/share/steamvr-monado`. Register it with SteamVR and start SteamVR with the headset plugged in.

```
~/.local/share/Steam/steamapps/common/SteamVR/bin/vrpathreg.sh adddriver /usr/local/share/steamvr-monado
```

Unit tests build with the default `BUILD_TESTING=ON` and run with `ctest --test-dir build`.

## License

Boost Software License 1.0, the same as Monado. See `LICENSE`. Third-party files keep their own licenses in `LICENSES/`.

Contact lbgos at contact@lbgos.dev.
