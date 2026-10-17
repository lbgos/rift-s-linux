#!/bin/sh
# Build Basalt and rift-s-linux against the bundled deps; stage the install under /work/stage.
set -eu
export CC=gcc-14 CXX=g++-14
P=/work/prefix H=/work/hostdeps/usr J=${J:-8}
export CMAKE_PREFIX_PATH="$P:$H"
cmake -S /work/src/basalt -B /work/build/basalt -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$P -DCMAKE_INSTALL_LIBDIR=lib -DBUILD_TESTS=OFF -DBUILD_TESTING=OFF \
  -DBASALT_INSTANTIATIONS_DOUBLE=OFF -DBASALT_BUILTIN_EIGEN=OFF -DEIGEN_ROOT=$H/include/eigen3 \
  -DBoost_INCLUDE_DIR=$H/include -DBASALT_BUILD_SHARED_LIBRARY_ONLY=ON -DBASALT_ENABLE_ROSBAG2=OFF \
  -DBASALT_BUILD_VISUALIZATION=OFF -DCXX_MARCH=x86-64 -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN"
cmake --build /work/build/basalt -j${BJ:-4} && cmake --install /work/build/basalt
nm -D /work/build/basalt/libbasalt.so | grep -q " T vit_tracker_supports_safe_recreation"
drivers=$(sed -n "s/.*(\(XRT_BUILD_DRIVER_[A-Z0-9_]*\) .*/-D\1=OFF/p" /work/src/monado/CMakeLists.txt | tr "\n" " ")
# bin/ and share/steamvr-monado/bin/linux64/ both reach the bundled lib/.
cmake -S /work/src/monado -B /work/build/monado -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/opt/rift-s-linux -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_INSTALL_LIBEXECDIR=lib \
  -DCMAKE_INSTALL_RPATH="\$ORIGIN/../lib:\$ORIGIN/../../../../lib" \
  -DBUILD_TESTING=ON -DXRT_BUILD_SAMPLES=OFF $drivers \
  -DXRT_BUILD_DRIVER_RIFT_S=ON -DXRT_BUILD_DRIVER_SIMULATED=ON \
  -DXRT_FEATURE_STEAMVR_PLUGIN=ON -DXRT_FEATURE_SLAM=ON \
  -DXRT_HAVE_SYSTEM_CJSON=OFF -DXRT_HAVE_GST=OFF -DXRT_HAVE_BLUETOOTH=OFF \
  -DXRT_HAVE_DBUS=OFF -DXRT_HAVE_HIDAPI=OFF -DXRT_HAVE_JPEG=OFF -DXRT_HAVE_LIBBSD=OFF \
  -DXRT_HAVE_LIBUVC=OFF -DXRT_HAVE_OPENVR=OFF -DXRT_MODULE_MERCURY_HANDTRACKING=OFF \
  -DRIFTS_DESKTOP_OVERLAY=ON -DOpenCV_DIR=$P/lib/cmake/opencv4 -DEigen3_DIR=$H/share/eigen3/cmake
cmake --build /work/build/monado -j$J
VIT_SYSTEM_LIBRARY_PATH=$P/lib/libbasalt.so ctest --test-dir /work/build/monado --output-on-failure -j4
rm -rf /work/stage && DESTDIR=/work/stage cmake --install /work/build/monado
echo build-ok
