#!/bin/sh
# Build the bundled third-party libraries into /work/prefix inside the sniper SDK.
set -eu
export CC=gcc-14 CXX=g++-14
P=/work/prefix
J=${J:-8}
common="-G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$P -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_POSITION_INDEPENDENT_CODE=ON"
cd /work/src
cmake -S fmt-11.2.0 -B /work/build/fmt $common -DFMT_TEST=OFF -DFMT_DOC=OFF -DBUILD_SHARED_LIBS=OFF
cmake --build /work/build/fmt -j$J && cmake --install /work/build/fmt
cmake -S oneTBB-2022.2.0 -B /work/build/tbb $common -DTBB_TEST=OFF -DTBB_EXAMPLES=OFF -DTBB_STRICT=OFF -DTBBMALLOC_BUILD=OFF
cmake --build /work/build/tbb -j$J && cmake --install /work/build/tbb
cmake -S opencv-4.12.0 -B /work/build/opencv $common \
  -DBUILD_LIST=core,imgproc,imgcodecs,highgui,video,calib3d,features2d,flann \
  -DWITH_QT=OFF -DWITH_GTK=OFF -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF -DWITH_V4L=OFF -DWITH_OPENCL=OFF \
  -DWITH_IPP=OFF -DWITH_ITT=OFF -DWITH_PROTOBUF=OFF -DWITH_TBB=OFF -DWITH_EIGEN=OFF -DWITH_LAPACK=OFF \
  -DWITH_TIFF=OFF -DWITH_WEBP=OFF -DWITH_OPENEXR=OFF -DWITH_OPENJPEG=OFF -DWITH_JASPER=OFF -DWITH_AVIF=OFF \
  -DWITH_IMGCODEC_HDR=OFF -DWITH_IMGCODEC_SUNRASTER=OFF -DWITH_IMGCODEC_PXM=OFF -DWITH_IMGCODEC_PFM=OFF \
  -DBUILD_PNG=ON -DBUILD_JPEG=ON -DBUILD_ZLIB=ON -DWITH_ADE=OFF -DWITH_VA=OFF -DWITH_1394=OFF \
  -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_apps=OFF -DBUILD_JAVA=OFF \
  -DBUILD_opencv_python3=OFF -DOPENCV_GENERATE_PKGCONFIG=OFF -DCPU_BASELINE=SSE3 -DOPENCV_SKIP_PYTHON_LOADER=ON
cmake --build /work/build/opencv -j$J && cmake --install /work/build/opencv
echo deps-ok
