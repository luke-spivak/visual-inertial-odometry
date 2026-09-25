#!/usr/bin/env bash
# Build the staged native app against the Pi's camera API and preserved OpenVINS library.
set -euo pipefail
stage=$(cd "$(dirname "$0")/../.." && pwd)
cmake -S "$stage/src" -B "$stage/build" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DVIO_BUILD_FLIGHT_APP=ON \
    -DVIO_MAVLINK_INCLUDE_DIR="$stage/deps/mavlink-headers" \
    -DOV_SRC="$stage/deps/open_vins" \
    -DOV_LIB="$stage/candidate/lib/libov_msckf_lib.so" \
    -DCMAKE_CXX_FLAGS=-mcpu=cortex-a76 \
    -DCMAKE_EXE_LINKER_FLAGS=-Wl,--disable-new-dtags
# Keep peak compiler memory within the Pi's available RAM.
cmake --build "$stage/build" -j1
ctest --test-dir "$stage/build" --output-on-failure
cp "$stage/build/vio_flight" "$stage/candidate/vio_flight"
ldd "$stage/candidate/vio_flight"
