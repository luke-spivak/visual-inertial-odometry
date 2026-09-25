#!/usr/bin/env bash
# Build the pinned OpenVINS dependency and compatibility runner bundle.

# Produces a local dependency bundle; does not deploy or restart the Pi.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
IMAGE="${IMAGE:-vio-build:trixie}"
PI="${PI:-viopi}" # Read-only library inventory for bundling.
OV_SHA=69488123ed9362dd44b6f28e7f4680abbff1442b
B=build/vio_live   # relative to the repo, which the container mounts at /work

docker image inspect "$IMAGE" >/dev/null 2>&1 || "$REPO/tools/buildenv/buildenv.sh" build

docker run --rm --platform linux/arm64 -v "$REPO":/work -w /work "$IMAGE" bash -c "
set -euo pipefail
B=$B
[ -d \$B/open_vins/.git ] || git clone -q https://github.com/rpng/open_vins.git \$B/open_vins
git -C \$B/open_vins checkout -q $OV_SHA
FLAGS='-mcpu=cortex-a76'
cmake -S \$B/open_vins/ov_msckf -B \$B/ov -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DENABLE_ROS=OFF -DENABLE_ARUCO_TAGS=OFF -DCMAKE_CXX_FLAGS=\"\$FLAGS\" \
      -DCMAKE_SHARED_LINKER_FLAGS=-Wl,--as-needed > \$B/ov-configure.log
# -j4, not more: at -j6 in Docker's 8 GB the OOM killer took a cc1plus
# (Eigen/Ceres translation units run ~1.5 GB each) and the build just stopped.
ninja -C \$B/ov -j4 ov_msckf_lib
cmake -S src/openvins_runner -B \$B/openvins_runner -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DOV_SRC=/work/\$B/open_vins -DOV_LIB=/work/\$B/ov/libov_msckf_lib.so -DCMAKE_CXX_FLAGS=\"\$FLAGS\" \
      -DCMAKE_EXE_LINKER_FLAGS=\"-Wl,--as-needed -Wl,--disable-new-dtags\" > \$B/openvins_runner-configure.log
ninja -C \$B/openvins_runner
rm -rf \$B/bundle \$B/deps && mkdir -p \$B/bundle/lib \$B/deps
cp \$B/openvins_runner/vio_live \$B/bundle/
cp \$B/ov/libov_msckf_lib.so \$B/bundle/lib/
LD_LIBRARY_PATH=\$B/bundle/lib ldd \$B/openvins_runner/vio_live | awk '/=> \\// {print \$3}' | while read -r f; do cp -L \"\$f\" \$B/deps/; done
"

# Ship only what the Pi lacks.
have=$(ssh "$PI" '/sbin/ldconfig -p' | awk '{print $1}' | sort -u)
n=0
for f in "$REPO/$B"/deps/*; do
    name=$(basename "$f")
    [ "$name" = libov_msckf_lib.so ] && continue
    grep -qxF "$name" <<<"$have" || { cp "$f" "$REPO/$B/bundle/lib/"; n=$((n + 1)); }
done
mkdir -p "$REPO/$B/bundle/config"
cp "$REPO"/src/config/*.yaml "$REPO/$B/bundle/config/"
echo "  bundle: vio_live + $(ls "$REPO/$B/bundle/lib" | wc -l | tr -d ' ') libs ($n the Pi lacked), $(du -sh "$REPO/$B/bundle" | cut -f1)"


echo "Dependency bundle ready at $REPO/$B/bundle; use the native Pi build for flight deployment."
