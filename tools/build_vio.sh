#!/usr/bin/env bash
# build.sh -- build vio_live for viopi in the buildenv container and deploy it.
#
#   tools/build_vio.sh          build, bundle, copy to viopi:~/vio_live, gate
#
# 1. OpenVINS, ROS-free, pinned to upstream 6948812 -- the parent of the VM's
#    local commit, which only ports ROS 2 includes and so changes nothing here.
#    ENABLE_ARUCO_TAGS=OFF (no tags in step 7; saves the contrib dependency).
# 2. -mcpu=cortex-a76, set explicitly: the container runs on an Apple core, so
#    -march=native would emit instructions the Pi 5 does not have.
# 3. Bundle: the binary, libov_msckf_lib.so, and any library it needs that
#    the Pi does not already have -- same Debian trixie builds, so same ABI,
#    and no apt install on the Pi.
#    Linked --disable-new-dtags: the default RUNPATH covers only the binary's
#    direct dependencies, so a bundled libceres could not find its bundled
#    libglog. Old-style RPATH is inherited by every library in the process.
#    --as-needed drops the NEEDED entries OpenVINS's link line adds for OpenCV
#    modules it never calls.
# 4. Gate: on the Pi, `ldd` finds every library and the binary runs to its
#    usage message.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
IMAGE="${IMAGE:-vio-build:trixie}"
PI="${PI:-viopi}"
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

ssh "$PI" 'mkdir -p ~/src'
rsync -a --delete "$REPO/$B/bundle/" "$PI:vio_live/"
scp -q "$REPO/src/vio_live.py" "$REPO/src/imu_log.py" "$REPO/src/vio_mavlink.py" \
    "$REPO/src/vio_flight.py" "$REPO/src/vio@.service" "$PI:src/"

ssh "$PI" 'missing=$(ldd ~/vio_live/vio_live | grep "not found" || true)
[ -z "$missing" ] || { echo "  FAIL: on the Pi:"; echo "$missing"; exit 1; }
out=$(~/vio_live/vio_live 2>&1 || true)
echo "$out" | grep -q "usage: vio_live" || { echo "  FAIL: binary did not run to its usage message:"; echo "$out" | head -5; exit 1; }
echo "  gate: every library resolves on viopi and vio_live runs"'
