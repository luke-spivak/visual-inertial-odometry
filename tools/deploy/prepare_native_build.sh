#!/usr/bin/env bash
# Install native build dependencies while retaining the Pi's installed camera ABI.
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then
    echo 'Run this script with sudo on the Pi.' >&2
    exit 1
fi
camera_version=$(dpkg-query -W -f='${Version}' libcamera0.7:arm64)
[ -n "$camera_version" ] || { echo 'Cannot determine installed libcamera version.' >&2; exit 1; }
# Refresh package indexes before resolving dependencies; stale versions can return 404.
apt-get update
# Match the running Pi pipeline; do not substitute Debian's generic libcamera build.
apt-get install -y --no-install-recommends --no-upgrade \
    "libcamera-dev=$camera_version" cmake ninja-build pkg-config \
    libeigen3-dev nlohmann-json3-dev libopencv-dev libceres-dev \
    libboost-system-dev libboost-filesystem-dev libboost-thread-dev libboost-date-time-dev
pkg-config --modversion libcamera
