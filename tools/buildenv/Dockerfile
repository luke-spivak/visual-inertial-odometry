# Build environment for anything heavy that has to run on viopi.
#
# On Apple Silicon this is not cross-compilation: the Mac is arm64 and the Pi is
# arm64, so --platform linux/arm64 runs natively at full speed. It is just a
# faster arm64 machine that happens to be on the desk.
#
# The tag has to match the Pi's release. viopi runs Debian 13 (trixie) with
# glibc 2.41; a bookworm container links against an older glibc and the
# mismatch does not fail at build time -- it produces binaries that look
# corrupt on the Pi. buildenv.sh asserts the match rather than trusting it.
FROM debian:trixie

# Compile/test the native app here, including the libcamera API. Deployment builds
# must still link against the Pi's own libcamera packages to avoid ABI/version skew.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build git pkg-config ca-certificates file nlohmann-json3-dev libcamera-dev \
        libeigen3-dev \
        libboost-filesystem-dev libboost-date-time-dev \
        libboost-thread-dev libboost-system-dev \
        libopencv-dev \
        libceres-dev libsuitesparse-dev libgoogle-glog-dev libgflags-dev \
        python3 python3-numpy \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
