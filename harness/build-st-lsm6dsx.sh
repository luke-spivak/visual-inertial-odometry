#!/usr/bin/env bash
# build-st-lsm6dsx.sh -- build the ISM330DHCX kernel driver out of tree.
#
# Pi OS does not ship it: the running kernel's config says
# "# CONFIG_IIO_ST_LSM6DSX is not set" and there is no module on disk, so the
# overlay creates a correct spi0.0 node that nothing ever binds to. Everything
# the driver links against IS present (IIO=m, IIO_KFIFO_BUF=m, REGMAP_SPI=m),
# so only the driver itself has to be built.
#
# The source is fetched rather than vendored -- it is GPL kernel code that
# belongs to the kernel, not to this repo -- but pinned to a commit, because a
# moving branch means a rebuild can silently differ from the one that was
# tested. To move the pin deliberately, change SHA below.
#
# Needs no root. Installing the result does; this script prints those steps.
#
#   ./build-st-lsm6dsx.sh            # build against the running kernel
#   KVER=6.18.34+rpt-rpi-2712 ./build-st-lsm6dsx.sh   # or another installed one
set -euo pipefail

SHA=0ac97ba3443f519b61bbc96079736cd8b881ea22
KVER="${KVER:-$(uname -r)}"
KDIR="/lib/modules/$KVER/build"
WORK="${WORK:-$HOME/build/st_lsm6dsx}"
BASE="https://raw.githubusercontent.com/raspberrypi/linux/$SHA/drivers/iio/imu/st_lsm6dsx"

[ -d "$KDIR" ] || { echo "no kernel headers at $KDIR"; exit 1; }

mkdir -p "$WORK"
cd "$WORK"

# i2c and i3c glue is deliberately not built: the part is on SPI, and the i3c
# variant would drag in CONFIG_I3C for nothing.
# Always re-fetch, so the patch below always applies to pristine source. A
# half-patched tree that still compiles is a worse outcome than a slow build.
for f in st_lsm6dsx.h st_lsm6dsx_core.c st_lsm6dsx_buffer.c \
         st_lsm6dsx_shub.c st_lsm6dsx_spi.c; do
  curl -fsSL -o "$f" "$BASE/$f"
done

# Local change: re-anchor ts_ref against the host clock as batches arrive.
# Stock, sample times are ts_ref + ticks * ts_gain with ts_ref taken once at
# buffer enable -- open loop, and measured here at -2003 ppm, which is 1.2 s of
# camera-IMU divergence over a ten minute flight. See the patch header.
PATCH="$(dirname "$(readlink -f "$0")")/st_lsm6dsx-reanchor.patch"
if [ -f "$PATCH" ]; then
  patch -p1 --no-backup-if-mismatch < "$PATCH" || {
    echo "patch failed -- upstream moved under the pinned SHA?"; exit 1; }
  echo "applied $(basename "$PATCH")"
else
  echo "WARNING: $PATCH not found, building stock (it will drift)"
fi

# Object layout copied from the in-tree Makefile: core, buffer and shub link
# into one module, the bus glue is its own.
cat > Makefile <<'MK'
st_lsm6dsx-y := st_lsm6dsx_core.o st_lsm6dsx_buffer.o st_lsm6dsx_shub.o
obj-m += st_lsm6dsx.o
obj-m += st_lsm6dsx_spi.o
MK

echo "building against $KVER"
make -C "$KDIR" M="$PWD" modules

echo
ls -la *.ko 2>/dev/null || { echo "no .ko produced"; exit 1; }
echo
echo "Built. Installing needs root:"
echo "  sudo make -C $KDIR M=$PWD modules_install"
echo "  sudo depmod -a"
echo "  sudo modprobe st_lsm6dsx_spi"
echo
echo "Then confirm it actually bound -- 'built' is not the goal:"
echo "  readlink /sys/bus/spi/devices/spi0.0/driver     # must stop saying NONE"
echo "  ls /sys/bus/iio/devices/"
