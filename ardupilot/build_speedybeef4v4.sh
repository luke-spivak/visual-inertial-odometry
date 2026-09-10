#!/usr/bin/env bash
# Build ArduCopter for the SpeedyBee F405 V4 with the features this project
# needs, and report flash usage against the board's real ceiling.
#
#   ./build_speedybeef4v4.sh                 # stock, for a baseline
#   ./build_speedybeef4v4.sh vio_min.dat     # + visual odom, EKF3 external nav
#
# Why build locally instead of using custom.ardupilot.org: the build server is a
# web form, and the question here is quantitative -- how many bytes does each
# feature cost, and what is left. A local build answers that directly and is
# reproducible from this repo.
#
# THE CEILING. hwdef.dat gives FLASH_SIZE_KB 1024 and FLASH_RESERVE_START_KB 48,
# so the application has ~976 KB. waf prints "Free Flash" against that; a build
# that does not fit fails with "region `flash' overflowed", it does not silently
# truncate.
#
# THE GATE THAT MATTERS. Both features this project depends on are compiled out
# on a 1 MB board by source default, not by anything in the board's hwdef:
#
#     HAL_VISUALODOM_ENABLED    HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#     EK3_FEATURE_EXTERNAL_NAV  EK3_FEATURE_ALL || HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#
# so 1024 > 1024 is false and the stock firmware has neither. That is the whole
# reason a custom build is needed -- not flash pressure, which turns out to be
# ample.
#
# Requires the ARM toolchain. On the sim VM it is at
# /opt/gcc-arm-none-eabi-10-2020-q4-major and is NOT on PATH by default.
#
# Also requires the python intelhex module, or waf silently produces no
# arducopter_with_bl.hex -- the only artifact that can flash a board coming from
# Betaflight. The gate is evaluated at CONFIGURE time, so install it first.
set -eo pipefail

AP="${ARDUPILOT_DIR:-$HOME/ardupilot}"
TC="${ARM_TOOLCHAIN:-/opt/gcc-arm-none-eabi-10-2020-q4-major/bin}"
EXTRA="${1:-}"
HERE="$(cd "$(dirname "$0")" && pwd)"

[ -d "$AP" ] || { echo "no ArduPilot checkout at $AP (set ARDUPILOT_DIR)" >&2; exit 1; }
[ -x "$TC/arm-none-eabi-gcc" ] || { echo "no ARM toolchain at $TC (set ARM_TOOLCHAIN)" >&2; exit 1; }
export PATH="$TC:$PATH"

ARGS=(--board speedybeef4v4)
if [ -n "$EXTRA" ]; then
  [ -f "$EXTRA" ] || EXTRA="$HERE/$EXTRA"
  [ -f "$EXTRA" ] || { echo "no such extra-hwdef: $1" >&2; exit 1; }
  ARGS+=(--extra-hwdef "$EXTRA")
  echo "==> features:"; grep -vE '^\s*(#|$)' "$EXTRA" | sed 's/^/    /'
fi

cd "$AP"
./waf configure "${ARGS[@]}" >/dev/null
./waf copter 2>&1 | tail -30

echo
echo "==> feature check (generated hwdef.h)"
H="$AP/build/speedybeef4v4/hwdef.h"
for f in HAL_VISUALODOM_ENABLED EK3_FEATURE_EXTERNAL_NAV AP_RANGEFINDER_ENABLED \
         AP_OPTICALFLOW_ENABLED EK3_FEATURE_OPTFLOW_FUSION; do
  # || true: a symbol at its source default is absent from hwdef.h, and under
  # `set -eo pipefail` that failing grep would exit the script mid-report.
  v="$(grep -E "^#define $f " "$H" 2>/dev/null | tail -1 | awk '{print $3}' || true)"
  printf "    %-30s %s\n" "$f" "${v:-<source default>}"
done
echo
echo "==> artifacts"
B="$AP/build/speedybeef4v4/bin"
printf "    %-24s %s\n" "arducopter.apj" "$([ -f "$B/arducopter.apj" ] && wc -c < "$B/arducopter.apj" || echo MISSING)"
if [ -f "$B/arducopter_with_bl.hex" ]; then
  printf "    %-24s %s\n" "arducopter_with_bl.hex" "$(wc -c < "$B/arducopter_with_bl.hex")"
else
  echo "    arducopter_with_bl.hex   MISSING -- no intelhex module at configure time."
  echo "    waf gates hex generation on HAVE_INTEL_HEX and skips it silently."
  echo "      pip3 install --user --break-system-packages intelhex, then re-run this."
fi
echo
echo "    .apj updates a board that already runs ArduPilot (bootloader present)."
echo "    A board arriving from Betaflight has no such bootloader: flash"
echo "    _with_bl.hex over DFU. dfu-util needs a raw binary, so convert first:"
echo "      arm-none-eabi-objcopy -I ihex -O binary --gap-fill 0xff \\"
echo "        $B/arducopter_with_bl.hex arducopter_with_bl.bin"
