#!/usr/bin/env bash
# kalibr_setup.sh — install Docker and build the Kalibr image on the sim VM.
#
# WHY DOCKER, and not a native build: Kalibr is a ROS 1 (catkin) package. The
# VM runs Ubuntu 24.04 with ROS 2 jazzy, and there is no ROS 1 for 24.04, so
# there is nothing to build against. The upstream Dockerfile carries its own
# ROS 1 noetic on Ubuntu 20.04. A ROS 2 community fork exists but validating
# an unvalidated fork defeats the point of validating the tool.
#
# WHY A DISK PREFLIGHT: 2026-09-02 recorded the sim host being eaten by its
# own artefacts, and a half-finished multi-GB image build that fills the VM
# disk is the same failure with a different cause. This refuses to start
# rather than stopping halfway.
#
# Run ON the VM:      ./kalibr_setup.sh
# Or from the Mac:    ssh luke@192.168.64.3 'bash -s' < tools/calibration/kalibr_setup.sh
set -euo pipefail

NEED_GB=${NEED_GB:-9}          # docker engine + kalibr image + build layers
KALIBR_DIR="${KALIBR_DIR:-$HOME/kalibr}"
IMAGE="${IMAGE:-kalibr:noetic}"

say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }

say "Preflight"
avail_gb=$(df -BG --output=avail / | tail -1 | tr -dc '0-9')
echo "  free on / : ${avail_gb} GB   (need ${NEED_GB} GB)"
if (( avail_gb < NEED_GB )); then
  cat <<EOF

REFUSING TO START -- not enough disk.

  free:     ${avail_gb} GB
  required: ${NEED_GB} GB

The image build would fail partway and leave dangling layers, which is worse
than not starting. Free space first, then re-run. Useful:

  docker system prune -a          # if docker is already installed
  du -sh ~/* | sort -rh | head    # find the big directories
  ls ~/euroc ~/bags 2>/dev/null   # dataset bags are the usual culprit

Override with:  NEED_GB=<n> ./kalibr_setup.sh
EOF
  exit 1
fi

arch=$(uname -m)
echo "  arch      : ${arch}"
[[ "$arch" == "aarch64" || "$arch" == "arm64" ]] && \
  echo "  note: arm64. Upstream's base image is amd64-only; the build below
        rebases it onto an official multi-arch ROS image."

say "Docker"
if ! command -v docker >/dev/null 2>&1; then
  echo "  installing docker.io from the Ubuntu archive (not Docker's own repo:"
  echo "  one less key and list file, and the archive version is sufficient here)"
  sudo apt-get update
  sudo apt-get install -y docker.io
  sudo systemctl enable --now docker
  sudo usermod -aG docker "$USER" || true
  echo "  added $USER to the docker group -- takes effect in new login shells"
fi
# Plain docker when this shell already has the group; sudo only when it does
# not, which is the run that just installed it. This used to be hardcoded to
# sudo, so every rerun stopped for a password the build never needed -- and
# the prompt then sat directly above an unrelated silent exit, which made the
# failure look like an authentication problem.
if docker info >/dev/null 2>&1; then
  DOCKER="docker"
  echo "  running as $USER, docker group active, no sudo needed"
else
  DOCKER="sudo docker"
  echo "  docker group not active in this shell yet -- using sudo"
fi

say "Kalibr source"
if [[ -d "$KALIBR_DIR/.git" ]]; then
  echo "  already cloned at $KALIBR_DIR"
else
  git clone --depth 1 https://github.com/ethz-asl/kalibr.git "$KALIBR_DIR"
fi
cd "$KALIBR_DIR"
git log --oneline -1 | sed 's/^/  at /'

say "Image build"
# The repo ships one Dockerfile per ROS release and no plain `Dockerfile`.
# This used to be `DF=$(ls a b c 2>/dev/null | head -1)`: ls exits non-zero
# when any operand is missing, pipefail carries that out of the pipeline, and
# set -e kills the script on the assignment -- with no message, since the
# error went to /dev/null. Same failure as build_speedybeef4v4.sh's feature
# grep. Test each candidate instead of asking ls.
DF=""
for cand in Dockerfile_ros1_20_04 Dockerfile_ros1_18_04 Dockerfile; do
  [[ -f "$cand" ]] && { DF="$cand"; break; }
done
[[ -n "$DF" ]] || { echo "no Dockerfile found in $KALIBR_DIR"; exit 1; }

# Build from a derived copy rather than upstream's file, and assert every edit:
# a sed that matches nothing exits 0, so an upstream change would otherwise
# pass straight through unnoticed.
DDF="$(mktemp /tmp/Dockerfile.kalibr.XXXX)"
cp "$DF" "$DDF"
ARM=0; [[ "$arch" == "aarch64" || "$arch" == "arm64" ]] && ARM=1

if (( ARM )); then
  # (1) Base image. Upstream builds FROM osrf/ros:noetic-desktop-full, which is
  # published for amd64 only -- a single-arch manifest, not a list. On an arm64
  # host with no binfmt emulation, docker does not refuse: it pulls the amd64
  # image anyway with a warning, several GB of it, and the first RUN dies with
  # "exec format error". `docker pull --platform linux/arm64` does not fail
  # fast either. ros:noetic-perception is the official multi-arch tag and
  # carries what Kalibr imports from ROS (roslib, rosbag, cv_bridge);
  # desktop-full's extras -- gazebo, rviz, rqt -- Kalibr never touches.
  sed -i 's|^FROM osrf/ros:noetic-desktop-full|FROM ros:noetic-perception|' "$DDF"
  grep -q '^FROM ros:noetic-perception' "$DDF" || {
    echo "  base swap did not apply -- upstream changed its FROM line:"
    grep '^FROM' "$DF" | sed 's/^/    /'
    exit 1
  }
  echo "  arm64: rebased onto ros:noetic-perception"
fi

# (2) Entrypoint. Upstream's is shell form (`ENTRYPOINT export ... && bash`),
# and a shell-form entrypoint discards every argument given to `docker run`:
# `docker run kalibr:noetic bash -lc 'exit 7'` returns 0, because the command
# is dropped and a bare bash reads EOF and exits cleanly. The first version of
# this script's verify step leaned on exactly that and reported success while
# Kalibr could not even import. Replaced with an exec-form entrypoint that
# sources the workspace and runs whatever it is handed.
sed -i '/^ENTRYPOINT/,$d' "$DDF"
if grep -q '^ENTRYPOINT' "$DDF"; then echo "  entrypoint strip did not apply"; exit 1; fi

if (( ARM )); then
  # (3) cv_bridge on arm64. ros-noetic-cv-bridge 1.16.2 fails at import with
  # "SystemError: initialization of cv_bridge_boost raised unreported
  # exception" -- in the pure ros:noetic-perception image as well, same package
  # versions, so neither Kalibr's apt layer nor the rebase causes it. It is a
  # known aarch64 problem, and cv_bridge imports cleanly once cv2 has been
  # imported first. A .pth import line runs at every interpreter start, which
  # fixes every Kalibr tool without touching upstream's scripts.
  echo 'RUN echo "import cv2" > /usr/lib/python3/dist-packages/zz_preload_cv2.pth' >> "$DDF"
fi
# The entrypoint is a small script rather than an inline
# `bash -c "source setup.bash && exec \"$@\""`, because `source` hands the
# caller's positional arguments to the sourced file. catkin's setup.bash
# forwards them to _setup_util.py and evals what it prints, so running
# `rosrun kalibr kalibr_calibrate_cameras --help` fed `--help` to catkin: it
# printed its own usage, the shell executed that text ("Generates: command not
# found"), ROS was never sourced, and rosrun did not exist -- exit 127. The
# exit-7 control passed straight through that, since bash is on PATH with or
# without ROS. Save the arguments, clear them, source, then exec.
cat >> "$DDF" <<'DOCK'
ENV KALIBR_MANUAL_FOCAL_LENGTH_INIT=1
WORKDIR /catkin_ws
RUN printf '%s\n' '#!/bin/bash' 'args=("$@")' 'set --' 'source /catkin_ws/devel/setup.bash' 'exec "${args[@]}"' > /kalibr_entrypoint.sh && chmod +x /kalibr_entrypoint.sh
ENTRYPOINT ["/kalibr_entrypoint.sh"]
CMD ["bash"]
DOCK

# Always build. The layer cache makes an unchanged build nearly free, whereas
# the old "skip if the image already exists" meant edits to this script never
# reached the image.
echo "  building from $DDF (the first build compiles Kalibr; later ones hit the cache)"
$DOCKER build -t "$IMAGE" -f "$DDF" .

say "Verify"
# The control comes first: prove the container executes the command it is
# given, using a command that must fail. Without it, a zero exit from the real
# check is uninterpretable -- which is how the first version passed.
set +e
$DOCKER run --rm "$IMAGE" bash -c 'exit 7' >/dev/null 2>&1
rc=$?
set -e
if [[ $rc -ne 7 ]]; then
  echo "  FAIL: told to exit 7, the image returned $rc -- arguments are not reaching the container"
  exit 1
fi
echo "  control: told to exit 7, got 7 -- arguments reach the container"

# The real check runs the exact top-level imports kalibr_calibrate_cameras
# makes -- read out of the script inside the image, so it tracks upstream --
# as a plain `python3 -c`, whose exit status means what it says.
#
# The tool's own --help cannot be gated on its exit status: it wraps
# parse_args() in a bare `except:` that also catches argparse's clean
# SystemExit(0), so a perfectly good --help exits 2, the same code as an
# argument error. And no pipe on either check: `cmd | head` reports head's
# exit status, which is how a Python traceback was once logged as a pass.
SCRIPT_IMPORTS=$($DOCKER run --rm "$IMAGE" bash -c \
  'grep -E "^(import|from) " "$(rospack find kalibr)/python/kalibr_calibrate_cameras"')
[[ -n "$SCRIPT_IMPORTS" ]] || { echo "  FAIL: could not read the tool's imports"; exit 1; }
out=$($DOCKER run --rm -e MPLBACKEND=Agg "$IMAGE" python3 -c "$SCRIPT_IMPORTS" 2>&1) && rc=0 || rc=$?
if [[ $rc -ne 0 ]]; then
  echo "  FAIL: kalibr_calibrate_cameras's top-level imports exit $rc"
  tail -8 <<<"$out" | sed 's/^/    /'
  exit 1
fi
echo "  imports: all $(wc -l <<<"$SCRIPT_IMPORTS" | tr -d ' ') top-level imports of kalibr_calibrate_cameras load, exit 0"

set +e
out=$($DOCKER run --rm "$IMAGE" rosrun kalibr kalibr_calibrate_cameras --help 2>&1)
set -e
if grep -q "Calibrate the intrinsics" <<<"$out" && ! grep -q "Traceback" <<<"$out"; then
  echo "  --help: prints the tool's usage, no traceback (exit status is 2 by Kalibr's design)"
else
  echo "  FAIL: --help did not print usage cleanly"
  tail -8 <<<"$out" | sed 's/^/    /'
  exit 1
fi

say "Next"
cat <<EOF
Generate the calibration target with Kalibr's own generator, so target and
detector cannot disagree:

  $DOCKER run --rm -v "\$PWD:/out" -w /out $IMAGE \\
    rosrun kalibr kalibr_create_target_pdf --type apriltag \\
      --nx 6 --ny 8 --tsize 0.045 --tspace 0.3 aprilgrid_6x8_45mm.pdf

That is sized for A2 with ~23 mm side margins. It writes the PDF only; the
target YAML Kalibr reads is written by hand. PRINT IT, THEN MEASURE IT -- see
tools/calibration/kalibr_compare.py's docstring for why the printed size, not the
requested size, is what goes in the YAML.
EOF
