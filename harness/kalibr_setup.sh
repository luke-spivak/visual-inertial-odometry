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
# Or from the Mac:    ssh luke@192.168.64.3 'bash -s' < harness/kalibr_setup.sh
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
  echo "  note: arm64. Upstream CI builds x86_64; the base images exist for arm64
        but this build is less travelled. If it fails on a missing package,
        that is the likely reason."

say "Docker"
if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
  echo "  already installed and running"
else
  echo "  installing docker.io from the Ubuntu archive (not Docker's own repo:"
  echo "  one less key and list file, and the archive version is sufficient here)"
  sudo apt-get update
  sudo apt-get install -y docker.io
  sudo systemctl enable --now docker
  # Group membership does not apply to the current shell; sudo is used below
  # rather than asking for a re-login mid-script.
  sudo usermod -aG docker "$USER" || true
  echo "  added $USER to the docker group -- log out and back in for it to"
  echo "  take effect in new shells. This script uses sudo regardless."
fi
DOCKER="sudo docker"

say "Kalibr source"
if [[ -d "$KALIBR_DIR/.git" ]]; then
  echo "  already cloned at $KALIBR_DIR"
else
  git clone --depth 1 https://github.com/ethz-asl/kalibr.git "$KALIBR_DIR"
fi
cd "$KALIBR_DIR"
git log --oneline -1 | sed 's/^/  at /'

say "Image build"
if $DOCKER image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "  $IMAGE already exists -- skipping. Force with: $DOCKER rmi $IMAGE"
else
  # The repo ships Dockerfiles per ROS release; prefer the 20.04/noetic one.
  DF=$(ls Dockerfile_ros1_20_04 Dockerfile_ros1_18_04 Dockerfile 2>/dev/null | head -1)
  [[ -n "$DF" ]] || { echo "no Dockerfile found in $KALIBR_DIR"; exit 1; }
  echo "  using $DF  (this takes a while -- it compiles Kalibr from source)"
  $DOCKER build -t "$IMAGE" -f "$DF" .
fi

say "Verify"
# A build that succeeds but produces an image whose entrypoint cannot find the
# tools is a real outcome, so check that a command actually resolves.
if $DOCKER run --rm "$IMAGE" bash -lc \
     'source /catkin_ws/devel/setup.bash 2>/dev/null; which kalibr_calibrate_cameras || rosrun kalibr kalibr_calibrate_cameras --help >/dev/null 2>&1 && echo via-rosrun'; then
  echo "  kalibr_calibrate_cameras resolves"
else
  cat <<'EOF'
  Could not resolve kalibr_calibrate_cameras inside the image.
  The image built, so this is an environment/path question, not a build
  failure. Inspect interactively:

    sudo docker run -it --rm kalibr:noetic bash
    source /catkin_ws/devel/setup.bash && rospack list | grep kalibr
EOF
  exit 1
fi

say "Next"
cat <<EOF
Generate the calibration target with Kalibr's own generator, so target and
detector cannot disagree:

  sudo docker run --rm -v "\$PWD:/out" $IMAGE bash -lc \\
    'source /catkin_ws/devel/setup.bash && cd /out && \\
     rosrun kalibr kalibr_create_target_pdf --type apriltag \\
       --nx 6 --ny 6 --tsize 0.03 --tspace 0.3'

That writes target.pdf plus the april_6x6.yaml that describes it. PRINT IT,
THEN MEASURE IT -- see harness/kalibr_compare.py's docstring for why the
printed size, not the requested size, is what goes in the YAML.
EOF
