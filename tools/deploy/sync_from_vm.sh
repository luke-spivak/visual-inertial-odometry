#!/usr/bin/env bash
# sync_from_vm.sh — pull sim/ros2/ packages back from the VM into this repo, for committing.
#
# Pairs with sync_to_vm.sh. Use this when editing in the VM (e.g. VS Code
# Remote-SSH, where rclpy actually resolves) and committing on the Mac.
#
# Excludes build artifacts; see .gitignore.
set -eo pipefail

HOST="${1:-luke@192.168.64.3}"
WS="${2:-~/ws_vio}"
DEST="$(cd "$(dirname "$0")/../.." && pwd)/sim/ros2/"

rsync -av --delete \
  --exclude '__pycache__' --exclude '*.pyc' --exclude '*.egg-info' \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  "$HOST:$WS/src/" "$DEST"

echo
echo "pulled <- $HOST:$WS/src/"
echo "review with: git -C $(dirname "$DEST") status"
