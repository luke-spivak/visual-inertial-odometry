#!/usr/bin/env bash
# sync_to_vm.sh — push ros2/ packages from this repo into the VM's colcon workspace.
# Canonical copy is here in git; the VM is a deploy target, not a source of truth.
#
# Usage: ./harness/sync_to_vm.sh [HOST] [REMOTE_WS]
set -eo pipefail

HOST="${1:-luke@192.168.64.3}"
WS="${2:-~/ws_vio}"
SRC="$(cd "$(dirname "$0")/.." && pwd)/ros2/"

ssh "$HOST" "mkdir -p $WS/src"
rsync -av --delete \
  --exclude '__pycache__' --exclude '*.pyc' --exclude '.DS_Store' \
  "$SRC" "$HOST:$WS/src/"

echo
echo "synced -> $HOST:$WS/src/"
echo "build with:  ssh $HOST 'source /opt/ros/jazzy/setup.bash && cd $WS && colcon build --symlink-install'"
