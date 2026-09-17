#!/usr/bin/env bash
# sync_to_vm.sh — push sim/ros2/ packages from this repo into the VM's colcon workspace.
# Canonical copy is here in git; the VM is a deploy target, not a source of truth.
#
# Usage: ./tools/deploy/sync_to_vm.sh [HOST] [REMOTE_WS]
set -eo pipefail

HOST="${1:-luke@192.168.64.3}"
WS="${2:-~/ws_vio}"
SRC="$(cd "$(dirname "$0")/../.." && pwd)/sim/ros2/"

ssh "$HOST" "mkdir -p $WS/src"
# --update: never overwrite a file that is NEWER on the VM. Editing happens in
#   the VM (VS Code Remote-SSH, where rclpy resolves), so a blind push would
#   clobber live work with a stale local copy.
# --backup: anything overwritten is kept in .sync-backup/ rather than lost.
# No --delete: removing remote files is not worth the blast radius.
rsync -av --update --backup --backup-dir=.sync-backup \
  --exclude '__pycache__' --exclude '*.pyc' --exclude '.DS_Store' \
  --exclude '.sync-backup' \
  "$SRC" "$HOST:$WS/src/"

echo
echo "synced -> $HOST:$WS/src/"
echo "build with:  ssh $HOST 'source /opt/ros/jazzy/setup.bash && cd $WS && colcon build --symlink-install'"
