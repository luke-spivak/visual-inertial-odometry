#!/usr/bin/env bash
# Run from the Mac while the aircraft is disarmed. Uses an already-built binary.
# Ship all CLI consumers together so config API changes cannot leave mixed versions.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
BINARY="$REPO/build/vio_live/openvins_runner/vio_live"
[ -f "$BINARY" ] || { echo 'Build the live harness first.' >&2; exit 1; }
TMP_STAGE=$(mktemp -d)
trap 'rm -rf "$TMP_STAGE"' EXIT
cp "$BINARY" "$TMP_STAGE/vio_live"
cp "$REPO/src/capture_session.py" "$REPO/src/cli.py" "$REPO/src/flight_config.py" "$REPO/src/imu_device.py" "$REPO/src/camera.py" "$REPO/src/flight_supervisor.py" "$REPO/src/mavlink_bridge.py" "$TMP_STAGE/"
cat > "$TMP_STAGE/install.sh" <<'INSTALL'
#!/bin/bash
set -euo pipefail
stage=/home/luke/vio-tracking-staged
cd "$stage"
sha256sum -c SHA256SUMS
[ -f /home/luke/src/config/flight.json ] || { echo "Run a full deployment first: flight.json is missing" >&2; exit 1; }
backup=$(mktemp -d /home/luke/vio-tracking-backup-XXXXXX)
cp -a /home/luke/vio_live/vio_live "$backup/vio_live"
cp -a /home/luke/src/capture_session.py /home/luke/src/cli.py /home/luke/src/flight_config.py /home/luke/src/imu_device.py /home/luke/src/camera.py /home/luke/src/flight_supervisor.py /home/luke/src/mavlink_bridge.py "$backup/"
was_active=0
if systemctl is-active --quiet vio@luke; then was_active=1; fi
systemctl stop vio@luke
restore() {
  echo "Deployment failed; restoring $backup" >&2
  cp -a "$backup/vio_live" /home/luke/vio_live/vio_live
  cp -a "$backup/capture_session.py" "$backup/cli.py" "$backup/flight_config.py" "$backup/imu_device.py" "$backup/camera.py" "$backup/flight_supervisor.py" "$backup/mavlink_bridge.py" /home/luke/src/
  if [ "$was_active" = 1 ]; then systemctl restart vio@luke; fi
}
trap restore ERR
install -o luke -g luke -m 755 vio_live /home/luke/vio_live/vio_live
install -o luke -g luke -m 644 capture_session.py cli.py flight_config.py imu_device.py camera.py flight_supervisor.py mavlink_bridge.py /home/luke/src/
if [ "$was_active" = 1 ]; then
  systemctl start vio@luke
  sleep 3
  systemctl is-active --quiet vio@luke
fi
trap - ERR
echo "Deployed; rollback files: $backup"
sha256sum /home/luke/vio_live/vio_live /home/luke/src/capture_session.py /home/luke/src/cli.py /home/luke/src/flight_config.py /home/luke/src/imu_device.py /home/luke/src/camera.py /home/luke/src/flight_supervisor.py /home/luke/src/mavlink_bridge.py
journalctl -u vio@luke -n 12 --no-pager
INSTALL
(cd "$TMP_STAGE" && shasum -a 256 vio_live capture_session.py cli.py flight_config.py imu_device.py camera.py flight_supervisor.py mavlink_bridge.py install.sh > SHA256SUMS)
ssh viopi 'mkdir -p /home/luke/vio-tracking-staged'
scp "$TMP_STAGE/vio_live" "$TMP_STAGE/capture_session.py" "$TMP_STAGE/cli.py" "$TMP_STAGE/flight_config.py" "$TMP_STAGE/imu_device.py" "$TMP_STAGE/camera.py" "$TMP_STAGE/flight_supervisor.py" "$TMP_STAGE/mavlink_bridge.py" "$TMP_STAGE/install.sh" "$TMP_STAGE/SHA256SUMS" viopi:/home/luke/vio-tracking-staged/
ssh viopi 'set -eu; cd /home/luke/vio-tracking-staged; sha256sum -c SHA256SUMS; export LD_LIBRARY_PATH=/home/luke/vio_live/lib; deps=$(ldd ./vio_live); if printf "%s\n" "$deps" | grep -q "not found"; then printf "%s\n" "$deps"; exit 1; fi; result=$(./vio_live 2>&1 || true); printf "%s\n" "$result"; printf "%s\n" "$result" | grep -q "usage: vio_live"'
ssh -t viopi 'sudo bash /home/luke/vio-tracking-staged/install.sh'
