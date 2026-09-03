#!/usr/bin/env bash
# make_scene.sh -- regenerate the obstacle field for a given flight altitude.
#
#   ./make_scene.sh 10       # the structured 10 m scene
#   ./make_scene.sh 4        # the low scene
#
# The field carries COLLISION geometry, so the mission altitude and the scene
# are not independent: flying the 20 m square at 4 m over the 10 m field means
# flying into objects up to 5.3 m tall, and there is one within 1 m of every
# leg. Height therefore has to be capped along the flight path, and how much
# can be left standing under the vehicle depends on the altitude.
#
# That coupling has a consequence worth stating plainly, because it inverts the
# obvious intuition. Depth diversity in frame is bounded by
#
#     alt / (alt - tallest object the vehicle can safely fly over)
#
# so flying LOWER does not put more 3D structure under the camera -- it forces
# shorter objects there. With this camera (80 deg HFOV, 640x400, f=381.3) the
# measured spread of scene depth in frame is
#
#     10 m, corridor capped at 7 m    3.0 .. 14.1 m    4.7:1
#      4 m, corridor capped at 2 m    2.0 ..  5.6 m    2.8:1
#
# and feature dwell time drops from 38 tracked frames to 15 at the same 5.8 m/s,
# because the footprint shrinks with altitude while the speed does not. Low
# flight is a harder front-end problem AND a flatter scene, which is the
# opposite of what "get closer to the obstacles" suggests.
#
# Objects inside the corridor are SHORTENED, not deleted: the camera looks
# straight down, so removing them would strip features from exactly the part of
# the image the vehicle flies over.
set -euo pipefail

ALT="${1:?usage: make_scene.sh ALTITUDE_M}"
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/../models/vio_feature_field/model.sdf"

# The mission's square-plus-diagonals, as flown by fly_sim_mission.py at
# --side 20. Symmetric under swapping north/east, so it is correct whichever
# way the gz frame maps onto NED.
PATHSPEC="0,0;20,0;20,20;0,20;0,0;20,20;0,0"

# Keep 3 m of vertical clearance under the vehicle, and never leave anything
# taller than the vehicle flies in the corridor.
CAP="$(python3 -c "print(max(0.5, min(7.0, $ALT - 3.0)))")"

echo "scene for ${ALT} m flight: corridor objects capped at ${CAP} m"
python3 "$HERE/make_feature_field.py" \
  --max-height 9.0 \
  --corridor "$PATHSPEC" \
  --corridor-halfwidth 2.0 \
  --corridor-max-top "$CAP" \
  --out "$OUT"

# The check is part of generation, not something to remember to run. It exits
# non-zero on a collision risk, so a caller that ignores it still fails loudly.
echo
python3 "$HERE/../../harness/field_clearance.py" "$OUT" "$ALT"
