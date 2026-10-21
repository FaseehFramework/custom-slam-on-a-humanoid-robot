#!/usr/bin/env bash
# Deploy the live localizer to soc1 and build it THERE (aarch64). No GTSAM needed: pgo
# stays on the laptop, only localizer_live/localizer_offline are built on the robot.
#
#   ./tools/deploy_nav_robot.sh                         # map01 from results/map01/map.pcd
#   ./tools/deploy_nav_robot.sh NAME path/to/map.pcd    # another map, as maps/NAME.pcd
#
# Writes only under /home/run/x2_ultra_slam (never /agibot). Read-only for the vendor stack.
set -eo pipefail
HOST="${X2_HOST:-soc1}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE=/home/run/x2_ultra_slam
MAP_NAME="${1:-map01}"; MAP_SRC="${2:-${ROOT}/results/map01/map.pcd}"
[[ -f "$MAP_SRC" ]] || { echo "no map at $MAP_SRC" >&2; exit 1; }

echo "==> syncing sources + map to ${HOST}:${REMOTE}"
ssh "$HOST" "mkdir -p ${REMOTE}/FASTLIO2_ROS2-main ${REMOTE}/maps ${REMOTE}/patches ${REMOTE}/tools"
rsync -az --delete "${ROOT}/nav_src/" "${HOST}:${REMOTE}/nav_src/"
rsync -az --delete "${ROOT}/FASTLIO2_ROS2-main/interface" "${ROOT}/FASTLIO2_ROS2-main/pgo" \
      "${ROOT}/FASTLIO2_ROS2-main/localizer" "${HOST}:${REMOTE}/FASTLIO2_ROS2-main/"
rsync -az "${ROOT}/patches/fastlio2_ros2.patch" "${HOST}:${REMOTE}/patches/"
rsync -az "${ROOT}/tools/bootstrap_nav_ws.sh" "${HOST}:${REMOTE}/tools/"
rsync -az "$MAP_SRC" "${HOST}:${REMOTE}/maps/${MAP_NAME}.pcd"

echo "==> live SLAM config on the robot vs here (localizer needs /cloud_registered)"
L=$(md5sum < "${ROOT}/slam_ws/src/x2_cloud_adapter/config/x2_ultra.yaml" | cut -d' ' -f1)
R=$(ssh "$HOST" "md5sum < ${REMOTE}/slam_ws/src/x2_cloud_adapter/config/x2_ultra.yaml" | cut -d' ' -f1)
if [[ "$L" == "$R" ]]; then echo "  ok  x2_ultra.yaml identical"
else echo "  !!  x2_ultra.yaml DIFFERS on the robot -- check scan_publish_en before a live run"; fi

echo "==> building on ${HOST} (nice, 2 workers)"
ssh "$HOST" bash -s -- "$REMOTE" <<'REMOTE_EOF'
set +u
cd "$1" && COLCON_WORKERS=2 nice -n 10 ./tools/bootstrap_nav_ws.sh --robot 2>&1 | grep -E "error|Summary|OK|MISSING|NOT built"
REMOTE_EOF
