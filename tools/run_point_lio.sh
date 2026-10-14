#!/usr/bin/env bash
# Run Point-LIO over a recorded X2 Ultra bag and capture the estimated trajectory.
#
#   ./tools/run_point_lio.sh data/bags/walk_20261007_102515 [--rviz] [--rate 1.0]
#
# Output goes to results/<bagname>_pointlio/ (a bag of /aft_mapped_to_init + /path).

set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:-}"; shift || true
RVIZ=false; RATE=1.0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rviz) RVIZ=true; shift ;;
    --rate) RATE="$2"; shift 2 ;;
    *) echo "unknown option: $1" >&2; exit 1 ;;
  esac
done
[[ -d "$BAG" ]] || { echo "usage: $0 <bag-dir> [--rviz] [--rate R]" >&2; exit 1; }

NAME="$(basename "$BAG")"
OUT="${ROOT}/results/${NAME}_pointlio"
mkdir -p "${ROOT}/results"
rm -rf "$OUT"

# set +u: ROS setup.bash dereferences unbound variables (README Lesson 8)
set +u
source /opt/ros/humble/setup.bash
source "${ROOT}/slam_ws/install/setup.bash"
set -u

LOG="${ROOT}/results/${NAME}_pointlio.log"
echo "==> launching Point-LIO (rviz=${RVIZ}); log: ${LOG}"
ros2 launch x2_cloud_adapter x2_point_lio.launch.py rviz:=${RVIZ} > "$LOG" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -INT ${REC_PID:-0} 2>/dev/null
  sleep 2
  kill -INT $LAUNCH_PID 2>/dev/null
  sleep 2
  pkill -f pointlio_mapping 2>/dev/null
  pkill -f cloud_adapter 2>/dev/null
}
trap cleanup INT TERM EXIT
sleep 6

echo "==> recording output to ${OUT}"
ros2 bag record -o "$OUT" /aft_mapped_to_init /path > /dev/null 2>&1 &
REC_PID=$!
sleep 3

echo "==> playing ${NAME} at rate ${RATE}"
# whitelist the INPUTS only: bags recorded during live runs also contain
# /aft_mapped_to_init, and replaying it would mix old poses into the recording.
ros2 bag play "$BAG" --rate "$RATE" --topics \
  /aima/hal/sensor/lidar_chest_front/lidar_pointcloud \
  /aima/hal/sensor/lidar_chest_front/imu \
  /aima/mc/leg_odometry 2>&1 | tail -3

sleep 4
kill -INT $REC_PID 2>/dev/null; sleep 3
kill -INT $LAUNCH_PID 2>/dev/null; sleep 2
pkill -f pointlio_mapping 2>/dev/null; pkill -f cloud_adapter 2>/dev/null
trap - INT TERM EXIT

echo
echo "==> throughput: adapter published vs Point-LIO received"
echo "  adapter: $(grep -c 'clouds;' "$LOG" | tr -d ' ')00+ (last: $(grep -o '[0-9]* clouds;' "$LOG" | tail -1))"
echo "  pointlio: $(grep -o '\[rx\] clouds received by Point-LIO: [0-9]*' "$LOG" | tail -1)"
grep -E "x2_cloud_adapter:" "$LOG" | tail -1
echo "==> Point-LIO errors/warnings"
grep -Ei "error|fail|segmentation|terminate" "$LOG" | head -5 || echo "  (none)"
echo
echo "==> result"
ros2 bag info "$OUT" 2>/dev/null | grep -E "Duration|Messages|Count:" | sed 's/^/  /'
