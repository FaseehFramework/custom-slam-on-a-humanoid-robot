#!/usr/bin/env bash
# Deterministic Point-LIO evaluation: no DDS in the input path.
#
#   ./tools/run_point_lio_det.sh data/bags/walk_20261007_102515 [outname]
#
# Converts the bag to Point-LIO's cloud layout (once, cached as <bag>_plio), then
# runs pointlio_mapping with `bag_path` set so it reads the bag in-process and calls
# its own callbacks in strict stored order. OMP_NUM_THREADS=1 removes floating-point
# non-associativity from parallel reductions. Two runs of the same bag should now
# produce the same trajectory; if they do not, something else is still non-deterministic.

set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:-}"; OUTNAME="${2:-}"
[[ -d "$BAG" ]] || { echo "usage: $0 <bag-dir> [outname]" >&2; exit 1; }
NAME="$(basename "$BAG")"; PLIO="${BAG%/}_plio"
[[ -n "$OUTNAME" ]] || OUTNAME="${NAME}_det"
OUT="${ROOT}/results/${OUTNAME}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT}/slam_ws/install/setup.bash"
set -u

if [[ ! -d "$PLIO" ]]; then
  echo "==> converting $NAME -> $(basename "$PLIO")"
  python3 "${ROOT}/tools/convert_bag.py" "$BAG" "$PLIO" || exit 1
else
  echo "==> using cached converted bag $(basename "$PLIO")"
fi

rm -rf "$OUT"; mkdir -p "${ROOT}/results"
LOG="${ROOT}/results/${OUTNAME}.log"

echo "==> recording output"
ros2 bag record -o "$OUT" /aft_mapped_to_init > /dev/null 2>&1 &
REC_PID=$!
trap 'kill -INT ${REC_PID:-0} 2>/dev/null; kill ${NODE_PID:-0} 2>/dev/null' INT TERM
sleep 3

echo "==> deterministic replay (OMP_NUM_THREADS=1)"
OMP_NUM_THREADS=1 ros2 run point_lio pointlio_mapping --ros-args \
  --params-file "${ROOT}/slam_ws/src/x2_cloud_adapter/config/x2_ultra.yaml" \
  -p bag_path:="$PLIO" \
  -p use_imu_as_input:=false \
  -p prop_at_freq_of_imu:=true \
  -p check_satu:=true \
  -p init_map_size:=10 \
  -p point_filter_num:=1 \
  -p space_down_sample:=true \
  -p filter_size_surf:=0.25 \
  -p filter_size_map:=0.5 \
  -p cube_side_length:=1000.0 \
  -p runtime_pos_log_enable:=false \
  -p mapping.dr_mode:=${DR_MODE:-1} \
  -p mapping.dr_vel_cov:=${DR_VEL_COV:-0.01} \
  -p mapping.dr_min_dt:=${DR_MIN_DT:-0.0} \
  -p mapping.dr_max_speed:=${DR_MAX_SPEED:-1.5} \
  > "$LOG" 2>&1
NODE_RC=$?

sleep 3; kill -INT $REC_PID 2>/dev/null; sleep 3
trap - INT TERM

echo "==> replay summary"
grep -E "\[det\]|\[dr\]" "$LOG" | sed 's/^.*\(\[det\]\|\[dr\]\)/  \1/' | tail -5
echo "==> errors"
grep -Ei "error|fail|segmentation|terminate" "$LOG" | head -4 || echo "  (none)"
echo "==> result (node exit ${NODE_RC})"
ros2 bag info "$OUT" 2>/dev/null | grep -E "Duration|Count:" | sed 's/^/  /'

# Exit status must reflect the NODE, not the last pipeline above (which is 0 whenever an
# output bag exists). A crash or truncated replay has to be visible to callers such as
# tools/offset_sweep.py, or it silently drops out of / contaminates their statistics.
if [[ $NODE_RC -ne 0 ]]; then
  echo "  FAILED: pointlio_mapping exited ${NODE_RC}" >&2; exit "$NODE_RC"
fi
if ! grep -q "replay complete" "$LOG"; then
  echo "  FAILED: replay did not complete (see $LOG)" >&2; exit 3
fi
exit 0
