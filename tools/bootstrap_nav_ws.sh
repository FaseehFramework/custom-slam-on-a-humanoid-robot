#!/usr/bin/env bash
# Rebuild nav_ws: FASTLIO2_ROS2's interface + pgo + localizer (patched) and our x2_nav.
#
#   ./tools/bootstrap_nav_ws.sh            # copy + patch + build (laptop: everything)
#   ./tools/bootstrap_nav_ws.sh --fresh    # also wipe build/install/log
#   ./tools/bootstrap_nav_ws.sh --robot    # on soc1: localizer sources + x2_nav only (no GTSAM)
#
# Kept OUT of slam_ws on purpose: slam_ws is also built on the robot (aarch64), and pgo
# needs GTSAM (`sudo apt install ros-humble-gtsam`). Their fastlio2 and hba packages are
# never built (Livox-only input, x10 accel, Sophus). FASTLIO2_ROS2-main/ stays pristine;
# our changes live in patches/fastlio2_ros2.patch.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${ROOT}/FASTLIO2_ROS2-main"; NAV="${ROOT}/nav_ws"
[[ -d "$SRC" ]] || { echo "missing $SRC" >&2; exit 1; }
ROBOT=false
for a in "$@"; do
  case "$a" in
    --fresh) rm -rf "$NAV"/{build,install,log} ;;
    --robot) ROBOT=true ;;
  esac
done

rm -rf "$NAV/src"; mkdir -p "$NAV/src"
for p in interface pgo localizer; do cp -r "$SRC/$p" "$NAV/src/"; done
( cd "$NAV/src" && patch -p1 --forward < "${ROOT}/patches/fastlio2_ros2.patch" )

set +u; source /opt/ros/humble/setup.bash; set -u
cd "$NAV"
if $ROBOT; then
  PKGS="x2_nav"; EXES="x2_nav/localizer_live x2_nav/localizer_offline"
else
  PKGS="interface pgo localizer x2_nav"
  EXES="pgo/pgo_node localizer/localizer_node x2_nav/pgo_offline x2_nav/localizer_offline x2_nav/localizer_live"
fi
colcon build --base-paths "$NAV/src" "${ROOT}/nav_src" \
  --packages-select $PKGS \
  --parallel-workers "${COLCON_WORKERS:-4}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DFASTLIO2_PGO_DIR="$NAV/src/pgo" -DFASTLIO2_LOC_DIR="$NAV/src/localizer"
for exe in $EXES; do
  [[ -x "$NAV/install/${exe%%/*}/lib/$exe" ]] || { echo "MISSING $exe" >&2; exit 1; }
done
echo "nav_ws OK: source $NAV/install/setup.bash"
