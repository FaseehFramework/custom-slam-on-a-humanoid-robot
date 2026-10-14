#!/usr/bin/env bash
# Rebuild slam_ws from scratch.
#
# slam_ws/src/point_lio_ros2 is a third-party clone and is gitignored, so any
# `git clean -xdf` removes it along with build/ install/ log/. Nothing in there is
# authored by us — this script reproduces all of it.
#
#   ./tools/bootstrap_slam_ws.sh            # clone if needed, patch, build
#   ./tools/bootstrap_slam_ws.sh --fresh    # also wipe build/install/log first

set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${ROOT}/slam_ws"
UPSTREAM="https://github.com/dfloreaa/point_lio_ros2.git"
PKG="${WS}/src/point_lio_ros2"
PATCHFILE="${ROOT}/patches/point_lio_ros2.patch"

FRESH=0
[[ "${1:-}" == "--fresh" ]] && FRESH=1

B=$'\e[1m'; G=$'\e[32m'; Y=$'\e[33m'; R=$'\e[31m'; N=$'\e[0m'
say(){ printf '%s\n' "${B}==>${N} $*"; }
ok(){  printf '  %sok%s  %s\n' "$G" "$N" "$*"; }
die(){ printf '%s\n' "${R}ERROR:${N} $*" >&2; exit 1; }

command -v colcon >/dev/null || die "colcon not found"
[[ -d /opt/ros/humble ]] || die "/opt/ros/humble not found"
[[ -d "${WS}/src/x2_cloud_adapter" ]] || die "our package ${WS}/src/x2_cloud_adapter is missing — that one is NOT reproducible from here, recover it from git"

say "Our own package"
for f in package.xml CMakeLists.txt src/cloud_adapter.cpp config/x2_ultra.yaml launch/x2_point_lio.launch.py; do
  [[ -f "${WS}/src/x2_cloud_adapter/$f" ]] || die "missing ${WS}/src/x2_cloud_adapter/$f"
done
ok "x2_cloud_adapter complete (5 files)"

say "Third-party: point_lio_ros2"
mkdir -p "${WS}/src"
if [[ -d "$PKG/.git" ]]; then
  ok "already cloned"
else
  rm -rf "$PKG"
  git clone --depth 1 "$UPSTREAM" "$PKG" >/dev/null 2>&1 || die "clone failed (network?)"
  ok "cloned $UPSTREAM"
fi

# Upstream requires pcl_ros but never includes it. Patching avoids an apt install.
if grep -qE '^\s*find_package\(pcl_ros REQUIRED\)' "$PKG/CMakeLists.txt"; then
  [[ -f "$PATCHFILE" ]] || die "patch missing: $PATCHFILE"
  ( cd "$PKG" && patch -p1 --forward --silent < "$PATCHFILE" ) \
    || die "patch failed — upstream CMakeLists.txt may have changed"
  ok "applied point_lio_ros2.patch (pcl_ros, cloud QoS, rx counter, bag replay, DR factor)"
else
  ok "patch already applied"
fi
grep -qE '^\s*find_package\(pcl_ros REQUIRED\)' "$PKG/CMakeLists.txt" \
  && die "pcl_ros requirement still present after patching"

if [[ $FRESH -eq 1 ]]; then
  say "Wiping build artifacts"
  rm -rf "${WS}/build" "${WS}/install" "${WS}/log"
  ok "removed build/ install/ log/"
fi

say "Building"
set +u
source /opt/ros/humble/setup.bash
set -u
# On the robot, cap parallelism: soc1 also runs the LiDAR HAL and perception, and a
# full-width build starves them. COLCON_WORKERS=2 is a good value there.
WORKERS_ARG=()
[[ -n "${COLCON_WORKERS:-}" ]] && WORKERS_ARG=(--parallel-workers "$COLCON_WORKERS")
( cd "$WS" && colcon build --symlink-install "${WORKERS_ARG[@]}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release ) \
  2>&1 | grep -Ev '^\s*$' | tail -8

for exe in "${WS}/install/x2_cloud_adapter/lib/x2_cloud_adapter/cloud_adapter" \
           "${WS}/install/x2_cloud_adapter/lib/x2_cloud_adapter/slam_publisher" \
           "${WS}/install/point_lio/lib/point_lio/pointlio_mapping"; do
  [[ -x "$exe" ]] || die "expected executable missing after build: $exe"
done
ok "cloud_adapter + slam_publisher + pointlio_mapping built"

echo
printf '%s\n' "${G}${B}Workspace ready.${N}"
echo "  ./tools/run_point_lio.sh data/bags/walk_20261007_102515"
