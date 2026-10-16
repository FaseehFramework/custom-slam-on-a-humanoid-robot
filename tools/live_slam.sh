#!/usr/bin/env bash
# Start / stop live SLAM on the robot.
#
# This is a PASSIVE observer: it reads the LiDAR and IMU and computes a pose. It never
# commands the robot. Walking with this running is no more dangerous than walking
# without it -- you drive the robot exactly as you normally would.
#
#   ./tools/live_slam.sh --start     # begin, leave running, exit
#   ./tools/live_slam.sh --status    # is it alive, and is it producing poses
#   ./tools/live_slam.sh --stop      # stop it, verify nothing is left behind
#
# --publish additionally fills the vendor's /slam/* slot. Off by default; it is the
# only part that writes to the robot's bus.
#
# --localize MAP [--init x,y,z,yaw_deg] also starts x2_nav/localizer_live against
# /home/run/x2_ultra_slam/maps/MAP.pcd: a passive observer like Point-LIO. It publishes
# /x2/localization/{odometry,status}, never /tf, never commands. --init is the rough start
# prior T_map<-camera_init (default 0,0,0,0 = "on the map's home spot, facing its way");
# it searches +-1 m / +-30 deg around it using the first 3 s of scans, so STAND STILL first.

set -uo pipefail
HOST="${X2_HOST:-soc1}"
REMOTE=/home/run/x2_ultra_slam
LOG=/tmp/live_slam.log
PUBLISH=false
RECORD=""
LOCALIZE=""
INIT="0,0,0,0"
MODE=""

B=$'\e[1m'; G=$'\e[32m'; Y=$'\e[33m'; R=$'\e[31m'; N=$'\e[0m'
say(){ printf '%s\n' "${B}==>${N} $*"; }
ok(){  printf '  %sok%s  %s\n' "$G" "$N" "$*"; }
bad(){ printf '  %sXX%s  %s\n' "$R" "$N" "$*"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --start) MODE=start; shift ;;
    --stop)  MODE=stop;  shift ;;
    --status)MODE=status;shift ;;
    --publish) PUBLISH=true; shift ;;
    --record)  RECORD="${2:?--record needs a NAME}"; shift 2 ;;
    --localize) LOCALIZE="${2:?--localize needs a MAP name}"; shift 2 ;;
    --init)    INIT="${2:?--init needs x,y,z,yaw_deg}"; shift 2 ;;
    -h|--help)
      sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done
[[ -z "$MODE" ]] && { sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 0; }

# shared remote preamble. set +u because ROS setup.bash dereferences unbound vars.
read -r -d '' RENV <<'ENVEOF' || true
set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/entry/cfg/ros_dds_configuration.xml
source /home/run/x2_ultra_slam/slam_ws/install/setup.bash >/dev/null 2>&1
ENVEOF

case "$MODE" in
start)
  say "Starting live SLAM on ${HOST} (publish_slam=${PUBLISH})"
  # NOTE: the remote script goes over stdin, never inline in the ssh command. If the
  # pattern appears in the remote shell's own command line, `pgrep -f` matches itself
  # and always reports "running", and `pkill -f` can kill the cleanup shell. The
  # [p]attern brackets are a second guard against the same thing.
  ssh "$HOST" bash -s -- "$PUBLISH" "$REMOTE" "$LOG" <<'EOF' 2>&1 | while read -r line; do
set +u
PUBLISH="$1"; REMOTE="$2"; LOG="$3"
source /opt/ros/humble/setup.bash >/dev/null 2>&1
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/entry/cfg/ros_dds_configuration.xml
source "${REMOTE}/slam_ws/install/setup.bash" >/dev/null 2>&1
if pgrep -f "[p]ointlio_mapping" >/dev/null; then echo ALREADY_RUNNING; exit 0; fi
[ -x "${REMOTE}/slam_ws/install/point_lio/lib/point_lio/pointlio_mapping" ] || { echo NOT_BUILT; exit 1; }
setsid nohup ros2 launch x2_cloud_adapter x2_point_lio.launch.py \
   rviz:=false publish_slam:="${PUBLISH}" > "${LOG}" 2>&1 < /dev/null &
sleep 20
if pgrep -f "[p]ointlio_mapping" >/dev/null; then echo STARTED; else echo FAILED; tail -5 "${LOG}"; fi
EOF
      case "$line" in
        ALREADY_RUNNING) ok "already running (use --stop first to restart)" ;;
        NOT_BUILT) bad "not built on the robot - run tools/bootstrap_slam_ws.sh there" ;;
        STARTED) ok "running; survives SSH and WiFi dropping" ;;
        FAILED)  bad "failed to start - log tail follows" ;;
        *) [[ -n "$line" ]] && echo "     $line" ;;
      esac
    done
  if [[ -n "$LOCALIZE" ]]; then
    echo
    say "Starting the localizer (map ${LOCALIZE}, prior ${INIT}) -- STAND STILL for its 3 s startup"
    ssh "$HOST" bash -s -- "$REMOTE" "$LOCALIZE" "$INIT" "${RECORD:-loc}" <<'EOF' 2>&1 | while read -r line; do
set +u
REMOTE="$1"; MAP="$2"; INIT="$3"; TAG="$4"
source /opt/ros/humble/setup.bash >/dev/null 2>&1
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/entry/cfg/ros_dds_configuration.xml
source "${REMOTE}/nav_ws/install/setup.bash" >/dev/null 2>&1
BIN="${REMOTE}/nav_ws/install/x2_nav/lib/x2_nav/localizer_live"
[ -x "$BIN" ] || { echo NOT_BUILT; exit 1; }
[ -f "${REMOTE}/maps/${MAP}.pcd" ] || { echo "NO_MAP ${REMOTE}/maps/${MAP}.pcd"; exit 1; }
if pgrep -x localizer_live >/dev/null; then echo ALREADY_RUNNING; exit 0; fi
IFS=, read -r X Y Z YAW <<< "$INIT"
# ROS 2 types "[0,0,0,0]" as an INTEGER array and the node declares doubles -> it aborts.
# Always send decimals.
printf -v X '%.6f' "$X"; printf -v Y '%.6f' "$Y"; printf -v Z '%.6f' "$Z"; printf -v YAW '%.6f' "$YAW"
# the BINARY, not `ros2 run`: SIGINT to `ros2 run` does not reach the node
setsid nohup "$BIN" --ros-args -p map_path:="${REMOTE}/maps/${MAP}.pcd" \
  -p config_path:="${REMOTE}/nav_src/x2_nav/config/localizer_x2.yaml" \
  -p init:="[${X},${Y},${Z},${YAW}]" -p log_path:="/home/run/slam_ws/loc_${TAG}.csv" \
  > /tmp/x2_localizer.log 2>&1 < /dev/null &
sleep 5
if pgrep -x localizer_live >/dev/null; then echo STARTED; else echo FAILED; tail -5 /tmp/x2_localizer.log; fi
EOF
      case "$line" in
        STARTED) ok "localizer running (log /tmp/x2_localizer.log on ${HOST})" ;;
        ALREADY_RUNNING) ok "localizer already running" ;;
        NOT_BUILT) bad "localizer not built on the robot -- run tools/deploy_nav_robot.sh" ;;
        NO_MAP*) bad "map missing on the robot: ${line#NO_MAP }" ;;
        FAILED) bad "localizer failed to start - log tail follows" ;;
        *) [[ -n "$line" ]] && echo "     $line" ;;
      esac
    done
  fi
  if [[ -n "$RECORD" ]]; then
    echo
    say "Starting the recorder too (${RECORD})"
    "$(dirname "$0")/../record_walk.sh" --detach -n "$RECORD" --max 900 || {
      bad "recorder failed to start -- STOPPING SLAM so you do not walk for nothing"
      "$0" --stop
      exit 1
    }
  else
    echo
    printf '%s\n' "${Y}  WARNING: nothing is recording.${N} live_slam only RUNS SLAM;"
    echo "  its output is not saved. For a walk you want:"
    echo "      ./tools/live_slam.sh --start --record live_walk_01"
  fi
  ;;
status)
  say "Status on ${HOST}"
  ssh "$HOST" bash -s -- "$LOG" <<'EOF' 2>&1 | while read -r line; do
set +u
LOG="$1"
if pgrep -f "[p]ointlio_mapping" >/dev/null; then
  echo ALIVE
  # only when alive: a stopped run would show stale counters as if live
  grep -E "\[rx\]|\[dr\]|\[slam_pub\] relayed" "${LOG}" 2>/dev/null | tail -3
else
  echo DEAD
fi
if pgrep -x localizer_live >/dev/null; then
  echo LOC_ON
  grep -E "LOCALISED|SEARCH|TRACKING" /tmp/x2_localizer.log 2>/dev/null | tail -2
fi
# is record_walk.sh's detached recorder alive? it leaves its pid here
if [ -f /home/run/slam_ws/.rec.pid ] && kill -0 "$(cat /home/run/slam_ws/.rec.pid)" 2>/dev/null; then
  echo REC_ON
else
  echo REC_OFF
fi
EOF
      case "$line" in
        ALIVE) ok "pointlio_mapping is running" ;;
        DEAD)  bad "NOT running" ;;
        REC_ON)  ok "a bag recording IS running" ;;
        LOC_ON)  ok "localizer_live is running" ;;
        REC_OFF) bad "NO recording -- the walk would NOT be saved" ;;
        *) [[ -n "$line" ]] && echo "     ${line##*laserMapping]: }" ;;
      esac
    done
  ;;
stop)
  say "Stopping live SLAM on ${HOST}"
  ssh "$HOST" bash -s <<'EOF' 2>&1 | while read -r line; do
set +u
pkill -INT -x localizer_live 2>/dev/null   # exact process name: cannot match this shell
pkill -INT -f "[r]os2 launch x2_cloud_adapter" 2>/dev/null
sleep 3
pkill -f "[p]ointlio_mapping" 2>/dev/null
pkill -f "[s]lam_publisher" 2>/dev/null
pkill -f "[c]loud_adapter" 2>/dev/null
sleep 2
if pgrep -f "[p]ointlio_mapping|[s]lam_publisher|[c]loud_adapter" >/dev/null || pgrep -x localizer_live >/dev/null; then
  echo STRAYS; pgrep -af "[p]ointlio_mapping|[s]lam_publisher|[c]loud_adapter|[l]ocalizer_live"
else
  echo CLEAN
fi
EOF
      case "$line" in
        CLEAN)  ok "stopped; nothing left running" ;;
        STRAYS) bad "processes still alive:" ;;
        *) [[ -n "$line" ]] && echo "     $line" ;;
      esac
    done
  ;;
esac
