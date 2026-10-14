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

set -uo pipefail
HOST="${X2_HOST:-soc1}"
REMOTE=/home/run/x2_ultra_slam
LOG=/tmp/live_slam.log
PUBLISH=false
RECORD=""
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
        REC_OFF) bad "NO recording -- the walk would NOT be saved" ;;
        *) [[ -n "$line" ]] && echo "     ${line##*laserMapping]: }" ;;
      esac
    done
  ;;
stop)
  say "Stopping live SLAM on ${HOST}"
  ssh "$HOST" bash -s <<'EOF' 2>&1 | while read -r line; do
set +u
pkill -INT -f "[r]os2 launch x2_cloud_adapter" 2>/dev/null
sleep 3
pkill -f "[p]ointlio_mapping" 2>/dev/null
pkill -f "[s]lam_publisher" 2>/dev/null
pkill -f "[c]loud_adapter" 2>/dev/null
sleep 2
if pgrep -f "[p]ointlio_mapping|[s]lam_publisher|[c]loud_adapter" >/dev/null; then
  echo STRAYS; pgrep -af "[p]ointlio_mapping|[s]lam_publisher|[c]loud_adapter"
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
