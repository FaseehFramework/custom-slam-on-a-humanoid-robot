#!/usr/bin/env bash
#
# record_walk.sh — capture a walking SLAM dataset from an AgiBot X2 Ultra.
#
#   you run this on your PC  ->  it starts the recorder on soc1
#   ->  you walk the robot   ->  you press ENTER (or a timer expires)
#   ->  it stops, verifies, and copies the bag to this PC.
#
# Read-only with respect to the vendor stack: it adds DDS subscribers and writes
# bags under /home/run/slam_ws on soc1. It never starts, stops or reconfigures a
# vendor process.
#
# See README.md for why each flag below is mandatory.

set -uo pipefail

# ----------------------------------------------------------------- configuration
HOST="${X2_HOST:-soc1}"
REMOTE_WS="${X2_REMOTE_WS:-/home/run/slam_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTDIR="${SCRIPT_DIR}/data/bags"

DURATION=0            # 0 = interactive (stop with ENTER)
NAME=""
KEEP_REMOTE=0
WITH_VENDOR=0
MIN_FREE_GB=5

# Topics with standard ROS 2 types. These MUST be captured.
CORE_TOPICS=(
  /aima/hal/sensor/lidar_chest_front/lidar_pointcloud   # sensor_msgs/PointCloud2  10 Hz
  /aima/hal/sensor/lidar_chest_front/imu                # sensor_msgs/Imu         200 Hz
  /aima/hal/imu/torso/state                             # sensor_msgs/Imu         500 Hz
  /aima/hal/imu/chest/state                             # sensor_msgs/Imu         500 Hz
  /aima/mc/leg_odometry                                 # nav_msgs/Odometry       167 Hz
  /tf
  /tf_static
)

# Topics with vendor aimdk_msgs types. Recorded when the prebuilt message tree
# resolves them; rosbag2 warns and skips any it cannot, without failing the run.
EXTRA_TOPICS=(
  /aima/hal/joint/leg/state                             # aimdk_msgs/JointStateArray
  /aima/hal/joint/waist/state
  /aima/mc/body_pose
  /aima/mc/locomotion/velocity
  /aima/hal/sensor/lidar_chest_front/lidar_status
)

# The point cloud alone is ~7.9 MB/s; everything together is ~12 MB/s.
EST_MB_PER_SEC=12

# ----------------------------------------------------------------- plumbing
BOLD=$'\e[1m'; RED=$'\e[31m'; GRN=$'\e[32m'; YEL=$'\e[33m'; DIM=$'\e[2m'; RST=$'\e[0m'
say()  { printf '%s\n' "${BOLD}==>${RST} $*"; }
warn() { printf '%s\n' "${YEL}WARNING:${RST} $*" >&2; }
die()  { printf '%s\n' "${RED}ERROR:${RST} $*" >&2; exit 1; }
ok()   { printf '%s\n' "  ${GRN}ok${RST}  $*"; }

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

  -d, --duration SEC   record for SEC seconds, then stop automatically.
                       Default 0 = record until you press ENTER.
  -n, --name NAME      bag name. Default: walk_<timestamp>
  -o, --outdir DIR     where to copy the bag on this PC.
                       Default: ${OUTDIR}
  -k, --keep           keep the bag on the robot after copying (default: delete)
  -V, --with-vendor    also pull the vendor drp .mcap segments covering the run
                       (raw MSOP packets; they rotate out after ~27 min)
  -h, --help           this message

Environment:
  X2_HOST       ssh host alias for the Jetson panel (default: soc1)
  X2_REMOTE_WS  workspace on the robot (default: /home/run/slam_ws)

Examples:
  ./$(basename "$0")                 # start, walk, press ENTER to stop
  ./$(basename "$0") -d 120          # record exactly 2 minutes
  ./$(basename "$0") -n straight_line_01 -V
EOF
  exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -d|--duration) DURATION="${2:-}"; shift 2 ;;
    -n|--name)     NAME="${2:-}"; shift 2 ;;
    -o|--outdir)   OUTDIR="${2:-}"; shift 2 ;;
    -k|--keep)     KEEP_REMOTE=1; shift ;;
    -V|--with-vendor) WITH_VENDOR=1; shift ;;
    -h|--help)     usage 0 ;;
    *) warn "unknown option: $1"; usage 1 ;;
  esac
done

[[ "$DURATION" =~ ^[0-9]+$ ]] || die "--duration must be a whole number of seconds"
[[ -n "$NAME" ]] || NAME="walk_$(date +%Y%m%d_%H%M%S)"
[[ "$NAME" =~ ^[A-Za-z0-9._-]+$ ]] || die "--name may only contain letters, digits, dot, dash, underscore"

REMOTE_BAG="${REMOTE_WS}/bags/${NAME}"
PIDFILE="${REMOTE_WS}/.rec.pid"
RECLOG="${REMOTE_WS}/.rec.log"

# Environment preamble for every remote ros2 invocation.
#   set +u       : ROS setup.bash dereferences unbound vars (README Lesson 8)
#   FASTRTPS...  : vendor DDS profile, interface whitelist 10.0.1.41 (Lesson 1)
#   AMENT/LD_... : makes aimdk_msgs types resolvable so EXTRA_TOPICS record
read -r -d '' RENV <<'ENVEOF' || true
set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/entry/cfg/ros_dds_configuration.xml
AIMDK=/home/agi/aimdk/src/aimdk_msgs/prebuilt_aarch64
if [ -d "$AIMDK" ]; then
  export AMENT_PREFIX_PATH="$AIMDK:${AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$AIMDK/lib:${LD_LIBRARY_PATH:-}"
fi
clean() { grep -v "\[Warning\]" | grep -v "^20[0-9][0-9]-"; }
ENVEOF

rsh() { ssh -o BatchMode=yes -o ConnectTimeout=10 "$HOST" "bash -s" -- "$@"; }

# ----------------------------------------------------------------- preflight
say "Preflight"

ssh -o BatchMode=yes -o ConnectTimeout=10 "$HOST" true 2>/dev/null \
  || die "cannot ssh to '$HOST'. Is the robot on and on the network?"
ok "ssh to $HOST"

# Every `ros2` invocation runs its own DDS discovery, and a discovery pass can come
# back empty under contention. Doing disk, process, publisher and type checks in
# separate calls made that flakiness likely. One call, one discovery pass, with a
# retry on the cloud probe.
CANDIDATES=( "${CORE_TOPICS[@]}" "${EXTRA_TOPICS[@]}" )
CLOUD_TOPIC=/aima/hal/sensor/lidar_chest_front/lidar_pointcloud

PRE="$(rsh <<EOF
${RENV}
echo "FREE_MB=\$(df --output=avail -m "${REMOTE_WS%/*}" 2>/dev/null | tail -1 | tr -d ' ')"
echo "HAL=\$(pgrep -c -f aima-hal_sensor-app-main 2>/dev/null || echo 0)"

MAP=""
for attempt in 1 2 3; do
  MAP=\$(timeout 40 ros2 topic list -t --no-daemon 2>/dev/null | clean)
  [ -n "\$MAP" ] && break
  sleep 3
done

PUBS=""
for attempt in 1 2 3; do
  PUBS=\$(timeout 25 ros2 topic info --no-daemon ${CLOUD_TOPIC} 2>/dev/null \
          | clean | awk '/^Publisher count:/{print \$3}')
  [ -n "\$PUBS" ] && break
  sleep 3
done
echo "CLOUD_PUBS=\${PUBS:-0}"

for t in ${CANDIDATES[*]}; do
  ty=\$(printf '%s\n' "\$MAP" | awk -v T="\$t" '\$1==T{gsub(/[][]/,"",\$2); print \$2; exit}')
  if [ -z "\$ty" ]; then echo "V:\$t|ABSENT|-"; continue; fi
  if ros2 interface show "\$ty" >/dev/null 2>&1; then echo "V:\$t|OK|\$ty"; else echo "V:\$t|BADTYPE|\$ty"; fi
done
EOF
)"
[[ -n "$PRE" ]] || die "preflight returned nothing from $HOST"

FREE_MB="$(sed -n 's/^FREE_MB=//p' <<<"$PRE")"; FREE_MB="${FREE_MB:-0}"
HAL="$(sed -n 's/^HAL=//p' <<<"$PRE")"; HAL="${HAL:-0}"
CLOUD_PUBS="$(sed -n 's/^CLOUD_PUBS=//p' <<<"$PRE")"; CLOUD_PUBS="${CLOUD_PUBS:-0}"
VALID="$(sed -n 's/^V://p' <<<"$PRE")"

[[ "$HAL" -ge 1 ]] || die "hal_sensor is not running on $HOST — no sensor data to record."
ok "hal_sensor running"

[[ "$CLOUD_PUBS" -ge 1 ]] \
  || die "no publisher on lidar_pointcloud after 3 attempts. Check the LiDAR and the sensor HAL."
ok "point cloud publisher present"

NEED_MB=$(( MIN_FREE_GB * 1024 ))
[[ "$DURATION" -gt 0 ]] && NEED_MB=$(( DURATION * EST_MB_PER_SEC + 2048 ))
if [[ "$FREE_MB" -lt "$NEED_MB" ]]; then
  die "only ${FREE_MB} MB free on $HOST, need ~${NEED_MB} MB. Delete old bags under ${REMOTE_WS}/bags."
fi
ok "disk: ${FREE_MB} MB free (need ~${NEED_MB} MB)"

mkdir -p "$OUTDIR" || die "cannot create $OUTDIR"
[[ -e "${OUTDIR}/${NAME}" ]] && die "${OUTDIR}/${NAME} already exists — pick another --name"
ok "output: ${OUTDIR}/${NAME}"

# ----------------------------------------------------------------- validate topics
# A topic whose typesupport library cannot be loaded does NOT get skipped politely:
# it raises "Failure in topics discovery" and aborts the discovery loop, so every
# topic not yet subscribed is silently lost. The vendor's prebuilt aimdk_msgs tree
# ships only a subset of .idl files, so this is a live hazard. `ros2 interface show`
# predicts it exactly, so validate everything up front.
say "Validating topics"

[[ -n "$VALID" ]] || die "topic validation returned nothing; is the ROS graph up on $HOST?"

RECORD_TOPICS=(); DROPPED=()
is_core() { local n="$1" c; for c in "${CORE_TOPICS[@]}"; do [[ "$c" == "$n" ]] && return 0; done; return 1; }

while IFS='|' read -r t st ty; do
  [[ -z "$t" ]] && continue
  case "$st" in
    OK) RECORD_TOPICS+=( "$t" ) ;;
    ABSENT)
        is_core "$t" && die "required topic $t is not published. Is the full stack up?"
        DROPPED+=( "$t (absent)" ) ;;
    BADTYPE)
        is_core "$t" && die "required topic $t has unloadable type $ty."
        DROPPED+=( "$t (type $ty not loadable)" ) ;;
  esac
done <<< "$VALID"

ok "${#RECORD_TOPICS[@]} topics will be recorded"
for d in "${DROPPED[@]:-}"; do [[ -n "$d" ]] && warn "dropped: $d"; done

# ----------------------------------------------------------------- start recorder
say "Starting recorder on $HOST"

rsh "$REMOTE_WS" "$NAME" <<EOF >/dev/null
${RENV}
WS="\$1"; BAGNAME="\$2"
mkdir -p "\$WS/bags"

# QoS overrides. Sensor topics here publish BEST_EFFORT; rosbag2 subscribes
# RELIABLE by default and would record ZERO messages without this, silently.
# A BEST_EFFORT reader matches both BEST_EFFORT and RELIABLE writers, so it is
# the safe default. /tf_static needs transient_local to get the latched messages.
{
  for t in ${RECORD_TOPICS[*]}; do
    case "\$t" in
      /tf_static)
        printf '%s:\n  history: keep_last\n  depth: 100\n  reliability: reliable\n  durability: transient_local\n' "\$t" ;;
      *)
        printf '%s:\n  history: keep_last\n  depth: 2000\n  reliability: best_effort\n  durability: volatile\n' "\$t" ;;
    esac
  done
} > "\$WS/qos_override.yaml"

cd "\$WS/bags"
rm -f "\$WS/.rec.pid" "\$WS/.rec.log"
setsid nohup ros2 bag record -o "\$BAGNAME" \
  --qos-profile-overrides-path "\$WS/qos_override.yaml" \
  ${RECORD_TOPICS[*]} \
  > "\$WS/.rec.log" 2>&1 < /dev/null &
echo \$! > "\$WS/.rec.pid"
EOF

sleep 6   # let subscriptions settle before reporting

SUBS="$(rsh "$REMOTE_WS" <<'EOF'
WS="$1"
grep -c "Subscribed to topic" "$WS/.rec.log" 2>/dev/null || echo 0
EOF
)"
SKIPPED="$(rsh "$REMOTE_WS" <<'EOF'
WS="$1"
grep -o "Topic '[^']*' has unknown type" "$WS/.rec.log" 2>/dev/null | sed "s/Topic '//;s/' has unknown type//" | sort -u | tr '\n' ' '
EOF
)"

RECPID="$(rsh "$REMOTE_WS" <<'EOF'
WS="$1"; cat "$WS/.rec.pid" 2>/dev/null
EOF
)"
[[ -n "${RECPID//[^0-9]/}" ]] || die "recorder failed to start; check ${REMOTE_WS}/.rec.log on $HOST"

ok "recorder pid ${RECPID}, subscribed to ${SUBS}/${#RECORD_TOPICS[@]} topics"
[[ "${SUBS:-0}" -lt "${#RECORD_TOPICS[@]}" ]] && warn "not all topics subscribed yet; check ${REMOTE_WS}/.rec.log if counts look wrong"
[[ -n "${SKIPPED// /}" ]] && warn "skipped (unresolvable type): ${SKIPPED}"

# Stop the remote recorder even if this script is interrupted.
stop_recorder() {
  rsh "$REMOTE_WS" <<'EOF' >/dev/null 2>&1
WS="$1"; PID=$(cat "$WS/.rec.pid" 2>/dev/null)
[ -n "$PID" ] && kill -INT "$PID" 2>/dev/null
for i in $(seq 1 30); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
kill -0 "$PID" 2>/dev/null && kill -TERM "$PID" 2>/dev/null
EOF
}
INTERRUPTED=0
on_int() { INTERRUPTED=1; echo; warn "interrupted — stopping recorder cleanly"; stop_recorder; exit 130; }
trap on_int INT TERM

# ----------------------------------------------------------------- the walk
echo
printf '%s\n' "${GRN}${BOLD}  RECORDING — walk the robot now.${RST}"
printf '%s\n' "${DIM}  Motors are torque-enabled; keep clear of the legs.${RST}"
if [[ "$DURATION" -gt 0 ]]; then
  printf '%s\n' "  Stopping automatically after ${DURATION}s."
  for (( i=DURATION; i>0; i-- )); do printf '\r  %-4ss remaining ' "$i"; sleep 1; done
  printf '\r%*s\r' 30 ''
else
  printf '%s' "  Press ENTER when the walk is finished... "
  read -r _
fi

# ----------------------------------------------------------------- stop + verify
say "Stopping recorder"
stop_recorder
trap - INT TERM
ok "stopped"

say "Verifying bag on robot"
INFO="$(rsh "$REMOTE_WS" "$NAME" <<'EOF'
set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1
AIMDK=/home/agi/aimdk/src/aimdk_msgs/prebuilt_aarch64
[ -d "$AIMDK" ] && export AMENT_PREFIX_PATH="$AIMDK:${AMENT_PREFIX_PATH}" && export LD_LIBRARY_PATH="$AIMDK/lib:${LD_LIBRARY_PATH:-}"
WS="$1"; BAGNAME="$2"
ros2 bag info "$WS/bags/$BAGNAME" 2>/dev/null | grep -v "\[Warning\]" | grep -v "^20[0-9][0-9]-"
EOF
)"
[[ -n "$INFO" ]] || die "could not read bag info; the recording may have failed. Check ${REMOTE_WS}/.rec.log"

DUR_S="$(sed -n 's/^Duration: *\([0-9.]*\)s.*/\1/p' <<<"$INFO" | head -1)"
printf '%s\n' "$INFO" | sed -n '/^Files:/,/^$/p' | sed 's/^/  /'

echo
printf '  %-52s %9s %10s\n' "TOPIC" "COUNT" "RATE"
CLOUD_COUNT=0
while IFS= read -r line; do
  t="$(sed -n 's/.*Topic: \([^ |]*\).*/\1/p' <<<"$line")"
  c="$(sed -n 's/.*Count: \([0-9]*\).*/\1/p' <<<"$line")"
  [[ -z "$t" || -z "$c" ]] && continue
  r="-"
  if [[ -n "$DUR_S" ]]; then r="$(awk -v c="$c" -v d="$DUR_S" 'BEGIN{if(d>0)printf "%.1f Hz",c/d; else print "-"}')"; fi
  [[ "$t" == *lidar_pointcloud ]] && CLOUD_COUNT="$c"
  printf '  %-52s %9s %10s\n' "${t:0:52}" "$c" "$r"
done < <(grep -o 'Topic: [^|]* | Type: [^|]* | Count: [0-9]*' <<<"$INFO")

echo
if [[ "${CLOUD_COUNT:-0}" -eq 0 ]]; then
  die "ZERO point clouds recorded. This is almost always a QoS mismatch —
       check that ${REMOTE_WS}/qos_override.yaml was applied (grep 'Overriding' in
       ${REMOTE_WS}/.rec.log). The bag is kept on the robot for inspection."
fi
ok "${CLOUD_COUNT} point clouds captured"

# ----------------------------------------------------------------- copy to PC
say "Copying to this PC"
scp -q -r "${HOST}:${REMOTE_BAG}" "${OUTDIR}/" || die "scp failed; bag left on robot at ${REMOTE_BAG}"

RSIZE="$(rsh "$REMOTE_BAG" <<'EOF'
du -sb "$1" 2>/dev/null | cut -f1
EOF
)"
LSIZE="$(du -sb "${OUTDIR}/${NAME}" 2>/dev/null | cut -f1)"
if [[ -n "$RSIZE" && -n "$LSIZE" && "$RSIZE" != "$LSIZE" ]]; then
  die "size mismatch after copy (robot ${RSIZE} B, local ${LSIZE} B). Bag kept on robot."
fi
ok "copied $(du -sh "${OUTDIR}/${NAME}" | cut -f1) to ${OUTDIR}/${NAME}"

# ----------------------------------------------------------------- vendor segments
if [[ "$WITH_VENDOR" -eq 1 ]]; then
  say "Pulling vendor drp segments covering the run"
  VDIR="${OUTDIR}/${NAME}_vendor"; mkdir -p "$VDIR"
  SEGS="$(rsh "$DUR_S" <<'EOF'
# segments whose mtime falls within the run window plus a segment of slack
D=$(ls -dt /agibot/data/bag/hal_sensor/aimrtbag_* 2>/dev/null | head -1)
[ -n "$D" ] && find "$D" -name '*.mcap' -newermt "-$(( ${1%.*} + 240 )) seconds" 2>/dev/null
EOF
)"
  if [[ -n "$SEGS" ]]; then
    while IFS= read -r f; do
      [[ -n "$f" ]] && scp -q "${HOST}:${f}" "$VDIR/" && echo "  pulled $(basename "$f")"
    done <<<"$SEGS"
    ok "vendor segments in ${VDIR}"
  else
    warn "no vendor segments matched the window"
  fi
fi

# ----------------------------------------------------------------- cleanup
if [[ "$KEEP_REMOTE" -eq 0 ]]; then
  rsh "$REMOTE_BAG" <<'EOF' >/dev/null 2>&1
rm -rf "$1"
EOF
  ok "removed bag from robot (use --keep to retain)"
else
  ok "bag kept on robot at ${REMOTE_BAG}"
fi

echo
printf '%s\n' "${GRN}${BOLD}Done.${RST}  ${OUTDIR}/${NAME}"
echo "  Inspect:  ros2 bag info ${OUTDIR}/${NAME}"
echo "  Replay :  ros2 bag play ${OUTDIR}/${NAME}"
