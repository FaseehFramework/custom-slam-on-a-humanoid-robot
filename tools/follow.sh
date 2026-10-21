#!/usr/bin/env bash
# Waypoint follower, from the laptop. Syncs the three follower files to soc1, then runs there
# on a terminal (ssh -tt): Ctrl-C (or Ctrl-\) here reaches the follower -> zeros -> stop.
#
#   ./tools/follow.sh mark NAME [--force]     save the current pose as a waypoint (no motion)
#   ./tools/follow.sh list                    saved waypoints
#   ./tools/follow.sh plan A B C [--from S]   preflight + legs (no motion)
#   ./tools/follow.sh shadow A B C [--from S] operator drives; follower computes, publishes NOTHING
#   ./tools/follow.sh go --from S A B C [--align-final] [--confirm]   WALKS THE ROBOT (~0.45 m/s)
#                                             (S = the waypoint the robot is standing on)
#   ./tools/follow.sh stop                    SIGINT to a running follower (from another terminal)
#   ./tools/follow.sh crash                   STAGE C TEST ONLY, start BEFORE `go`, in a 2nd terminal:
#                                             SIGKILLs the follower 4 s after it starts walking
#                                             (no zeros possible: tests mc's own command expiry)
#
# Options before the subcommand: --map NAME (default map01). Out of WiFi range: X2_HOST=soc1ap.
# Losing the laptop link does NOT stop a running `go`: the controller stick does (and the e-stop).
set -eo pipefail
HOST="${X2_HOST:-soc1}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE=/home/run/x2_ultra_slam
if [[ "${1:-}" == stop ]]; then
  ssh "$HOST" bash -s <<'REMOTE_EOF'
pid=$(cat /tmp/x2_waypoint_follower.pid 2>/dev/null) || { echo "no follower running"; exit 0; }
if grep -q waypoint_follower.py "/proc/$pid/cmdline" 2>/dev/null; then
  kill -INT "$pid" && echo "sent SIGINT to follower pid $pid (it sends zeros and exits)"
else
  echo "stale pid file; no follower running"
fi
REMOTE_EOF
  exit 0
fi
if [[ "${1:-}" == crash ]]; then
  ssh "$HOST" bash -s < "${ROOT}/tools/follower_crash_test.sh"
  exit $?
fi
# loco_cmd_test.py too: the two motion tools exclude each other only if BOTH robot copies take
# /tmp/x2_motion.lock
rsync -az --checksum "${ROOT}/tools/follower_core.py" "${ROOT}/tools/waypoint_follower.py" \
      "${ROOT}/tools/follow_robot.sh" "${ROOT}/tools/loco_cmd_test.py" "${HOST}:${REMOTE}/tools/"
exec ssh -tt -o ServerAliveInterval=2 -o ServerAliveCountMax=3 "$HOST" "${REMOTE}/tools/follow_robot.sh" "$@"
