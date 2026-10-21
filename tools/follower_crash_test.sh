#!/usr/bin/env bash
# STAGE C crash test. Runs ON soc1 (piped there by `./tools/follow.sh crash`).
#
# Waits for a `go` follower to start (its PID file), then for it to start WALKING (its log
# prints "turned:"), waits CRASH_DELAY s more (default 4: past mc's ~3 s creep, so the kill
# lands at ~0.45 m/s), then SIGKILLs it. A SIGKILL leaves no chance to
# send zeros, so only mc's own command expiry can stop the robot: exactly the backstop under
# test. It prints wall-clock times for the analysis. It never publishes anything.
#
# Refuses to kill if the follower already arrived (leg too short) or exited.
PIDFILE=${X2_PIDFILE:-/tmp/x2_waypoint_follower.pid}
LOGS=${X2_LOGS:-$HOME/slam_ws}
DELAY=${CRASH_DELAY:-4.0}   # mc creeps ~3 s before walking speed: kill at speed
WAIT=${CRASH_WAIT:-120}
now() { date '+%s.%N'; }

echo "crash test: waiting up to ${WAIT} s for a follower to start (start 'go' now)"
pid=""
for _ in $(seq 1 $((WAIT * 10))); do
  p=$(cat "$PIDFILE" 2>/dev/null)
  if [ -n "$p" ] && grep -q waypoint_follower.py "/proc/$p/cmdline" 2>/dev/null; then pid=$p; break; fi
  sleep 0.1
done
[ -n "$pid" ] || { echo "crash test: no follower started; nothing done"; exit 1; }
echo "crash test: follower pid $pid; waiting for it to start walking"

log=""
for _ in $(seq 1 $((WAIT * 10))); do
  [ -d "/proc/$pid" ] || { echo "crash test: follower exited before walking; nothing done"; exit 1; }
  [ -z "$log" ] && log=$(ls -t "$LOGS"/follow_*_"$pid".log 2>/dev/null | head -1)
  if [ -n "$log" ] && grep -q "turned:" "$log"; then break; fi
  sleep 0.1
done
grep -q "turned:" "$log" 2>/dev/null || { echo "crash test: follower never started walking; nothing done"; exit 1; }
echo "crash test: walking at $(now); killing in ${DELAY} s"
sleep "$DELAY"
if grep -q -E "arrived|ARRIVED|\[abort\]|\[result\]" "$log"; then
  echo "crash test: follower already arrived or stopped; NOT killed (use a longer leg)"
  exit 1
fi
[ -d "/proc/$pid" ] || { echo "crash test: follower already exited; nothing done"; exit 1; }
t=$(now)
kill -9 "$pid"
echo "crash test: SIGKILL sent to $pid at $t  (wall clock, PTP)"
echo "crash test: no zeros were sent. mc must stop the robot by itself within ~1.3 s."
echo "            If it keeps walking: E-STOP (the stick alone may hand control back)."
