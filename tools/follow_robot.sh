#!/usr/bin/env bash
# ON soc1: environment for tools/waypoint_follower.py, then exec it (same PID, so signals and
# the PID file refer to the python process). Vendor aimdk type support is used READ-ONLY.
set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/entry/cfg/ros_dds_configuration.xml
V=/agibot/software/common
export PYTHONPATH="$V/local/lib/python3.10/dist-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="$V/lib:$LD_LIBRARY_PATH"
export AMENT_PREFIX_PATH="$V:$AMENT_PREFIX_PATH"
[ -t 0 ] && stty -ixon 2>/dev/null   # Ctrl-S must not be able to freeze the follower's output
exec python3 -u "$(dirname "$(readlink -f "$0")")/waypoint_follower.py" "$@"
