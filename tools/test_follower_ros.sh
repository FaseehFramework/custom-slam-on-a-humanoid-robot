#!/usr/bin/env bash
# End-to-end ROS test of the waypoint follower, ISOLATED on this laptop: ROS_LOCALHOST_ONLY=1,
# a private domain, no vendor DDS profile, and the laptop-only fake aimdk_msgs (built here).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
if [[ ! -f "$ROOT/test_ws/install/setup.bash" ]]; then
  ( cd "$ROOT/test_ws" 2>/dev/null || { mkdir -p "$ROOT/test_ws"; cd "$ROOT/test_ws"; }
    colcon build --base-paths "$ROOT/tools/test_fixtures" --packages-select aimdk_msgs >/dev/null )
fi
source "$ROOT/test_ws/install/setup.bash"
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=87
exec python3 -u "$ROOT/tools/test_follower_ros.py" "$@"
