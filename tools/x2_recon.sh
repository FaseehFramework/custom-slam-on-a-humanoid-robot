#!/usr/bin/env bash
# x2_recon.sh — READ-ONLY reconnaissance of an AgiBot X2 Ultra compute panel.
#
# Writes nothing outside /tmp. Starts no services, stops no services,
# sends no motion commands, loads no drivers.
#
# Usage (on the panel):
#   bash x2_recon.sh 2>&1 | tee /tmp/recon_$(hostname).txt
#
# Run on each panel separately:
#   10.0.1.40  RK3588
#   10.0.1.41  Jetson Orin NX   <-- most likely perception host
#   10.0.1.42  RK3588
#   192.168.4.31

set -u
TIMEOUT=10
hr() { printf '\n===== %s =====\n' "$1"; }
have() { command -v "$1" >/dev/null 2>&1; }
run() { if have "${1%% *}"; then timeout "$TIMEOUT" bash -c "$*" 2>&1 | head -80; else echo "(not installed: ${1%% *})"; fi; }

echo "x2_recon  host=$(hostname)  user=$(whoami)  date=$(date -Is)"

# ---------------------------------------------------------------- system
hr "SYSTEM"
uname -a
[ -r /etc/os-release ] && cat /etc/os-release
[ -r /etc/nv_tegra_release ] && cat /etc/nv_tegra_release   # Jetson L4T version
nproc; free -h | head -2; uptime

# ---------------------------------------------------------------- network
# LiDAR units are very often on their own Ethernet subnet. Look for an
# interface with an address unrelated to 10.0.1.x, and for neighbours on it.
hr "NETWORK INTERFACES"
run "ip -br addr"
hr "ROUTES"
run "ip route"
hr "ARP / NEIGHBOURS  (candidate sensor IPs)"
run "ip neigh"
hr "LISTENING SOCKETS"
run "ss -tulnp"

# ---------------------------------------------------------------- devices
hr "USB DEVICES"
run "lsusb"
hr "SERIAL DEVICES"
ls -l /dev/serial/by-id/ 2>/dev/null || echo "(no /dev/serial/by-id)"
ls -l /dev/tty{USB,ACM}* 2>/dev/null || echo "(no USB/ACM tty)"
hr "CAN INTERFACES"
run "ip -d link show type can"
hr "VIDEO / CAMERA NODES"
ls -l /dev/video* 2>/dev/null || echo "(none)"
hr "I2C / SPI"
ls /dev/i2c-* /dev/spidev* 2>/dev/null || echo "(none)"

# ---------------------------------------------------------------- processes
hr "TOP PROCESSES BY CPU"
ps -eo pid,ppid,pcpu,pmem,etime,cmd --sort=-pcpu | head -25
hr "PROCESSES MATCHING SENSOR / MIDDLEWARE KEYWORDS"
ps -eo pid,cmd | grep -Ei 'lidar|rslidar|robosense|livox|hesai|ouster|velodyne|unitree|slam|cartograph|nav|aimdk|agibot|ros|dds|rmw|imu|point' | grep -v grep | head -40

# ---------------------------------------------------------------- services
hr "RUNNING SYSTEMD SERVICES"
run "systemctl list-units --type=service --state=running --no-pager"
hr "ENABLED SERVICES MATCHING KEYWORDS"
systemctl list-unit-files --no-pager 2>/dev/null | grep -Ei 'lidar|agibot|aimdk|ros|nav|slam|robot' | head -30

# ---------------------------------------------------------------- containers
hr "DOCKER"
run "docker ps -a"

# ---------------------------------------------------------------- ROS
hr "ROS ENVIRONMENT"
env | grep -Ei '^(ROS|RMW|AMENT|CYCLONEDDS|FASTRTPS|FASTDDS|LD_LIBRARY)' || echo "(no ROS env in this shell)"
hr "ROS INSTALLATIONS ON DISK"
ls -d /opt/ros/* 2>/dev/null || echo "(no /opt/ros)"
ls -d /home/*/[a-z_]*_ws 2>/dev/null | head -20

# Try each ROS distro found, in its own subshell so nothing leaks.
for setup in /opt/ros/*/setup.bash; do
  [ -r "$setup" ] || continue
  distro=$(basename "$(dirname "$setup")")
  hr "ROS TOPICS — $distro"
  ( set +u; source "$setup" >/dev/null 2>&1
    if have ros2; then
      timeout "$TIMEOUT" ros2 topic list 2>&1 | head -60
      hr "ROS NODES — $distro"
      timeout "$TIMEOUT" ros2 node list 2>&1 | head -40
    elif have rostopic; then
      timeout "$TIMEOUT" rostopic list 2>&1 | head -60
      hr "ROS NODES — $distro"
      timeout "$TIMEOUT" rosnode list 2>&1 | head -40
    else
      echo "(no ros2/rostopic binary after sourcing)"
    fi )
done

# ---------------------------------------------------------------- DDS
hr "DDS CONFIG FILES"
find /etc /opt /home /usr/local -maxdepth 6 \
     \( -iname '*cyclonedds*' -o -iname '*fastdds*' -o -iname '*fastrtps*' \) \
     2>/dev/null | head -20

# ---------------------------------------------------------------- vendor stack
hr "AGIBOT / AIMDK INSTALL LOCATIONS"
find / -maxdepth 5 \( -iname '*aimdk*' -o -iname '*agibot*' \) \
     -not -path '/proc/*' -not -path '/sys/*' 2>/dev/null | head -40

hr "LAUNCH / CONFIG FILES MENTIONING LIDAR"
grep -rIl --include='*.launch' --include='*.launch.py' --include='*.yaml' --include='*.yml' --include='*.xml' --include='*.json' \
     -Ei 'lidar|rslidar|livox|hesai|ouster|velodyne|point_cloud|pointcloud' \
     /opt /etc /home /usr/local 2>/dev/null | head -40

hr "SHARED LIBRARIES MATCHING SENSOR VENDORS"
ldconfig -p 2>/dev/null | grep -Ei 'rslidar|robosense|livox|hesai|ouster|velodyne|lidar|aimdk|agibot' | head -30

# ---------------------------------------------------------------- locomotion
# READ ONLY. This only looks for the command interface; it sends nothing.
hr "LOCOMOTION / CONTROL INTERFACE CANDIDATES"
ps -eo pid,cmd | grep -Ei 'rosbridge|websocket|locomo|walk|gait|motion|control|cmd_vel|sdk' | grep -v grep | head -30
find / -maxdepth 5 \( -iname '*rosbridge*' -o -iname '*locomotion*' -o -iname '*motion_control*' \) \
     -not -path '/proc/*' -not -path '/sys/*' 2>/dev/null | head -20

hr "DONE"
echo "Saved nothing. Pipe this through tee if you want a transcript."
