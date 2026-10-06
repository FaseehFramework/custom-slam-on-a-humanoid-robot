# AgiBot X2 Ultra — Custom SLAM

Goal: run **our own SLAM algorithm** on an AgiBot X2 Ultra, consuming the robot's
LiDAR + IMU and publishing pose estimates back into the slot the vendor stack
already expects.

This document is the result of a read-only reconnaissance of a live robot. It is
split into **What we know** (the system as it actually is) and **What we learnt**
(the traps we fell into getting there — read this one, it will save you hours).

Status: recon complete. No vendor files modified. Nothing started or stopped.
Next step: record a bag.

---

## Quick start

```bash
ssh-keygen -t ed25519 -N '' -f ~/.ssh/id_ed25519     # if you have no key
ssh-copy-id -i ~/.ssh/id_ed25519.pub run@192.168.4.31  # soc1 first
# then, with ~/.ssh/config below in place:
ssh-copy-id -i ~/.ssh/id_ed25519.pub soc0
ssh-copy-id -i ~/.ssh/id_ed25519.pub soc2
```

`~/.ssh/config`:

```
Host soc1
    HostName 192.168.4.31
    User run
    IdentityFile ~/.ssh/id_ed25519

Host soc0
    HostName 10.0.1.40
    User run
    IdentityFile ~/.ssh/id_ed25519
    ProxyJump soc1

Host soc2
    HostName 10.0.1.42
    User run
    IdentityFile ~/.ssh/id_ed25519
    ProxyJump soc1
```

Then, **on soc1**, to see the real ROS 2 graph:

```bash
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/entry/cfg/ros_dds_configuration.xml

ros2 topic list --no-daemon                                     # NOTE: --no-daemon
ros2 topic echo --no-daemon --qos-reliability best_effort --once \
  /aima/hal/sensor/lidar_chest_front/lidar_pointcloud            # NOTE: best_effort
```

Both flags are mandatory. Omitting either makes the robot look broken. See
[Lesson 1](#lesson-1-the-ros-2-daemon-was-lying-to-us) and
[Lesson 2](#lesson-2-qos-best_effort-vs-reliable).

---

# What we know

## Compute topology

Three panels, all `aarch64`, all with hostname `agi`, all logged in as user `run`.

| Panel | Role | develop0 | Other interfaces |
|---|---|---|---|
| **soc0** | Motion control | `10.0.1.40` | `ethercat0` (UP), `can0`/`can1` (DOWN), `hotspot0` `192.168.88.88` |
| **soc1** | **Perception** — Jetson Orin | `10.0.1.41` | `wifi0` `192.168.4.31`, `sensor0` `10.11.1.1/24`, `can0` (DOWN) |
| **soc2** | Audio / interaction | `10.0.1.42` | — |

- **soc1 is the only way in.** It is dual-homed: your WiFi on `wifi0`, the robot's
  internal panel LAN on `develop0`. soc0 and soc2 live on `10.0.1.0/24` and are not
  routable from your laptop — every connection to them hops through soc1.
- Each panel also has `ssh0@develop0` on `10.0.200.4x/24`, a VLAN child of develop0
  that appears to be a management network.
- **`sensor0` (`10.11.1.1/24`) is a dedicated sensor subnet** with soc1 as `.1`.
  The LiDAR is the only device on it, at `.100`.

soc1 is **JetPack 6.2** (L4T `R36.4.3`, Ubuntu 22.04 base) with **ROS 2 Humble** at
`/opt/ros/humble`. soc0 and soc2 are Ubuntu 22.04.5.

## Robot identity

From `/agibot/software/param/info/`:

```
sku              = X2 Ultra
series           = x2
hardware_version = t2d5
form             = with_casing
project_id       = lx2501_3
omp_server       = emqx.agibot.com     # vendor MQTT broker — a phone-home path
```

## Sensors

### LiDAR — RoboSense E1R

| | |
|---|---|
| Address | `10.11.1.100` on `sensor0` (MAC `08:48:57:0a:d7:ea`) |
| MSOP (point data) | UDP **6699**, 1200-byte payloads, magic `55 aa 5a a5` |
| DIFOP (device info) | UDP **7788**, 256-byte payloads |
| Firmware | `3.0.2` |
| Serial | `3031BEDF37DE` |
| TCP ports | **none open** — pure UDP streaming device, no web UI |

The port pair 6699/7788 is RoboSense-specific (Velodyne/Hesai use 2368/8308, Ouster
7502/7503, Livox the 56000s), which is how the vendor was identified before any
config file confirmed it.

Link health on `sensor0`: MTU 1500 (device max 9194), and over one session
**9.2 GB / 7.9M packets with 0 errors and 0 drops**. The existing pipeline is not
losing data and does not need jumbo frames — payloads are 1200 bytes, so nothing
fragments.

### IMUs — there are four

| Topic | Rate | Source | Notes |
|---|---|---|---|
| `/aima/hal/sensor/lidar_chest_front/imu` | — | E1R built-in | **what the SLAM uses** |
| `/aima/hal/imu/torso/state` | ~501 Hz | `soc0_hal_imu` | body IMU |
| `/aima/hal/imu/chest/state` | ~515 Hz | `soc0_hal_imu` | body IMU |
| `/aima/hal/sensor/stereo_head_front/imu` | 200 Hz | stereo head | |

The vendor SLAM uses **only the LiDAR's built-in IMU**, not the body IMUs. That is
the right call for tightly-coupled LIO — the IMU is rigidly co-located with the
LiDAR, so the extrinsic is tiny and stable. The 500 Hz body IMUs are available to
you as *additional* signal, not as a replacement.

Characterised from a live sample while stationary:

```
frame_id: lidar_imu_chest_front
orientation: (0,0,0,w=1), covariance all zeros   → 6-axis, NO orientation fusion
angular_velocity:    (0.0112, -0.0029, 0.0003) rad/s
linear_acceleration: (9.159, -0.213, 3.603) m/s²    ‖a‖ = 9.845 ≈ g
```

Two things that matter when configuring a LIO frontend:
- **Units are m/s², not g.** Several LIO configs have a flag for this; getting it
  wrong fails silently and looks like divergence.
- **6-axis, no orientation.** Your filter estimates attitude itself.
- Gravity splits between x and z (tilt ≈ 21.5°) — the chest LiDAR is angled downward.

### Cameras

| Topic namespace | What |
|---|---|
| `/aima/hal/sensor/rgbd_head_front/` | Orbbec Gemini 330 — depth, rgb, imu, camera_infos |
| `/camera/` | the same Orbbec in its native namespace |
| `/aima/hal/sensor/stereo_head_front_left` / `_right` | stereo pair, 2064×1552 |
| `/aima/hal/sensor/rgb_head_front_center` | mono |
| `/aima/hal/sensor/rgb_head_rear` | mono |

Cameras are **hardware frame-synced via PWM at 40 Hz** (`camera_external_trigger_40hz`,
`pwm_out_camera_frame_sync.sh` in the HAL's `em_run.sh`). Good news for any later
visual-inertial work.

Also present: `/aima/hal/sensor/gnss`, `/aima/hal/sensor/touch_head`,
`/aima/hal/sensor/lidar_chest_front/lidar_status` (use this for health rather than
reinventing it).

## Time synchronisation — already solved

This is usually the hardest thing to get right on a multi-computer robot, and here
it is done for you. From the process list on soc1:

```
ptp4l -i sensor0  -f .../cfg/master0_time_sync.conf -H     # soc1 is MASTER to the LiDAR
ptp4l -i develop0 -f .../cfg/slave1_time_sync.conf  -H     # soc1 is SLAVE on the panel LAN
sys_time_sync_to_phc.sh sensor0                            # system clock -> sensor0 PHC
phc_time_sync_to_sys.sh develop0:1                         # develop0 PHC -> system clock
```

So there is a single PTP chain: **something upstream on the panel LAN (almost
certainly soc0) → soc1 → the LiDAR.** All three panels and the LiDAR share one
timebase. Confirmed on the wire — PTPv2 announce/sync/follow_up on `sensor0` with
grandmaster clock identity `0xa82bddfffed77012`, which is soc1's own `sensor0` MAC
(`a8:2b:dd:d7:70:12`).

Caveat worth knowing: that master advertises `clock class 248`, `time source 0xa0`
(internal oscillator) — it is free-running, not traceable to GPS. Fine for SLAM,
which needs *relative* consistency, not absolute truth.

The HAL is configured `use_lidar_clock: false`, i.e. points are stamped with the
PTP-disciplined system clock rather than the LiDAR's internal clock.

## Software stack

Vendor software lives in **`/agibot/software/`**. There is also an SDK source tree
at **`/home/agi/aimdk/src/`**.

The middleware is **AimRT** (AgiBot's own framework), bridged to ROS 2. Topics
suffixed `pb_3Aaimdk_2Eprotocol_2E...` are AimRT URL-encoding protobuf type names
(`pb:aimdk.protocol.X`) — those carry protobuf and need vendor `.proto` files.
**The topics we care about (LiDAR, IMU, odometry, TF) are native ROS 2 types**, so
the protobuf layer can be ignored entirely.

Processes are supervised by `em-server` (HTTP API on `0.0.0.0:50080`), driven by
`/agibot/software/entry/cfg/run_agibot.yaml`.

### The nav/no_nav profile switch

There are three manifests in `entry/cfg/`:

```
802c00a8cc402381a4e2ad3c2a069129  run_agibot.yaml          <- active
c4ccba5081ef78a8899d1a6a54d3a113  run_agibot.yaml.nav
802c00a8cc402381a4e2ad3c2a069129  run_agibot.yaml.no_nav   <- identical to active
```

The robot ships running **`no_nav`**. The entire difference between the profiles is
five commented-out lines in `default_apps`:

```diff
-    # - slam              +    - slam
-    # - map_manager       +    - map_manager
-    # - perception        +    - perception
-    # - pnc               +    - pnc
-    # - auto_integrated   +    - auto_integrated
```

All five apps are fully defined in the `apps:` catalog and their binaries are on
disk. Autonomous navigation is gated by a profile switch, not by absent software.

**Two notes on this.** First, flipping it runs *AgiBot's* Faster-LIO — their paid
product, and a licensing question for the robot's owner, not a technical one.
Second, and more to the point: **it is not the path to our goal.** We want our own
algorithm in that slot. What we actually need is already running in `no_nav` —
`hal_sensor_orin` publishes the sensors regardless.

Currently active apps: `orbbec_camera`, `hal_sensor_orin`, `drp`, `ota`,
`housekeeper`, `hds_slave`, `hds_master`, `atop`, `cobridge`, `ec`, `dm`.

### The sensor HAL

`/agibot/software/hal_sensor_orin/bin/aima-hal_sensor-app-main`, config selected by
`AGIBOT_SOC_INDEX` in `em_run.sh` (soc1 = `1` → `cfg/hal_sensor_module_config_orin.yaml`).

```yaml
lidar_module:
  global:
    timeout_usec: 500000
    pcap_path: "/data/lidar_capture.pcap"
    use_lidar_clock: false
    data_source: 0          # 0 = online lidar, 1 = rosbag, 2 = pcap
  modules:
    - id: "chest_lidar"
      type: E1R
      ip_address: "10.11.1.100"
      msop_port: 6699
      difop_port: 7788
      topic_namespace: "/aima/hal/sensor/lidar_chest_front"
      frame_id: "lidar_chest_front"
      imu_frame_id: "lidar_imu_chest_front"
      range_min: 0.1
      range_max: 150.0
```

Note `data_source` — the HAL can replay from a **rosbag or a pcap** instead of the
live sensor. That is a supported offline development path.

## The vendor SLAM

`/agibot/software/nav/bin/slam`, launched by
`/agibot/software/nav/scripts/slam/start_slam.sh`. Config:
`nav/config/slam/slam_config.yaml` (AimRT wiring) +
`nav/config/slam/paramsE1R.yaml` (algorithm).

```yaml
frontend_type: fasterlio     # options: lio-sam / fasterlio
enable_backend: true
enable_balm: true            # BALM bundle adjustment
enable_lidar_loop: true
ivox_grid_resolution: 0.5
ivox_nearby_type: 18         # iVox = Faster-LIO's signature structure
point_filter_num: 3
max_iteration: 5
filter_size_surf: 0.25
filter_size_map: 0.5
```

It is **Faster-LIO** with a BALM backend, LiDAR loop closure, NDT-OMP localisation
against a prior map, branch-and-bound relocalisation, and a TensorRT VPR model
(`orin_vpr_fp16.engine`). A complete, serious stack.

Operationally: the SLAM process pins its main thread to **CPU cores [5,6,7]** (of
the Orin's 8) and opens HTTP 50001 / TCP 51001 / UDP 52001.

## The integration contract

This is the whole point of the recon. The vendor's `soc0_task_manager` **already
subscribes** to `/slam/localization/odometry` and `/slam/mapping/odometry`, and with
the `no_nav` profile active **nothing publishes them** (`Publisher count: 0,
Subscription count: 1`). The slot is vacant and well-defined.

**Subscribe:**

| Topic | Type |
|---|---|
| `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud` | `sensor_msgs/msg/PointCloud2` |
| `/aima/hal/sensor/lidar_chest_front/imu` | `sensor_msgs/msg/Imu` |
| `/aima/mc/leg_odometry` | `nav_msgs/msg/Odometry` |

**Publish:**

| Topic | Type |
|---|---|
| `/slam/localization/odometry` | `nav_msgs/msg/Odometry` |
| `/slam/mapping/odometry` | `nav_msgs/msg/Odometry` |
| `/slam/lidar_odom` | `nav_msgs/msg/Odometry` |
| `/tf` | `tf2_msgs/msg/TFMessage` |
| `/heartbeat/loc_state` | liveness (MQTT + ROS 2) |

`/aima/mc/leg_odometry` has a live publisher (`mc_ros2_node`) but produces nothing
while the robot is idle — leg odometry only flows once the controller is actually
walking. This has not yet been observed in motion.

### Point cloud format

```
x          float32   offset 0
y          float32   offset 4
z          float32   offset 8
intensity  float32   offset 16
ring       uint16    offset 20
timestamp  float64   offset 24     <- per-point timestamp
point_step 32   width ~24583   height 1   is_dense true   frame_id lidar_chest_front
```

This is the Velodyne-style layout FAST-LIO / Faster-LIO consume natively, and it
**includes a per-point timestamp**, so motion deskewing is available. ~786 KB per
message.

## Factory calibration — do not re-derive this

From `paramsE1R.yaml`, the `mapping:` block (see Lesson 5 before using any other
block in that file):

```yaml
# T_il : lidar -> (the LiDAR's own) IMU
extrinsic_T: [ 0.00427, -0.01575, -0.01121 ]
extrinsic_R: [  0.005948, -0.999979,  0.002737,
                0.000525,  0.002741,  0.999960,
               -0.999982, -0.005946,  0.000542 ]
extrinsic_est_en: false        # vendor trusts it enough not to re-estimate online

# IMU noise (comments indicate kalibr-derived)
acc_cov:   0.1       gyr_cov:   0.01
b_acc_cov: 0.0001    b_gyr_cov: 0.0001

# T_odom_lidar : leg-odometry frame -> lidar
Tol: [ 1, 0, 0, 0.094027,
       0, 0, 1, 0.0,
       0,-1, 0, 0.18159,
       0, 0, 0, 1 ]
```

`extrinsic_R` is a near-perfect axis permutation (≈ `[0,-1,0; 0,0,1; -1,0,0]`) with
sub-degree corrections — what a real factory calibration looks like.

**Independent cross-check:** the HAL reads an extrinsic from the device itself over
DIFOP and logs it at startup:

```
parser_e1r.cpp:360  IMU TF Static Transform: t(-0.011, 0.004, 0.016),
                    q(0.500910, -0.501071, -0.498764, 0.499250)
```

That quaternion is ≈ the same axis permutation. Two independent sources agree.

Other assets:
- URDF: `/agibot/data/info/etc/body/basic/urdf/model.urdf_1`
- VPR model: `nav/config/slam/orin_vpr_fp16.engine`
- Camera intrinsics (QR module): `fx 903.52, fy 903.94, cx 656.0, cy 366.0` (1280×720)

## DDS configuration

Fast DDS, **domain 0**, discovery multicast `239.255.0.1:7400`, user traffic on
`7410–7433+`. Profile: `/agibot/software/entry/cfg/ros_dds_configuration.xml`.

```xml
<interfaceWhiteList>
  <address>10.0.1.41</address>      <!-- develop0 ONLY -->
</interfaceWhiteList>
<!-- SHM segment 33554432, UDP send/recv buffers 10388608 -->
```

**Consequence for your workflow:** vendor publishers advertise only on `develop0`.
Your laptop on WiFi (`192.168.4.x`) **cannot join this DDS graph**. So:

- develop and run nodes **on soc1**,
- record bags on soc1 and `scp` them off for offline algorithm work.

(A route to `10.0.1.0/24` via soc1 could change this later. Not needed yet.)

---

# What we learnt

These are the things that cost us time. Each one produced a *confidently wrong*
conclusion before it was understood.

## Lesson 1: the ROS 2 daemon was lying to us

**Symptom.** `ros2 topic list` showed 106 topics. None of them was a point cloud.
`ros2 node list` showed nodes from soc0 and soc2 — but **no soc1 node at all**,
even though soc1's `hal_sensor_orin` was clearly running.

**The wrong conclusion we reached.** That the HAL was publishing only on AimRT's
shared-memory backend and never bridging the cloud to ROS 2 — and therefore that we
would have to sniff UDP 6699 with libpcap or stop the vendor HAL and bind the port
ourselves. We went some distance down this path, reading HAL channel configs and
comparing them against soc2's, looking for a `pub_topics_options` rule that was
filtering the cloud out. There was no such rule. The configs were identical in
shape. That was the clue that the premise was wrong.

**Evidence that contradicted it.** The HAL's own log showed a clean handshake with
the LiDAR:

```
parser_e1r.cpp:206  Lidar Software Version: 3.0.2
parser_e1r.cpp:280  Lidar Serial Number: 3031BEDF37DE
```

and `ss -ulnp` showed the HAL process holding **both** UDP 6699 and 7788. A process
that owns the sockets, parses DIFOP, and logs no errors after startup is not a
process that has failed to publish.

**The actual cause.** `ros2 topic list` and `ros2 node list` query a background
**daemon** that caches the graph. Ours had been started in an earlier session, the
robot's stack had since restarted, and the daemon was serving a stale, partial view.

**The fix.**

```bash
ros2 daemon stop
ros2 topic list --no-daemon
```

Topic count went **106 → 180** immediately, and every LiDAR topic appeared:

```
/aima/hal/sensor/lidar_chest_front/lidar_pointcloud
/aima/hal/sensor/lidar_chest_front/lidar_pointcloud_down_sampling
/aima/hal/sensor/lidar_chest_front/lidar_raw_data
/aima/hal/sensor/lidar_chest_front/lidar_status
/aima/hal/sensor/lidar_chest_front/imu
```

There was never a blocker. Everything was published, in native ROS 2 types, the
whole time.

**Rule: always pass `--no-daemon` on this robot.** The vendor stack restarts apps
independently of your shell, so the daemon's view goes stale constantly. Also set
`FASTRTPS_DEFAULT_PROFILES_FILE` to the vendor profile so you are looking at the
same DDS configuration they are.

**Transferable heuristic:** when a tool reports absence, verify the absence with a
tool that works differently before theorising about *why* the thing is missing. The
kernel (`ss`), the process table (`ps`) and the application's own log all disagreed
with `ros2 topic list`. Three to one should have ended the argument sooner.

## Lesson 2: QoS BEST_EFFORT vs RELIABLE

**Symptom.** With the daemon problem fixed and the topics visible,
`ros2 topic hz <pointcloud>` printed **nothing at all**. Not a low rate — no output.

**The trap.** Sensor topics here are published **BEST_EFFORT**. `ros2 topic hz`,
`ros2 topic echo` and `ros2 bag record` all default to **RELIABLE**. A RELIABLE
subscriber and a BEST_EFFORT publisher are **incompatible** under DDS: they never
match, so your subscriber receives exactly zero messages and reports no error. It
looks identical to a dead sensor.

**The fix — add explicit QoS:**

```bash
ros2 topic echo --no-daemon --qos-reliability best_effort --once \
  /aima/hal/sensor/lidar_chest_front/lidar_pointcloud
```

which immediately returned a real message.

**Diagnostic worth remembering:** `ros2 topic info -v` reports
`Publisher count: 1` *regardless* of QoS compatibility, because it reads discovery
data rather than subscribing. So:

> `topic info` shows a publisher **but** `topic hz` shows nothing
> ⇒ suspect QoS mismatch, not a dead publisher.

**This is a live hazard for the next step.** `ros2 bag record` with default QoS will
produce a bag containing **zero point clouds**, and will do so without warning. A
QoS override file is mandatory for recording.

## Lesson 3: Fast DDS floods stdout

Every `ros2` invocation emits hundreds of lines like:

```
2026-10-05 17:22:15.541 [Warning] [38080] [RTPS_WRITER] Adding reader ...
```

These drowned the first `ros2 topic list` entirely — we got a wall of DDS internals
and no topic list. They are interleaved with real output, so filter them:

```bash
ros2 topic list --no-daemon 2>/dev/null | grep -v '\[Warning\]' | grep -v '^20[0-9][0-9]-'
```

## Lesson 4: `sudo` does not apply to shell redirects

This failed with `Permission denied` despite passwordless sudo:

```bash
sudo -n tr '\0' '\n' < /proc/2682/environ      # WRONG
```

The **shell** opens the file for `<`, as your user, before `sudo` ever runs. Correct:

```bash
sudo -n cat /proc/2682/environ | tr '\0' '\n'  # RIGHT
```

`sudo -n` is passwordless for `run` on soc1, which makes `tcpdump`, reading other
users' `/proc` entries, and `ss -ulnp` process attribution all available.

## Lesson 5: `paramsE1R.yaml` contains dead configuration

The vendor's algorithm config is a working file with substantial leftover material.
**Reading the wrong block will give you plausible but wrong extrinsics.**

- The **`mapping:`** block is live — it is what `frontend_type: fasterlio` consumes.
- The **`agi_sam:`** block belongs to the *LIO-SAM* frontend, which is **not in use**.
  It carries its own `extrinsicTrans`/`extrinsicRot` that differ from the live values
  **by a factor of 10** (`0.000427` vs `0.00427`). Easy to grab by mistake.
- `sensor: mid360` in that block refers to a Livox MID-360, not this robot's E1R.
  (A `paramsLivoxIMU.yaml` sits alongside — the stack supports both vendors.)
- Other leftovers: `/home/ccx/Downloads/...` paths, and `/ranger_base_node/odom`,
  which is an AgileX Ranger wheeled base — a different platform entirely.

## Lesson 6: network recon told us the architecture before any file did

Worth repeating as method. Before reading a single config, `ip -br addr` on each
panel established the whole division of labour:

- soc0 had `ethercat0` UP → actuator bus → motion control.
- soc1 had a `sensor0` subnet with one ARP neighbour → perception.
- soc2 had nothing but the panel LAN → neither.

And the LiDAR vendor was identified from **UDP port numbers and a 4-byte payload
magic** (`6699`/`7788`, `55 aa 5a a5`) before any file confirmed `type: E1R`.
On an unfamiliar robot, the wire is often more honest than the filesystem.

## Lesson 7: check uptime before trusting yesterday's notes

The robot was rebooted between sessions (visible as `etime` in `ps`, and as a fresh
timestamp on `/agibot/log/latest/`). PIDs, the DDS graph and the daemon cache all
changed. Startup errors in the HAL log —

```
[Error][4542][lidar_parser.cpp:322] publish timeout
[Error][4543][lidar_parser.cpp:320] receive timeout
```

— looked alarming until we noticed every one was stamped within 2 seconds of
startup and **nothing had been written to the log in the 13 minutes since**. They
were transient startup noise, self-recovered. Check whether a log is still *growing*
before acting on what it says.

---

# Open questions

- **Point cloud rate.** `ros2 topic hz` never reported one even with correct QoS.
  Needs measuring by timing message counts directly.
- **`/aima/mc/leg_odometry` in motion.** Advertised, publisher present, silent while
  idle. Unverified while walking.
- **QoS of the `/slam/*` topics** we need to publish — must match what
  `soc0_task_manager` expects, or it will not receive our poses (Lesson 2, in reverse).
- **Disk space on soc1** for bag recording (~0.5–1 GB/min at 786 KB/scan).
- **`em-server` HTTP API** on `0.0.0.0:50080` — presumably how apps are started and
  stopped cleanly. Undocumented so far.
- **Vendor cloud coupling.** `emqx.agibot.com`, the MQTT plugin wired into the SLAM
  module itself, `soc0_cloud_proxy`, `cobridge`. Extent of any backend dependency is
  not yet mapped.

# Safety notes

- The robot's motors are **torque-enabled and damped**, not free. Anything that
  stops or restarts motion-control modules can cause physical movement. All recon to
  date has been strictly read-only for exactly this reason.
- Keep our work out of `/agibot/` (vendor-managed, and OTA-updated). Prefer
  `/home/agi/` or `/data`.
- No vendor file has been modified. The only changes made anywhere are an SSH
  `authorized_keys` entry on each panel and a local `~/.ssh/config` on the dev laptop.
