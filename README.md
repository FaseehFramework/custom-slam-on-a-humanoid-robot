# AgiBot X2 Ultra — Custom SLAM

Goal: run **our own SLAM algorithm** on an AgiBot X2 Ultra, consuming the robot's
LiDAR + IMU and publishing pose estimates back into the slot the vendor stack
already expects.

This document is the result of a read-only reconnaissance of a live robot. It is
split into **What we know** (the system as it actually is) and **What we learnt**
(the traps we fell into getting there — read this one, it will save you hours).

Status: **Point-LIO running on real X2 Ultra data.** ~0.097 m drift over 120 s of walking
(~0.5 %), 1 mm vertical drift. Recon complete, walking dataset captured and verified. No
vendor files modified, nothing started or stopped on the robot.

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
| `/aima/hal/sensor/lidar_chest_front/imu` | **200 Hz** | E1R built-in | **what the SLAM uses** |
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

### Walking dynamics (measured, 2026-10-06)

Measured from the LiDAR IMU over a ~2 minute walk, against our static baseline.
**This is the single most important measurement for algorithm selection.**

| | **Walking** | **Static** |
|---|---|---|
| `\|accel\|` mean | 9.933 m/s² | 9.840 m/s² |
| `\|accel\|` **sd** | **2.424** | **0.004** |
| `\|accel\|` min / max | **2.581 / 29.597** | 9.825 / 9.854 |
| `\|gyro\|` max | **3.080 rad/s** (176 °/s) | 0.014 |
| Δ`\|accel\|` per sample, max | **9.542 m/s²** | 0.010 |
| samples >5 m/s² off gravity | **5.37 %** | 0.00 % |

Interpretation:

- **Variance rises ~600×.** The static IMU is near-noiseless (sd 0.004 m/s²), so
  essentially all of the walking variance is real motion, not sensor noise.
- **The range spans 0.26 g to 3.02 g.** The 2.58 m/s² minimum is the chest going
  near-ballistic between footfalls; the 29.6 m/s² peaks are the impacts.
- **Mean gravity is preserved** (9.93 vs 9.84) ⇒ the accelerometer is **not
  saturating**, so the data is trustworthy rather than clipped.
- 200 Hz IMU / 10 Hz LiDAR = **20 IMU samples per scan**, with up to 9.5 m/s² change
  between consecutive samples. Intra-scan motion is large, so **deskewing is
  mandatory**, not an optimisation.

**Caution:** peaks at 3.02 g are sampled cleanly now, but if the E1R accelerometer is
configured at ±4 g, a faster gait or a stumble would clip. Confirm the configured
range before trusting data from more dynamic runs.

Reproduce with `/home/run/slam_ws/imu_walk.py`.

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

### Making `aimdk_msgs` types usable

By default `AMENT_PREFIX_PATH` is only `/opt/ros/humble`, so **no vendor message
type resolves** and any tool touching them fails. A prebuilt ament tree fixes it:

```bash
AIMDK=/home/agi/aimdk/src/aimdk_msgs/prebuilt_aarch64
export AMENT_PREFIX_PATH="$AIMDK:$AMENT_PREFIX_PATH"
export LD_LIBRARY_PATH="$AIMDK/lib:$LD_LIBRARY_PATH"
```

**It is only a partial tree — 57 `.idl` files, not the full message set.**
Verified behaviour:

| Type | `ros2 interface show` | `.idl` present | Recordable |
|---|---|---|---|
| `aimdk_msgs/msg/JointStateArray` | OK | yes | **yes** |
| `aimdk_msgs/msg/McLocomotionVelocity` | OK | yes | **yes** |
| `aimdk_msgs/msg/ModuleInfo` | FAIL | no | **no** |
| `aimdk_msgs/msg/McBodyPose` | FAIL | no | **no** |

`ros2 interface show <type>` predicts recordability exactly, and is the cheap
preflight check — it runs locally with no DDS discovery.

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

`/aima/mc/leg_odometry` publishes at **~167 Hz even while the robot is idle and
damped** — it does not require the robot to be walking.

> **This corrects an earlier conclusion in this document's history.** We first
> recorded that leg odometry "only flows once walking", because `ros2 topic hz`
> showed nothing. That silence was our own tooling — stale daemon plus a RELIABLE
> subscriber against a BEST_EFFORT publisher (Lessons 1 and 2) — not robot state.
> Recording the topic into a bag with correct QoS produced 3589 messages in 21.5 s.
> When a measurement disagrees with your mental model of the robot, suspect the
> measurement first; here the same root cause produced two wrong conclusions.

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
message, published at **10.0 Hz**.

### Per-point timestamp semantics (measured, not assumed)

> `timestamp` is **float64 nanoseconds, relative to scan start**.
> `header.stamp` is the **scan start time**.

Measured from a recorded bag:

```
cloud 0:  header 1791266255.515525   pt ts [   498000 .. 95176000 ]
cloud 1:  header 1791266255.615349   dt from prev header = 99.824 ms
cloud 2:  header 1791266255.715364   dt from prev header = 100.015 ms
```

The point span is ~94.8e6, which is 94.8 ms in nanoseconds — one 10 Hz scan period.
This confirms `relative_cloud_pt_time: true` in `paramsE1R.yaml`.

**Integration consequence:** FAST-LIO's Velodyne handler expects a `time` field in
**seconds** relative to scan start. The conversion is one multiply:

```
time_seconds = timestamp * 1e-9
```

No absolute-epoch conversion and no sign flip. The field is also *named*
`timestamp` (float64) where FAST-LIO expects `time` (float32) and Ouster-style
drivers expect `t` (uint32), so a small republisher or a patched preprocess is
needed — but the maths is trivial.

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

### The static frame tree (from `/tf_static`)

```
base_link          -> pelvis                    identity
pelvis             -> imu_in_pelvis_link        (+0.0239, -0.0002, +0.0417)
torso_link         -> imu_in_torso_link         (-0.0249, +0.0020, +0.1059)
torso_link         -> lidar_chest_front         (+0.1026, +0.0000, +0.1816)
lidar_chest_front  -> lidar_imu_chest_front     (-0.0113, +0.0043, +0.0157)
head_pitch_link    -> imu_in_head_link          (+0.0260, -0.0001, +0.0176)
head_pitch_link    -> rgb_head_center           (+0.0684, -0.0002, +0.0500)
head_pitch_link    -> rgb_head_rear             (-0.0834, +0.0003, +0.0000)
head_pitch_link    -> rgbd_head_front           (+0.0576, -0.0112, -0.0484)
head_pitch_link    -> stereo_head_front         (+0.0680, +0.0298, +0.0500)
stereo_head_front  -> stereo_head_front_right   (-0.0594, +0.0013, -0.0010)
stereo_head_front  -> stereo_imu_head_front     identity
```

Note `pelvis` ↔ `torso_link` is **not** static — it is driven by the waist joint and
arrives on dynamic `/tf` at 48 Hz. Record `/tf` as well as `/tf_static`.

**Third independent confirmation of the LiDAR→IMU extrinsic.** Three sources, three
conventions, same transform:

| Source | Value |
|---|---|
| `/tf_static` | `(-0.0113, +0.0043, +0.0157)` |
| HAL log (read from device DIFOP) | `(-0.011, 0.004, 0.016)` |
| `paramsE1R.yaml` `extrinsic_T` | `(0.00427, -0.01575, -0.01121)` — same magnitudes, permuted axes |

One discrepancy worth noting: `/tf_static` puts `torso_link -> lidar_chest_front` at
x = **0.1026**, while `paramsE1R.yaml`'s `Tol` has x = **0.094027** (z matches exactly
at 0.1816). An 8.6 mm difference — most likely because `Tol` is odom→lidar rather than
torso→lidar, but do not assume the two frames are interchangeable.

### `leg_odometry` is already in the LiDAR IMU frame

```
header.frame_id = "leg_odom"   child_frame_id = "lidar_imu_chest_front"
```

The vendor already transforms leg odometry into the LiDAR IMU frame, so it can be fed
to a LIO backend as a DR prior **without applying any extrinsic**. Convenient, and easy
to get wrong if you assume it is at `base_link`.

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

# Datasets

Bags live in `data/bags/` (gitignored). Analyse any of them with
`tools/analyze_bag.py <bag>` — continuity, IMU dynamics, trajectory, frames, sync.

## `walk_20261007_102515` — the primary dataset

2:10 of walking, 1.2 GiB, 321,371 messages, all 10 topics. **Verified good.**

| Check | Result |
|---|---|
| Point clouds | 1297 @ **10.00 Hz**, dt sd **0.23 ms**, **0 dropped scans** |
| Cloud width | 24547–25928 (mean 25416) |
| Per-point ts span | 94.68 ms per 100 ms scan — consistent with ~5 ms inter-scan dead time |
| LiDAR IMU | 25932 @ 199.9 Hz |
| Body IMUs | 64842 / 64851 @ 500.0 Hz |
| Leg odometry | 22040 @ 169.9 Hz |
| `/tf` | 6245 @ 48.2 Hz; `/tf_static` 3 latched |
| Clock alignment | cloud/IMU/odom all start within **52 ms** — one PTP timebase, confirmed in data |

**Trajectory:** ~**19.2 m travelled** (speed-integrated) in a **5.43 × 5.45 m** area,
moving 83.2 % of samples, mean 0.148 m/s, max 0.775 m/s. It wanders out and returns:
0.74 m from start at t=60 s and **0.15 m at t=119.7 s** — a real **loop-closure
opportunity**, revisited twice.

**Walking dynamics** (matches the independent vendor-bag measurement closely, so the
numbers are reproducible):

| | LiDAR IMU | Torso IMU |
|---|---|---|
| `\|a\|` mean / sd | 9.936 / 2.370 | 9.973 / 2.465 |
| `\|a\|` min / max | 2.957 / 27.916 | 2.751 / 29.477 |
| `\|w\|` max | 3.291 rad/s | 3.756 rad/s |
| Δ`\|a\|` p99 / max | 2.983 / 9.321 | 1.944 / 5.843 |

The LiDAR IMU sees **sharper transients** than the torso IMU (Δ`|a|` p99 2.983 vs
1.944) despite similar overall variance — it is mounted closer to the impact path.
Since that is the IMU your LIO uses, treat it as the harsher of the two.

### Caution: `leg_odometry` path length is not distance travelled

Naively summing position deltas gives **64.47 m**, against 19.2 m of actual travel and
3.08 m net. That is **not** noise — downsampling from 170 Hz to 2 Hz only reduces it to
53.32 m, so it is real low-frequency motion. The cause is gait: the odometry is
expressed at the chest IMU, which physically oscillates **17.7 cm peak-to-peak
vertically at 1.79 Hz** (the step frequency).

Use the twist (`speed × time`) for distance travelled; use the pose as a DR prior,
which is what a LIO backend wants anyway.

## `office_corridor_02` — the corridor dataset

4 minutes, 2.2 GiB, 590,173 messages, recorded detached over the robot's own AP.
**Clean capture, but Point-LIO fails on it — see below.**

| Check | Result |
|---|---|
| Point clouds | 2383 @ **10.00 Hz**, dt sd **0.21 ms**, **0 dropped** |
| LiDAR IMU | 47,668 @ 200 Hz |
| Clock alignment | cloud/IMU/odom within **158 ms** |
| Extrinsics + frame tree | identical to `walk_20261007_102515` (calibration stable across sessions) |

**Coverage is much better than the first bag:**

| | `walk_20261007_102515` | `office_corridor_02` |
|---|---|---|
| XY extent | 5.4 x 5.4 m | **17.6 x 17.5 m** |
| Max distance from start | 3.9 m | **18.9 m** |
| Travelled (speed-integrated) | ~19 m | **~31 m** |
| Returns to start | 0.15 m | 1.84 m @ t=206 s |
| **Median point range** | 3.40 m | **1.92 m** |
| Max range seen | 12.8 m | 20.3 m |

The corridor is *narrower* than the room in sensor terms: median range dropped to
1.92 m (per-scan medians as low as 0.67 m) even though the sensor saw 20.3 m down the
corridor at times.

## `static_20261006_095735`

21.5 s, 179 MiB, stationary baseline. Useful as the noise floor: `|a|` sd 0.004 m/s²
against 2.4 while walking.

## `vendor_walk_20261006/`

Two vendor `drp` MCAP segments (516 MiB) covering a 10:17–10:19 walk, pulled before
the rolling buffer discarded them. Contains the 200 Hz LiDAR IMU, but the LiDAR only
as undecoded `LidarRawData` packets, and no `leg_odometry`/`/tf`. Superseded by
`walk_20261007_102515`; kept as an independent cross-check.

---

# Vendor continuous recording (`drp`)

The `drp` app runs in the **active `no_nav` profile** and continuously records
rolling MCAP bags to `/agibot/data/bag/`. This is free data for every run you do —
but it does **not** cover everything, so read the gaps before relying on it.

| Stream | Panel | Retention | Key contents |
|---|---|---|---|
| `hal_sensor` | soc1 | **4 GB** (~27 min) | `lidar_raw_data`, `lidar_chest_front/imu` @200 Hz, `touch_head`, 2× `camera_info` |
| `rt_control_link` | soc0 | **9 GB** | all joint `state`+`command` (leg/arm/waist/head/hand), **`imu/torso`**, **`imu/chest`**, `mc/rl/debug`, `mc/common/state` |
| `external_control_link` | soc0 | 1 GB | `mc/body_pose`, `mc/locomotion/velocity`, `teleop_bridge/vr_data` |
| `software` | all | 4 GB | diagnostics / process info only |

Config: `/agibot/software/drp/bin/cfg/lx2501_3/<soc>/*.yaml`
(`lx2501_3` matches `param/info/project_id`).

### What the vendor does NOT record

- **`/aima/mc/leg_odometry`** — recorded nowhere.
- **`/tf` and `/tf_static`** — recorded nowhere.
- **`PointCloud2`** — see below; only raw packets are stored.

Those three gaps are exactly why we still need our own recorder.

### Retention is a rolling buffer — copy promptly

`hal_sensor` keeps ~4 GB in 256 MB segments of ~108 s each: roughly **27 minutes of
history**. Data older than that is gone. After any run worth keeping, copy the
covering segments off immediately.

```bash
ls -la /agibot/data/bag/hal_sensor/aimrtbag_*/        # find segments by timestamp
scp soc1:/agibot/data/bag/hal_sensor/<dir>/<seg>.mcap data/bags/
ros2 bag info -s mcap <file>.mcap                     # note: -s mcap
```

### `lidar_raw_data` is raw packets, not points

```
# aimdk_msgs/msg/LidarRawData
MessageHeader header
uint8[] msop_data     # raw MSOP bytes, unparsed
uint8[] difop_data    # raw DIFOP bytes
```

~232 KB/message at 10 Hz, versus 786 KB for the decoded `PointCloud2`. To get points
back there are two routes:

1. **Replay through the vendor HAL.** `hal_sensor_module_config_orin.yaml` has
   `data_source: 1` (rosbag) alongside `0` (online) and `2` (pcap). Copy the config
   to `/home/run/slam_ws/`, point it at the bag, and run
   `aima-hal_sensor-app-main --cfg_file_path=<our copy>`. Nothing vendor-owned is
   modified. **Untested so far.**
2. **Parse MSOP ourselves** — the format is known (1200-byte payloads, magic
   `55 aa 5a a5`), but this is strictly more work than route 1.

---

# Recording bags

## Recording out of WiFi range (detached mode)

When the robot walks out of range of the office WiFi, connect the laptop to **the
robot's own access point** and record detached, so the recording survives the link
dropping entirely.

### The robot's AP

`hostapd` runs on **soc0**, serving `hotspot0` at `192.168.88.88`, DHCP
`192.168.88.100–200`. The recorder runs on **soc1**, which is on a different subnet
(`10.0.1.41`) — and **soc0 has `ip_forward=0`**, so `10.0.1.x` is *not* IP-routable
from the hotspot.

**ProxyJump works regardless**, because soc0's sshd opens the connection to soc1
itself rather than routing packets. `~/.ssh/config` has `soc0ap` / `soc1ap` / `soc2ap`
entries for this. Select them with `X2_HOST`:

```bash
X2_HOST=soc1ap ./record_walk.sh --list
```

### The walk

```bash
# 1. join the robot's AP, then start recording and walk away
X2_HOST=soc1ap ./record_walk.sh --detach -n office_corridor_01 --max 900

# 2. ... walk the robot. The link may drop; recording continues on the robot. ...

# 3. back in range (AP or office WiFi — either works)
X2_HOST=soc1ap ./record_walk.sh --list      # is it still running? how big?
X2_HOST=soc1ap ./record_walk.sh --stop      # stop it
X2_HOST=soc1ap ./record_walk.sh --fetch     # verify, copy here, delete there
```

`--fetch` with no name reads `.rec.name` from the robot, so you do not have to
remember what you called it. It refuses to delete the robot's copy unless the local
copy matches byte-for-byte, and refuses outright if the bag contains zero point clouds.

### The watchdog

`--detach` arms an auto-stop on the robot (`--max`, default 1800 s). The recorder
writes **~12 MB/s**, so an unattended run that is never stopped would eventually fill
the disk. With 383 GB free that is ~9 hours of headroom, but the watchdog means a
forgotten recording stops by itself rather than filling the disk. Verified: it fires.

Set `--max` a bit longer than you expect to walk; stopping early costs nothing.

## The easy way: `./record_walk.sh`

Run it **on your PC**. It starts the recorder on soc1, waits while you walk the
robot, stops cleanly, verifies the bag, and copies it here.

```bash
./record_walk.sh                      # start, walk, press ENTER to stop
./record_walk.sh -d 120               # record exactly 2 minutes
./record_walk.sh -n straight_line_01  # name it
./record_walk.sh -k                   # keep the bag on the robot too
./record_walk.sh -V                   # also pull the vendor drp .mcap segments
./record_walk.sh -h                   # all options
```

**Typical session:**

```
$ ./record_walk.sh -n walk_01
==> Preflight
  ok  ssh to soc1
  ok  hal_sensor running
  ok  point cloud publisher present
  ok  disk: 383000 MB free (need ~5120 MB)
==> Validating topics
  ok  10 topics will be recorded
WARNING: dropped: /aima/mc/body_pose (type aimdk_msgs/msg/McBodyPose not loadable)
==> Starting recorder on soc1
  ok  recorder pid 14812, subscribed to 10/10 topics

  RECORDING — walk the robot now.
  Motors are torque-enabled; keep clear of the legs.
  Press ENTER when the walk is finished...
```

then, after you press ENTER, it prints a per-topic count/rate table, refuses to
continue if **zero** point clouds were captured, and copies the bag to `data/bags/`.

**What it records** (10 topics, all verified working):

| Topic | Type | Rate |
|---|---|---|
| `lidar_chest_front/lidar_pointcloud` | `sensor_msgs/PointCloud2` | 10 Hz |
| `lidar_chest_front/imu` | `sensor_msgs/Imu` | 200 Hz |
| `imu/torso/state`, `imu/chest/state` | `sensor_msgs/Imu` | ~497 Hz |
| `mc/leg_odometry` | `nav_msgs/Odometry` | 168 Hz |
| `joint/leg/state`, `joint/waist/state` | `aimdk_msgs/JointStateArray` | ~497 Hz |
| `mc/locomotion/velocity` | `aimdk_msgs/McLocomotionVelocity` | 48 Hz |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 46 Hz / latched |

Safety checks it performs, each because of a real failure we hit:

- aborts if `hal_sensor` is down or the cloud has no publisher (retries 3×);
- aborts if disk is short;
- **validates every topic's type before recording** — see [Lesson 9](#lesson-9-one-unloadable-type-poisons-the-whole-recording);
- **aborts if zero point clouds landed** — the QoS trap in [Lesson 2](#lesson-2-qos-best_effort-vs-reliable);
- verifies byte-for-byte size after copying before deleting from the robot;
- stops the remote recorder even if you Ctrl-C the script.

Note `-d N` records slightly longer than N: there is a ~6 s settle before the
countdown plus drain time at the end. Harmless pre-roll.

## Doing it by hand

Our workspace on soc1 is **`/home/run/slam_ws/`** (`/home/agi` is not writable by
`run`; `/agibot/` is vendor-managed and OTA-updated — keep out of both).

```
/home/run/slam_ws/qos_override.yaml   QoS overrides (mandatory, see below)
/home/run/slam_ws/bags/               recordings
/home/run/slam_ws/ts_probe.py         per-point timestamp analysis
```

## The QoS override is not optional

`ros2 bag record` subscribes **RELIABLE** by default. Every sensor topic here is
**BEST_EFFORT**. Without an override you get a bag containing **zero point clouds**,
with no error message. `/home/run/slam_ws/qos_override.yaml`:

```yaml
/aima/hal/sensor/lidar_chest_front/lidar_pointcloud:
  history: keep_last
  depth: 200
  reliability: best_effort
  durability: volatile
/aima/hal/sensor/lidar_chest_front/imu:
  history: keep_last
  depth: 2000
  reliability: best_effort
  durability: volatile
# ... same shape for the body IMUs, leg_odometry and /tf ...
/tf_static:
  history: keep_last
  depth: 100
  reliability: reliable
  durability: transient_local      # transient_local to receive the latched messages
```

A BEST_EFFORT **reader** matches both BEST_EFFORT and RELIABLE writers, so it is the
safe default for everything except `/tf_static`, where `transient_local` is what
actually delivers the already-latched transforms. (Only `leg_odometry` is actually
published BEST_EFFORT — see the correction in Lesson 2 — but a permissive reader costs
nothing and protects against the vendor changing a QoS profile.)

Confirm the override took effect — the recorder logs one line per topic:

```
[rosbag2_recorder]: Overriding subscription profile for /aima/hal/...
```

## Command

```bash
source /opt/ros/humble/setup.bash          # NOTE: do this with `set +u`, see Lesson 8
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/entry/cfg/ros_dds_configuration.xml
cd /home/run/slam_ws/bags

ros2 bag record -o "static_$(date +%Y%m%d_%H%M%S)" \
  --qos-profile-overrides-path /home/run/slam_ws/qos_override.yaml \
  /aima/hal/sensor/lidar_chest_front/lidar_pointcloud \
  /aima/hal/sensor/lidar_chest_front/imu \
  /aima/hal/imu/torso/state \
  /aima/hal/imu/chest/state \
  /aima/mc/leg_odometry \
  /tf /tf_static
```

## Measured rates and cost

From `static_20261006_095735` (21.46 s, 30,440 messages, 179 MiB):

| Topic | Count | Rate |
|---|---|---|
| `lidar_chest_front/lidar_pointcloud` | 215 | **10.0 Hz** |
| `lidar_chest_front/imu` | 4293 | **200.0 Hz** |
| `/aima/hal/imu/torso/state` | 10672 | 497.2 Hz |
| `/aima/hal/imu/chest/state` | 10676 | 497.4 Hz |
| `/aima/mc/leg_odometry` | 3589 | **167.2 Hz** |
| `/tf` | 992 | 46.2 Hz |
| `/tf_static` | 3 | latched |

**Storage: ~8.3 MB/s ≈ 500 MB/min ≈ 30 GB/hr.** soc1 has 384 GB free, so roughly
12 hours of recording capacity — but bags should be copied off and deleted promptly.

Copying off runs at ~10.5 MB/s over WiFi (179 MiB in 17 s), marginally faster than
the generation rate. Copy after recording; do not try to stream live.

```bash
scp -r soc1:/home/run/slam_ws/bags/<name> data/bags/
```

`data/bags/` is gitignored.

---

# Choosing an algorithm

We do not need to write a SLAM frontend from scratch to get a proof of concept. The
cloud format is the Velodyne-style layout the whole FAST-LIO family consumes, and
the vendor's own choice (Faster-LIO) is open source, so their tuned parameters in
`paramsE1R.yaml` map onto a stock build.

| Candidate | Verdict |
|---|---|
| **KISS-ICP** | Best first step. ROS 2 native, LiDAR-only, no IMU, no extrinsics, minimal config. Proves the data pipeline end to end. Not the final answer. |
| **Point-LIO** | **Start here for the real PoC.** Built for exactly this regime: it drops the fixed-scan-period assumption and treats IMU readings as measurements of state rather than propagation inputs, which is what 3 g footfall transients at 200 Hz demand. See [Walking dynamics](#walking-dynamics-measured-2026-10-06) for the evidence. |
| **FAST-LIO2** | Well-proven baseline; uses the factory extrinsics and kalibr noise params directly. But its propagation assumes the IMU adequately samples motion between scans — strained here at 20 samples/scan with 3 g transients. Worth running as a comparison, not as the first choice. |
| **Faster-LIO** | What the vendor runs. Use it to reproduce their baseline and compare. |
| **LIO-SAM** | **Not compatible as-is.** Requires a 9-axis IMU with valid orientation; ours is 6-axis with `orientation = (0,0,0,1)` and zero covariance. The vendor's dead `agi_sam:` block specifies `imuType: 1 #9-axis` and they chose `fasterlio` instead — same constraint. |

Whatever is chosen needs the `timestamp` -> `time` adapter described above.

---

# Running Point-LIO

Working end-to-end on `walk_20261007_102515`. Everything lives in `slam_ws/`.

## Build

```bash
./tools/bootstrap_slam_ws.sh            # clone if needed, patch, build
./tools/bootstrap_slam_ws.sh --fresh    # also wipe build/ install/ log/ first
```

That is the whole build. It clones upstream, applies our patch, builds, and verifies
both executables exist. Takes ~40 s from nothing.

**Why a script and not instructions:** `slam_ws/src/point_lio_ros2/` is a third-party
clone and is **gitignored**, as are `build/`, `install/` and `log/`. A `git clean -xdf`
therefore deletes all of them. Nothing in there is authored by us, so it should be
reproducible with one command rather than remembered. (This already happened once.)

The script refuses to run if `slam_ws/src/x2_cloud_adapter/` is missing — **that** one
is ours and is not reproducible from upstream, so recover it from git.

**The patch exists** because upstream's `CMakeLists.txt` does
`find_package(pcl_ros REQUIRED)` while **nothing in `src/` includes `pcl_ros`**. Rather
than `apt install ros-humble-pcl-ros` (which needs root), we comment out the three
references. `patches/point_lio_ros2.patch` is a `git diff`, applied with
`-p1`. The script fails loudly if it does not apply, rather than building something
unexpected.

No other system packages are needed — `libpcl-dev`, `pcl_conversions` and
`libeigen3-dev` are already present.

### What is and is not recoverable

| Path | In git? | If deleted |
|---|---|---|
| `slam_ws/src/x2_cloud_adapter/` | **should be** | **not recoverable** — ours |
| `tools/`, `patches/`, `record_walk.sh`, `README.md` | **should be** | not recoverable |
| `slam_ws/src/point_lio_ros2/` | ignored | `./tools/bootstrap_slam_ws.sh` |
| `slam_ws/{build,install,log}/` | ignored | `./tools/bootstrap_slam_ws.sh --fresh` |
| `data/bags/`, `results/` | ignored | **not recoverable** — re-record on the robot |

`data/bags/` is gitignored and `.gitignore` also lists `*.db3`/`*.mcap`, so
`git clean -xdf` **will delete recorded bags**. They are large and cannot be
regenerated without the robot. Back them up outside the repo before any aggressive
clean.

## Run

```bash
./tools/run_point_lio.sh data/bags/walk_20261007_102515            # headless
./tools/run_point_lio.sh data/bags/walk_20261007_102515 --rviz     # with RViz
./tools/eval_trajectory.py results/<name>_pointlio data/bags/<name>
```

Output lands in `results/<bagname>_pointlio/` as a bag of `/aft_mapped_to_init`
(`nav_msgs/Odometry`) and `/path`.

## `x2_cloud_adapter` — why it is needed

Point-LIO's Velodyne handler expects `velodyne_ros::Point`, which is the **same 32-byte
stride** as the vendor cloud but with **`time` and `ring` swapped**:

```
vendor  : x,y,z(f32) intensity(f32@16) ring(u16@20)  timestamp(f64@24)
velodyne: x,y,z(f32) intensity(f32@16) time(f32@20)  ring(u16@24)
```

and the per-point value must become **float32 seconds** rather than float64
nanoseconds. `slam_ws/src/x2_cloud_adapter` does exactly that and nothing else. It
resolves field offsets **by name at runtime**, so a vendor firmware change produces a
clear error instead of silently garbled geometry.

## Config choices that are not the stock defaults

`slam_ws/src/x2_cloud_adapter/config/x2_ultra.yaml`. Four of these matter a great deal:

| Parameter | Stock | Ours | Why |
|---|---|---|---|
| `satu_acc` | 30.0 | **40.0** | Walking peaks reach **29.5 m/s²**. With `check_satu: true`, the stock value makes Point-LIO discard real footfall impacts as saturated IMU readings. We verified the IMU is not actually saturating (mean \|a\| stays at g). |
| `scan_line` | 32 | **96** | The E1R reports `ring` 0–95. Too small risks an out-of-bounds write into `is_first[layer]` on the no-offset-time path. |
| `imu_time_inte` | 0.01 | **0.005** | IMU is 200 Hz, not 100 Hz. |
| `blind` | 2.0 | **0.5** | Observed point range is 1.05–12.8 m indoors; 2.0 discards much of a small room. |
| `timestamp_unit` | 2 (µs) | **0 (s)** | The adapter emits seconds. |

Noise terms (`b_acc_cov`, `b_gyr_cov`, `gyr_cov_input`, `acc_cov_input`, `plane_thr`)
and the extrinsics are taken from the vendor's own `paramsE1R.yaml`. The vendor runs
Faster-LIO, which shares FAST-LIO's `extrinsic_T`/`extrinsic_R` convention
(LiDAR→IMU), so their numbers transfer directly.

## Results on `walk_20261007_102515`

> **Read the reproducibility section below before quoting any number from here.**
> The first two runs of the same bag with the same binaries produced wildly different
> trajectories.

| Run | Poses (of 1297 clouds) | Return-to-start | z drift at end |
|---|---|---|---|
| `--rate 2.0` | 1146 | **0.251 m** (≈0.097 m drift) | −0.001 m |
| `--rate 1.0` | 1115 | **11.701 m** (diverged) | +1.681 m |

The robot physically returned to within 0.153 m of its start at t=119.7 s, so the
first run corresponds to ~0.097 m of drift over 120 s and ~19 m travelled (~0.5 %) —
and the second to complete divergence.

**Do not compare the x/y extents of the two trajectories directly.** Point-LIO's world
frame is gravity-aligned with an arbitrary yaw at initialisation, while `leg_odom` has
its own yaw convention, so the two are rotated relative to each other. The
return-to-start metric is rotation-invariant, which is why it is the one quoted.
A proper comparison needs Umeyama alignment — not yet done.

## Reproducibility: SOLVED by a deterministic harness

**Use `./tools/run_point_lio_det.sh` for all evaluation and tuning.** Repeated runs
are now bit-identical, and it is ~9x faster than real-time replay.

```bash
./tools/run_point_lio_det.sh data/bags/walk_20261007_102515 det_1
./tools/eval_trajectory.py results/det_1 data/bags/walk_20261007_102515
```

Verified across three runs — every shared pose matches exactly:

```
det_2 vs det_3: 1287/1287 identical poses | max diff 0.000e+00 m
det_1 vs det_2: 1287/1287 identical poses | max diff 0.000e+00 m
```

### How it works

Two changes, both required:

1. **`tools/convert_bag.py`** converts the vendor cloud layout to Point-LIO's offline,
   file-to-file, with no DDS. Cached as `<bag>_plio`. This is the same transform
   `x2_cloud_adapter` does live; the adapter remains the on-robot path.
2. **A `bag_path` parameter on Point-LIO** (`patches/point_lio_ros2.patch`). When set,
   the main loop reads the bag in-process and calls its own `standard_pcl_cbk` /
   `imu_cbk` in strict stored order, one message per iteration, single threaded. No
   DDS in the input path at all. `rate.sleep()` is skipped since the loop is
   data-driven, which is where most of the speedup comes from.

`OMP_NUM_THREADS=1` in the runner also removes floating-point non-associativity from
OpenMP parallel reductions.

### What this fixed, measurably

| | DDS replay | Deterministic harness |
|---|---|---|
| Clouds fed | ~1200 of 1297 | **1297 of 1297** |
| Poses out | 1201–1238 | **1287–1292** |
| Wall time | ~130 s | **~15 s** |
| Run-to-run | **~50 % diverged** | **bit-identical** |

The DDS path was also *losing* scans even with RELIABLE QoS and deep queues — it fed
~1200 of 1297. The harness feeds all of them.

### New baseline

With deterministic input, `walk_20261007_102515` gives **1.441 m return-to-start
drift** (reference: robot physically back within 0.153 m), reproducibly. That is worse
than the best lucky DDS run (0.232 m) and better than the worst (13.2 m) — it is
simply the true value for this config, which is what makes it useful.

**Known minor gap:** the *output* is still captured with `ros2 bag record` over DDS,
so the pose count varies by a few at the tail (1292 vs 1287) depending on when the
recorder attaches. Every shared pose is identical, so this does not affect the
estimate — but writing a trajectory file directly from the node would remove the last
bit of nondeterminism from the pipeline.

## Historical: the DDS-replay investigation

Kept because the diagnostic path is instructive, and because the scan-loss and
point-density findings are real and still apply to the live on-robot pipeline.

Six runs of the same bag with the same binaries:

| Run | Rate | Transport | `point_filter_num` | `fov_degree` | Drops | Return-to-start |
|---|---|---|---|---|---|---|
| 1 | 2.0 | lossy | 4 | 180 | yes | 0.097 m |
| 2 | 1.0 | lossy | 4 | 180 | yes | **11.5 m ✗** |
| A | 1.0 | lossless | 4 | 180 | none | 1.172 m |
| B | 1.0 | lossless | 4 | 180 | none | **13.218 m ✗** |
| C | 1.0 | lossless | **1** | **123** | none | **0.232 m** |
| D | 1.0 | lossless | **1** | **123** | none | **6.045 m ✗** |

Two hypotheses were tested and **both were wrong as explanations of the variance**:

1. **Dropped scans.** Point-LIO subscribed with `SensorDataQoS()` (BEST_EFFORT,
   depth 5) and was losing ~180 of 1297 scans. Fixed — both ends are now deep and
   RELIABLE, and the `[rx]` counter confirms 1200 published / 1200 received. **Runs
   still diverge.** Worth keeping regardless: silent scan loss is never acceptable.
2. **Too few points for a narrow FOV.** `point_filter_num: 4` used only ~6.3k of
   25.2k points. Fixed (now 1). This produced the **best** result so far (0.232 m),
   so it is worth keeping — but the very next run with the identical config gave
   6.045 m.

### What the divergence actually looks like

It is a **discrete failure, not accumulating drift**. Run B tracked the reference
within ±0.1 m until t=30 s, jumped to +4.8 m by t=40 s, and never recovered:

```
 t(s) |  runA |d|  runB |d| |  ref |d| |   A-err   B-err
   20 |     3.199     3.149 |    3.245 |  -0.047  -0.096
   30 |     2.673     2.578 |    2.523 |  +0.151  +0.056
   40 |     3.987     8.663 |    3.850 |  +0.136  +4.812   <-- B breaks
```

And **that moment is not exceptional**. Per-10 s windows show t=30–40 has the *lowest*
max speed (0.396 m/s) of any moving window, and below-average peak acceleration and
yaw rate. So the trigger is not dynamics — the filter is marginal throughout and fails
at a semi-random point.

### Remaining suspects (untested)

With scan loss eliminated, the remaining non-determinism is in the replay pipeline
itself:

- **IMU/cloud interleaving.** Replaying over live DDS means the exact set of 200 Hz
  IMU samples bundled with each scan by `sync_packages` depends on thread scheduling.
  Different grouping, different estimate.
- **OpenMP floating-point non-associativity.** Parallel reductions sum in
  scheduling-dependent order. Normally this is last-bit noise, but a filter operating
  near its stability limit amplifies it.

Both are inherent to replaying through DDS. **The fix is a deterministic harness** —
read the bag in-process and feed Point-LIO's callbacks in strict timestamp order with
no DDS in the loop. That is the prerequisite for any further tuning. A cheaper
stopgap worth testing first is replaying at `--rate 0.25` or slower, so each scan's
IMU batch is fully delivered before the next scan arrives.

### Why the sensor makes this hard

Measured from the data, the E1R is **122.8° × 89.1°, forward-facing** — not a 360°
spinner — with **median range 3.40 m** (max 12.8 m) in a 5.4 × 5.4 m room. Facing a
flat wall, translation along the wall and yaw are weakly observable. Combined with 3 g
footfall transients, the estimator has little margin. This is a genuinely hard
configuration, and it argues for a larger, more feature-rich recording environment as
much as for better tuning.

## Earlier finding: the pipeline was losing scans

A single good run does not characterise a SLAM system. Replaying the same bag twice
gave 0.097 m drift once and 11.5 m the next time, which pointed at non-determinism
rather than tuning.

**Cause (suspected, under test):** Point-LIO subscribed to the cloud with
`rclcpp::SensorDataQoS()` — **BEST_EFFORT, depth 5**. At ~786 KB per cloud, a brief
stall silently drops scans. A dropped scan doubles the inter-scan motion, which on a
legged platform with 3 g transients is exactly how a filter diverges. And drops depend
on thread scheduling, so the same input gives different answers run to run.

Both ends are now deep and RELIABLE (`patches/point_lio_ros2.patch` and the adapter),
and Point-LIO logs how many clouds it actually received:

```
[rx] clouds received by Point-LIO: N
```

Compare that against the adapter's published count. **If they differ, scans are being
lost and no trajectory number from that run means anything.**

Also noted while reading `standard_pcl_cbk`: the loop-back guard is broken upstream.

```cpp
if (get_time_sec(msg->header.stamp) < last_timestamp_lidar) { ...; return; }
last_timestamp_lidar = msg->header.stamp.sec;   // only .sec — nanoseconds discarded
```

It compares a full-precision time against one truncated to whole seconds, so it only
fires on a backward jump of more than ~1 s. Not our drop source, but it is not doing
the job it looks like it is doing.

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

**The trap.** A RELIABLE subscriber and a BEST_EFFORT publisher are **incompatible**
under DDS: they never match, so the subscriber receives exactly zero messages and
reports no error. It looks identical to a dead sensor. `ros2 topic hz`, `ros2 topic
echo` and `ros2 bag record` all default to **RELIABLE**.

> **Correction.** An earlier version of this document said "every sensor topic here is
> BEST_EFFORT". **That is wrong.** Verified against the live robot with
> `ros2 topic info -v` and against the recorded `offered_qos_profiles`:
>
> | Topic | Reliability | Durability |
> |---|---|---|
> | `lidar_chest_front/lidar_pointcloud` | **RELIABLE** | TRANSIENT_LOCAL |
> | `lidar_chest_front/imu` | **RELIABLE** | TRANSIENT_LOCAL |
> | `/tf`, `/tf_static` | RELIABLE | VOLATILE / TRANSIENT_LOCAL |
> | **`/aima/mc/leg_odometry`** | **BEST_EFFORT** | TRANSIENT_LOCAL |
>
> So `hz` failing on the *point cloud* was **Lesson 10** (transient discovery failure),
> not QoS — we mis-attributed it. The QoS override is still genuinely required, because
> `leg_odometry` really is BEST_EFFORT and a default RELIABLE recorder silently drops
> it. Right conclusion, wrong reason; the distinction matters when diagnosing the next
> failure. It also matters in reverse: Point-LIO subscribes to IMU **RELIABLE**
> (`create_subscription<Imu>(imu_topic, 200000, ...)`) while its cloud subscriber uses
> `SensorDataQoS` (BEST_EFFORT). Because the vendor IMU is RELIABLE, that happens to
> work — had it been BEST_EFFORT, Point-LIO would have received no IMU at all.

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

## Lesson 8: `set -u` breaks `source /opt/ros/humble/setup.bash`

Our first bag-recording script died immediately after writing its config file, with
exit code 1 and no error message. Cause: the script opened with `set -u`, and ROS 2's
`setup.bash` dereferences unbound variables (`AMENT_TRACE_SETUP_FILES`, `COLCON_*`
and friends). Under `set -u` that is a fatal error, and since we had redirected the
output, it was silent.

The robot's own `x2_recon.sh` already guarded against this — it sources each ROS
distro inside `( set +u; source ... )`, a subshell that both relaxes the check and
prevents the environment leaking between distros. Worth copying that pattern:

```bash
( set +u; source /opt/ros/humble/setup.bash; ... )
```

General form of the mistake: a strictness flag turned a third-party script's benign
habit into a fatal error, and output redirection hid the evidence. When a script dies
early with no output, re-run the first few lines without redirection before
theorising.

## Lesson 9: one unloadable type poisons the whole recording

The most expensive bug in the recording script, and the least obvious.

**Symptom.** A test recording produced 9 topics. `/tf`, `/tf_static` and
`/aima/mc/body_pose` were simply **absent** from the bag — no error at the top
level, no non-zero exit, just missing data.

**What we expected.** We had already tested this: asking rosbag2 to record a topic
whose *package* is not on the ament path produces a polite warning and the topic is
skipped —

```
[WARN] Topic '/aima/hal/joint/leg/state' has unknown type 'aimdk_msgs/msg/JointStateArray'.
       Only topics with known type are supported. Reason: package 'aimdk_msgs' not found
```

so we assumed listing extra topics was harmless.

**What actually happens** when the package *is* found but one message type's
typesupport library is missing:

```
[ERROR] [rosbag2_recorder]: Failure in topics discovery.
Error: Something went wrong loading the typesupport library for message type
       aimdk_msgs/ModuleInfo. Library could not be found.
```

This is **not** a skip. It aborts the discovery loop and retries forever, so every
topic that had not yet been subscribed is never subscribed. One bad topic
(`lidar_status`, type `ModuleInfo`) silently cost us `/tf` — a topic with a
perfectly ordinary `tf2_msgs` type that had recorded fine the day before.

**The fix.** Validate every topic's type *before* recording and drop the bad ones.
`ros2 interface show <type>` predicts it exactly and needs no DDS discovery. The
script now does this and refuses to start if a *required* topic fails.

**Transferable:** we generalised from one observed failure mode ("unknown package →
graceful skip") to a different one ("unloadable type → fatal"). Two superficially
similar errors, opposite consequences. The damage was invisible because the victims
were unrelated topics.

## Lesson 10: every `ros2` call does its own DDS discovery

**Symptom.** A preflight check that had worked moments earlier started failing with
"no publisher on lidar_pointcloud". Running the exact same command by hand printed
`Publisher count: 1`. Re-running the extraction immediately afterwards returned
nothing at all.

**Cause.** It was not a parsing bug and not stderr-vs-stdout — we checked both. Each
`ros2 topic list` / `ros2 topic info` invocation spins up its own DDS participant and
performs a fresh discovery pass. Several in quick succession contend, and one can
simply come back empty. Our script was doing three separate passes for disk, publisher
and type checks, which made an occasional failure near-certain.

**The fix.** One remote call, one `ros2 topic list -t` discovery pass, reuse its
output for every type check, and retry the probes up to 3×.

> On this robot, treat each `ros2` CLI invocation as expensive and unreliable.
> Batch them.

---

# Open questions

- **QoS of the `/slam/*` topics** we need to publish — must match what
  `soc0_task_manager` expects, or it will not receive our poses (Lesson 2, in reverse).
  Not yet inspected.
- **Dead-reckoning fusion.** The measured blocker — see *The blocking limitation*
  above. ZUPT and/or `leg_odometry` fusion, before any more parameter tuning.
- **Tuning against the deterministic baseline** (1.441 m). Now meaningful, since a
  config change can be distinguished from a lucky draw.
- **Trajectory output as a file** rather than a DDS-recorded bag, to remove the last
  few poses of nondeterminism at the tail.
- **E1R accelerometer full-scale range.** Walking peaks hit 3.02 g. If configured at
  ±4 g, a faster gait or stumble clips. Readable from DIFOP or the LiDAR's config.
- **HAL rosbag replay (`data_source: 1`)** — the route from vendor `lidar_raw_data`
  back to `PointCloud2`. Looks designed for this; untested.
- **`em-server` HTTP API** on `0.0.0.0:50080` — presumably how apps are started and
  stopped cleanly. Undocumented so far.
- **Vendor cloud coupling.** `emqx.agibot.com`, the MQTT plugin wired into the SLAM
  module itself, `soc0_cloud_proxy`, `cobridge`. Extent of any backend dependency is
  not yet mapped.
- **Dynamic `/tf` chain.** `/tf_static` is now known (below), but the articulated
  links (`pelvis` ↔ `torso_link`, driven by the waist joint) come through dynamic
  `/tf` at 48 Hz and have not been walked through.
- **Kinematic odometry from joint states.** soc0 records all leg joint states at
  9 GB retention. With the URDF, leg odometry could be computed independently of
  `/aima/mc/leg_odometry` — useful both as a cross-check and as a fallback.

### Resolved

| Question | Answer | Where |
|---|---|---|
| Point cloud rate | **10.0 Hz** | bag message counts |
| LiDAR IMU rate | **200 Hz** | bag message counts |
| Per-point timestamp units | **float64 ns, relative to scan start** | [measured](#per-point-timestamp-semantics-measured-not-assumed) |
| Does leg odometry need walking? | **No** — ~167 Hz while idle | bag message counts |
| Disk space for bags | 384 GB free; **~500 MB/min** | `df` + bag size |
| Where can we write on soc1? | `/home/run` (**not** `/home/agi`) | writability test |
| Can we record vendor `aimdk_msgs` topics? | **Partly** — only the 57 types in the prebuilt tree | type table above |
| How violent is walking for the IMU? | **sd ×600, 0.26–3.02 g, no saturation** | [measured](#walking-dynamics-measured-2026-10-06) |
| Is walking data recorded automatically? | **Partly** — raw LiDAR + all IMUs yes; `leg_odometry`, `/tf`, `PointCloud2` no | `drp` configs |
| What is `lidar_raw_data`? | Raw MSOP/DIFOP byte arrays | `LidarRawData.msg` |

---

# The blocking limitation: Point-LIO has no dead-reckoning input

On `office_corridor_02`, Point-LIO drifts **14.1 m over ~31 m travelled (~45 %)**.
That is not a tuning problem, and the shape of the failure says exactly why.

**Vertical is fine, horizontal is not.** z drift at the end is **0.101 m** over four
minutes. Gravity is well observed; the failure is purely in the horizontal plane.

**It hallucinates motion when the robot is nearly still.** Distance travelled per 20 s
window, against leg odometry:

| window | leg odom | Point-LIO | ratio | median range |
|---|---|---|---|---|
| 0-20 s | 10.99 m | 10.69 m | 0.97 | 2.50 m |
| 80-100 s | 13.02 m | 13.49 m | 1.04 | 1.69 m |
| **100-120 s** | **1.83 m** | **7.66 m** | **4.19** | **1.25 m** |
| 160-180 s | 13.54 m | 26.56 m | 1.96 | 2.26 m |
| **220-240 s** | **0.03 m** | **1.62 m** | **60** | 3.37 m |

While actually walking, Point-LIO tracks distance to within a few percent. The error
concentrates in the windows where the robot **barely moved**: in the last 20 s it
moved **3 cm** and the estimator reported **1.62 m**.

**Why.** The X2 Ultra stands by actively balancing, so a "stationary" robot still
produces real footfall and oscillation on the IMU. Point-LIO propagates on that IMU.
The LiDAR should correct it — but with a 123 deg FOV at 1.25 m median range against
close corridor walls, scan matching slides freely along those surfaces and cannot.
Nothing in the filter says "you are not translating."

## What this means for the project

**LiDAR-inertial odometry alone is not sufficient on this platform.** That is not a
Point-LIO defect; it is the wrong tool for a legged robot in close quarters, used
alone.

Note what the vendor does. Their `LocalizationModule` config has:

```yaml
sub_dr_topic: /aima/mc/leg_odometry      # dead reckoning
```

Their Faster-LIO-based stack consumes **leg odometry as a dead-reckoning prior**.
Point-LIO has no such input — it is LiDAR + IMU only.

Leg odometry's strengths are exactly LIO's weaknesses here:

| | leg odometry | LiDAR-inertial |
|---|---|---|
| Stationary | **no drift** (foot contact) | drifts (IMU noise + degenerate geometry) |
| Short-term translation | good | good when geometry is rich |
| Long-term / global | drifts, no loop closure | corrects against the map |
| Degenerate geometry | **unaffected** | fails |

They are complementary. Fusing them is the architecture this robot needs, and the
vendor reached the same conclusion independently.

### Options, roughly in order of effort

1. **Zero-velocity updates (ZUPT)** — detect "not walking" from the `leg_odometry`
   twist or joint states and clamp velocity. Cheap, and targets the single biggest
   measured error source.
2. **Fuse after the fact** — run Point-LIO, then combine its pose with
   `/aima/mc/leg_odometry` in a pose graph or EKF. No changes inside Point-LIO.
3. **Add a DR factor inside Point-LIO** — most faithful to the vendor architecture,
   most work.

Try 1 and 2 before any further parameter tuning: the table above shows tuning cannot
supply a missing constraint.


# What to record next

The limitation is **geometry, not duration**. More minutes in the same room gives the
same weak constraints.

Measured: the E1R is **122.8 deg x 89.1 deg forward-facing**, median range **3.40 m**,
in a 5.4 x 5.4 m space. The failure mode is a large flat surface filling the FOV: one
dominant plane constrains its own normal and little else, leaving translation along
the surface and yaw weakly observable.

**Most useful next recording — one loop, 3-5 minutes:**

1. **office -> corridor -> back to start.** Varied geometry, real distance, and a
   genuine loop closure over a longer baseline than the current 5.4 m room.
2. **Keep structure in view.** Desks, chairs, doorways, corners, boxes. Clutter helps
   LIO; a clean empty room hurts it.
3. **Avoid long stretches facing a blank wall**, and especially avoid turning in place
   while facing one — that is the degenerate case.
4. If the corridor is long and bare, **a single there-and-back** beats looping inside it.

**On corridors:** expect them to be *harder*, not easier. Two long parallel walls leave
motion along the corridor axis weakly observable, and the classic symptom is drift that
stretches the trajectory lengthwise. Worth recording precisely because it tests drift
over distance — but it is a harder case, not a fix for the current one.

Record with `./record_walk.sh -n office_corridor_loop_01`.

# Safety notes

- The robot's motors are **torque-enabled and damped**, not free. Anything that
  stops or restarts motion-control modules can cause physical movement. All recon to
  date has been strictly read-only for exactly this reason.
- Keep our work out of `/agibot/` (vendor-managed, and OTA-updated). Prefer
  `/home/agi/` or `/data`.
- No vendor file has been modified. The only changes made anywhere are an SSH
  `authorized_keys` entry on each panel and a local `~/.ssh/config` on the dev laptop.
