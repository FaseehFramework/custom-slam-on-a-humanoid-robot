# CLAUDE.md — AgiBot X2 Ultra custom SLAM

Goal: run our own SLAM on an X2 Ultra, publishing into the vacant
`/slam/localization/odometry` + `/slam/mapping/odometry` slot the vendor's
`soc0_task_manager` already subscribes to.

**`README.md` is the detailed record** (findings, measurements, lessons). This file is
the operating manual. Read README.md before changing anything non-obvious.

## Layout

```
record_walk.sh              record datasets from the robot (the main tool)
tools/bootstrap_slam_ws.sh  rebuild slam_ws from scratch (~40 s)
tools/run_point_lio_det.sh  DETERMINISTIC eval  <- use this
tools/run_point_lio.sh      live DDS replay (non-deterministic, demo only)
tools/convert_bag.py        vendor cloud layout -> Point-LIO layout, offline
tools/analyze_bag.py        bag sanity: continuity, IMU dynamics, trajectory, frames
tools/eval_trajectory.py    estimate vs leg-odometry reference
tools/eval_loop_closure.py  drift vs a physically marked start/end (real ground truth)
tools/live_slam.sh          start/stop/status live SLAM on the robot
RUNBOOK_live_walk.md        step-by-step live walking run (hand to the operator)
tools/dr_payoff.py          bounds what a leg-odometry DR factor would recover
patches/point_lio_ros2.patch  our changes to upstream (pcl_ros, QoS, rx counter, bag_path)
slam_ws/src/x2_cloud_adapter/  OURS: cloud adapter + slam_publisher + config + launch
slam_ws/src/point_lio_ros2/    third party, gitignored, reproducible
data/bags/, results/           gitignored, NOT reproducible without the robot
```

## Robot

| Panel | Role | Office WiFi | Robot AP |
|---|---|---|---|
| soc0 | motion control, EtherCAT, body IMUs, hostapd | `soc0` | `soc0ap` |
| soc1 | **perception** (Jetson Orin, LiDAR, our workspace) | `soc1` | `soc1ap` |
| soc2 | audio / interaction | `soc2` | `soc2ap` |

soc1 is the only way in; soc0/soc2 are reached by ProxyJump through it.
Our workspace on the robot: `/home/run/slam_ws` (**not** `/home/agi` — not writable).

## Commands

```bash
# --- record ---------------------------------------------------------------
./record_walk.sh                              # interactive: start, walk, ENTER
./record_walk.sh -d 120 -n name               # fixed duration
X2_HOST=soc1ap ./record_walk.sh --detach -n name --max 900   # out of WiFi range
X2_HOST=soc1ap ./record_walk.sh --list        # what is on the robot
X2_HOST=soc1ap ./record_walk.sh --fetch       # stop, verify, copy here, delete there

# --- build ----------------------------------------------------------------
./tools/bootstrap_slam_ws.sh                  # clone + patch + build
./tools/bootstrap_slam_ws.sh --fresh          # also wipe build/install/log

# --- evaluate -------------------------------------------------------------
./tools/run_point_lio_det.sh data/bags/<bag> <outname>     # ~15 s, repeatable
./tools/eval_trajectory.py results/<outname> data/bags/<bag>
./tools/analyze_bag.py data/bags/<bag>
./tools/dr_payoff.py results/<outname> data/bags/<bag>   # DR payoff bound
./tools/eval_loop_closure.py results/<outname> data/bags/<bag>  # needs a marked walk

# --- on the robot ---------------------------------------------------------
ssh soc1
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/entry/cfg/ros_dds_configuration.xml
ros2 topic list --no-daemon                   # --no-daemon is MANDATORY
```

## DO NOT

1. **Never `git clean -xdf`.** `data/bags/`, `*.db3`, `*.mcap` are gitignored — it
   deletes recordings that cannot be regenerated without the robot. Back them up
   outside the repo first. (This has already bitten once.)
2. **Never omit `--no-daemon`** for `ros2 topic/node list` on the robot. A stale daemon
   showed 106 topics instead of 180 and hid the entire LiDAR, costing hours.
3. **Never use `run_point_lio.sh` (DDS) to evaluate or tune.** It loses scans and ~50 %
   of runs diverge. Use `run_point_lio_det.sh`. A single good run means nothing —
   always run twice and compare.
4. **Never `set -u` around `source /opt/ros/humble/setup.bash`.** ROS setup scripts
   dereference unbound vars; the shell dies silently. Use `( set +u; source ... )`.
5. **Never `sudo cmd < file`.** The shell opens the redirect as *you*, not root.
   Use `sudo cat file | cmd`.
6. **Never record a topic without validating its type first.** One unloadable
   `aimdk_msgs` type raises "Failure in topics discovery" and **aborts discovery for
   every topic not yet subscribed** — it silently cost us `/tf`. `record_walk.sh`
   validates; do the same anywhere else.
7. **Never subscribe RELIABLE to `/aima/mc/leg_odometry`** — it is published
   BEST_EFFORT and you will receive nothing, with no error. A BEST_EFFORT reader
   matches both, so prefer it. (The point cloud and IMU *are* RELIABLE.)
8. **Never read extrinsics from the `agi_sam:` block of `paramsE1R.yaml`.** It is dead
   LIO-SAM config whose values differ by a factor of 10. Use the `mapping:` block.
9. **Never write to `/agibot/`.** Vendor-managed and OTA-updated. Work in
   `/home/run/slam_ws`.
10. **Never stop/start vendor motion modules.** Motors are torque-enabled and damped —
    the robot can move. All robot work to date is read-only apart from our own bags.
11. **Never batch several `ros2` CLI calls expecting reliability.** Each does its own
    DDS discovery and can return empty under contention. One call, one discovery pass,
    retry.
12. **Never declare a Point-LIO parameter top-level if it lives under `mapping:` in
    the YAML.** Upstream declares `mapping.satu_acc`, `mapping.fov_degree`, ... but
    `use_imu_as_input` bare. A mismatch does not error — ROS 2 silently ignores the
    YAML entry and you run with the default. Our `dr_*` params are `mapping.dr_*`;
    verify with the `[dr]` log line, which prints the values actually in force.
13. **Never publish to `/slam/*` casually.** `slam_publisher` is the only component we
    have that *writes* to the robot's bus; everything else is read-only. It is off by
    default (`publish_slam:=false`). Publish RELIABLE + TRANSIENT_LOCAL — measured, all
    three subscribers are BEST_EFFORT/VOLATILE, and a RELIABLE/TRANSIENT_LOCAL writer
    is the strictly most compatible. A QoS mismatch here fails **silently**.
14. **Never claim `soc0_task_manager` consumes our odometry.** Measured: the subscriber
    on `/slam/localization/odometry` is `soc1_drp_ros2_node3065`, the vendor *recorder*.
    task_manager is running but its subscriptions could not be enumerated (cross-panel
    discovery is intermittent and asymmetric). Unconfirmed, not false.
15. **Never leave a run in `/agibot/data/bag/` uncopied.** It is a ~27 min rolling
    buffer; older segments are gone.

## Sensor facts worth not re-deriving

- **LiDAR**: RoboSense E1R, 10.0 Hz, UDP 6699/7788 on `sensor0` (10.11.1.100).
  **122.8° × 89.1° forward-facing**, `ring` 0–95, median range 1.9–3.4 m indoors.
- **Cloud `timestamp` field** is float64 **nanoseconds relative to scan start**;
  `header.stamp` is scan start. Point-LIO wants float32 seconds → `× 1e-9`.
- **LiDAR IMU**: 200 Hz, **m/s² not g**, 6-axis (no orientation). Body IMUs 500 Hz.
- **Walking dynamics**: 0.26–3.0 g, |a| sd ~2.4 vs 0.004 standing. `satu_acc` must be
  > 30 or real footfalls get discarded as saturated.
- **`/aima/mc/leg_odometry` is already in the `lidar_imu_chest_front` frame** — no
  extrinsic needed to use it as a prior.
- **PTP** disciplines all three panels and the LiDAR to one clock. Time sync is solved.

## State / next step

Working: deterministic harness (bit-identical runs), recording incl. detached AP mode,
cloud adapter, extrinsics confirmed three ways.

**Working (2026-10-12): the leg-odometry DR factor.** Validated against **physical
ground truth** on `marked_loop_03` (24.6 m marked loop): closure error **1.524 m -> 0.142 m**
raw, **1.560 m -> 0.085 m** after correcting for a measured 6 cm repositioning offset
(**0.35 % of path**), and yaw **40.11 -> 2.71 deg**. It beats leg
odometry itself (0.292 m / -7.30 deg), which proves fusion rather than passthrough --
a passthrough cannot beat its own source. On `office_corridor_02` (leg-odometry-scored):
13.75 m -> 0.79 m, path inflation 1.34x -> 1.00x, peak speed 5.395 -> 0.971 m/s.
Deterministic, and inert when off (`dr_mode:=0` reproduces the old baseline exactly).

How it works: velocity is state index 12..14 of the 30-state output filter, so the
residual `z = R*v_legodom - v_state` has an identity H and the gain is a slice of P --
`update_iterated_dyn_share_DR()` in esekfom.hpp mirrors `..._IMU()` exactly.

**DO NOT use `dr_mode: 2`** (speed magnitude only). It scales along the filter's *own*
velocity direction, which is a feedback loop: a wrong direction gets driven confidently
at the right speed. Measured 29.76 m, worse than no DR at all. The offline proxy ranked
it *best* (88 %) because post-hoc rescaling has no feedback term -- a proxy without
feedback cannot rank in-filter designs.

**DO NOT tune `dr_vel_cov` against `dr_payoff.py`'s `err` column.** It measures distance
from leg odometry's own answer, so it rewards a passthrough by construction. Judge by
end displacement and by peak speed instead. Results are insensitive across 1e-4..1.0
anyway (0.26-1.40 m); even cov 1.0 -- looser than the platform's top speed -- recovers
13.75 m to 0.71 m, which is what shows it constrains rather than dominates.

Next, in order:

1. **Ground truth, on the corridor.** `marked_loop_02` established the first real anchor:
   Point-LIO closes a 14.1 m loop to **0.077 m (0.55 %)** against a ~5 cm repositioning
   floor. But it does **not** validate the DR factor — baseline peaked at 1.194 m/s there
   (no divergence) vs 5.395 m/s on the corridor, so DR and baseline differ by 1.1 cm,
   below the floor. Repeat the marked walk **through the corridor**. Yaw from that run is
   uninterpretable: the foot offsets imply a 4-20 deg yaw floor.
2. **Confirm frame ids** `soc0_task_manager` expects, then set `header_frame_id` /
   `child_frame_id` on `slam_publisher` (currently passes `camera_init -> aft_mapped`
   through unchanged).
3. **Stage 3 DONE (2026-10-12).** Published into the vendor slot for ~45 s with the robot
   live and torque off: 10.5 Hz confirmed on `/slam/localization/odometry`, clean revert
   (publisher count back to 0, no strays), vendor stack unaffected. **Still unknown:**
   whether `soc0_task_manager` reacts -- no before/during/after instrumentation was taken.
   Filling this slot does NOT enable the paywalled nav stack (`/loc_tele/*`).

## No vendor buffer records a PointCloud2

`/agibot/data/bag/{hal_sensor,software,autonomous}` hold `lidar_raw_data` (raw UDP),
the lidar IMU and `leg_odometry` -- but **zero PointCloud2 topics**. So a walk that was
not recorded by `record_walk.sh` is **not recoverable** without writing an E1R MSOP
packet decoder. Always start the recorder:
`./tools/live_slam.sh --start --record NAME`.

## Never `pgrep -f`/`pkill -f` a pattern that is in your own command line

Bitten twice. A watcher looped forever because its `pgrep -f "run_point_lio_det"` matched
the shell running it; `live_slam.sh` reported "running" with nothing running for the same
reason. `pkill -f` is worse -- it can kill the cleanup shell mid-cleanup. Pipe remote
scripts over **stdin** (`ssh host bash -s <<'EOF'`), never inline in the ssh command, and
use `[p]attern` brackets as a second guard. Verify with `ps -C <name>`.

## Trust recorded data, not `ros2` CLI counts

`ros2 topic info` reported `pub=0` for a topic that was simultaneously delivering 203
recorded messages. `hz` has twice reported silence on topics that were demonstrably
flowing. Endpoint counts, `hz` and `node list` are all intermittently wrong here.
**Record a bag and count messages** -- that is the only measurement that has never lied.
