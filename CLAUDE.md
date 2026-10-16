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
tools/loco_cmd_test.py      FIRST MOTION TEST (on soc1): tiny velocity command, interlocked
tools/decode_locomotion_cmd.py  decode recorded locomotion commands (no aimdk typesupport)
tools/follow.sh             WAYPOINT FOLLOWER from the laptop: mark/list/plan/shadow/go/stop
tools/follower_core.py      follower logic, pure python (caps, gates, legs, watchdogs)
tools/waypoint_follower.py  follower ROS side on soc1 (run via follow.sh / follow_robot.sh)
tools/test_follower.py      follower vs simulated robot + mc, every stop path (~20 s)
tools/test_follower_ros.sh  real follower program vs fake robot node, localhost only (~4 min)
tools/test_fixtures/aimdk_msgs_fake/  LAPTOP-ONLY fake vendor types for that test. Never deploy.
tools/cmd_tracking.py       remote command vs leg-odometry motion (turn sign, lag, speed)
tools/heading_check.py      turn sign leg-odom vs map; robot heading from map pose vs travel
RUNBOOK_live_walk.md        step-by-step live walking run (hand to the operator)
RUNBOOK_mapping_walk.md     mapping walk (both directions) + test walk from the home mark
RUNBOOK_live_localization.md  live localization walk (operator steps)
tools/deploy_nav_robot.sh   sync + build the localizer on soc1 (no GTSAM there), copy maps
tools/dr_payoff.py          bounds what a leg-odometry DR factor would recover
tools/offset_sweep.py       start-offset ensemble (DR on/off) -- use before any claim
tools/sweep_report.py       cross-bag table; merges baseline + --tag experiment JSONs per bag
tools/dr_vertical_check.py  does DR leak horizontal velocity into v_z (corr is NOT specific;
                            compare against DR off)
patches/fastlio2_ros2.patch   our changes to FASTLIO2_ROS2 pgo (init, isotropic noise, loop log)
nav_src/x2_nav/               OURS: pgo_offline, localizer_offline, localizer_live (shared
                              reloc_core.hpp), configs
patches/point_lio_ros2.patch  our changes to upstream (pcl_ros, QoS, rx counter, bag_path,
                            DR factor, publish_tf gate, deterministic replay)
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
RECORD_CLOUDS=1 ./tools/run_point_lio_det.sh <bag> <out>   # + /cloud_registered (for pgo)
./tools/bootstrap_nav_ws.sh                   # nav_ws: pgo/localizer (patched) + x2_nav
source nav_ws/install/setup.bash
ros2 run x2_nav pgo_offline results/<out> nav_src/x2_nav/config/pgo_x2.yaml results/<out>_pgo
ros2 run x2_nav localizer_offline <map.pcd> results/<walk> nav_src/x2_nav/config/localizer_x2.yaml \
     results/<reloc_out> --search          # or --init / --local x,y,z,yaw_deg (= T_map<-camera_init)
./tools/compare_traj.py <est_traj_bag> <ref_traj_bag> [--ref-shift x,y,z,yaw]
./tools/closure_in_robot_frame.py results/<run> data/bags/<bag>   # closure as forward/left (vs operator)
./tools/dwell_vs_home.py <map_final> <map_bag> <walk_traj> <walk_bag>  # walk start/end vs HOME mark
./tools/stance_heading.py data/bags/<bag>     # waist/hip yaw at start vs end (chest-vs-feet)
./tools/eval_trajectory.py results/<outname> data/bags/<bag>
./tools/analyze_bag.py data/bags/<bag>
./tools/dr_payoff.py results/<outname> data/bags/<bag>   # DR payoff bound
./tools/eval_loop_closure.py results/<outname> data/bags/<bag>  # needs a marked walk
./tools/offset_sweep.py data/bags/<bag> [--loop] --dr 1,0          # baseline ensemble
./tools/offset_sweep.py data/bags/<bag> [--loop] --dr 3,4 --tag x  # experiment, own JSON
./tools/sweep_report.py                                           # merged table

# --- on the robot ---------------------------------------------------------
./tools/deploy_nav_robot.sh                   # localizer + maps/map01.pcd -> soc1, build there
./tools/live_slam.sh --start --record NAME --localize map01 [--init x,y,z,yaw]
./tools/follow.sh mark NAME | list | plan A B | shadow A B   # no motion (needs the localizer)
./tools/follow.sh go A B [--align-final]      # WALKS: operator on the controller, clear space
./tools/follow.sh stop                        # SIGINT a running follower from another terminal
./tools/test_follower.py && ./tools/test_follower_ros.sh    # before any follower change ships
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
15. **Never command motion except through `tools/loco_cmd_test.py` or the waypoint
    follower's `go` (`tools/follow.sh go` -> `tools/waypoint_follower.py` on soc1).** They are
    the only code that can move the robot. `mark`, `list`, `plan` and `shadow` have no publisher.
    - **Source.** Both publish as `source: 'pnc'`, the registered planning-and-control
      input (priority 40, below the remote's 80). mc silently DROPS any unregistered
      source name.
    - **Interlock.** Both check the `GetMcAction` description: `STAND_DEFAULT` or
      `LOCOMOTION_DEFAULT` + RUNNING to start. mc auto-switches stand <-> walk on commands.
      The numeric action field reads 0, so never gate on it.
    - **loco_cmd_test limits.** Caps 0.2 m/s and 3 s, BEST_EFFORT+VOLATILE (no latching),
      5 Hz mode re-check, explicit zeros, then a stop check.
    - **Follower limits, all in code (`follower_core.py`).**
      - Caps: forward 0..0.2 m/s (never backwards), turn +-0.3 rad/s, sideways 0, legs <= 10 m,
        a time limit per leg.
      - Stops with zeros and never resumes on any of: stick input from the remote; another
        program publishing as pnc; the remote's stream silent > 1 s; mode not
        STAND/LOCOMOTION_DEFAULT (RUNNING or TRANSITION) or GetMcAction silent > 1 s; pose
        silent > 0.5 s, > 1 s old or not finite; status not TRACKING or silent > 3 s; 8 ICP
        rejections in a row; a pose jump > 0.4 m or 15 deg; rotating 25 deg against the turn
        command, walking 0.3 m away, or no progress for 6 s; > 1 m off the leg's line;
        Ctrl-C, Ctrl-\, SIGTERM or SIGHUP.
      - Refuses to start unless all of that holds with the robot standing and the remote
        idle, AND the robot is localised on the `--from` waypoint it stands on (0.3 m /
        15 deg), AND the map file is the one the waypoints were captured on (sha1). It
        holds `/tmp/x2_motion.lock` (shared with loco_cmd_test.py) from before the typed GO
        (or `--confirm`) until exit. Every exit after preflight ends with 1 s of zeros,
        sent before anything is printed.
      - Change `follower_core.py` or `waypoint_follower.py` only with
        `tools/test_follower.py` AND `tools/test_follower_ros.sh` passing.
    - **Never call `SetMcAction` or `SetMcInputSource`.** Never publish as `rc` (the
      remote must keep priority).
    - **People and space.** Any non-dry run needs an operator holding the controller and
      clear space. Losing the laptop link does NOT stop a running `go`; the stick does.
16. **Never leave a run in `/agibot/data/bag/` uncopied.** It is a ~27 min rolling
    buffer; older segments are gone.
17. **Never run FASTLIO2_ROS2 `pgo`/`localizer` on the robot's bus unremapped.** Both
    broadcast `map -> camera_init` on `/tf` unconditionally (no switch). Offline: run them
    under `ROS_LOCALHOST_ONLY=1` with a private `ROS_DOMAIN_ID`. Never run the two together.
    **Never use their `fastlio2`** (Livox-only input, accel x10). Details: README,
    "Mapping and relocalisation".
18. **Never run the localizer with 6-DOF ICP** (`yaw_only: false`). On the X2 it settled on
    a solution tilted 3.7 deg / 14 cm high that passed every fitness gate. The 4-DOF
    estimator (`yaw_estimation.h` in the patch) is the fix.
19. **Never run pgo without the heading gate** (`loop_max_angle_deg: 60`). Forward-only
    LiDAR: loops between opposite views pass the score gate and are false. They broke
    `map_walk_01` (closure 0.10 -> 0.44 m).
20. **Never stop `localizer_live` via `ros2 run`'s PID.** SIGINT to `ros2 run` does not reach
    the node, and it keeps running. Start the binary directly; stop it with
    `pkill -INT -x localizer_live` (exact name; cannot self-match).

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

Working: deterministic harness (bit-identical, verified under 6-way concurrency since
2026-10-13), recording incl. detached AP mode,
cloud adapter, extrinsics confirmed three ways.

**The leg-odometry DR factor** (`dr_mode: 1`): a robustness fix -- see "Closure numbers are
single draws" below for what it does and does not do. The 2026-10-12 headline figures
(1.524 -> 0.142 m, "91 %", "0.35 % of path", yaw 40 -> 2.7 deg) were single draws comparing a
slipped baseline run with an unslipped DR run, and the yaw part was a gimbal-lock artefact.
Do not quote them.

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

## Closure numbers are single draws -- never quote one as "the error"

Run `tools/offset_sweep.py data/bags/<bag> [--loop]` (11 start offsets, DR on/off) and
`tools/sweep_report.py`; compare distributions, never single runs. Closure is **bimodal**
(re-registers or slips) -- report slip rate + good-mode median. Offsets are correlated:
bags, not offsets, are the unit of generalisation.

What the ensemble established: **DR is a robustness fix, not an accuracy fix.** Unslipped
closure is the same with and without it; DR cuts slips (10/11 -> 5/11, 7/11 -> 0/11) and
divergence (corridor DR-off: 100 m height error, 735 m/s). A diverged filter can SEGFAULT
in ikd-Tree's background rebuild thread -- the threaded (live) build only; DR prevents it by
preventing the divergence. It **costs mid-walk height** on hard walks (8/11
worse). Cause UNEXPLAINED. Tested by intervention under the full ensemble and refuted:
`dr_mode 3` (no direct z row) is worse on 5/5 bags; `dr_mode 4` (body-frame residual WITH
attitude Jacobian, numerically verified) is a coin flip vs mode 1 (26/55). Keep mode 1.
Do not re-try either. Routes confirmed level (max 3 cm threshold), so `dz` is real filter
error for DR on and off alike -- parked, not dismissed.

**DO NOT** read single-orientation yaw on this robot: the IMU frame sits at pitch ~-85 deg
(gimbal lock). Use `rel_rot()` (world-frame delta) in `tools/eval_loop_closure.py`. Every
yaw figure before 2026-10-13 was wrong.

**DO NOT** trust end-point-only metrics. Closure hid the height problem; check `dz` (height
error while walking) too.

The harness is deterministic **only since 2026-10-13**: ikd-Tree's background rebuild
thread made replays depend on CPU load (fixed: synchronous rebuild in replay mode), and the
recorder dropped leading poses (fixed: replay waits for its subscriber). Verified with six
concurrent identical replays. The harness now exits with the node's status; a crash is a
failure, never a scored row.

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
