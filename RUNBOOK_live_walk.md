# Live walking run — step by step

Running SLAM while you walk the robot. **The SLAM stack never commands the robot.** It
reads the LiDAR and IMU and computes a pose; you drive the robot exactly as you normally
would. Walking with this running is no more dangerous than walking without it.

Measured cost on the robot: 32 % of *one* core out of eight, and soc1's load average did
not move. Motion control runs on soc0, a different panel.

---

## Before you start

- Robot on, **torque on** (you need it to walk).
- You on the same WiFi as the robot, or on the robot's own AP (see step 6).
- One-time, if the robot has never been built on:
  ```bash
  rsync -az tools patches soc1:/home/run/x2_ultra_slam/
  rsync -az slam_ws/src/x2_cloud_adapter soc1:/home/run/x2_ultra_slam/slam_ws/src/
  ssh soc1 'cd /home/run/x2_ultra_slam && COLCON_WORKERS=2 nice -n 10 ./tools/bootstrap_slam_ws.sh'
  ```

---

## The run

**1. Mark the floor.** Tape both foot outlines. Add a straight edge ~1 m along the
robot's side and mark that line too — it is what makes the yaw number meaningful
(+-1 cm over 1 m is +-0.57 deg, versus +-8 deg from foot outlines alone).

**2. Start SLAM *and* the recorder, in one command.**
```bash
./tools/live_slam.sh --start --record live_walk_01
```
`--record` is what saves the walk. **`--start` on its own runs SLAM but saves nothing** —
it now warns you loudly if you do that. `--max 900` auto-stops the recorder after 15 min
so a forgotten recording cannot fill the disk.

**3. Confirm BOTH are running.** Do not skip this.
```bash
./tools/live_slam.sh --status
```
You need **both** green lines:
```
  ok  pointlio_mapping is running
  ok  a bag recording IS running
```
plus an `[rx] clouds received` count that **increases** if you run it again. If either
line is red, stop and fix it before walking.

**4. Place the robot on the mark. Stand still 10 s.** Not optional — the analysis finds
the endpoints from this.

**5. Walk the loop.** Office -> corridor -> back. If you leave WiFi range, join the
robot's AP and prefix later commands with `X2_HOST=soc1ap`.

**6. Return to the mark**, align both feet and the 1 m edge. **Stand still 10 s.**

**7. Stop and collect.**
```bash
./record_walk.sh --fetch live_walk_01
./tools/live_slam.sh --stop
```
`--stop` must print `ok  stopped; nothing left running`.

**8. Tell me how precisely you repositioned it** — per foot, and the heading edge. That
sets the error floor; without it the numbers cannot be interpreted.

---

## If something looks wrong

Stopping SLAM is always safe and instant. It is one process; killing it removes the DDS
endpoints and changes nothing else:
```bash
./tools/live_slam.sh --stop
```
Nothing is written to disk by the SLAM stack, no vendor config is touched, nothing in
`/agibot/` is modified. The robot's own stack is unaffected — verified after a live run:
34 nodes up, `leg_odometry` 168.5 Hz, LiDAR 10.6 Hz, unchanged.

**Do not** restart any vendor process to "fix" something. Ask instead.

---

## What this run gives us

Both the live SLAM output *and* the raw sensor data land in the same bag. That lets us
replay the identical inputs offline, deterministically, and compare:

- if live matches the offline replay, the live path is sound;
- if it does not, DDS is costing us something, and the difference tells us what.

That comparison is the point of the exercise — it is the one thing bag-replay testing
cannot tell us.

```bash
./tools/run_point_lio_det.sh data/bags/live_walk_01 live01_offline
./tools/eval_loop_closure.py results/live01_offline data/bags/live_walk_01
```

## Publishing to the vendor slot (optional, not needed for this)

`./tools/live_slam.sh --start --publish` also fills `/slam/*`. Leave it off unless you
specifically want that; it is the only part that writes to the robot's bus.
