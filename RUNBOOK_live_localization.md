# Live localization walk: step by step

The robot finds itself on the saved map (`map01`, from `map_walk_01`) **while you walk
it**. Nothing new moves the robot. Point-LIO and the localizer only read sensors and
compute a pose; you drive with the controller exactly as usual.

What runs on soc1: Point-LIO (as in the earlier live run), plus `localizer_live`. The
localizer publishes `/x2/localization/odometry` and `/x2/localization/status`. It never
writes `/tf` and never commands anything. Stopping is instant and safe.

---

## Before you start

- Robot on, torque on, you on the office WiFi or the robot's AP. On the AP, put
  `X2_HOST=soc1ap` in front of every command below.
- **I deploy and build first** (`./tools/deploy_nav_robot.sh`). Tell me when the robot is
  reachable.

## The walk

**1. A new start mark.** The old home tape is gone, and that is fine. Tape a new one
(both foot outlines, heading arrow, 1 m edge):
- **in the same spot as the old home, as close as you can manage**, and
- **facing the same way** the mapping walk started.

The localizer searches **+-1 m and +-30 deg** around the old home, so "about there" is
enough. This new mark is this walk's own start = end reference.

**2. Robot on the new mark, standing still.** Keep it still from here until step 4 says
LOCALISED. The localizer uses the first 3 s of scans to find itself.

**3. Start everything with one command.**
```bash
./tools/live_slam.sh --start --record loc_walk_01 --localize map01
```

**4. Check. Do not skip this.**
```bash
./tools/live_slam.sh --status
```
You need all of:
```
  ok  pointlio_mapping is running
  ok  localizer_live is running
     ... LOCALISED after ... s search: x ... y ... yaw ...
  ok  a bag recording IS running
```
Run it again after ~15 s. A `TRACKING ...` line should appear, with the ICP accepted count
rising.
- **`SEARCH FAILED` repeating:** the start is too far from the old home (or facing too
  differently). Stop (step 8) and tell me roughly where it stands; I will give you an
  `--init` for it.
- **No LOCALISED line after ~60 s:** stop and tell me.

**5. Stand still another 10 s.**

**6. Walk the loop: office -> corridor -> back to the office.** Normal walking. Please
avoid the door threshold that made it stumble last time if there is a way around it; if
not, just note when you crossed it.

**7. Back on the new mark.** Align the feet and the edge, facing the arrow. **Stand still 15 s.**

**8. Stop and collect.**
```bash
./record_walk.sh --fetch loc_walk_01
./tools/live_slam.sh --stop
```
`--stop` must print `ok  stopped; nothing left running`.

**9. Tell me** how precisely it came back on the new mark (per foot + edge), and anything
unusual (stumble, threshold, people in front, doors).

---

## What I do with it (no robot needed)

- **Live vs offline.** Replay the same raw data on the laptop (Point-LIO + localizer,
  deterministic) and compare it with what the robot computed live. The localizer code is
  identical (bit-identical on the laptop when fed the same input), so a difference
  isolates what live Point-LIO + DDS cost.
- **Start = end through the map.** The walk's start and end must land on the same map spot,
  within your repositioning.
- **Latency and load.** How far behind real time the live pose runs, and the CPU it costs
  on soc1. The waypoint follower will feed on this pose, so its latency matters.
