# Waypoints + shadow run (stage B): step by step

**Nothing in this runbook makes the robot move by itself.** You drive with the controller the
whole time, exactly as on the earlier walks. The follower only *listens*:
- `mark` saves where the robot stands;
- `plan` prints the legs;
- `shadow` works out the commands it *would* send and logs them. It has no publisher, so it
  cannot send anything.

Run every command from the repo on the laptop. On the robot's AP, put `X2_HOST=soc1ap` in
front of each one.

---

## Before you start

- Robot on, torque on, **standing**. Controller connected. Laptop on the office WiFi or the
  robot AP.
- **The home mark.** Use the tape from `loc_walk_01` if it is still there. If not, tape a new
  one as close as you can to the old home, facing the same way (both foot outlines + heading
  arrow).
- **Pick 3-4 waypoints along office -> door -> corridor**, before you start:
  - **Each leg must be a clear straight line.** The follower will later walk exactly that line.
  - **Each leg is at most 10 m.** `plan` refuses longer ones.
  - **Put waypoints at turns:** just inside or outside the office door, at the corridor
    corner, at the far point of the loop.
  - **The 3-4 cm door threshold:** if a leg has to cross it, place the waypoints so the
    leg crosses it **straight on, not at an angle**. Half a foot on the threshold is what
    made it stumble before. Tell me which leg crosses it.
  - **Names:** short, letters/digits/`_` only, e.g. `office_door`, `corner1`, `far_end`.

## 1. Start localisation on the home mark

Robot on the home tape, standing still, facing the arrow.
```bash
./tools/live_slam.sh --start --record shadow_walk_01 --localize map01
./tools/live_slam.sh --status        # after ~15 s
```
You need `LOCALISED ...`, then a `TRACKING ...` line, plus `a bag recording IS running`.
**Keep the robot still until LOCALISED appears.**

## 2. Save `home` (still on the tape)

```bash
./tools/follow.sh mark home
```
It needs about 2 s of a still robot. It prints `saved home: x ... y ... heading ...`.

## 3. Save the other waypoints

For each one: drive there, stop, **wait 3 s with the robot completely still**, then:
```bash
./tools/follow.sh mark office_door      # your names
```
- If it says `robot not still`, wait a moment and run it again.
- If you put one in the wrong spot, drive to the right spot and re-run it with `--force`.

## 4. Check the plan

Drive back onto the home tape, still, then:
```bash
./tools/follow.sh list
./tools/follow.sh plan --from home office_door corner1 far_end office_door home
```
Use your own names, in walking order, ending at `home`. It prints each leg's length and the
turn it would make first, then `preflight: OK` or the reasons it would refuse. **Send me the
output** before step 5.

## 5. Shadow run (you drive; the follower only watches)

Robot on the home tape. Start this, then pick up the controller:
```bash
./tools/follow.sh shadow --from home office_door corner1 far_end office_door home
```
Drive the route **the way the follower would**. That makes its commands comparable with yours:
- At each waypoint, **first turn in place** toward the next one.
- **Then walk straight** to it, at a slow normal walk.
- **Stop on each waypoint for 3 s.** The follower prints `ARRIVED at ...` and moves on to the
  next leg.
- `WOULD ABORT: ...` lines are information, not a problem. Keep driving; I'll go through them
  afterwards.

It ends by itself after `home` with `[result] COMPLETED`. If it doesn't, press Ctrl-C when
you're back on the tape.

## 6. Stop and collect

```bash
./record_walk.sh --fetch shadow_walk_01
./tools/live_slam.sh --stop          # must print: ok  stopped; nothing left running
```

## 7. Tell me

- Where the robot ended against the home tape (per foot + edge, as before).
- Which leg crossed the door threshold.
- Anything unusual: a stumble, people in the way, a door.

---

## What I do with it (no robot needed)

- **Compare commands.** Your stick commands (in the bag) against the follower's would-be
  commands (its CSV on soc1, `~/slam_ws/shadow_*.csv`): turn direction, when it would stop
  turning, and how fast it would walk.
- **Check the arrivals.** Its `ARRIVED` positions against the waypoints, and any `WOULD
  ABORT` reasons.
- **Measure timing.** The DDS start-up time and the GetMcAction round trip on soc1, from the
  follower's logs.
- **If all of that is clean**, stage C (first motion) is next, again only with your OK.
