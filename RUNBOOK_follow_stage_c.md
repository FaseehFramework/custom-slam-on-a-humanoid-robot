# Stage C: first follower-driven motion, step by step

**This is the first time the follower moves the robot.** Three short tests, each one only
after the previous one is clean and we have talked:

- **C1:** home -> waypoint `c1` (3.06 m, saved 2026-10-21). Tests the turn in place, the walk
  and the stop. **Follower v2 (2026-10-21):** turns in place at ~30-55 deg/s, walks at ~0.45 m/s
  (mc's slowest walk; the first try at "0.12 m/s" could not turn at all, see README "Stage C1").
- **C2:** c1 -> home, and the follower is **killed mid-walk on purpose**. Only mc's own
  timeout can stop the robot then. Tests the last untested backstop.
- **C3:** home -> office_door (5 m), the leg you walked straight in the shadow run.

**Your job in every test:** hold the controller, hands on the sticks.
- **Any stick input cancels the run immediately.** It then stays cancelled; it does not
  resume when you let go.
- **E-stop** if the robot does anything you don't like and the stick does not stop it.
- Keep the area of each leg clear of people, and keep away from the other robot and the sofas.

Run every command from the repo on the laptop. On the robot's AP, put `X2_HOST=soc1ap` in
front of each one.

---

## Before you start

- Robot on, torque on, standing. Controller connected.
- **The home tape.** The same tape as stage B.
- **Choose the c1 spot:** 1.5-2 m from home, **off to one side**, so the robot has to turn
  about 45-90 deg first. The floor in between and around it must be clear, with no threshold.
  Tape it like home: both feet + an arrow.

## 1. Start localisation and recording on the home mark

Robot on the home tape, still, facing the arrow.
```bash
./tools/live_slam.sh --start --record follow_c_01 --localize map01
./tools/live_slam.sh --status      # after ~15 s: LOCALISED, then TRACKING, and the recording
```
**Paste the status output to me.**

## 2. Save c1 (already done 2026-10-21: skip this step unless the c1 tape has moved)

Drive the robot onto the c1 tape and stand it still for 3 s:
```bash
./tools/follow.sh mark c1
```
Then drive back onto the home tape, aligned with the arrow, still.

## 3. Check the plan

```bash
./tools/follow.sh plan --from home c1
```
It should show one leg of 1.5-2 m with a turn first, and `preflight: OK`.
**Paste it to me and wait for my go-ahead.**

## 4. C1: home -> c1

Hands on the controller. Then:
```bash
./tools/follow.sh go --from home c1
```
Type `GO`, then hold the controller.

**What should happen:**
1. Within about a second, the robot starts stepping (mc switches to walking by itself).
2. It turns in place toward c1, briskly (about 30-55 deg/s, like your half stick), and stops
   turning a little early. It may still be up to ~15 deg off; it fixes that while walking.
3. It walks: up to speed in under a second, then your normal slow walk (~0.5 m/s). (Measured
   in C1-C3; the 3 s creep seen in the calibration was at smaller commands.)
4. **It stops close to c1: within ~0.2 m short of it, or slightly past.** Forward is cut
   0.35 m out and the robot coasts in.
5. It stands again after about 4 s. The terminal shows `ARRIVED`, `[result] COMPLETED` and
   `after the stop: stopped`.

**If it turns the wrong way, walks toward anything, or does not stop: flick a stick.** (A wrong
turn direction is caught by the follower too, but only after up to ~60 deg of rotation.)

**Then tell me:** where it stopped against the c1 tape (cm, per foot), and paste the terminal
output.

## 5. C2: c1 -> home with a deliberate crash

**Only after my go-ahead on C1.** Robot on the c1 tape or near it, still.

The leg first:
```bash
./tools/follow.sh plan --from c1 home
```
Then, in a **second terminal**, start the crash helper **first**:
```bash
./tools/follow.sh crash
```
It waits for the follower. Then, in the first terminal:
```bash
./tools/follow.sh go --from c1 home
```
Type `GO`, then hold the controller.

**What should happen:**
1. The robot turns toward home and starts walking.
2. 4 s into the walk (at walking speed), the crash helper kills the follower. The first
   terminal shows the connection end, with no `[result]`.
3. **The robot stops by itself within about 1.5 s** and stands again a few seconds later.
   (Done 2026-10-21: mc dropped the command 0.21 s after the kill; the robot was nearly still
   1.25 s after it and had travelled 0.37 m.)

**If it keeps walking more than ~2 s after the kill message: E-STOP.** Not just the stick:
if mc did not drop the dead follower's command, letting go of the stick could hand control
back to it.

**Tell me** whether and how quickly it stopped, and paste both terminals.

## 6. C3: home -> office_door

Only after my go-ahead. Drive the robot back onto the home tape, then:
```bash
./tools/follow.sh plan --from home office_door
./tools/follow.sh go --from home office_door
```

## 7. C4: two legs, home -> c1 -> office_door, then align at the door

**Done 2026-10-22, clean** (bag `follow_c_03`; README "Stage C with follower v2"). Added
2026-10-21 after C1-C3 were clean. Tests chaining two waypoints, a new 2.2 m leg
(c1 -> door; operator: clear and straight) and the first `--align-final`. It avoids the ~160
deg turn at the door until the drift after big turns (README "Stage C with follower v2") is
fixed.

1. Robot on the home tape, still, facing the arrow:
   ```bash
   ./tools/live_slam.sh --start --record follow_c_03 --localize map01
   ./tools/live_slam.sh --status      # after ~15 s: TRACKING, and the recording running
   ```
   **Paste the status to me.**
2. ```bash
   ./tools/follow.sh plan --from home c1 office_door
   ```
   Two legs, `preflight: OK`. **Paste it and wait for my go-ahead.**
3. Hands on the controller:
   ```bash
   ./tools/follow.sh go --from home c1 office_door --align-final
   ```

**What should happen:**
1. Leg 1 exactly as C1: turn ~49 deg left, walk, stop near the c1 tape.
2. **At c1 it stops and stands for ~2-5 s.** That pause is the follower measuring where it
   stopped, not a fault.
3. Leg 2: it turns right ~55-60 deg toward the door, stopping the turn ~20 deg early, and walks.
   It may wander up to ~0.4-0.6 m off the straight line in the first 2 s before the steering
   takes hold (seen in C2). **If it heads for the door frame or anything else: flick a stick.**
4. About 0.35 m before the door it stops walking and **turns left in place, ~40 deg**, toward
   the direction saved at the door (35 deg in the map). It will likely stop 10-25 deg short of
   it: the turn stops early by design.
5. `[result] COMPLETED`, two `ARRIVED` lines, `after the stop: stopped`.

**Then tell me** where it stopped at c1 (cm against the tape) and how it faced at the door,
and paste the terminal.

## 8. Stop and collect

```bash
./record_walk.sh --fetch follow_c_03      # the name you recorded under
./tools/live_slam.sh --stop
```

---

## If something goes wrong

- **`[abort] ...` in the terminal:** the follower stopped itself and sent zeros. Paste the
  reason to me. Nothing else is needed.
- **`REFUSING ...`:** nothing was published. Paste it to me.
- **Leaving the follower:** after any stop, it never moves the robot again until you type a
  new `go`.
