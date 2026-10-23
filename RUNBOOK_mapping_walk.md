# Mapping walk + test walk — step by step

Two recordings. Nothing runs on the robot except the recorder. You drive with the
controller exactly as usual; no SLAM, no commands from us.

- **Mapping walk** (`map_walk_01`): builds the map. It must see every route **in both
  directions**, because the LiDAR only looks forward (123 deg).
- **Test walk** (`test_walk_01`): a *different* walk, starting and ending on the same mark.
  It tests whether the robot can find itself on that map. The mark is the ground truth.

Do both in one session, back to back, so the furniture stays the same.

If you are out of office WiFi range, join the robot's AP and put `X2_HOST=soc1ap` in front
of every `./record_walk.sh` command below.

---

## 0. The home mark (once)

- Pick a spot with **things in view** (desks, shelves, door frames), not facing a bare
  wall. You will turn the robot in place here, and turning in front of a blank wall is the
  one motion LiDAR odometry handles badly.
- Tape **both foot outlines**, a **heading arrow** (the way the robot faces), and a
  **straight edge ~1 m long** along the robot's side. The edge is what makes the heading
  measurable.
- This mark becomes the map's origin, the "home" every later test starts from. **Leave it
  on the floor** after today.

---

## 1. Mapping walk

**1. Start the recorder.**
```bash
./record_walk.sh --detach -n map_walk_01 --max 900
./record_walk.sh --list
```
`--list` must show `>>> RECORDING IN PROGRESS: map_walk_01`. If it doesn't, stop here.

**2. Robot on the mark, facing the arrow. Stand still 15 s.**

**3. Loop 1, in one direction** (say clockwise). Walk every part of the floor you will
later want it to navigate. A dead-end corridor or room: walk in, turn, walk out. That
already covers both directions of that stretch.

**4. Back on the mark, facing the arrow. Stand still 10 s.** This is a checkpoint, so align
the feet as well as you reasonably can.

**5. Turn 180 deg in place, slowly** (take ~10 s for the half turn).

**6. Loop 2: the same route in the opposite direction** (counter-clockwise). Same
dead-ends, same rooms.

**7. Back on the mark.** It now arrives facing *against* the arrow. **Turn 180 deg in place,
slowly**, so it faces the arrow again. Align feet and edge. **Stand still 15 s.**

So yes: you turn at the mark between the loops, it comes back facing the opposite way,
and **at the very end you turn it once more** so it finishes exactly as it started. Same
spot, same heading, which is the cleanest possible check.

**8. Stop and collect.**
```bash
./record_walk.sh --fetch map_walk_01
```

Tips:
- Normal walking speed is fine. Turns at corners: normal. Turns in place: slow.
- Keep people (yourself included) **beside or behind** the robot where you can. Anyone in
  front of it gets baked into the map as a wall.
- 3-6 minutes total is plenty. Longer is fine (`--max 900` auto-stops at 15 min).

---

## 2. Test walk (right after)

It does **not** need both directions. Its job is to be **different** from the mapping
walk, while staying inside the area the mapping walk covered (the robot can only find
itself where the map has been).

**1. Start the recorder.**
```bash
./record_walk.sh --detach -n test_walk_01 --max 600
./record_walk.sh --list
```
**2. Robot on the mark, facing the arrow. Stand still 15 s.**

**3. Walk a different path** through the mapped area: cut across, do a figure-8, go part
way and come back, one loop in either direction with a detour. 1-3 minutes.

**4. Back on the mark, facing the arrow** (turn in place if needed), align feet and edge.
**Stand still 15 s.**

**5. Collect.**
```bash
./record_walk.sh --fetch test_walk_01
```

---

## 3. Tell me

- **Repositioning at each stop:** mapping walk step 4 (mid checkpoint), step 7 (end), and
  test walk step 4. Per foot, plus the heading edge, as before (e.g. "left foot 3 cm ahead,
  right foot on the line, edge 1 cm off at the far end").
- Anything unusual: a stumble, someone walking in front for long, a door that was open in
  one walk and closed in the other.

## What I do with it (no robot needed)

1. Map: replay `map_walk_01` -> pgo -> `map.pcd`. Its mid and end checkpoints give two
   closure measurements against the mark.
2. Test: localise `test_walk_01` on that map, twice:
   - starting from the mark (known start), and
   - with the global search (unknown start).
   Both must put the test walk's start **and** end on the map's home pose, within the
   repositioning error. That is the first relocalisation result with real ground truth.
