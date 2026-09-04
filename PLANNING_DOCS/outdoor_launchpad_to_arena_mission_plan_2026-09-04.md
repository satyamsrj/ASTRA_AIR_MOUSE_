# Mission plan: launch pad outside → enter arena → map → return to pad

**Date:** 2026-09-04
**Requested:** spawn the drone outside the arena on `simulation/LAUNCHED PAD.STL`, navigate inside
using camera + front-facing LiDAR, map the arena with LiDAR, then return to the launch station.
**Status:** design proposal. Nothing in here is implemented yet.

This document does two things: §1 lists the problems with the proposal as stated (each backed by
something measured or read out of the tree, not opinion), and §2–§4 give the architecture and a
phased build order that works around them.

---

## 1. Problems with the proposal as stated

### 1.1 There is no camera on this vehicle — at all

The model Gazebo actually spawns is
`simulation/PX4-Autopilot-v1.14.3/Tools/.../models/iris_vlp16/iris_vlp16.sdf`. It contains exactly
three things: `model://iris`, a `velodyne_link` carrying the VLP-16 ray sensor, and the
`tfmini_lidar` downward rangefinder. `grep -c camera` on `iris.sdf` returns **0**.

So "using camera" is not a configuration change. It means adding a Gazebo camera sensor and
plugin, an image transport pipeline, a perception algorithm, and the CPU to run all of it — on a
machine that was already observed at 12 GB of 14 GB RAM plus 4 GB of swap during a plain
exploration run. The repo currently contains zero image-processing code.

### 1.2 A front-facing LiDAR would add nothing the VLP-16 does not already provide

The VLP-16 is a 360° puck — 1800 horizontal samples over a full circle. Forward coverage is
already there. Adding a forward LiDAR gives no new information.

**The real problem you were feeling was `blind: 2`.** `catkin_ws/src/FAST_LIO/config/velodyne.yaml`
discarded every return within **2 metres** of the sensor before FAST-LIO ever saw it, which is
exactly the close-range blindness that makes a forward sensor feel necessary. That has been fixed
today (`blind: 0.5`) — see `fuel_repeat_scan_root_cause_2026-09-04.md` §3. Please re-evaluate the
need for extra hardware *after* flying with that fix; the requirement may simply evaporate.

### 1.3 "LiDAR makes it wander outside" is a misdiagnosis, and the fix is already available

A LiDAR does not choose where to fly. What makes the vehicle wander in open space is **FUEL's
frontier objective**: outdoors every direction is an unexplored frontier, so the ATSP tour has no
reason to prefer the arena.

This matters practically: **if you add a camera but keep running FUEL outside, it will still
wander.** The correct fix costs nothing — FUEL already has `sdf_map/box_min_*` / `box_max_*`, which
bound where frontiers may be searched. Set the box to the arena interior and FUEL is structurally
incapable of targeting anything outside it, even while the vehicle is physically outside.

### 1.4 This arena has no door to find

Flood-filling the wall mesh (`nidar_arena/meshes/mapdraw.stl`, 878 triangles, 15.00 × 15.00 m
footprint, walls 2.44 m tall) at cruise altitude z = 1.4 m shows the **entire south face
(y = −7.5) is open**, broken only by two ~0.10 m posts:

| span on y = −7.5 | width |
|---|---|
| x = −7.10 … −1.10 | 6.00 m |
| x = −1.00 … 0.75 | 1.75 m |
| x = 0.85 … 7.20 | 6.35 m |

There is nothing to detect. Entry is a straight line flown northward across y = −7.5 anywhere
except two thin posts. The deleted `scripts/entry_detection_module.py` was built around detecting a
**0.5–1.2 m door** (`door_min_width` / `door_max_width`) — that premise does not hold for this
mesh, which is worth knowing before anyone restores it.

Because you plan to test with several arena STLs, §3.3 proposes deriving the aperture from
whichever mesh is loaded rather than hard-coding it or trying to see it with a camera.

### 1.5 Moving the spawn silently breaks three hard-coded frame constants

The spawn pose (world `(0, −6.5)`, yaw 1.5708) is currently written down independently in **three**
places, with a fourth derived by hand:

1. `scripts/test_takeoff.sh` — `SPAWN_X=0.0`, `SPAWN_Y=-6.5`, `SPAWN_YAW=1.5708`
2. `launch/nidar_fuel_upstream.launch` — `static_transform_publisher ... 0.0 -6.5 0.1 1.5707963 0 0 world map`
3. `scripts/flight_envelope_guard.py` — `camera_to_world()` / `world_to_camera()` with **6.5 as a
   bare literal** (`yw = xc - 6.5`, `xc = yw + 6.5`)
4. the FUEL box `box_min_x = -0.3 … box_max_x = 13.3`, which is the hand-computed camera_init
   pre-image of world ±6.8 *under that specific spawn*

Move the pad and all four must change together. Miss one and the guard converts poses into a frame
the planner is not using — the same class of bug as the double-rotation defect fixed on 2026-09-04.
**This must be fixed first** (§3.1); it is also the "single source of truth" P0 item already open in
`nidar_phase_plan_to_mission_complete.md` §0.2.

### 1.6 The envelope guard will reject every command outside the arena

`config/flight_envelope_guard.yaml` sets world bounds ±7.0 with a 0.2 m margin → an effective
envelope of ±6.8 m, and the guard rejects any command outside it. A pad at, say, world (0, −10) is
3.2 m outside that. **Every setpoint during transit and return would be rejected.** The guard needs
to become phase-aware (§3.4) — this is not optional, and it is easy to overlook until the vehicle
refuses to leave the arena.

### 1.7 The launch pad mesh needs unit, axis and size fixes before it can be used

`simulation/LAUNCHED PAD.STL` is a 12-triangle box measuring **609.60 × 150.00 × 609.60**, with the
**150 on the Y axis**. Three consequences:

- **Units.** Gazebo reads STL vertices as metres, so as-is this is a 610 m pad. Needs `<scale>`.
- **Axis.** The 150 (thickness) is on **Y**, but Gazebo is Z-up. Needs a roll rotation.
- **Size.** 0.6096 m is **smaller than the airframe**. Iris rotors sit at (±0.13, ±0.22) → 0.256 m
  radius, plus ~0.13 m propellers → roughly **0.75 m span**. The props would overhang the pad, and
  landing tolerance would be near zero — which collides directly with §1.9.

Recommended `<include>` (scales to a 1.5 m square, 0.03 m thick, and rotates Y-up → Z-up):

```xml
<include>
  <name>launch_pad</name>
  <uri>model://launch_pad</uri>
  <!-- roll +90deg maps mesh +Y (its up axis) onto world +Z -->
  <pose>PAD_X PAD_Y 0 1.5707963 0 0</pose>
</include>
<!-- inside the model's <mesh>: 1.5/609.6 = 0.00246 across, 0.03/150 = 0.0002 in thickness -->
<scale>0.00246 0.0002 0.00246</scale>
```

The 0.03 m thickness is deliberate — see §1.9.

### 1.8 Outdoors, LiDAR localization is weakly observable — this *is* a LiDAR problem, just not the one you named

At 1.4 m altitude with a ±15° vertical FOV, the downward beams terminate on the ground at
1.4 / tan 15° ≈ **5.2 m** radius. Outside the arena, most of the scan is therefore flat ground,
which constrains altitude, roll and pitch but gives **no horizontal information at all**.

Horizontal constraint comes only from the arena structure. A single flat wall fixes distance-to-wall
and yaw but leaves sliding *along* the wall unobservable — the classic corridor degeneracy. Since
FAST-LIO2 has no loop closure, that drift is never corrected.

Mitigations, in order of effectiveness:

- **Keep the transit short.** Put the pad 2–3 m outside the aperture, not 10 m. Less time outside
  is less accumulated drift.
- **Park where two non-parallel surfaces are visible** — near a corner of the arena, so both faces
  constrain the solution.
- Initialise FAST-LIO stationary on the pad with the arena in view, so `camera_init` is anchored
  against real geometry rather than open ground.

### 1.9 The pad will fight the altitude pin

`laserMapping.cpp` pins published Z from the TFmini:
`g_pinned_height_camera_init = range + 0.05 − 0.1`. Flying from over the floor to over a raised pad
*reduces* the measured range by the pad's thickness, so the pinned Z drops and the controller
climbs to compensate — a spurious altitude step exactly during the most delicate part of the
mission (terminal descent onto a 1.5 m target).

With a **0.15 m** pad that step is 0.15 m. Making the pad **0.03 m** thick (§1.7) reduces it to
noise. If a thick pad is required later, the Z-pin must be frozen during `DESCEND` instead.

### 1.10 Return-to-pad accuracy is currently unbounded and unmeasured

FAST-LIO2 has no loop closure, so absolute drift grows without correction. Note that the
`diverge=` column in `logs/mission_telemetry_*.csv` does **not** measure this: `relay_odometry.py`
feeds FAST-LIO's pose into EKF2 as `/mavros/vision_pose/pose`, so the two estimates are coupled by
construction and small values prove only that the coupling works. The only honest metric is
estimate-vs-`vehicle_local_position_groundtruth`, which is not currently logged continuously.

Until that is measured, "return to camera_init (0,0)" has **no known landing accuracy**. §3.7
covers this.

### 1.11 There is no mission end today, and "no frontiers left" is not a safe trigger

Nothing in the stack declares exploration complete; `mission_telemetry_logger.py` loops until ROS
dies. Worse, before today's fix roughly **60 % of frontiers were permanently dormant**, so a
"wait until no frontiers remain" condition would have waited forever. Even with that fixed,
RETURN must have a **mission-clock backstop** rather than depending solely on FUEL's own opinion.

---

## 2. What actually needs building (and what does not)

| Task | Proposed approach | Recommended approach |
|---|---|---|
| Stop wandering outside | camera + front LiDAR | **FUEL `box_*` = arena interior only** (free) |
| Find the entrance | camera detection | **derive aperture from the arena STL offline** (§3.3) |
| Close-range obstacles | front-facing LiDAR | **`blind: 0.5`** — already fixed today |
| Map the arena | LiDAR + FUEL | unchanged, now that §1 of the root-cause doc is fixed |
| Return to pad | — | deterministic reverse transit + §3.7 accuracy work |
| Land on pad | — | enlarge pad first; ArUco only if tolerance demands it |

**Where a camera genuinely earns its place:** victim detection (the actual scored objective of
NIDAR RescueSwarm) and precision landing. Not entry navigation. Recommend deferring the camera
until the mission skeleton flies end-to-end, then adding it for detection — that ordering also
keeps the CPU budget free while the harder localization work is being validated.

---

## 3. Proposed architecture

### 3.1 Phase A — single source of truth for the spawn/frame chain *(prerequisite)*

Create `config/mission_frames.yaml`:

```yaml
mission_frames:
  spawn_world:  {x: PAD_X, y: PAD_Y, z: 0.05, yaw: 1.5707963}
  arena_bounds: {x_min: -7.0, x_max: 7.0, y_min: -7.0, y_max: 7.0}
  cruise_altitude_world: 1.5
  pad: {size: 1.5, thickness: 0.03}
```

Then:
- `flight_envelope_guard.py` builds `camera_to_world` / `world_to_camera` from `spawn_world`
  (rotation by `yaw`, translation by `x,y`) instead of the hard-coded 6.5.
- the `world → map` static TF is published from the same params.
- the FUEL `box_*` values are computed from `arena_bounds` + `spawn_world`, not hand-derived.
- `test_takeoff.sh` reads its spawn args from the same file.

**Acceptance:** change `spawn_world` alone, spawn at three different poses, and confirm
`world_to_camera(camera_to_world(p)) == p` for each, plus a guard that accepts the arena centre in
all three. Nothing else in the tree should need editing.

### 3.2 Phase B — put the pad in the world

Add `simulation/custom_models/launch_pad/` (mesh + `model.sdf` with the §1.7 scale/pose), reference
it from `nidar_competition.world`, and set `spawn_world` to sit on it.

Set the pad's collision `<mu>`/`<mu2>` to **1.0** explicitly — the arena mesh shipped with `mu=100`
and caused a multi-session wall-sticking bug.

**Acceptance:** vehicle rests level on the pad; TFmini reads `cruise_alt − pad_thickness` on the
pad and `cruise_alt` off it, with the difference under 0.05 m.

### 3.3 Phase C — derive the entry aperture from whatever arena STL is loaded

`scripts/derive_arena_apertures.py`: slice the arena mesh at cruise altitude, rasterise, flood-fill
from outside, and report every opening in the outer boundary with its centre, width and inward
normal. Emit the chosen aperture plus suggested transit waypoints into `mission_frames.yaml`.

This is the piece that makes your "test with various arena STL files" plan work: drop in a new
mesh, re-run the script, and the mission reconfigures itself. It replaces camera-based entry
detection with a deterministic, testable computation.

**Acceptance:** run against `mapdraw.stl` → reports the south face open with the three spans in
§1.4. Run against an arena with a real 1 m door → reports one span of ~1 m.

### 3.4 Phase D — phase-aware envelope guard

Two envelopes instead of one:

- **MISSION** — convex hull of {pad, transit corridor, arena interior}, active in
  `TAKEOFF / TRANSIT_IN / TRANSIT_OUT / DESCEND`
- **EXPLORE** — today's arena interior ±6.8, active only in `EXPLORE`

The guard subscribes to `/mission/phase` and switches. Keep the existing rejection and
fallback-hold behaviour unchanged; only the bounds move. **Fail closed:** if `/mission/phase` is
stale, fall back to the tighter EXPLORE envelope.

### 3.5 Phase E — mission manager FSM

`scripts/mission_manager.py`, publishing `/mission/phase`:

```
IDLE → ARM → TAKEOFF → TRANSIT_IN → EXPLORE → TRANSIT_OUT → DESCEND → LAND → DISARM
                                        ↓ (any failsafe)
                                    ABORT → TRANSIT_OUT
```

- **TRANSIT_IN / TRANSIT_OUT** — a deterministic waypoint follower publishing to
  `/planning/pos_cmd`, the same interface the guard already validates. FUEL is *not* running.
- **EXPLORE** — publish the FUEL trigger; hand `/planning/pos_cmd` to `traj_server`.
- The FSM owns the **mission clock** (open item §2.3 of the phase plan).

Only one node may drive `/planning/pos_cmd` at a time — a mux with the FSM as arbiter. Two
publishers on that topic would produce exactly the kind of contention already seen when duplicate
FAST-LIO nodes fought over `/Fast_LIO/odometry`.

### 3.6 Phase F — termination

Enter `TRANSIT_OUT` on the **first** of:

1. FUEL reports zero visitable **and** zero dormant frontiers for ≥10 s consecutively;
2. mapped-volume growth < 1 % over 60 s (coverage plateau);
3. mission clock expiry (hard backstop);
4. battery/failsafe.

Conditions 2 and 3 exist because condition 1 alone is exactly what could not be trusted before
today's dormancy fix.

### 3.7 Phase G — landing accuracy *(do last, size it from data)*

Before building anything: log estimate-vs-groundtruth continuously for a full-duration flight and
measure actual terminal error at the pad. Then choose:

- error ≪ pad radius → land open-loop, done;
- error comparable to pad radius → enlarge the pad, or add a TFmini-based pad-edge confirmation;
- error ≫ pad radius → add the downward camera + ArUco (`aruco_cam` model already ships with PX4).

Do not build the camera pipeline before this measurement exists.

---

## 4. Build order and risk

| Phase | Depends on | Risk if skipped |
|---|---|---|
| A — frame single-source | — | everything downstream silently mis-transforms |
| B — pad in world | A | — |
| C — aperture derivation | — (parallelisable) | mission hard-codes one arena |
| D — phase-aware guard | A | vehicle cannot leave the arena at all |
| E — mission FSM | A, B, C, D | no mission, only free exploration |
| F — termination | E | never returns |
| G — landing accuracy | E, F | lands somewhere near the pad, unquantified |

**Do A first and alone.** It is the prerequisite for B, D and E, it is independently testable, and
it closes an already-open P0 item.

### Open risks not resolved by this plan

1. **Outdoor localization degeneracy (§1.8)** is mitigated, not solved. If drift outside proves
   unacceptable, the options are GPS/baro fusion outside the arena (toggling `EKF2_EV_CTRL` by
   phase — invasive and easy to get wrong) or adding features to the outdoor area.
2. **FAST-LIO has no loop closure.** Long missions accumulate unbounded drift. Out of scope here,
   but it caps how long a mission can run before RTL becomes unreliable.
3. **CPU/RAM headroom.** A run already reaches 12 GB of 14 GB. Adding a camera pipeline needs a
   measured budget first, not an assumption.
4. **`pcd_save_en: true` with `interval: -1`** accumulates every scan in RAM for the whole flight;
   its own config comment warns of a memory crash. Recommend disabling before any long mission.
