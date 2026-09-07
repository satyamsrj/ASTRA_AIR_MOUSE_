# Airframe / arena fit — root cause of the stuck mapping and the wall crashes (2026-09-06)

Status: **plan, not yet implemented.** Decision taken: shrink the airframe.

Evidence base: `~/.ros/log/2026-09-06/12_23_19.ulg` (run A, crash at t=339),
`~/.ros/log/2026-09-06/14_58_08.ulg` (run B, stuck 406 s then crash at t=502),
`/tmp/fuel.log`, `/tmp/fast_lio.log`, `simulation/custom_models/arina_nidar/meshes/arina_nidar.stl`.

---

## 1. What was actually wrong

### 1.1 It is not the PID tuning

Run B, 430 s of flight, all three cascaded loops measured against their own setpoints:

| loop | median error | p90 |
|---|---|---|
| rate (roll) | 1.69 °/s | 15.1 °/s |
| attitude (roll / pitch) | 0.54° / 0.44° | 2.49° / 1.57° |
| **position** | **0.392 m** | 0.673 m |

The inner loops are tight. A 0.5° attitude error with a 0.39 m position error means the
position reference is being commanded where the airframe cannot physically go. Retuning
gains cannot fix this and must not be attempted as a remedy.

### 1.2 The vehicle does not fit the arena

```
x500 rotor hub          (±0.174, ±0.174)  -> arm 0.246 m
x500 rotor blade box    0.2792 m long     -> half 0.1396 m
PROP-TIP RADIUS         0.386 m           -> PROP DISC 0.771 m
   (this is exactly vehicle.collision_radius = 0.386 in mission_config.yaml)

arena at cruise height 1.75 m: 606 wall segments, walls 2 cm thick,
   median clearance 0.400 m  ->  typical corridor ~0.80 m
```

Published guidance for conventional (non-morphing) quadrotors is a corridor of
**1.3–1.5× the vehicle diameter**. The x500 needs 1.00–1.16 m and has 0.80 m.

Flood fill from the arena entrance (x = 0, y = −7.5), 5 cm grid:

| required clearance | reachable | % of 158 m² |
|---|---|---|
| 0.30 | 121.2 m² | 76.7% |
| 0.35 | 108.6 m² | 68.7% |
| 0.38 | 100.9 m² | 63.9% |
| **0.40** | **95.8 m²** | **60.6%** |
| 0.45 | 82.7 m² | 52.4% |
| **0.50** | **0.9 m²** | **0.6%**  ← arena disconnects |

`sdf_map/obstacles_inflation` is pinned at **0.400** — the only value that keeps the arena
connected (see the comment at `algorithm.xml:51`). Against a 0.386 m prop radius that is a
**14 mm** margin, versus a measured localisation error of **101 mm median / 212 mm p90**
(run A) and **160 mm / 314 mm** (run B).

**Consequence, measured in run B:** median wall clearance 0.342 m — *below* the prop radius.
**50.8% of the flight had the propeller disc inside a wall.** The vehicle spent 226 s (49.7%)
pinned in two adjacent 1 m cells and visited 39 of ~225 cells. Coverage froze at 38.3% /
97.5 m² left from t=95 to t=501 — 406 s of zero progress. That is the "mapping one room for
five minutes" symptom, exactly.

### 1.3 When it does touch, the contact is a simulation artifact

```
x500 rotor collision  = BOX 0.2792 x 0.0169 x 0.00085 m, spinning to 1000 rad/s
iris / px4vision      = CYLINDER r = 0.128 (the full swept disc)

world max_step_size = 0.004 s:
   620 rad/s (cruise) -> 142 deg per step, blade tip teleports 0.346 m
  1000 rad/s (max)    -> 229 deg per step, blade tip teleports 0.558 m
```

A thin blade that jumps a third of a metre per physics step cannot be collision-checked.
Most steps it misses the wall; one step it appears deeply embedded, and ODE resolves that as
an impulsive torque. Run A, t=332.612, one 40 ms sample:

```
t=332.572   yaw rate  +5.1 °/s    |a| = 1.002 g
t=332.612   yaw rate -68.4 °/s    |a| = 0.938 g    <- pure torque, NO linear impact
            -> runs away to -427 °/s while the rate controller is pinned at its +60 limit
            -> allocator saturates (m = [1.00, 0.66, 0.16, 0.05]), thrust collapses
            -> ground strike t=334.68 (8.5 g), overshoot to 6.81 m, free fall at -6 m/s
            -> second impact t=338.65 (10.1 g), FD roll trip t=339.06, terminate
```

Blade-tip clearance computed against the real STL with full attitude confirms contact begins
at exactly t=332.60 (0.011 m) and persists to t=334.30. A cylinder disc has no angular
dependence, so the same graze becomes a smooth bounded scrape.

### 1.4 Secondary defects found along the way

- **FSM never reaches FINISH.** `grep -c "state: FINISH" /tmp/fuel.log` = 0. It only transitions
  on `NO_FRONTIER` (`fast_exploration_fsm.cpp:214`); stale viewpoints keep `planExploreMotion`
  returning SUCCEED. Run A reached "0.0 m² left" at t=297.9 and crashed at t=332.6 — **34.7 s
  of flying with nothing left to map.**
- **Simulation does not stop on crash.** At flight termination the PX4↔Gazebo lockstep
  handshake stalls: RTF collapses to exactly 0.0040 (one 0.004 s step per wall second),
  `[simulator_mavlink] poll timeout` ~200×/sim-second, gzserver 5% CPU with 72 threads in
  `futex_wait`. Not CPU load and not swap thrash — an idle deadlock. Nothing in the launch
  stack watches for `system_status == 8`.
- **Altitude runs 0.2 m above the guard ceiling.** Ground truth cruise 1.71–1.80 m world vs
  `flight_envelope_guard.yaml world_z_max: 1.55`, unnoticed because the EKF reads 0.26 m low.

---

## 2. Decision

**Shrink the airframe.** Target the `px4vision` class:

```
px4vision  rotor hub (±0.0935, ±0.107) -> arm 0.142 m
           rotor collision CYLINDER r = 0.128
           PROP-TIP RADIUS 0.270 m  ->  DISC 0.540 m
           mass 1.54 kg, thrust 41.4 N
```

0.540 m disc in 0.80 m corridors = **1.48×**, inside the recommended 1.3–1.5 band, and it
already uses cylinder rotor collisions, so §1.3 is fixed by the same change.

**Chosen operating point: R = 0.270, inflation = 0.40 → margin 130 mm, reachable 60.6%.**
That is a ~9× increase in margin over today for the same reachable area.

---

## 3. Implementation steps

### Step 1 — new vehicle model `nidar_x` (replaces `x500_vlp16`)

Base it on `px4vision` proportions, **not** a straight copy — two things must be corrected:

1. `px4vision` `base_link_inertia_collision` is a **box 0.47 × 0.47 × 0.11**, whose corners
   reach 0.332 m — *outside* the 0.270 m rotor tips. That box is the whole airframe
   footprint, not the centre body. Replace with a centre body ≤ 0.20 × 0.20 × 0.11
   (half-diagonal 0.141 m), so the true collision radius really is 0.270 m.
2. `px4vision` has **no landing gear collision** — the same defect that drove the move off the
   iris. Add skid rails in the x500 style, sized to stay inside 0.270 m radius and to leave
   belly clearance for the VLP-16 / TFmini / camera.

Carry over unchanged from `x500_vlp16.sdf`: the velodyne mount at +0.20, the TFmini pose
(−0.0328, −0.0005, −0.0573), the camera link, and the generated mast.

### Step 2 — re-derive mass, inertia and thrust (do not copy x500 values)

```
frame              1.54 kg   (px4vision)
VLP-16             0.83 kg
camera             0.095 kg
TOTAL              2.465 kg  -> 24.18 N

px4vision thrust   41.4 N    -> T/W = 1.71   (x500_vlp16 today: 2.02)
MPC_THR_HOVER      1/1.71    = 0.585
```

T/W 1.71 is flyable at a 0.6 m/s cruise but below the current 2.02. Either accept it, or
scale `motorConstant` by 48.4/41.4 = 1.17 to restore T/W 2.0. **Decide explicitly and record
the choice in the airframe file**, per the existing convention in
`1025_gazebo-classic_x500_vlp16` ("Change one and re-derive the other").

Re-derive `CA_ROTOR*_PX/PY` from the new hub positions — the current file's 0.1515/0.245
values must not be carried over.

### Step 3 — propagate the new radius through the config chain

`mission_config.yaml` is the single source; `apply_mission_config.py` regenerates the rest
and already has guardrails that refuse on inconsistency.

```
vehicle.collision_radius   0.386  ->  0.270
planner.obstacles_inflation 0.400 ->  0.400   (unchanged; margin goes 14 mm -> 130 mm)
vehicle.model              x500_vlp16 -> nidar_x
```

Add a **new guardrail**: refuse to generate if
`obstacles_inflation - collision_radius < 0.10` (the measured median localisation error).
That single assertion would have caught this class of bug before either crash.

### Step 4 — the two independent fixes (safe to do regardless of Step 1)

- Enter `FINISH` when the coverage telemetry reports 0 m² left, not only on `NO_FRONTIER`.
- Crash watchdog: tear the sim down on `system_status == 8`.

### Step 5 — reduce the localisation error (parallel workstream, NOT optional long-term)

Measured decomposition (run B): per-sample **noise 0.092 m std**, slow drift only 0.039 m std,
and **no correlation with speed (r = +0.028)**.

Consequences for the plan:
- This is jitter, not accumulating SLAM drift — loop closure and better initialisation will
  not help.
- The missing motion de-skew (`lidar_type: 4` disables it) is **not** the dominant term. Do
  not spend effort there first; the earlier hypothesis is not supported by this measurement.
- With 0.092 m noise, the p90 error (0.31 m) still exceeds even the improved 130 mm margin
  ~30% of the time. Contacts will become rare and non-catastrophic, not impossible.

Candidate causes to investigate, in order: FAST-LIO `filter_size_map` / `filter_size_surf`
smearing the 2 cm walls; `point_filter_num` decimation; `EKF2_EV_DELAY = 0` against real
ROS→MAVROS transport latency. **This is a hypothesis list, not a conclusion.**

---

## 4. Verification criteria (all measured from ground truth, not from the estimator)

A run is only acceptable if **all** of these hold:

1. `min blade-tip clearance to the arena STL > 0` for the whole flight — zero wall contacts.
2. Fraction of flight with clearance < collision_radius: **0%** (was 50.8%).
3. No 10 s window with churn ratio (path ÷ net displacement) > 10 (worst was 37.7).
4. Coverage strictly increasing; no window > 60 s with < 1 m² progress (was 406 s at zero).
5. FSM reaches `FINISH` and `/exploration_completed` publishes.
6. No `Attitude failure` / flight termination in the ulog.
7. RTF stays > 0.3 to the end of the run.

Reachable-area ceiling for this arena at inflation 0.40 is **60.6%** — a run that maps ~60%
and terminates cleanly is a *success*, not a regression. Do not chase 100%: the remaining
39% is behind gaps that disconnect at 0.50 m clearance.

---

## 5. Known risks

- **T/W drops to 1.71** with the VLP-16 on a 540-class frame. Real, but a 0.83 kg lidar on a
  0.54 m airframe is at the heavy end of what that class carries. If it proves marginal, the
  repo already vendors `livox_ros_driver` — a Mid-360-class sensor (~0.27 kg) would restore
  T/W above 2.2 and is the conventional choice for this vehicle class. Larger change; not in
  this plan.
- **PX4 rate/attitude gains are scale-dependent.** The current gains were validated on the
  x500; a 0.54 m frame with different inertia will need a tuning pass. §1.1 shows the present
  gains are good — do not discard them, re-check them.
- **The arena itself was not re-verified.** 0.80 m corridors with 2 cm walls is a demanding
  maze; if `ARINA_NIDAR.STL` does not match the real NIDAR spec, the sizing target changes.
  This plan assumes the mesh is correct, as decided.
