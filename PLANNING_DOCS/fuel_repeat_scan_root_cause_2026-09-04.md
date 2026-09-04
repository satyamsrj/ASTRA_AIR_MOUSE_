# Why the drone re-scans the same corridor and then ends up "stuck at the wall"

**Date:** 2026-09-04
**Trigger:** user report — "drone spawn, map the area and at last stuck to the wall … it is
performing SLAM at a corridor 2 to 3 times then move to next spot, it is totally waste of time."
**Method:** PX4 ulog analysis (`pyulog`) + FUEL's own `rosout` diagnostics + arena mesh geometry.
**Status:** two independent root causes found, both fixed. See §6 for verification.

---

## 1. What the logs actually show

This is **not** the wall-friction problem fixed in
`guard_state_validation_and_wall_stuck_fix_2026-09-04.md`, and it is not a guard problem — the
guard logged only 3 events across the whole failing run.

Measured on `~/.ros/log/2026-09-04/09_54_28.ulg` (438 s airborne, headless):

| metric | value |
|---|---|
| 1 m cells touched | 41 |
| total cell visits (>15 s gap = new visit) | 92 |
| **mean visits per cell** | **2.24** |
| cells visited ≥3 times | 12 |
| worst single cell | **10 visits** |
| heading reversals >120° | 38 = 5.21/min |
| stalls (speed <0.05 m/s for >3 s) | **0** |
| region actually covered | world x ∈ [2.2, 6.0], y ∈ [−5.6, −1.0] |

Two things follow immediately:

- The user's "scans a corridor 2–3 times" is real and quantified: **mean 2.24 visits per cell**.
- The vehicle **never explored the arena**. It stayed inside a ~3.8 × 4.6 m patch — roughly **9 %
  of the 175 m² of reachable free space** — for the entire 438 s flight. It was not stuck in the
  physical sense (zero stalls, mean speed 0.43 m/s); it was flying continuously and getting nowhere.

FUEL's own diagnostic in `rosout` shows the exploration had frozen solid. The last several samples
of the run are bit-identical:

```
Frontiers detected: 15 | Visitable: 10 | Dormant: 15 | Total Viewpoints: 243
Frontiers detected: 15 | Visitable: 10 | Dormant: 15 | Total Viewpoints: 243
...
```

Aggregated over the run: **dormant mean 16.25 (max 27) vs visitable mean 10.69** — about **60 % of
all frontiers had no usable viewpoint at all**. Dormant frontiers are excluded from the ATSP tour,
so the vehicle could only ever cycle between the same ~10 local frontiers. The FSM confirms it:
353 `EXEC_TRAJ` + 200 `PUB_TRAJ` transitions, **zero** finish states.

The viewpoint sampler's own rejection breakdown over the whole run (20 868 candidates):

| outcome | share |
|---|---|
| **Accepted** | **6.8 %** |
| Collision (inside inflated obstacle) | 32.5 % |
| **LowVis (`visib_num <= min_visib_num`)** | **30.6 %** |
| Clearance (`isNearUnknown`) | 16.0 % |
| OutMap (outside `box_*`) | 14.0 % |

## 2. Root cause A — the planner's sensor model does not match the sensor

Three separate mismatches, all in the same direction: FUEL believed the LiDAR could see things it
physically cannot.

### 2.1 Vertical field of view overstated by 2.14×

The VLP-16 actually spawned in Gazebo (`iris_vlp16.sdf`, sensor `velodyne-VLP16`) scans
`vertical min_angle/max_angle = ±0.261799 rad` — **±15°**, 16 beams.

`algorithm.xml` told the planner `perception_utils/top_angle = 0.56125` rad — **32.16°**.

`PerceptionUtils::insideFOV()` accepts a cell when `atan2(|dz|, dxy) <= top_angle_`. So every
frontier cell sitting between 15° and 32.16° of elevation was counted as "this viewpoint will
observe it" when the sensor can never return it.

### 2.2 The map's vertical band was taller than the sensor can close

`box_min_z/box_max_z` was `1.0 … 1.9` (0.9 m) with the vehicle hard-pinned to z = 1.4. To observe
a cell 0.5 m above the vehicle at ±15° you must stand **0.5 / tan 15° = 1.87 m** away horizontally.
Viewpoints were sampled at radii `{0.4, 0.84, 1.28, 1.72, 2.16, 2.6}` — **four of the six are
inside that blind cone**, yet the 32.16° model claimed anything beyond 0.79 m was fine.

The consequence is a permanent unknown shell that follows the vehicle. A frontier cluster spanning
the full box height has ~5 of its 9 z-layers (56 %) in that shell, so it can never reach
`min_view_finish_fraction = 0.6` and **never clears**. FUEL re-targets it, flies there, still
cannot see it, leaves, and comes back. That is precisely the "scan the same corridor 2–3 times"
behaviour, and precisely why the frontier count froze.

### 2.3 `min_visib_num` was compared against a downsampled cluster

`countVisibleCells()` iterates `frontier.filtered_cells_`, which is the cluster **after**
voxel-grid downsampling at `resolution * down_sample = 0.1 × 3 = 0.3 m`. A minimum-size cluster
(`cluster_min = 30` raw 0.1 m cells) collapses to roughly **5** filtered cells. Requiring
`visib_num > 10` was therefore unsatisfiable for small and mid-size clusters *no matter where the
viewpoint stood* — which is most of the 30.6 % LowVis rejections and a large part of the dormancy.

### 2.4 Fix

| parameter | was | now | why |
|---|---|---|---|
| `perception_utils/top_angle` | 0.56125 (32.16°) | **0.261799 (15°)** | match the sensor actually spawned |
| `box_min_z` / `box_max_z` | 1.0 / 1.9 | **1.2 / 1.6** | 0.25 m offset needs only 0.93 m standoff |
| `frontier/candidate_rmin` | 0.4 | **1.0** | keep every sampled radius outside the blind cone |
| `frontier/min_visib_num` | 10 | **3** | threshold is against ~5–20 *downsampled* cells |
| `perception_utils/max_dist` | 4.5 | **6.0** | was under-crediting standoff viewpoints; map raycasts to 6.5 |

All five are ROS parameters — **no rebuild required**. The 0.4 m slab is what the mission actually
needs (a 2D occupancy map at cruise altitude); the arena walls are 2.44 m tall so they are fully
captured in it.

## 3. Root cause B — FAST-LIO went blind in tight spaces and took PX4's estimator with it

This is the failure the user has been describing as "at last stuck to the wall", and it is
**pre-existing** — it predates every change in this document.

`catkin_ws/src/FAST_LIO/config/velodyne.yaml` set `blind: 2`. `Preprocess::sim_handler()` compares
squared range against `blind * blind`, so **every LiDAR return within 2 m of the sensor was
discarded before FAST-LIO ever saw it**.

In a 15 × 15 m arena with interior rooms, the vehicle is routinely within 2 m of several walls at
once. In tight spots essentially the entire scan was thrown away, leaving scan matching nothing to
register against. Caught red-handed on run `10_50_06`:

```
[WARN] [... 153.896]: No Effective Points!      (repeated)
```

At that moment the vehicle sat **0.20 m from the nearest wall with 187 wall cells inside the 2 m
blind radius**. The pose then propagated on IMU alone and drifted ~8 m. Because
`scripts/relay_odometry.py` feeds FAST-LIO's pose to PX4 as `/mavros/vision_pose/pose` and EKF2
fuses it as its position source (`EKF2_EV_CTRL = 11`), **EKF2 diverged with it**:

| t = 150 s | position | speed | altitude |
|---|---|---|---|
| **groundtruth** | stationary at world ≈ (1.8, 4.0) | ~0 | **1.01 m, descending** |
| **EKF2 estimate** | (13.88, 7.42) NED | **14.63 m/s** | **−2.30 m** |

The vehicle was hovering. The estimator thought it was doing 14.6 m/s underground. The controller
acted on that and the vehicle sank into the wall.

This is intermittent but common — **three of the six most recent runs** ended this way, with peak
EKF/FAST-LIO divergence of 15.07 m, 11.47 m and 10.52 m (from `logs/mission_telemetry_*.csv`). Two
of those three runs predate any change in this document.

### 3.1 Fix

`blind: 2` → **`blind: 0.5`** in `catkin_ws/src/FAST_LIO/config/velodyne.yaml`.

0.5 m still rejects self-returns: the VLP-16 sits 0.12 m above `base_link`, the rotors are ~0.1 m
below it at ~0.26 m radius with ~0.13 m props, so the lowest −15° beam can clip a prop tip out to
~0.38 m. The Gazebo sensor itself reports from `<min>0.1</min>`. 2.0 m was roughly 5× larger than
anything the airframe justifies.

Note this also retroactively validates `candidate_rmin = 1.0` from §2.4 — with a 0.5 m blind
radius, a 1.0 m viewpoint can genuinely see its frontier. Had `blind` stayed at 2.0, `rmin` would
have had to be raised above 2.2 m instead.

## 4. What was *not* the cause

Worth recording, because each was checked and cleared:

- **Arena connectivity.** Flood-fill of the wall mesh at z = 1.4 gives 175.3 m² of reachable free
  space, and it is *identical* from the spawn point, the arena centre, and the region the vehicle
  actually explored. Nothing was geometrically walled off.
- **The flight envelope guard.** 3 events in the whole failing run.
- **Wall friction.** Already fixed (`mu` 100 → 1.0); zero stalls measured, so no contact-sticking.
- **`mapping/fov_degree: 180`.** Looks wrong for a 360° sensor, but `laserMapping.cpp` clamps
  `FOV_DEG` to 179.9 regardless, so it is effectively "full". Left alone.

## 5. Files changed

- `catkin_ws/src/fuel/fuel_planner/exploration_manager/launch/algorithm.xml` — `top_angle`,
  `max_dist`, `candidate_rmin`, `min_visib_num`.
- `launch/nidar_fuel_upstream.launch` — `box_min_z` / `box_max_z`.
- `catkin_ws/src/FAST_LIO/config/velodyne.yaml` — `blind`.

## 6. Verification

_(filled in from live runs — see §6.1)_

### 6.1 Run 10_50_06 — FUEL fix only (`blind` still 2)

| metric | before (09_54_28 / 09_15_51) | after |
|---|---|---|
| dormant frontiers | mean 16.25, max 27 | **2–4** |
| visitable frontiers | mean 10.69 | **12** |
| viewpoints accepted per frontier | frequently **0**; 6.8 % aggregate | **23–41 of 222 (10–18 %)** |
| arena traverse | never left one quadrant in 438 s | **crossed to the opposite corner in 97 s** |

The exploration pathology is resolved. This run then hit root cause B at t = 153.9 s
("No Effective Points!"), which is what motivated §3.

### 6.2 Run 13 (`10_58_38.ulg`) — both fixes, 482 s airborne, headless

| metric | before (`09_54_28` / `09_15_51`) | after | change |
|---|---|---|---|
| **dormant frontiers** | mean 16.25, max 27 | **mean 0.94, max 2** | **−94 %** |
| visitable frontiers | mean 10.69, max 17 | **mean 14.62, max 20** | +37 % |
| viewpoint acceptance | 6.8 % | **14.9 %** | ×2.2 |
| **max EKF↔FAST-LIO divergence** | **15.07 m** (run lost) | **0.20 m** | — |
| `No Effective Points!` | fired, run lost | **0** | — |
| guard events | 3 | 2 | — |
| worst single cell | 10 visits | **7 visits** | −30 % |
| 1 m cells entered | 26 in 432 s | **34 in 482 s** | +31 % |
| mean visits per cell | 2.58 | 2.26 | −12 % |

**Resolved:** the dormancy trap, the frozen frontier set, the estimator blow-up, and the loss of
control. The vehicle flew 482 s with sub-0.2 m estimator agreement and **did not crash** — the run
ended in the **OOM killer** (`Killed` on all three roslaunch processes), not in a flight failure.

**Not resolved:** exploration still does not complete. Two observations from the same run:

- After t ≈ 100 s the vehicle stays pinned at **world y ≈ −6.0** (range −5.87 … −6.42) for the
  remaining 400 s, sliding in x between −3.25 and −6.33. It never enters the northern two-thirds or
  the eastern half of the arena.
- `Visitable: 15` holds essentially constant for the whole run while `Frontiers detected` (new
  clusters per cycle) sits at 1. So ~15 frontiers are permanently on the list, being deferred
  rather than cleared, and heading reversals rose to **7.46/min** — one every 8 s, which is tour
  thrashing rather than progress.

Caveat on the "cells entered" metric: it counts where the vehicle *flew*, not what the LiDAR
*observed*. With a 6.5 m raycast the vehicle can map far more than it overflies, so this
understates real coverage. Run 14 measures observed map coverage directly.

### 6.3 Next lever (not yet applied)

The remaining behaviour looks like ATSP tour instability rather than a coverage-model defect:
`exploration/refine_local = true` with `refined_num = 7` / `refined_radius = 5.0` re-solves the
local refinement on every replan (`fsm/replan_time = 0.2`), and with ~15 near-equal-cost frontiers
clustered together the chosen target can flip each cycle. Candidate experiments, in order:

1. Add hysteresis — do not switch target unless the new one is materially cheaper.
2. Raise `fsm/replan_time` / `thresh_replan*` so the vehicle commits to a target for longer.
3. Reduce `refined_radius` so refinement stops reshuffling distant frontiers.

Each needs its own measured A/B; none should be applied blind.

## 7. Known remaining issues (not fixed here)

1. **No mission end.** FUEL has no terminal condition wired up, and `mission_telemetry_logger.py`
   loops forever. Runs still end only by external kill. Tracked in
   `nidar_phase_plan_to_mission_complete.md` §2.3 / Phase 7.
2. **`pcd_save_en: true` with `interval: -1`** in `velodyne.yaml` accumulates every scan in RAM for
   the whole flight — the config's own comment warns "may lead to memory crash". Memory was
   observed at 12 GB of 14 GB plus 4 GB of swap during a run. Left enabled for now so this
   session's A/B stayed single-variable; **recommend disabling** for long runs.
3. **`scan_line: 32`** in `velodyne.yaml` for a 16-beam sensor. Harmless on the `sim_handler` path
   (over-allocation only), but wrong and worth correcting.
4. **FAST-LIO2 has no loop closure**, so absolute drift over a long flight is unbounded and
   currently unmeasured. The `diverge=` column in the telemetry CSV compares EKF2 against
   FAST-LIO, but `relay_odometry.py` *feeds* one into the other, so small values there prove
   coupling, **not** absolute accuracy. Groundtruth-vs-estimate is the only honest metric and is
   not currently logged continuously.
