# Execution plan for mission plan v3 — measured against the current tree

**Date:** 2026-09-05
**Implements:** `mission_implementation_plan_2026-09-04-v3.md`
**New assets:** `simulation/ARINA_NIDAR.STL`, `simulation/A8_mini_Camera.stl`
**Primary acceptance test:** the vehicle spawns on a pad outside the arena and **flies through the
arena door autonomously**, verified in a realtime run.

---

## 0. Corrections to v3 — it was written against a stale clone

v3 is based on commit `852482e5` (2026-08-20). Several of its statements are no longer true, and
two of its recommended numbers are wrong for the new arena. Recording these so the plan is not
followed blindly.

| v3 says | Actual state now | Impact |
|---|---|---|
| "`blind: 2` still in repo, needs pushing" | Already `0.5`, committed in `15bedce5` | Phase 0 partly done |
| "`entry_detection_module.py` already exists (559 lines)" | **Deleted** in `15bedce5` repo cleanup; recovered from git for this work | Must be restored, not just edited |
| "`collision_retreat_behavior_plan_2026-08-18.md`" | Also deleted in cleanup | Phase 0 reference is gone; superseded by `fuel_repeat_scan_root_cause_2026-09-04.md` |
| `door_max_width` → 8.0 m | The **new** arena has a clean **1.90 m** door, not a 15 m open face | 8.0 would match wall gaps that are not the door |
| Camera STL "scale 0.001 if mm" | `A8_mini_Camera.stl` is **already in metres** (0.052 × 0.061 × 0.084) | Scale **1.0**; scaling by 0.001 would make it invisible |

### 0.1 A safety defect in the recovered EDM

`entry_detection_module.py` publishes setpoints on **`/mavros/setpoint_raw/local`** — straight to
MAVROS, **bypassing `flight_envelope_guard.py` completely**. Every other motion source in this
stack goes through `/planning/pos_cmd` where the guard validates it. As written, the EDM can fly
the vehicle anywhere with no envelope enforcement at all.

**This must be re-pointed at `/planning/pos_cmd` before it is ever armed.** Non-negotiable.

---

## 1. Measured facts about the new assets

### 1.1 `ARINA_NIDAR.STL`

| Property | Value |
|---|---|
| Raw extents | 15000 × 15000 × 2540 **mm** → scale **0.001** |
| Placed | centred on origin, spans −7.5 … +7.5 m, walls 2.54 m tall |
| Triangles | 1394 (606 wall segments at cruise altitude, 301.9 m of wall) |
| Free area (>0.15 m from wall) | 158.0 m² |
| **Median clearance** | **0.48 m** |
| **Max clearance anywhere** | **1.36 m** |

**Entrance: exactly one**, on the south wall (y = −7.5), spanning x = −0.95 … +0.95 —
**1.90 m wide, centred at x = 0.00**. This is a proper door, which makes it a much better entry
test than the old arena's fully-open south face.

Navigable + connected area vs planner inflation (Iris collision radius is 0.384 m):

| `obstacles_inflation` | connected navigable |
|---|---|
| 0.199 (pre-fix) | 155.9 m² — fictitious, counts gaps the airframe cannot pass |
| 0.384 (exact fit) | 104.6 m² |
| **0.42 (current setting)** | **85.5 m²** |
| 0.45 | 81.5 m² |
| 0.50 | 17.3 m² — collapses |

The 0.42 setting carried over from the previous arena remains correct here, with more usable area
than the old arena (85.5 vs 70.5 m²). **Do not exceed 0.45.**

### 1.2 `A8_mini_Camera.stl`

ASCII STL, **already in metres**: 0.052 × 0.061 × 0.084 m, against a real SIYI A8 mini of
55 × 55 × 70 mm. Use `<scale>1 1 1</scale>`.

**51,176 triangles** — very heavy for a cosmetic mesh on a CPU-bound machine. It is `<visual>`
only (collision stays a primitive box), but it still costs render time. Mitigation: the vehicle
runs headless for tests, so the mesh is only paid for when the GUI is on.

---

## 2. Package structure

New files go into a proper catkin package rather than loose `scripts/`:

```
catkin_ws/src/nidar_mission/
├── package.xml
├── CMakeLists.txt
├── config/
│   └── mission_config.yaml          <- THE central config (§3)
├── launch/
│   └── nidar_mission.launch         <- loads config, brings up mission nodes
└── scripts/
    ├── entry_detection_module.py    <- recovered, re-pointed at /planning/pos_cmd
    ├── mission_manager.py           <- phase FSM (later phase)
    └── apply_mission_config.py      <- renders derived files from the central config
```

Gazebo models follow the existing `simulation/custom_models/` convention:

```
simulation/custom_models/
├── arina_nidar/{model.config, model.sdf, meshes/arina_nidar.stl}
└── launch_pad/{model.config, model.sdf, meshes/launch_pad.stl}
```

The **camera-equipped vehicle must live in the PX4 tree**, because that is what Gazebo actually
spawns — `simulation/custom_models/` is dead weight for the vehicle (established earlier this
week; the launch resolves `$(find mavlink_sitl_gazebo)/models/...`):

```
simulation/PX4-Autopilot-v1.14.3/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
└── iris_vlp16_cam/{model.config, iris_vlp16_cam.sdf, meshes/siyi_a8_mini.stl}
```
plus a PX4 airframe file `1024_gazebo-classic_iris_vlp16_cam` alongside the existing `1023_…iris_vlp16`.

---

## 3. The central config — what it can and cannot do

`config/mission_config.yaml` is the **single source of truth**. But it cannot be the *only* place
values live, because four different consumers read four different formats:

| Consumer | Reads | Can it take ROS params? |
|---|---|---|
| Mission nodes (EDM, manager, guard, detector) | ROS param server | **Yes** — `rosparam load` directly |
| FUEL | roslaunch `<param>` in `algorithm.xml` | Yes, via `$(arg)` substitution |
| FAST-LIO | its own `velodyne.yaml` | **No** — reads a file path |
| Gazebo | SDF/world XML | **No** — static XML |

So the honest design is: **one YAML, plus a generator** (`apply_mission_config.py`) that renders
the two formats that cannot read ROS params. Running the generator is a build step, not something
that happens silently at launch — it prints exactly which files it rewrote.

Config sections: `arena`, `launch_pad`, `vehicle`, `lidar`, `camera`, `flight`, `planner`,
`entry`, `detection`, `mission`. Adding a new arena becomes: drop the STL in, add an `arena:`
block, re-run the generator.

---

## 4. Phased execution

| # | Phase | Deliverable | Verification |
|---|---|---|---|
| **1** | Arena model | `arina_nidar` in Gazebo, correct scale/placement | World loads; door at (0, −7.5) present in the map |
| **2** | Launch pad | Pad at world (0, −9.5), 1.5 m, 0.03 m thick, `mu=1.0` | Vehicle rests level; TFmini reads expected |
| **3** | Central config + generator | `mission_config.yaml` + `apply_mission_config.py` | Change a value → regenerate → the derived file changes |
| **4** | Camera vehicle | `iris_vlp16_cam` spawns, `/camera/image_raw` publishes | `rostopic hz /camera/image_raw` ≈ 15 Hz |
| **5** | Frame single-source | Spawn pose no longer hardcoded in 3 places | `world_to_camera(camera_to_world(p)) == p` at 3 spawn poses |
| **6** | **EDM entry** | Recovered EDM, re-pointed at `/planning/pos_cmd`, params fixed for a 1.90 m door | **Realtime run: vehicle crosses y = −7.5 into the arena** |
| 7 | Mission FSM | `mission_manager.py`, phase-aware guard | Full IDLE→…→LAND sequence |
| 8 | Survivor detection | YOLOv8-nano, map tagging | Detection appears at correct grid coord |

**Phases 1–6 are this session's scope** — they are exactly what "test in realtime that it is able
to enter the arena" requires. Phases 7–8 follow once entry is proven.

### 4.1 EDM parameter values for *this* arena

| Parameter | Old | New | Why |
|---|---|---|---|
| `door_min_width` | 0.50 | **0.80** | Below the 1.90 m door with margin; rejects narrow interior gaps |
| `door_max_width` | 1.20 | **3.00** | Door is 1.90 m. **Not v3's 8.0** — that was sized for the old open-face arena and would match almost any wall gap |
| `target_door_width` | 1.00 | **1.90** | The actual door |
| `search_forward_speed` | 0.35 | **0.40** | Pad is 2 m out; the vehicle is 0.767 m wide passing a 1.90 m door, so ~0.55 m per side. Slower than v3's 0.5 |
| `max_search_duration` | 45 | **30** | Door is directly ahead of the pad |
| `min_clearance` | 0.45 | **0.42** | Match `obstacles_inflation` |

### 4.2 Geometry of the entry

Pad at world (0, −9.5); door centre at world (0, −7.5). The vehicle flies **due north** 2.0 m,
crossing the threshold. Door is 1.90 m wide, vehicle is 0.767 m wide → **0.57 m clearance per
side**. Tight but well within the 0.42 m inflation, and the approach is head-on.

---

## 5. Risks

1. **Localization on the pad.** Outside the arena, ground returns dominate and constrain only
   altitude. With the pad only 2 m from a wall containing the door, the arena's south face and
   both interior edges of the door are in view, so this is far better constrained than a pad in
   open ground. Still the weakest link in the chain.
2. **0.57 m per-side clearance through the door.** With 2.6–4.0 cm measured position jitter this
   is fine on paper, but a mis-detected gap centre costs half the margin. The EDM must steer to
   the *measured* gap centre, not a hardcoded x = 0.
3. **The guard envelope must be widened** to include the pad and transit corridor, or every
   setpoint outside y > −6.8 is rejected and the vehicle can never leave the pad. Phase 6 depends
   on this.
4. **Camera CPU cost** on an already-loaded machine. 640×480 @ 15 fps headless; the 51k-triangle
   mesh is visual-only.
5. **`pcd_save_en`** is now `false`, which removed the OOM that ended earlier runs — but long runs
   remain unproven past ~8 minutes.
