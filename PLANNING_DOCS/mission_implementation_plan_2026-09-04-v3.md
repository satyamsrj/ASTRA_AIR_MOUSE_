# Implementation plan v3: camera-first, fully autonomous mission

**Date:** 2026-09-04 (v3 — replaces v2)
**Basis:** fresh clone of `singhayush5062-star/ASTRA_AIR_MOUSE_`, commit `852482e5` (2026-08-20),
cross-referenced against the full NIDAR AirMouse mission brief.

---

## Why v2 was wrong

v2 treated the camera as a deferrable Phase G item. The NIDAR rules make it **mandatory from
mission start — no camera means disqualification**, regardless of how well navigation works:

| Rule | Exact requirement | Consequence |
|---|---|---|
| §1 | "The Mission Planner / GCS must display a **live camera feed** from the drone throughout the mission" | No camera = rule violation |
| §6 | "The drone must **detect survivors** using onboard sensing and processing" | Survivors are humans/dummies — LiDAR cannot distinguish them from furniture |
| §5 | GCS must display "live camera feed", "tagged locations of detected survivors", "grid coordinate or grid box containing each detected survivor" | Detection results must appear on the live 2D map during flight |
| §4 | "Any manual control input, path correction, waypoint adjustment, survivor tagging input, or operator-assisted navigation during the mission shall be considered manual intervention" | Everything must be autonomous — entry, mapping, detection, tagging, exit |

The camera is as fundamental as the propellers. It is not a nice-to-have.

---

## 1. SIYI A8 mini — what it is and how it maps to simulation

Confirmed specs (from manufacturer datasheets):

| Parameter | Value |
|---|---|
| Weight | 95 g |
| Dimensions | 55 × 55 × 70 mm |
| Sensor | Sony 1/1.7" CMOS, 8 MP |
| FOV | 93° diagonal (81° horizontal) |
| Resolution | 4K @ 25 fps, 1080p @ 30 fps |
| Gimbal axes | 3 (pitch −135°/+45°, yaw ±160°, roll ±30°) |
| Control input | S.Bus / UART / UDP |
| Video output | Ethernet, HDMI, CVBS (AV) |
| Voltage | 11–25.2 V |
| Power | avg 5 W, peak 12 W |
| AI | Built-in NPU for tracking (SIYI proprietary) |
| PX4 compatibility | Yes — ships with UART cable for PX4/ArduPilot FC |

**For the physical drone:** the SIYI A8 mini mounts directly, controlled via UART from PX4 (MAVLink
gimbal protocol). The Ethernet video output feeds to the companion computer for detection and to the
GCS link for live feed. Its built-in NPU handles object tracking; survivor detection inference runs
on the companion computer. Weight (95 g) is well within the 10 kg AUW limit.

**For the Gazebo simulation:** we don't simulate the physical camera housing — we simulate its
**sensor characteristics** (FOV, resolution, frame rate) and its **gimbal kinematics** (3-axis
joint chain). Two ready-made references already exist in the PX4 tree:

1. `models/fpv_cam/fpv_cam.sdf` — a camera sensor with both `libgazebo_ros_camera.so` (publishes
   `/usb_cam/image_raw` to ROS) and `libgazebo_gst_camera_plugin.so` (streams to GCS via UDP 5600).
   Currently 320×240 @ 30 fps — needs resolution and FOV adjustment.

2. `models/typhoon_h480/typhoon_h480.sdf.jinja` — a complete 3-axis gimbal implementation using
   `libgazebo_gimbal_controller_plugin.so` with revolute joints for roll/pitch/yaw, controlled via
   the PX4 MAVLink gimbal channels.

**Do you need to provide the SIYI A8 mini STL/CAD?** Only for visual cosmetics in Gazebo (so the
sim model *looks* like the real drone). The sensor behavior and gimbal mechanics come from Gazebo
plugins, not the mesh. If you want visual fidelity, yes, provide the STL and it gets used as the
`<visual>` geometry on the camera link — but functionally, the sim works identically without it.

---

## 2. What actually needs building — revised

| Task | v2's approach (wrong) | v3's approach (correct) |
|---|---|---|
| Live camera feed to GCS | "defer camera" | **Camera on day 1** — `libgazebo_gst_camera_plugin.so` → UDP → QGC/GCS |
| Survivor detection | not addressed | **YOLOv8-nano on camera feed** — runs on companion computer, publishes detections to ROS, tags on 2D map |
| Stop wandering outside | FUEL `box_*` only | **LiDAR-based gap detection + FUEL `box_*`** — `entry_detection_module.py` (already exists, needs parameter fixes) detects the arena opening from VLP-16 point cloud, drives the drone through, then hands off to FUEL |
| Find the entry | "pre-compute from STL" | **Autonomous LiDAR gap detection** — the VLP-16 sees the arena walls and the opening; no STL needed, no manual waypoint entry. Camera provides supplementary visual context but isn't the primary sensor for this |
| Close-range obstacles | `blind: 0.5` | unchanged (needs to be pushed — still `blind: 2` in repo) |
| Map the arena | LiDAR + FUEL | unchanged |
| Return to pad | "deterministic reverse transit" | **Reverse of entry path** stored by EDM, replayed by mission manager |
| Land on pad | "enlarge pad" | Camera-assisted if drift measured > pad radius (Phase G) |
| Front-facing LiDAR | "not needed" | **Confirmed not needed** — VLP-16's 1800 horizontal samples already cover 360°; a front-facing LiDAR adds no new coverage |

---

## 3. Architecture — how the pieces connect

```
                                   ┌──────────────────┐
                                   │  Mission Manager  │ publishes /mission/phase
                                   │  (mission_manager │ owns /planning/pos_cmd mux
                                   │   .py — NEW)      │
                                   └───────┬──────────┘
                           ┌───────────────┼───────────────┐
                           │               │               │
                    ┌──────▼──────┐ ┌──────▼──────┐ ┌──────▼──────┐
                    │ TRANSIT_IN  │ │   EXPLORE   │ │ TRANSIT_OUT │
                    │ EDM drives  │ │ FUEL drives │ │ reverse     │
                    │ /pos_cmd    │ │ /pos_cmd    │ │ /pos_cmd    │
                    └──────┬──────┘ └──────┬──────┘ └──────┬──────┘
                           │               │               │
                    ┌──────▼───────────────▼───────────────▼──────┐
                    │          flight_envelope_guard.py            │
                    │  phase-aware: MISSION envelope during        │
                    │  transit, EXPLORE envelope during mapping    │
                    └──────────────────┬──────────────────────────┘
                                       │ validated /planning/pos_cmd
                                       ▼
                                  traj_server → MAVROS → PX4

     ┌─────────────────────────────────────────────────────────────┐
     │ Camera pipeline (runs in ALL phases after TAKEOFF)          │
     │                                                             │
     │  Gazebo camera sensor                                       │
     │    → /camera/image_raw (ROS topic, 640×480 or 1280×720)     │
     │    → survivor_detector.py (YOLOv8-nano inference)           │
     │      → /detections (bounding boxes + class + confidence)    │
     │      → tag on 2D occupancy grid at drone pose + camera      │
     │        geometry → /survivor_markers (MarkerArray on map)    │
     │    → GStreamer UDP stream → QGC/GCS (live feed)             │
     └─────────────────────────────────────────────────────────────┘

     ┌─────────────────────────────────────────────────────────────┐
     │ LiDAR pipeline (runs in ALL phases)                         │
     │  VLP-16 → /velodyne_points → FAST-LIO2 → odometry + map    │
     │  VLP-16 → EDM gap detection (TRANSIT_IN only)               │
     │  VLP-16 → FUEL frontier exploration (EXPLORE only)          │
     └─────────────────────────────────────────────────────────────┘
```

---

## 4. How autonomous entry actually works — LiDAR-driven, no manual waypoints

The `entry_detection_module.py` **already exists** (559 lines, fully functional FSM with
TAKEOFF → ENTRY_SEARCH → ENTRY_CONFIRMATION → EXPLORATION states). It uses the **VLP-16 point
cloud** (not a camera) to detect openings in walls — exactly what's needed to find the arena
entrance autonomously.

### What it does today

1. Subscribes to `/cloud_registered` (FAST-LIO registered point cloud) and `/Fast_LIO/odometry`
2. Slices the point cloud at cruise altitude (±0.8 m band around the drone)
3. In the forward body-frame sector (0.3–4.0 m ahead, ±2.5 m lateral), separates left-boundary
   and right-boundary obstacle points
4. If the gap between them falls within `door_min_width` (0.5 m) to `door_max_width` (1.2 m),
   declares an opening detected
5. Steers laterally toward the gap center while maintaining forward velocity (0.35 m/s)
6. Once confidence crosses the threshold (opening detected + crossed + stable localization +
   obstacle clearance), transitions to ENTRY_CONFIRMATION → triggers FUEL

### What needs to change

The parameters were tuned for a **0.5–1.2 m door** (the design spec when it was written). Your
actual test arena (`mapdraw.stl`) has the **entire south face open** — three spans of 6.0, 1.75,
and 6.35 m, broken only by two thin posts. And competition arenas have corridors ≥ 1 m wide, so
the entry could be anything from 1 m to several meters.

| Parameter | Current value | Required value | Why |
|---|---|---|---|
| `door_min_width` | 0.5 m | **0.8 m** | Minimum corridor width per rules is 1 m; 0.8 gives margin for angled approach |
| `door_max_width` | 1.2 m | **8.0 m** | The opening could be as wide as the full south face (15 m in theory); cap conservatively |
| `search_forward_speed` | 0.35 m/s | **0.5 m/s** | Faster approach since the pad is 2–3 m from the arena, not inside it |
| `max_search_duration` | 45 s | **30 s** | With the pad close to the arena, 30 s is ample; if not found by then, something is wrong |

The EDM's approach — fly forward, detect gap in LiDAR, steer toward gap center, fly through —
works regardless of arena layout because it's reactive to whatever geometry the LiDAR actually
sees. No pre-computed coordinates, no manual waypoint entry, no camera needed for this step.

### What the camera adds to entry (supplementary, not primary)

During ENTRY_SEARCH, the camera feed is streamed to the GCS (mandatory rule) and
`survivor_detector.py` is already running — if the competition puts visual markers at the entry
(e.g., signage), it could detect those too, but this is bonus, not required.

### After entry

Once inside the arena (ENTRY_CONFIRMATION → EXPLORATION):
- EDM saves the entry path as a list of (position, yaw) waypoints at ~1 Hz
- FUEL takes over for interior exploration with `box_*` set to the arena interior
- Camera runs survivor detection continuously
- When FUEL reports no frontiers (or mission clock expires), `mission_manager.py` transitions to
  TRANSIT_OUT and replays the saved entry path in reverse

---

## 5. Survivor detection pipeline — camera + YOLO

### 5.1 Detection algorithm

| Component | Choice | Rationale |
|---|---|---|
| Model | YOLOv8-nano (or YOLOv11-nano) | ~3 MB, runs at 30+ fps on CPU, trained on COCO `person` class, fine-tunable for dummies |
| Input | `/camera/image_raw` (ROS Image topic) | From the Gazebo camera sensor (sim) or SIYI A8 Ethernet feed (hardware) |
| Framework | `ultralytics` Python package | Single `pip install`, inference in ~5 lines of code |
| Output | `/detections` (custom msg: bounding box, class, confidence, timestamp) | Consumed by the map-tagging node |

### 5.2 Localization of detected survivors on the 2D map

When YOLOv8 detects a person in the camera frame:

1. The bounding box center gives the pixel (u, v) of the detection
2. The camera intrinsics (focal length, principal point) give the bearing ray from the camera
3. The drone's pose in map frame (from FAST-LIO odometry) + camera gimbal angles give the ray's
   origin and direction in world frame
4. The ray is intersected with the 2D occupancy grid at floor level (z = 0) — or with the LiDAR
   depth at that bearing — to get the 3D world position of the detection
5. That position is quantized to the arena grid (2 m × 2 m rooms per rules) to get the grid
   coordinate
6. A marker is published on `/survivor_markers` and displayed on the 2D map in the GCS

### 5.3 Avoiding double-counting

Detections within 1.5 m of an already-tagged survivor are suppressed (the drone may see the same
person from multiple angles as it explores). This radius is half the room width (2 m), so two
different survivors in adjacent rooms are never merged.

### 5.4 Sim environment: survivor models in Gazebo

The current `nidar_competition.world` has no survivor models. For testing, add Gazebo person models
(e.g., `person_standing`, `person_walking` from the Gazebo model database, or simple colored
cylinders as stand-ins) at known grid positions inside the arena. This lets you validate the full
pipeline: camera sees person → YOLO detects → position projected → grid coordinate tagged → appears
on GCS map.

---

## 6. Camera integration in Gazebo — exact steps

### 6.1 Create `iris_vlp16_cam` model

A new model that combines the existing `iris_vlp16` (VLP-16 + IMU + MAVLink) with a forward-facing
camera, using the already-proven `fpv_cam` sensor block with parameters adjusted to match the
SIYI A8 mini:

**New file:** `simulation/custom_models/iris_vlp16_cam/iris_vlp16_cam.sdf`

The camera link gets:
- FOV: 1.623 rad (93°) to match SIYI A8 mini
- Resolution: 640 × 480 in sim (save CPU — enough for YOLO; the real A8 mini does 1080p/4K)
- Frame rate: 15 fps in sim (save CPU; real camera does 30 fps)
- `libgazebo_ros_camera.so` — publishes `/camera/image_raw` for the detection pipeline
- `libgazebo_gst_camera_plugin.so` — streams to GCS via UDP (QGC connects to port 5600)
- Mounted at a slight downward pitch (~15°) to see rooms/floor ahead — simulates a gimbal
  pointed forward-down. Full gimbal articulation (using revolute joints like typhoon_h480) is a
  later refinement; fixed-angle works for MVP

**Joint:** fixed to `iris::base_link`, positioned at `(0.05, 0, -0.02)` — slightly forward and
below center of mass (where the real SIYI A8 mini mounting plate sits on a quad).

### 6.2 Update launch infrastructure

- `test_takeoff.sh`: change `vehicle:=iris_vlp16` → `vehicle:=iris_vlp16_cam`
- Ensure the Gazebo model path includes the new model directory
- No changes to FAST-LIO, FUEL, or the envelope guard — the camera is a parallel pipeline

### 6.3 CPU budget

The machine was at 12/14 GB RAM during plain exploration. Camera adds:
- Gazebo rendering: ~200–400 MB GPU memory, ~10% CPU for 640×480 @ 15 fps (the plugin note in
  `fpv_cam.sdf` itself warns: "needs a lot of CPU! Consider lowering the camera image size")
- YOLOv8-nano inference: ~50–100 MB RAM, ~15% single-core CPU for 15 fps
- GStreamer encoder: ~5% CPU

**Mitigation (same as prior docs):** run headless (`GUI_ARG=false`), Release-build FAST-LIO,
disable `pcd_save_en`. These three changes alone free ~2 GB RAM and multiple CPU cores. The
camera fits within that recovered budget.

---

## 7. Revised phase plan — camera mandatory from Phase 1

| Order | Phase | What | Needs team sign-off? |
|---|---|---|---|
| 0 | Collision-retreat fix | Apply existing plan from `collision_retreat_behavior_plan_2026-08-18.md` — the corridor oscillation and wall-stuck bugs block everything | **Yes** (FUEL source) |
| 1 | Camera integration | `iris_vlp16_cam` model, `/camera/image_raw` publishing, GStreamer feed to GCS working | No (new model + plugin config) |
| 2 | Frame single-source (Phase A) | `mission_frames.yaml`, eliminate hardcoded 6.5 in guard, single-source TF, box params | No (config + scripts) |
| 3 | Launch pad in world (Phase B) | Pad model with corrected mesh, `mu=1.0` | No (new model) |
| 4 | EDM parameter fix + mission FSM (Phases D+E) | Fix `door_max_width`, add phase-aware guard, build `mission_manager.py` FSM that hands control between EDM (transit) and FUEL (explore) | No (scripts + config) |
| 5 | Survivor detection pipeline | `survivor_detector.py` (YOLOv8-nano), map-tagging node, survivor models in Gazebo world | No (new scripts) |
| 6 | Termination + return (Phase F) | Mission clock backstop, coverage plateau detection, TRANSIT_OUT replaying saved entry path | No (scripts) |
| 7 | Landing accuracy (Phase G) | Measure actual drift, decide if camera-assisted landing is needed | No |

---

## 8. What does NOT need building — confirmed

| Idea | Why it's unnecessary |
|---|---|
| **Front-facing LiDAR** | VLP-16 is 360° with 1800 horizontal samples. Forward coverage already exists. A second LiDAR adds zero new information and consumes CPU/weight |
| **Camera-based entry detection** | The VLP-16 + EDM detects the opening geometrically — the gap in the wall is a literal absence of LiDAR returns. Camera can't do this more reliably than a 360° range sensor |
| **Manual waypoint entry for the entry point** | EDM flies forward and reactively steers through whatever gap the LiDAR finds. No pre-programming of coordinates needed |
| **Pre-computing entry from STL** | Only useful in sim for validation. The EDM approach works identically on unknown arenas — it detects the opening in real time from the live point cloud |
| **Camera for navigation** | LiDAR + FAST-LIO handles localization; FUEL handles exploration planning. Camera does survivor detection and GCS feed — two different jobs that don't overlap |

---

## 9. About the SIYI A8 mini CAD model

You mentioned having the CAD model. Here's what to do with it:

- **If it's an STL:** use it as the `<visual>` mesh in the camera link of
  `iris_vlp16_cam.sdf` — purely cosmetic, makes the sim model look like the real drone.
  Scale it to match the real dimensions (55 × 55 × 70 mm = 0.055 × 0.055 × 0.07 m in SDF).
- **If it's a STEP/IGES:** convert to STL or DAE first (FreeCAD, Blender, or `assimp` CLI).
- **Not needed for functionality:** the Gazebo camera sensor plugin and gimbal joints are what
  produce the actual camera image and gimbal behavior — the mesh just affects what it looks like.

If you want to provide it, drop it in `simulation/custom_models/iris_vlp16_cam/meshes/siyi_a8_mini.stl`
and the SDF references it as a visual.

---

## 10. End-to-end mission flow — fully autonomous, zero manual intervention

```
1. IDLE          Drone on launch pad, powered, all nodes running
2. ARM           Operator presses one button → PX4 arms
3. TAKEOFF       Climb to cruise altitude (1.5 m), hover, FAST-LIO initializes
                 Camera begins streaming to GCS immediately
                 (Rule: "live camera feed throughout the mission")
4. TRANSIT_IN    EDM takes control of /planning/pos_cmd
                 • Flies forward at 0.5 m/s
                 • VLP-16 detects arena walls ahead
                 • Identifies gap (opening) in the LiDAR scan
                 • Steers toward gap center
                 • Crosses threshold → inside arena
                 • Saves path as breadcrumbs for return
                 Camera: streaming to GCS + running YOLOv8 (detection active)
5. EXPLORE       FUEL takes control of /planning/pos_cmd
                 • Frontier-based exploration within box_* bounds
                 • LiDAR builds the 2D map continuously → displayed on GCS
                 • Camera detects survivors → tagged on map with grid coordinates
                 • Collision-retreat handles tight spaces (Phase 0 fix)
6. TRANSIT_OUT   Triggered by: no frontiers for 10s, OR coverage plateau, OR
                 mission clock (30 min hard limit per rules), OR battery failsafe
                 • Replay saved entry path in reverse
                 • EDM drives /planning/pos_cmd back to pad location
7. DESCEND       Lower to landing altitude above pad
8. LAND          PX4 land mode
9. DISARM        PX4 disarm

Total manual intervention: one button press (ARM). Everything else autonomous.
```

---

## 11. Open items needing your input

1. **Team sign-off on Phase 0** — the collision-retreat fix touches FUEL source
   (`fast_exploration_fsm.cpp`, `planner_manager.cpp`, `expl_data.h`). Can you get approval?
2. **Upload the telemetry/logs** from the stuck run — I'll confirm the oscillation signature
   before the retreat fix is applied
3. **Push local changes** — `blind: 0.5` and any other fixes from previous sessions aren't in the
   repo yet, so my analysis may be against stale code
4. **SIYI A8 mini CAD file** — optional, only for visual fidelity. If you have it as STL, drop it
   in the repo; if STEP/IGES, let me know and I'll help convert
5. **Survivor dummy models** — do you have specific models the competition uses, or should we use
   generic Gazebo person models for testing?
