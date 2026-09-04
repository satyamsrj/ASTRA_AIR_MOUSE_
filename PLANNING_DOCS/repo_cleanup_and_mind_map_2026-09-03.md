# Repo Map & Cleanup Plan (2026-09-03)

Status: **analysis only — no files changed, no code touched.** This is the implementation plan for a future cleanup pass. Everything below was verified by tracing actual references (`grep` across every launch/script/config file), not by guessing from file names.

Companion visual mind map: see the published Artifact linked in chat (same content as the "Live execution graph" section below, drawn as a diagram).

Verification harness for every phase in this plan is the same command the team already uses:
```
docker start ros_workspace
docker exec -it ros_workspace bash -lc 'cd /home/developer/NIDAR && ./scripts/test_takeoff.sh true 0.0 -6.5 0.1 1.5708'
```
A phase is only "done" once this still reaches `Clean Upstream FUEL Autonomous Exploration Running!` with the same four topics coming up (`/mavros/imu/data`, `/velodyne_points`, `/Fast_LIO/odometry`, `/cloud_registered`).

---

## 1. Headline numbers

| Metric | Value |
|---|---|
| Total git-tracked files | 31,685 |
| Total git-tracked size (working tree) | 966 MB |
| `.git` history size | 2.9 GB |
| `simulation/` tracked size | 698 MB (72% of all tracked bytes) |
| `simulation/PX4-Autopilot-v1.14.3/` tracked files | 30,439 (**96% of every tracked file in the repo**) |
| `catkin_ws/` (real active ROS workspace) | 448 MB, 1,172 files |
| `scripts/` | 25 files |
| `PLANNING_DOCS/` | 15 files |

The single biggest fact about this repo's size is that 96% of its tracked files are one vendored copy of PX4 firmware. Everything else discussed below is comparatively small. See §4.4 for why that one is high-risk to touch and everything else is not.

---

## 2. Live execution graph (what must not break)

This is the actual runtime call graph, traced from the command in §0, not the architecture diagram in the README (which has drifted — see §4.1).

```
docker exec ros_workspace ./scripts/test_takeoff.sh
│
├─ source scripts/setup_env.sh
│
├─ roslaunch px4 mavros_posix_sitl.launch  (package lives in simulation/PX4-Autopilot-v1.14.3)
│    vehicle=iris_vlp16   world=/home/developer/NIDAR/nidar_competition.world  ← ROOT FILE, LIVE
│    │
│    ├─ nidar_competition.world → model://nidar_arena
│    │    → simulation/custom_models/nidar_arena/{model.sdf, meshes/mapdraw.stl, meshes/nidar_world_arena.dae}
│    │         (these meshes were authored from the raw files in ARENA/ — see §4.5)
│    └─ vehicle model → simulation/custom_models/iris_vlp16/
│
├─ roslaunch launch/fast_lio/nidar_mapping.launch
│    ├─ config/fast_lio/nidar_sim.yaml
│    ├─ config/iris_vlp16.urdf
│    ├─ config/nidar_lidar.rviz
│    └─ catkin_ws/src/FAST_LIO  (fastlio_mapping node, flight_path_publisher.py)
│
├─ scripts/relay_odometry.py        (FAST-LIO odometry → MAVROS vision-pose fusion)
├─ rosparam load config/flight_envelope_guard.yaml
├─ scripts/flight_envelope_guard.py (FUEL trajectory → MAVROS offboard, safety envelope)
├─ scripts/cpu_repin_loop.sh        (CPU affinity pinning)
│
├─ roslaunch launch/nidar_fuel_upstream.launch
│    ├─ exploration_manager/launch/algorithm.xml
│    ├─ plan_manage       (traj_server node)
│    ├─ waypoint_generator (waypoint_generator node)
│    └─ supporting fuel_planner libs: active_perception, plan_env, bspline, bspline_opt,
│         path_searching, poly_traj, traj_utils, utils
│
└─ scripts/mission_telemetry_logger.py

Sensor/driver dependency layer (feeds the above, not launched directly by test_takeoff.sh):
  catkin_ws/src/livox_ros_driver     — Livox LiDAR ROS driver
  catkin_ws/src/velodyne_simulator   — Gazebo VLP-16 plugin + meshes
  catkin_ws/src/ikd-Tree             — nearest-neighbor structure used by FAST-LIO
  catkin_ws/src/PX4-Autopilot        — NOT the firmware; a thin 12-file, 1.3MB shim that only
                                        carries velodyne_vlp16 Gazebo meshes so rospack can find them
```

**Do-not-touch list** (everything named above): `nidar_competition.world`, `config/*`, `launch/*`, `scripts/test_takeoff.sh`, `scripts/setup_env.sh`, `scripts/relay_odometry.py`, `scripts/flight_envelope_guard.py`, `scripts/cpu_repin_loop.sh`, `scripts/mission_telemetry_logger.py`, `simulation/custom_models/*`, `simulation/PX4-Autopilot-v1.14.3/*` (build target), all of `catkin_ws/src/{FAST_LIO,fuel,livox_ros_driver,velodyne_simulator,ikd-Tree,PX4-Autopilot}` (their actual package contents — a couple of stray scratch files inside are called out separately below).

---

## 3. Why the README's diagram is already stale

`README.md`'s "Repository Structure" section (lines 75-98) doesn't mention `config/flight_envelope_guard.yaml`, `launch/nidar_fuel_upstream.launch`'s real dependents, or any of the root-level files below — it was written before the repo was restructured into `config/`/`launch/` and never updated. That drift is exactly how the root-level duplicates in §4.1 were able to sit unnoticed. Worth regenerating this section from §2 once cleanup lands.

---

## 4. Verified clutter inventory

Every item below was confirmed **zero-referenced by any live script, launch file, or config** (only self-references, README mentions, or historical `PLANNING_DOCS/*.md` entries turned up in a repo-wide grep). Nothing here was flagged on file-name guesswork alone.

### 4.1 Root-level orphaned duplicates — safe to delete, zero references

These were superseded when the repo moved to `config/` + `launch/fast_lio/`, but the old copies were never removed:

| Root file | Superseded by | Evidence |
|---|---|---|
| `nidar_lidar.rviz` | `config/nidar_lidar.rviz` | `launch/fast_lio/nidar_mapping.launch:33` loads the `config/` copy by absolute path |
| `nidar_sim.yaml` | `config/fast_lio/nidar_sim.yaml` | same launch file, line 6 |
| `iris_vlp16.urdf` | `config/iris_vlp16.urdf` | same launch file, line 19 |
| `nidar_mapping.launch` | `launch/fast_lio/nidar_mapping.launch` | `test_takeoff.sh:83` calls the `launch/` copy, never the root one |
| `relay_odometry.py` | `scripts/relay_odometry.py` | `test_takeoff.sh:88` calls the `scripts/` copy (also newer + larger: Aug 17 vs Aug 12, 7.6KB vs 4KB) |
| `nidar_airmouse_arena.sdf` | `simulation/custom_models/nidar_arena/model.sdf` | zero references anywhere in the repo |
| `nidar_world_arena.dae` | `simulation/custom_models/nidar_arena/meshes/nidar_world_arena.dae` | only mentioned in one PLANNING_DOCS postmortem, never loaded by any world/launch file |
| `testing_methods` | — (delete outright) | plain-text scratch note with no extension; references `/home/ayush/Desktop/NIDAR/...`, a path from a different machine/user, not this environment |

### 4.2 `scripts/` — legacy and experimental, zero references

Of 25 files in `scripts/`, these are never invoked by `test_takeoff.sh`, `setup_env.sh`, `docker_dev_start.sh`, `build_px4.sh`, any `.launch` file, or any other still-referenced script:

- **Abandoned "EDM" feature cluster:** `entry_detection_module.py`, `test_edm_cues.py`, `test_edm_transition.sh`
- **Dead "phase0" diagnostic pair:** `phase0_collect_mapping.sh` (nothing calls it) → `phase0_tf_audit.py` (only called by the former)
- **Legacy pre-FUEL adapter:** `minimal_fuel_adapter.py` — superseded by the "Upstream FUEL" stack (`launch/nidar_fuel_upstream.launch`). Its only remaining trace is a defensive `pkill -f minimal_fuel_adapter.py` at `test_takeoff.sh:11`, which is harmless to leave or remove.
- **Standalone, unreferenced:** `calibrate_landmarks.py`, `monitor_flight.py`, `strict_monitor.py`, `collect_crash_timeline.sh`, `run_sim.sh`
- **Legacy manual "square flight" bench-test cluster** (predates the FUEL exploration pipeline): `run_obstacle_test.sh`, `test_square_flight.sh`, `robust_square_flight.py`, `square_flight.py` — these call each other but nothing in the live pipeline calls any of them.
- **Stale bytecode cache:** `scripts/__pycache__/` — already untracked and covered by `.gitignore`'s `__pycache__/` rule; safe to delete locally any time, will regenerate.

Recommend confirming with the team before hard-deleting the square-flight cluster and `test_flight_envelope_guard.py`/`record_mission.sh`-adjacent tools specifically — they read as manual bench-test utilities someone may still reach for by hand, not provably dead the way the EDM/phase0 clusters are. Everything else in this section has no plausible manual use left (superseded or feature abandoned).

### 4.3 Tiny vendored scratch files

`catkin_ws/src/fuel/fuel_planner/test.txt` (empty) and `log.md` (68-line TODO scratchpad from the upstream FUEL project, not NIDAR-specific) are tracked in git but are not part of any package's build (not referenced by any `CMakeLists.txt`/`package.xml`). Trivial, but free to remove.

### 4.4 The vendored PX4 tree — flagged, not actioned

`simulation/PX4-Autopilot-v1.14.3/` is committed as **plain files, not a git submodule** (this is called out explicitly in `README.md`'s troubleshooting section as a deliberate "zero-friction onboarding" choice, and `scripts/build_px4.sh` has special-case logic to bootstrap fake local `.git` repos in five nested paths specifically because there's no real PX4 git history vendored in). It accounts for 30,439 of the repo's 31,685 tracked files and roughly 700MB of tracked bytes.

Telling evidence this was already recognized as a problem once: **`.gitignore_container`** (present at repo root, itself gitignored/local-only) contains the line `simulation/PX4-Autopilot*/` under a comment `# Ignore the massive PX4 firmware repository` — but the real `.gitignore` was never updated to match. That fix was drafted and never landed.

This is **not** a Phase-0/1 action — ignoring or removing it now would break `./scripts/build_px4.sh` and the onboarding flow the README documents on purpose. Options, in increasing order of effort/risk, for a deliberate future decision (not this pass):
1. **Leave as-is.** Zero risk, status quo.
2. **Convert to a real git submodule** pinned at the PX4 `v1.14.3` tag. Removes the need for `build_px4.sh`'s git-bootstrap hack, stops the tree from growing `.git` history further, but requires network access at clone time and changes the "3-command" onboarding story in the README.
3. **Purge historical blobs from `.git` history** (`git filter-repo`/BFG) if the 2.9GB `.git` size itself becomes a problem (e.g., for GitHub hosting limits). This rewrites history and needs a force-push + full team re-clone — a team decision, not something to do unilaterally.

Related, but purely local (not git-tracked, not repo bloat): `simulation/PX4-Autopilot-v1.14.3/build/` is 1.1GB of local SITL build output, already gitignored — safe to `rm -rf` any time to reclaim disk space; it regenerates on the next `build_px4.sh` run. Same for `logs/` (579MB of `flight_envelope_guard_*.csv` telemetry dumps, already covered by `**/logs/` in `.gitignore`) — worth a periodic manual prune, not a repo change.

### 4.5 Not clutter — but worth relocating for clarity

- **`ARENA/`** (`mapdraw.stl`, `rooms for drone denied simulation.glb`, `result.dae`, 772KB) — these are the *source* design assets that `simulation/custom_models/nidar_arena/meshes/{mapdraw.stl, nidar_world_arena.dae}` were built from (per `PLANNING_DOCS/stl_arena_integration.md`). Not loaded at runtime directly, but this is the editable original — don't delete it, consider moving it under `simulation/custom_models/nidar_arena/source/` so the relationship is obvious instead of implicit.
- **`graphify/` and `graphify-out/`** — a Claude Code skill/tool and its analysis cache, not NIDAR project code. Neither is currently git-tracked. `graphify-out/` (97MB, regenerable cache) should get an explicit `.gitignore` entry so it's never accidentally committed. Whether `graphify/` itself belongs checked into this repo at all vs. installed as a personal tool elsewhere is a call for the team, not something to assume.
- **Root PDFs** (`Mission Brief - NIDAR RescueSwarm.pdf`, `NIDAR 26-27 - Rulebook.pdf`) — legitimate competition reference material, just loose at repo root. Optional: move alongside `PAPERS/` into one `docs/` location.
- **`PLANNING_DOCS/`** (15 files) — real engineering history (root-cause analyses, implementation plans), not noise. Recommend archiving the ones describing already-resolved issues (e.g. `Phase_4_completion.md`, `mapping_stall_crash_root_cause.md`, `premature_landing_root_cause_2026-08-17.md`, `realtime_test_verification_2026-08-17.md`) into `PLANNING_DOCS/archive/`, keeping open/active plans at the top level (e.g. `tfmini_downward_rangefinder_integration_2026-08-23.md`, the currently-open doc, which is a not-yet-implemented plan).
- **`Makefile_px4`** and **`.gitignore_container`** at repo root — both already gitignored/untracked, so they're not repo bloat, just local disk clutter. `Makefile_px4` is a stray copy of PX4's own top-level `Makefile`, unused by any script (`build_px4.sh` builds from inside `simulation/PX4-Autopilot-v1.14.3` directly). Safe to delete locally whenever.

---

## 5. Recommended sequencing

**Phase 0 — zero-risk deletions** (nothing in §2's live graph references any of these):
- Delete the 8 root-level files in §4.1.
- Delete `catkin_ws/src/fuel/fuel_planner/{test.txt,log.md}`.
- Add `graphify-out/` to `.gitignore`.
- Local-only, no git impact: clear `scripts/__pycache__/`, `simulation/PX4-Autopilot-v1.14.3/build/`, `Makefile_px4`.
- Verify: rerun the test command in §0 once; confirm identical behavior (nothing here is even in the call graph, so this is a formality, not a real risk check).

**Phase 1 — legacy script cleanup** (confirm with team first per §4.2's caveat):
- Remove the EDM cluster, phase0 pair, `minimal_fuel_adapter.py`, and the 5 standalone unreferenced scripts outright.
- Move the square-flight cluster and any other "might still be used by hand" tools to `scripts/legacy/` instead of deleting, so history isn't lost but they're out of the way.
- If `minimal_fuel_adapter.py` is removed, also drop its now-pointless `pkill` line at `test_takeoff.sh:11` (the only edit to a live file in this whole plan).
- Verify: rerun the test command in §0; also manually confirm none of the moved/removed scripts are things a teammate runs by hand for bench testing.

**Phase 2 — documentation organization** (no functional risk, pure filesystem hygiene):
- Archive resolved `PLANNING_DOCS/*.md` per §4.5.
- Relocate `ARENA/` under `simulation/custom_models/nidar_arena/source/`.
- Optionally consolidate root PDFs with `PAPERS/`.
- Regenerate `README.md`'s "Repository Structure" section from §2.

**Phase 3 — vendored PX4 tree** (§4.4): a deliberate team decision, not a mechanical cleanup step. Do not schedule until Phases 0-2 are verified stable.

---

## 6. Execution record — 2026-09-04

Phases 0-2 executed. **27 files removed, all git-tracked, so every one is recoverable via
`git checkout HEAD~1 -- <path>`.** Each was re-verified unreferenced immediately before deletion
against live `scripts/`, `launch/`, `config/`, `*.world`, `Dockerfile` and `CMakeLists.txt` only —
the apparent "references" that showed up were basename collisions between a root duplicate and its
`config/`/`launch/` replacement, or members of a cluster deleted together.

| | before | after |
|---|---|---|
| `scripts/` | 25 files | **8**, every one reachable from `test_takeoff.sh` |
| root non-hidden files | 11 | **3** (`README.md`, `CLAUDE.md`, `nidar_competition.world`) |

Removed: the 8 root duplicates (§4.1), `fuel_planner/{test.txt,log.md}` (§4.3), the EDM cluster,
the phase0 pair, `minimal_fuel_adapter.py` (plus its now-pointless `pkill` line in
`test_takeoff.sh`), the 5 standalone unreferenced scripts, and — approved by the team on
2026-09-04 — the manual bench-test cluster (`run_obstacle_test.sh`, `test_square_flight.sh`,
`robust_square_flight.py`, `square_flight.py`) together with `test_flight_envelope_guard.py` and
`record_mission.sh`.

Also: `graphify-out/` and `graphify/` added to `.gitignore`; resolved plans moved to
`PLANNING_DOCS/archive/`; README "Repository Structure" regenerated from the live call graph.

**Verified** by a full GUI run (`./scripts/test_takeoff.sh true 0.0 -6.5 0.1 1.5708`) after the
deletions: reached `Clean Upstream FUEL Autonomous Exploration Running!`, altitude held at
1.4125 m (sd 0.0093), zero flight-envelope-guard events.

### 6.1 Not actioned — pre-existing uncommitted deletions

`ARENA/` (3 files), `PAPERS/` (4 files) and both root PDFs are **tracked in git but already absent
from the working tree**, deleted by someone else and never committed (10 ` D` entries predating
this pass). §4.5 explicitly says ARENA holds the editable source assets the arena meshes were built
from and should be kept. Left untouched rather than silently committing someone else's deletion:
restore with `git checkout -- ARENA PAPERS '*.pdf'`, or commit the removal deliberately.

### 6.2 Compute findings that belong with this plan

The clutter has a direct, measured compute cost, which is the missing link in the "high-end PC but
CPU-bound" question:

* **cpptools ~211 % CPU** continuously parsing the 19,252 vendored PX4 C/C++ files (97 % of all
  C/C++ in the repo). VS Code indexes the working directory, not git, so `.gitignore` does not help.
  Fixed with `.vscode/settings.json` exclusions.
* **rviz ~290 % CPU** (~3 of 12 hardware threads) purely for visualisation. Now off by default and
  decoupled from the Gazebo GUI; `RVIZ=1` re-enables it.
* **Leaked processes**: `test_takeoff.sh` never killed `rviz`, `mission_telemetry_logger.py`,
  `robot_state_publisher` or `static_transform_publisher`, and spawned replacements before the
  SIGKILLs landed. A re-run inherited 3 gzserver / 3 px4 / **2 fastlio_mapping**, and the duplicate
  FAST-LIO nodes publishing competing solutions onto one topic produced a convincing but entirely
  fake "SLAM divergence". Fixed. **Treat any back-to-back test result from before this as suspect.**

The simulation itself only needs ~1.5-2 cores. Roughly 5 threads were being burned on waste.
