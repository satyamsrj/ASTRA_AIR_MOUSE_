# Fixed-Altitude 2D Flight Architecture — Implementation Plan (2026-09-03)

Status: **plan only, no code or config changed while producing this document.** Every claim below was verified
against the current committed code and live `git log` history, not against what earlier planning docs *said* was
done — several turned out to have been implemented later than their own doc's "status" line suggests, and one
already-shipped mechanism (a hard Z clamp) was silently lost in a later refactor. See §1.

**Companion reading, already produced today:** `PLANNING_DOCS/repo_cleanup_and_mind_map_2026-09-03.md` (overall repo
map). This doc assumes that one's live-execution-graph findings (`test_takeoff.sh` → `nidar_fuel_upstream.launch` →
`exploration_manager`/`plan_manage`/`flight_envelope_guard.py`) and doesn't re-derive them.

---

## 0. The ask, precisely

Move the drone in a genuinely **2D horizontal search** — a fixed cruise altitude, not "3D exploration inside a
shrunken vertical box" — using a downward-facing TFmini rangefinder for height, because every prior attempt at
constraining Z (frontier band clamp, virtual ceiling, guard clamp) has still let the vehicle exceed the arena's
height limit in live testing. This plan has to close every one of the specific ways those prior attempts leaked,
not just add a sensor on top of a stack that still has those leaks.

---

## 1. Why the last three attempts didn't hold — the actual failure chain

This is not one bug. It's **five independent single points of failure**, three of which were each individually
"fixed" once and still leaked, one of which regressed silently, and one of which is orthogonal to all of them and
can defeat any of the other four at once. A fixed-altitude architecture has to close all five or it will repeat
this exact history.

| # | Layer | What was tried | Why it still leaks | Evidence |
|---|---|---|---|---|
| 1 | Frontier candidate sampling | `frontier/min_candidate_z=1.2` / `max_candidate_z=1.6` (camera_init) clamps the **viewpoint target** `FrontierFinder::sampleViewpoints()` picks | Only constrains the *destination* of a plan, not any of the intermediate control points the flown B-spline actually passes through | `height_hold_baro_fastlio_analysis_2026-08-17.md`, confirmed live in current `frontier_finder.cpp:683-686` |
| 2 | SDF map search box | `sdf_map/box_max_z`: `2.2→1.9` (camera_init) | Only a **logical** bound on `FrontierFinder`/kinodynamic-A* candidate search (`isInBox()`); the actual flown trajectory comes from `BsplineOptimizer`, which never references `box_min_z`/`box_max_z` at all | `z_axis_boundary_recovery_plan.md` — "tested; did not stop the crash" |
| 3 | Virtual ceiling | `sdf_map/virtual_ceil_height=1.9` stamps a real *occupied* voxel layer so `calcDistanceCost` in the B-spline optimizer treats it as a wall | It's a **soft, weighted cost term**, not a hard constraint — NLopt's unconstrained optimization can trade it off against smoothness/goal/waypoint costs when they dominate. **Confirmed actually violated live**: `mapping_stall_crash_root_cause.md` records a real reject at camera_init `z=2.47` — 0.57 m *above* the 1.9 ceiling | `mapping_stall_crash_root_cause.md`; current `sdf_map.cpp:464-465`; `bspline_optimizer.cpp` cost list (§3.3 below) has no Z-hold term at all, only obstacle-repulsion |
| 4 | Downstream guard | `flight_envelope_guard.py` rejects any command with Z outside `[eff_zw_min, eff_zw_max]` | `validate_command()` (current file, lines 237-240) is a **hard reject**, not a clamp — the never-implemented "Phase 3" from `z_axis_boundary_recovery_plan.md`. On reject, the guard replays the last *pre-violation* setpoint at zero velocity while the vehicle's real, undamped momentum carries it past that stale point — the resulting correction is a discontinuity, not a smooth stop | `z_axis_boundary_recovery_plan.md` §2.3; confirmed still true by reading current `flight_envelope_guard.py` — `validate_command` still returns `False` outright, `fuel_cb`'s clamp block only runs after validation already passed |
| 5 | State estimation (orthogonal to 1-4) | N/A — no existing mitigation catches this | A **CPU-contention-triggered FAST-LIO stall** (confirmed live, see §5) leaves PX4's EKF2 free-integrating on IMU alone with its only aiding source gone. Velocity was observed climbing to ~22.7 m/s and position diverging 26.65 m before recovery. **Any Z bound-check running against that same diverged pose is checking against fiction** — layers 1-4 all assume the reported position is roughly true, and this failure mode breaks that assumption for all of them simultaneously | `realtime_test_verification_2026-08-17.md` §"Result 2" |

**One more thing worth knowing before designing the fix**: a hard Z clamp already existed once, before the
"upstream FUEL" reintegration. `Phase_4_completion.md` / `implementation_plan.md` documented a **1.5 m altitude
cap enforced directly in `fuel_to_mavros_bridge.py`** ("clamped B-spline trajectory setpoint Z targets to a maximum
height of 1.5 m"), alongside `box_min_z=0.5`/`box_max_z=1.8`. That script no longer exists anywhere in the repo —
confirmed via `find . -iname "*fuel_to_mavros*"` (zero results) — it was replaced by the upstream `traj_server` +
`flight_envelope_guard.py` pipeline (commit `b4a93a5b`, "reintegration of official upstream FUEL exploration
framework"), and nothing in that migration re-established an equally hard clamp. **This plan is, in part, restoring
something that used to work and was lost, not inventing a new mechanism from nothing.**

---

## 2. Target architecture: four independent layers, not one fix

No single layer below is assumed sufficient on its own — that assumption is exactly what failed three times
already. Each layer must hold *even if every other layer fails*, matching the "defense in depth" pattern the
codebase already half-applies elsewhere (baro kept enabled as a cross-check alongside vision, per
`height_hold_baro_fastlio_analysis_2026-08-17.md` — same philosophy, extended to actually be complete this time).

```
Layer 1 — SENSING           TFmini (simulated) → PX4 EKF2, independent of FAST-LIO/ROS
Layer 2 — SEARCH SPACE      Frontier candidate Z + sdf_map box narrowed to a thin slab around cruise altitude
Layer 3 — TRAJECTORY        New hard(er) altitude-hold cost term in the B-spline optimizer itself
Layer 4 — ENFORCEMENT       Guard clamps-and-holds Z-only violations instead of hard-rejecting
```

Layer 1 fixes failure #5 (the only layer-1-4-independent one). Layers 2-4 each independently close one of
failures #1-4. None of them depend on the others succeeding.

---

## 3. Layer 1 — Sensing: TFmini in Gazebo SITL (not real hardware)

### 3.1 The existing `tfmini_downward_rangefinder_integration_2026-08-23.md` doc does not apply here — corrected

That doc describes wiring a physical TFmini to a **real flight controller over UART** (`SENS_TFMINI_CFG`, pin-level
wiring, "confirm the physical port-to-`/dev/ttyS*` mapping for your specific flight controller board"). This
repo's entire test workflow is **100% PX4 SITL + Gazebo Classic simulation** — there is no real flight controller,
no real UART, and confirmed by direct inspection of this repo's actual airframe file
(`simulation/PX4-Autopilot-v1.14.3/ROMFS/px4fmu_common/init.d-posix/airframes/1023_gazebo-classic_iris_vlp16`),
**zero rangefinder-related parameters are set anywhere in it**. `SENS_TFMINI_CFG` configures a real serial driver
module that is never invoked in SITL at all. Following that doc's hardware steps literally would build nothing.

**Keep that doc's PX4-parameter table for future real-hardware deployment** (it's correct for that case) — but the
simulation path below is a different mechanism entirely, verified by reading the actual SITL bridge source.

### 3.2 How a downward rangefinder actually reaches PX4 in this SITL setup

Traced end to end through the vendored PX4 source:

1. **Gazebo side**: a `<sensor type="ray">` (the stock `lidar`/`sf10a` model pattern) is read by
   `libgazebo_lidar_plugin.so` (`gazebo_lidar_plugin.cpp`), which publishes a Gazebo-transport
   `sensor_msgs::msgs::Range`.
2. **Bridge**: `gazebo_mavlink_interface.cpp:826` (`LidarCallback()`) converts that into a MAVLink
   `DISTANCE_SENSOR` message and sends it over the same lockstep TCP link (port 4560) already used for every other
   sensor in this stack (the same plugin instance already present in
   `simulation/custom_models/iris_vlp16/iris_vlp16.sdf` for the mavlink interface).
3. **Auto-subscription requires a name match** — `gazebo_mavlink_interface.h:78-79` only wires up a lidar/sonar
   model whose name matches `.*(lidar|sf10a).*` or `.*(sonar|mb1240-xl-ez4).*`. **The attached model's name must
   contain one of those substrings or the bridge silently never forwards it.** This is a non-obvious constraint
   worth a code comment when implemented.
4. **PX4 side**: `src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp:357-359` handles
   `MAVLINK_MSG_ID_DISTANCE_SENSOR` → `publish_distance_topic()` (line 1468) → uORB `distance_sensor` topic,
   completely bypassing `SENS_TFMINI_CFG` and the real serial driver.
5. **EKF2 consumption**: `EKF2_RNG_CTRL` (`ekf2_params.c:730`) already **defaults to `1`** (enabled/conditional) at
   the firmware level — nothing in the current airframe file overrides it to `0`. Once a `distance_sensor` uORB
   message exists, EKF2 is already willing to use it; the sensor is the only missing piece, not the fusion
   enablement.

**Working template already vendored in this exact tree** (no new plugin code needed):
`Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_opt_flow/iris_opt_flow.sdf` (lines 25-34) attaches
the stock `model://lidar` via a fixed joint to `iris::base_link`, pitched straight down:
```xml
<include>
  <uri>model://lidar</uri>
  <pose>0 0 -0.05 0 0 0</pose>
</include>
<joint name="lidar_joint" type="fixed">
  <parent>iris::base_link</parent>
  <child>lidar::link</child>
</joint>
```
`models/lidar/model.sdf` itself (`libgazebo_lidar_plugin.so`, `min_distance=0.2`, `max_distance=15.0`) is a
reasonable stand-in for a TFmini's range envelope (TFmini-S: 0.1-12m) — either use it as-is (accept the 0.2m near
clip) or clone it into `simulation/custom_models/tfmini_lidar/` with `min_distance` tightened to `0.1` and a
model name containing `lidar` (required for step 3 above) to be semantically honest about what it represents. No
new C++/plugin code is required either way.

**No extra `GAZEBO_MODEL_PATH` plumbing needed** — `scripts/build_px4.sh` already exports
`$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models` (where `model://lidar` lives) onto
`GAZEBO_MODEL_PATH`, so `model://lidar` (or a custom model placed under `simulation/custom_models/`, also already
on that path) resolves without touching `test_takeoff.sh` or `build_px4.sh` at all.

### 3.3 Concrete changes

1. **`simulation/custom_models/iris_vlp16/iris_vlp16.sdf`**: add the `model://lidar` include + fixed joint to
   `iris_vlp16::base_link`, pitched straight down (`0 1.5707963 0` rotation, matching the `sf10a` template's
   pitched-sensor pose convention), mounted away from the velodyne_link and prop wash (e.g. `0 0 -0.08 0
   1.5707963 0` — centered under the fuselage, verify no collision-mesh overlap once built).
2. **PX4 params** (via `mavparam set`, same pattern as the existing `MIS_TAKEOFF_ALT` call in
   `test_takeoff.sh:133`, or added explicitly to the airframe file for auditability):
   - `EKF2_RNG_CTRL = 1` — explicit, even though it's already the firmware default; the existing airframe file
     sets everything else explicitly (`EKF2_EV_CTRL`, `EKF2_BARO_CTRL`, ...) and should do the same here rather
     than rely on an unstated default.
   - `EKF2_MIN_RNG = 0.1` (matches TFmini's rated minimum).
   - **Deliberately keep `EKF2_HGT_REF = 3` (Vision) — do not switch to `2` (Range) as the original hardware-only
     doc recommended.** FAST-LIO/vision is still the only source of X/Y localization this stack has; there's no
     reason to also make it non-primary for height. The point of Layer 1 is **redundancy**, not replacement: with
     `EKF2_RNG_CTRL=1` and Vision still primary, EKF2 gets a second, independent height input that keeps
     reporting real ground-truth-quality range **even during a FAST-LIO/ROS-side stall** (failure #5), because the
     range sensor's data path runs entirely inside the Gazebo↔PX4 SITL process pair, never touching the ROS graph
     or FAST-LIO's IKFoM filter that failure #5 actually stalled. This is the concrete mechanism by which Layer 1
     helps with failure #5, not just "TFmini is less noisy than baro."
   - Keep `EKF2_BARO_CTRL = 1` (already set) — same "don't remove existing redundancy" rule already established
     for baro.
3. **Verification** (adapt §3 "Verification plan" from the original tfmini doc, corrected for SITL): confirm
   `distance_sensor` uORB topic is populated (`listener distance_sensor` in the PX4 shell, or `ekf2 status`
   showing range fusion active) after spawning, at a few known heights, before wiring this into the flight test —
   a bench check inside the sim, not a real bench test.

---

## 4. Layer 2 — Search space: collapse the exploration volume to a thin slab

Only `algorithm.xml` values change; no code.

| Param | Current | New | Why |
|---|---|---|---|
| `frontier/min_candidate_z` | 1.2 (camera_init) | **1.35** | Tightens the 0.4m band to 0.1m, centered on the cruise altitude below |
| `frontier/max_candidate_z` | 1.6 | **1.45** | Same |
| `sdf_map/box_min_z` (passed from `nidar_fuel_upstream.launch`, currently `0.2`) | 0.2 | **1.0** | Shrinks the logical search volume close to cruise altitude — reduces how often A*/frontier search even considers off-band points, cheap defense even though (per §1, failure #2) it's not sufficient alone |
| `sdf_map/box_max_z` | 1.9 | **1.9** (unchanged) | Leave matching `virtual_ceil_height` — no reason to move the physical ceiling wall itself |
| `sdf_map/virtual_ceil_height` | 1.9 | **1.9** (unchanged) | Still a useful cost-term deterrent even though Layer 3 is now the real enforcement |

**Cruise altitude choice: 1.5 m world (1.4 m camera_init, using the guard's existing `zw = zc + 0.1` transform).**
Not a new number — this reuses `MIS_TAKEOFF_ALT` (`test_takeoff.sh:133`), `default_altitude` in
`flight_envelope_guard.yaml`, `traj_server/init_z` (`nidar_fuel_upstream.launch:9`), **and the original,
pre-3D-exploration altitude cap directive** documented in `Phase_4_completion.md`/`implementation_plan.md`
("restrict flight and mapping altitude strictly to 1.5 meters maximum"). Every part of the stack already agrees
on 1.5m as *the* altitude; this plan is the first to make every layer actually enforce it simultaneously.

---

## 5. Layer 3 — Trajectory optimization: a real altitude-hold cost term (the fix nobody implemented)

`height_hold_baro_fastlio_analysis_2026-08-17.md` listed this as **Option 3** ("add a soft altitude-hold bias
term... most faithful to 'prefer 1.5m'... most implementation effort") and explicitly went with the cheaper
Option 1 (frontier band clamp) instead. Given Option 1 + the virtual ceiling (Option-1-adjacent) have now both
been tried and both still leak (§1, failures #1 and #3), Option 3 is no longer optional — it's the only layer that
constrains every control point of the *actually flown* spline, not just its endpoint.

**Exact insertion point**, verified against the current file:

- `bspline_opt/include/bspline_opt/bspline_optimizer.h`: alongside the existing `calcSmoothnessCost`,
  `calcDistanceCost`, `calcFeasibilityCost`, `calcViewCost` declarations, add:
  ```cpp
  void calcAltitudeCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient_q);
  ```
- `bspline_opt/src/bspline_optimizer.cpp`: alongside the existing `ld_smooth_`/`ld_dist_`/... reads (currently
  lines 26-34), add `nh.param("optimization/ld_alt", ld_alt_, -1.0);` and a new `nh.param("optimization/z_cruise",
  z_cruise_, 1.4)` (camera_init frame, matching §4's cruise altitude).
- `calcAltitudeCost()` implementation, structurally mirroring `calcSmoothnessCost` (quadratic penalty +
  analytic gradient, the same pattern every other cost term in this file already uses):
  ```cpp
  void BsplineOptimizer::calcAltitudeCost(const vector<Eigen::Vector3d>& q, double& cost,
                                           vector<Eigen::Vector3d>& gradient_q) {
    cost = 0.0;
    gradient_q.resize(q.size(), Eigen::Vector3d::Zero());
    for (size_t i = 0; i < q.size(); i++) {
      double dz = q[i](2) - z_cruise_;
      cost += dz * dz;
      gradient_q[i](2) += 2.0 * dz;
    }
  }
  ```
- `combineCost()` (`bspline_optimizer.cpp:518`): add a dispatch block in the same style as the existing
  `calcDistanceCost` block (lines 582-587):
  ```cpp
  if (cost_function_ & ALTITUDE) {
    calcAltitudeCost(g_q_, f_alt, g_alt_);
    f_combine += ld_alt_ * f_alt;
    for (int i = 0; i < ...) grad[dim_ * i + j] += ld_alt_ * g_alt_[i](j);
  }
  ```
  and add `ALTITUDE` to the existing cost-function bitmask (wherever `SMOOTHNESS`/`DISTANCE`/... are `#define`d or
  enumerated) so it can be toggled per-call the same way the others already are.
- `algorithm.xml`: `optimization/ld_alt` — **start at `5.0`** (between `ld_waypt=0.3` and `ld_smooth=20.0` in
  magnitude: dominant enough to matter against a 0.1m-tall legal band, not so large it overrides feasibility/
  smoothness entirely and produces jerky vertical corrections). This is a starting point for empirical tuning
  during verification (§7), not a derived constant — say so explicitly when implementing, don't present it as
  final.

**Why this is the layer most likely to actually close the gap**: every other layer either bounds a single point
(the frontier endpoint) or applies a cost the optimizer can trade away (the virtual ceiling). A per-control-point
quadratic pull toward `z_cruise_` is minimized precisely by holding altitude constant — it doesn't compete with
"avoid the ceiling," it makes "stay near 1.5m" the optimizer's actual objective, the same way `calcSmoothnessCost`
already makes "don't jerk" an objective rather than a boundary check.

---

## 6. Layer 4 — Enforcement: guard clamps instead of rejecting, with a re-derived margin

### 6.1 Tighten the band to match

`config/flight_envelope_guard.yaml`: `world_z_min: 0.3 / world_z_max: 2.0` → **`world_z_min: 1.45 / world_z_max:
1.55`**. This is not a new value invented for this plan — it's **exactly** the band already hardcoded in
`scripts/test_flight_envelope_guard.py`'s test suite, which `z_axis_boundary_recovery_plan.md` §2.6 flagged as
"stale" only because the live config had since been *widened* to `[0.3, 2.0]` to accommodate 3D exploration. Since
this plan removes the 3D requirement, reverting to that original band makes the test file correct again instead
of stale — no separate test-data update needed for the bounds themselves (see §6.3 for what *does* still need
updating there).

### 6.2 Re-derive `boundary_margin_z` — the existing formula doesn't fit a 0.1m band

`boundary_margin_z = 0.2` was sized against the old 1.7m-tall band (`[0.3, 2.0]`). Applied naively to a 0.1m band
via the existing `eff_zw_min = world_z_min + boundary_margin_z` / `eff_zw_max = world_z_max - boundary_margin_z`
formula, `0.2` on each side of a `0.1`-wide band **inverts the effective bounds to an empty set**
(`eff_zw_min=1.65 > eff_zw_max=1.35`) — every command would be rejected, permanently. This must be re-derived, not
reused:
- Set `boundary_margin_z` to something on the order of one control-loop's worth of vertical travel at the
  guard's `publish_rate=20Hz` and PX4's typical `MPC_Z_VEL_MAX_UP`/`_DN` (check the actual current value in the
  running PX4 instance before finalizing — this is exactly the "not yet checked" open item
  `z_axis_boundary_recovery_plan.md` §5 already flagged and never closed). As a starting point pending that check:
  `boundary_margin_z = 0.03` (leaves `eff_z ∈ [1.48, 1.52]`, a 4cm working band) — small enough to leave room for
  velocity damping to matter, large enough not to be pure noise given `sdf_map/resolution=0.1`.
- **This is the single most important number to verify empirically before trusting this plan** — too tight and
  the guard will reject/clamp almost continuously (excessive `CLAMPED_Z` events per §6.3, likely visible as
  jittery vertical micro-corrections); too loose and it defeats the point of tightening the band at all.

### 6.3 Implement the never-shipped Phase 3: clamp-and-hold instead of hard reject, for Z only

This was explicitly deferred as "optional hardening" in `z_axis_boundary_recovery_plan.md` back when the band was
1.7m tall and Z violations were rare. **With a 4-10cm effective band, the vehicle will be near a Z bound during
essentially all of active flight, not just at emergencies** — the existing hard-reject-and-freeze behavior (§1,
failure #4) would now fire constantly instead of occasionally, which would make tightening the band actively
*worse* for flight smoothness unless this is fixed at the same time. Promoting this from optional to **required**:

In `validate_command()` (`flight_envelope_guard.py`, currently lines 216-254): when only Z is out of
`[eff_zw_min, eff_zw_max]` and X/Y both pass, return a new distinct outcome (e.g. a `code="CLAMPED_Z"` result)
instead of `False`. In `fuel_cb()`, treat that outcome like an accepted command whose Z has already been clamped
to the boundary — i.e., let it fall into the existing accept path (lines 293+) rather than the reject path (lines
365+), so `last_valid_command`/`last_valid_pos_world` **keep updating continuously** instead of freezing at a
stale pre-approach position. Keep the existing hard-reject behavior for X/Y violations and for the
already-existing `VISION_STALE` case — this change is scoped specifically to "Z-only, otherwise-valid" commands,
per the original Phase 3 design note.

### 6.4 Test coverage

`scripts/test_flight_envelope_guard.py` needs updates regardless of the rest of this plan — it currently has zero
coverage for the yaw-rate clamp, the vision-staleness watchdog, or Z velocity damping (flagged and never closed in
both `z_axis_boundary_recovery_plan.md` §2.6 and `realtime_test_verification_2026-08-17.md` "Known gap"). Add:
- A test confirming the reverted `[1.45, 1.55]` bounds are read correctly (should already pass once §6.1 lands).
- A new test for the `CLAMPED_Z` path from §6.3 — assert a Z-only-violating command is clamped and forwarded, not
  dropped, and that `last_valid_pos_world` advances.
- A new test asserting `boundary_margin_z` (§6.2) never produces an inverted/empty effective range for the
  configured band — a regression guard against exactly the bug described in §6.2.

---

## 7. Sequencing and verification

Each layer is independently testable; land and verify in this order (cheapest/lowest-risk first, matching the
project's own established pattern from `z_axis_boundary_recovery_plan.md` §4):

1. **Layer 2** (`algorithm.xml` param changes only) — restart the stack, no rebuild needed (roslaunch XML is read
   fresh every run, same note as the original virtual-ceiling fix).
2. **Layer 4** (`flight_envelope_guard.yaml` + `flight_envelope_guard.py` changes) — Python, no rebuild.
3. **Layer 3** (`bspline_opt` C++ change) — requires `catkin build bspline_opt`; verify it succeeds cleanly before
   any live test, same discipline `height_hold_baro_fastlio_analysis_2026-08-17.md` and
   `realtime_test_verification_2026-08-17.md` both already used for their C++ changes.
4. **Layer 1** (Gazebo model + PX4 params) — needs a Gazebo respawn (SDF change), independently bench-verifiable
   inside the sim (§3.3) before it's ever load-bearing for a real flight test, since Layers 2-4 already constrain
   altitude without it — Layer 1 only needs to prove it keeps the *estimate* honest during a stall, which requires
   deliberately inducing CPU contention during a test (see §8) to actually exercise the case it's meant to catch.

**Use the existing four verification channels** documented in `cpu_bottleneck_implementation_plan_2026-08-17.md`
(telemetry logger, `get_model_state` ground truth, guard CSV, rosbag record) — nothing new to build there. Add one
new thing to watch for specifically: count `CLAMPED_Z` events in the guard CSV per run (§6.3) — a high rate is the
signal that `boundary_margin_z` (§6.2) needs loosening, a near-zero rate together with continued smooth flight is
the signal this is working.

**Do all of this with the run's own process-discipline rule already established and then abandoned once** (item 4
of `cpu_bottleneck_implementation_plan_2026-08-17.md`, "dropped per direction... GUI stays on"): for *verification
of this specific plan*, run headless (`./scripts/test_takeoff.sh false ...`) at least once per layer, specifically
to remove CPU contention as a confound while confirming the Z-band holds — re-enable GUI afterward for whatever
the team's normal workflow is. This isn't re-litigating that earlier decision for production use, only for
isolating whether a Z excursion during verification is this plan's layers failing or the already-known CPU issue
recurring (§8).

---

## 8. The CPU bottleneck question, answered directly

**Yes — confirmed real, confirmed to have already caused a full mission-ending failure once, and only partially
mitigated. It is not a closed issue, and it is mechanically linked to the height problem this plan addresses
(failure #5 in §1).**

- **Machine confirmed**: AMD Ryzen 5 7640HS, 6 physical cores / 12 threads (`lscpu`), matching exactly what
  `cpu_bottleneck_implementation_plan_2026-08-17.md` assumed.
- **Already fixed and verified live**: FAST-LIO Release build (`catkin_ws/src/FAST_LIO/CMakeLists.txt:4`,
  confirmed still `"Release"` today) and per-thread CPU pinning across 6 named processes
  (`test_takeoff.sh`'s `pin_process()` + `scripts/cpu_repin_loop.sh`, confirmed present and matching the plan's
  design exactly, including the two real bugs that plan's implementation found and fixed: wrapper/child PID
  mismatch for `gzserver`, and per-thread-not-per-process affinity).
- **Confirmed NOT fully closed, by the project's own later documentation**: `realtime_test_verification_2026-08-17.md`
  reproduced a live FAST-LIO stall **after** the pinning fix was already active (GUI on, as shipped) — `uptime`
  showed `load average: 12.60, 15.22, 9.90` on the 12-thread machine during the stall, FAST-LIO froze for ~35s,
  and PX4's EKF2 (vision being its only aiding source) free-integrated to a fictitious position 26.65m off truth
  before recovering. `premature_landing_root_cause_2026-08-17.md` independently notes the structural reason
  pinning alone can't fully close this: **`taskset` only restricts which cores a process *may* run on — it does
  not reserve those cores exclusively.** True isolation would need a cgroup/cpuset, which this environment cannot
  provide (`/sys/fs/cgroup` confirmed read-only even as root).
- **New evidence gathered today, machine idle**: `uptime` right now reads `load average: 9.68, 15.38, 9.44` with
  **nothing from the NIDAR stack running** (`ps aux` confirms no `gzserver`/`px4`/`mavros`/`fastlio` processes
  alive) — a spike from this same dev machine's own VSCode remote-server tooling (`cpptools` observed at 211%
  CPU during this investigation) is enough on its own to produce load averages in the same range that previously
  caused a live stall. Since `taskset` pinning doesn't exclude other processes from a pinned core, a background
  IDE indexing spike is a plausible, currently-unmitigated way to steal cycles from FAST-LIO's dedicated core
  (0,1) during an actual test run.
- **Why this matters for the plan above, specifically**: failure #5 (§1) — an EKF runaway during a FAST-LIO
  stall — defeats Layers 2-4 simultaneously, since all three assume the reported position is roughly true. Layer 1
  (TFmini via the SITL-native MAVLink path, §3) is the only layer immune to *this specific* failure mode, because
  its data path never touches the ROS graph or FAST-LIO. It still doesn't fix X/Y drift during a stall — only
  Z. **Recommend closing the loop by finally reconsidering the previously-rejected headless-default change**
  (item 1 of `cpu_bottleneck_implementation_plan_2026-08-17.md`) at least for competition-relevant verification
  runs, given the stakes changed from "mapping completeness" to "hard rule compliance."

---

## 9. Explicitly out of scope — don't assume these are covered

- **Collision-retreat behavior's own Z handling**: `collision_retreat_behavior_plan_2026-08-18.md` describes a
  retreat vector computed from the full 3D ESDF gradient (can include vertical component), clamped afterward to
  `min_candidate_z_`/`max_candidate_z_`. Confirmed via `git log` and direct grep that **this plan was never
  actually implemented** — no `retreat` logic exists in current `fast_exploration_fsm.cpp`. If/when it is,
  it must also respect Layer 2's tightened band (§4) and ideally route through Layer 3's altitude-hold cost
  (§5) rather than a separate clamp, or it becomes a sixth single point of failure this plan didn't close.
- **The corridor-deadlock retreat bug itself** (`premature_landing_root_cause_2026-08-17.md` root cause #5,
  "FUEL has no retreat behavior after repeated NO_PATH/collision loops... dominant, last-remaining known cause of
  incomplete coverage") is a **separate, still-open bug**, unrelated to altitude. Not addressed here.
- **`kMaxTrackingError=1.0m`** in the FSM's real-odometry fallback (`realtime_test_verification_2026-08-17.md`)
  was flagged as "chosen as a reasonable starting bound, not empirically tuned" and observed firing more often
  than expected. Not re-tuned here; worth revisiting once this plan's layers are live, since a tighter Z band
  changes how often small trajectory-tracking divergences occur.
- **Doorway-exit candidate sampling** (`fuel_exploration_parameters.md`) — already fixed separately
  (`candidate_rmin`, `candidate_rnum`, `candidate_dphi`, `min_candidate_clearance` all already at their
  recommended values in the live `algorithm.xml`, confirmed while reading it for this plan). Unrelated to
  altitude; mentioned only to confirm it's not accidentally re-broken by §4's changes (it isn't — none of the
  touched params overlap).

---

## 10. Rollback

Every change in this plan is config (`algorithm.xml`, `flight_envelope_guard.yaml`) or additive/isolated code
(one new cost-term function in `bspline_optimizer.cpp`, one new `validate_command` outcome in
`flight_envelope_guard.py`, one new Gazebo model include). None of it modifies FAST-LIO, MAVROS, or PX4 core
behavior. If Layer 3's new cost term destabilizes the optimizer (e.g. `optimization/ld_alt` too aggressive), set
it to `0.0` to disable without reverting the rest of the plan — the same pattern the codebase already uses for
`optimization/ld_view=0.0` (present, disabled) in the current `algorithm.xml`.

---

# Addendum (2026-09-03, same day) — Hard pin at the publish boundary, single-pass build plan

Direct follow-up request: don't just bias FUEL toward a fixed height — make FUEL **directly publish** the fixed
value so there is structurally no deviation, and specify the matching FAST-LIO and PX4 changes as one buildable
package. This addendum supersedes Layers 3-4 above as the *primary* enforcement mechanism (they remain in place as
defense-in-depth, demoted from "the fix" to "keeps the internal solve consistent with what gets forced out the
door"). Everything below was verified against the current committed code before being specified — no guessing from
file names or past doc claims.

## 11. New finding: a live, unconditional 1-meter Z bounce at every mission start

Reading `plan_manage/src/traj_server.cpp` in full (not excerpted) surfaced something none of the five failure
modes in §1 account for. In `main()`, **after** `percep_utils_.reset(...)` and **before** `ros::spin()`, completely
unconditional, no flag or param gating it:

```cpp
// traj_server.cpp:492-502 (current, unmodified)
for (int i = 0; i < 100; ++i) {
  cmd.position.z += 0.01;
  pos_cmd_pub.publish(cmd);
  ros::Duration(0.01).sleep();
}
for (int i = 0; i < 100; ++i) {
  cmd.position.z -= 0.01;
  pos_cmd_pub.publish(cmd);
  ros::Duration(0.01).sleep();
}
```

Starting from `cmd.position.z = init_pos[2]` (= `traj_server/init_z` = **1.5**, set at `nidar_fuel_upstream.launch:9`),
this publishes a **1.0 m climb, then a 1.0 m descent**, directly to `/position_cmd` (remapped to
`/planning/pos_cmd`, the exact topic `flight_envelope_guard.py` consumes) — reaching a commanded **2.5** before
returning to 1.5. This runs on every `nidar_fuel_upstream.launch` start, which `test_takeoff.sh:175` fires
**after the drone has already taken off, armed, and is in OFFBOARD** — so this isn't dead code exercised only in
simulation setup; the vehicle is airborne and listening when it fires. It bypasses every one of Layers 1-4 above
entirely: it never touches `frontier_finder.cpp`, `sdf_map.cpp`, or `bspline_optimizer.cpp` (no bspline is even
being evaluated yet — `receive_traj_` is still `false` at this point in `main()`), so no candidate-Z clamp, virtual
ceiling, or altitude-hold cost ever sees it. Only `flight_envelope_guard.py` could catch it, and only after the
fact, message by message, via the hard-reject-and-freeze path already identified as broken (§1, failure #4) — by
the time a reject fires, several earlier ramp steps have already been accepted and forwarded.

This reads as leftover bring-up/demo code from the upstream FUEL project (visually confirming Z control works on
first boot), never removed during integration. It is the single highest-confidence, most directly actionable
finding in this investigation — independent of TFmini, independent of CPU load, independent of the optimizer.
**Delete it.** `flight_envelope_guard.py`'s own `timer_cb` already keeps OFFBOARD alive with a safe hold stream
whenever no fresh command has arrived (that's its documented purpose), so nothing needs to replace this loop —
removing it is not removing a safety behavior, it's removing an unrequested one.

## 12. FUEL repo changes (`catkin_ws/src/fuel/fuel_planner`)

### 12.1 `plan_manage/src/traj_server.cpp`

**(a) Remove the startup wiggle** — delete lines 492-502 (both `for` loops) entirely. Nothing replaces them.

**(b) Hard-pin Z at the one real publish site**, `cmdCallback()` (the 100Hz timer callback that is the actual
production path — the `test()` function at lines 345-434 is dead code, never called, leave it alone). Immediately
after the existing block that sets `cmd.position.z/velocity.z/acceleration.z` from the evaluated bspline
(lines 304-312), force Z regardless of what the 3D solve produced:

```cpp
// after existing lines 304-312 (cmd.position.x/y/z, velocity, acceleration all set from pos/vel/acc)
cmd.position.z = z_cruise_;
cmd.velocity.z = 0.0;
cmd.acceleration.z = 0.0;
```

Declare `double z_cruise_;` alongside the other file-scope state (near `traj_duration_` etc.) and read it in
`main()` next to the existing `init_pos` params:
```cpp
nh.param("traj_server/z_cruise", z_cruise_, 1.4);  // camera_init frame; world = z_cruise_ + 0.1
```
Apply the same three-line override to the initial-value block (lines 477-479, `cmd.position.z = init_pos[2]`) so
the very first published command is already consistent — simplest is to just set `traj_server/init_z` to the same
`1.4` in the launch file (§12.2) rather than maintaining two separate constants.

**Why this is the real fix, and why Layers 2-3 from the base plan still matter anyway**: this override makes it
*structurally impossible* for `/planning/pos_cmd` to ever carry a non-cruise Z, independent of whatever the
optimizer internally solved. Layers 2 (tight frontier/box bounds) and 3 (the `calcAltitudeCost` term) are not
redundant with this — they keep the *3D solve itself* close to flat, so this override is a near-no-op correction
rather than a large last-instant jump that would otherwise fight the optimizer's own velocity/acceleration profile
for X/Y (which was implicitly solved assuming a slightly different Z coupling). Keep both.

### 12.2 `launch/nidar_fuel_upstream.launch`

- `init_z` arg: `1.5` → **`1.4`** (camera_init, matching §12.1's `z_cruise_` default so `traj_server/init_z` and
  `traj_server/z_cruise` never disagree).
- Add `<param name="traj_server/z_cruise" value="1.4" type="double"/>` to the `traj_server` node block.

### 12.3 `algorithm.xml` / `bspline_optimizer` (unchanged from the base plan, §4-§5 above)

Still apply Layer 2's tightened `frontier/min_candidate_z=1.35` / `max_candidate_z=1.45` and Layer 3's new
`calcAltitudeCost` with `optimization/z_cruise=1.4` — same values, now explicitly in service of "keep the solve
consistent with what §12.1 forces out" rather than being the enforcement mechanism itself.

## 13. FAST-LIO changes (`catkin_ws/src/FAST_LIO`)

FUEL doesn't only receive commands — `exploration_node` and `traj_server` both consume `/Fast_LIO/odometry`
directly as their own position truth (`nidar_fuel_upstream.launch:12`, remapped to `/odom_world` for both nodes).
If FAST-LIO's own Z estimate drifts (LiDAR-SLAM height estimation is inherently noisier than a direct rangefinder,
especially against a large flat arena floor with few vertical features), FUEL's internal map-building and the
FSM's tracking-divergence check (`kMaxTrackingError`, `realtime_test_verification_2026-08-17.md`) are working off
a shifting notion of height even with §12's output pin in place. Pinning the input, not just the output, closes
that gap — same "override at the publish boundary, not the estimator internals" philosophy as §12.1, applied one
node upstream.

**Do not touch FAST-LIO's IKFoM filter/estimator** — same reasoning `height_hold_baro_fastlio_analysis_2026-08-17.md`
already gave for why baro was never fused into FAST-LIO's core: adding a new measurement type to the filter's
residual/Jacobian model is invasive, third-party-internals surgery. This change is not that — it overrides the
already-computed output pose's one field, immediately before it's published, exactly as surgical as §12.1.

### 13.1 Recommended: patch `laserMapping.cpp`'s publish step directly

This repo already treats `FAST_LIO`'s build config as its own to modify (`CMAKE_BUILD_TYPE` was changed directly,
per `cpu_bottleneck_implementation_plan_2026-08-17.md`) — patching this one function follows that precedent and
needs no new node, topic, or remap.

`publish_odometry()` (`src/laserMapping.cpp:589-595`, current):
```cpp
void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    ...
```
Add, immediately after `set_posestamp(odomAftMapped.pose);` and before the publish call:
```cpp
// Fixed-altitude 2D flight: pin published Z to the TFmini-informed height instead of FAST-LIO's raw
// LiDAR-SLAM Z estimate, so FUEL's own map-building/tracking-divergence checks never see a drifting
// height. X/Y/yaw remain FAST-LIO's own untouched SLAM solve — only this one field is overridden, at
// the publish boundary, not inside the IKFoM filter. See
// PLANNING_DOCS/fixed_altitude_2d_flight_architecture_2026-09-03.md §13.
odomAftMapped.pose.pose.position.z = g_pinned_height_camera_init;
```
Add near the top of the file (file-scope, alongside the other globals like `odomAftMapped`):
```cpp
double g_pinned_height_camera_init = 1.4;   // fallback: cruise altitude, camera_init frame
ros::Time g_last_range_time;
const double kRangeStaleTimeout = 1.0;      // seconds — mirrors flight_envelope_guard's vision_timeout pattern
void rangeCb(const sensor_msgs::Range::ConstPtr& msg) {
  // TFmini publishes distance-to-ground (world-ish, sensor pointed straight down at the floor); convert
  // to camera_init using the same rigid offset flight_envelope_guard.py already uses (zw = zc + 0.1).
  g_pinned_height_camera_init = msg->range - 0.1;
  g_last_range_time = ros::Time::now();
}
```
and, in `main()`, alongside the existing `sub_pcl`/`sub_imu` subscribers:
```cpp
ros::Subscriber sub_range = nh.subscribe("/tfmini/range", 10, rangeCb);
```
with a staleness fallback checked in `publish_odometry()` right before the override line — if
`(ros::Time::now() - g_last_range_time).toSec() > kRangeStaleTimeout` (or the topic has never been seen),
fall back to the hardcoded `1.4` constant rather than an unbounded-stale reading, mirroring
`flight_envelope_guard.py`'s existing `vision_timeout` watchdog idiom exactly.

### 13.2 Alternative, if patching vendored FAST-LIO source is unwanted: a separate relay node

Add a small `scripts/height_pin_relay.py` (same shape as `scripts/relay_odometry.py`, which already does the
"subscribe to one topic, republish a corrected version" pattern) subscribing to `/Fast_LIO/odometry` and
`/tfmini/range`, republishing to a new topic (e.g. `/Fast_LIO/odometry_pinned`) with `.pose.pose.position.z`
overridden the same way as §13.1. Then repoint `odom_topic` in `nidar_fuel_upstream.launch:12` from
`/Fast_LIO/odometry` to `/Fast_LIO/odometry_pinned`. Zero changes to FAST-LIO's own source; one new node, one new
topic, one launch-arg change. Slightly more moving parts than §13.1, strictly lower risk. **Pick one — don't do
both.** §13.1 is the primary recommendation (fewer parts); use this if the team would rather never touch
`catkin_ws/src/FAST_LIO/src/`.

## 14. PX4 parameters — ready to run

**Superseded by direct decision (2026-09-03, confirmed): TFmini becomes PX4's sole height reference, not a
blended aid.** This reverses §3.3/§14's earlier "keep Vision primary" recommendation — the user explicitly wants
height *measurement* to come from TFmini only, which for PX4's own onboard EKF2 (the estimator that actually
flies the vehicle, feeding `/mavros/local_position/pose`) means switching the height reference itself, not just
enabling range as a secondary aid.

```bash
rosrun mavros mavparam set EKF2_HGT_REF 2      # Range — TFmini becomes PX4's authoritative height source
rosrun mavros mavparam set EKF2_RNG_CTRL 1
rosrun mavros mavparam set EKF2_MIN_RNG 0.1
# EKF2_BARO_CTRL stays 1 — baro remains enabled as a passive cross-check/consistency signal inside
# EKF2, it just no longer anchors the primary height state. Vision (EKF2_EV_CTRL) stays as-is too —
# it still supplies X/Y, only the height axis changes reference.
```

**Important corollary of "only," not present in the blended design**: with no automatic secondary height source,
a bad/stale TFmini reading (floor out of range, reflective surface, sensor fault) has nothing else backing it up
inside PX4 unless PX4's own EKF2 has built-in reference-fallback logic. **This needs verifying against the actual
v1.14.3 `ecl`/EKF2 estimator source before relying on it** — flagged, not yet confirmed, add to the next research
pass rather than assumed either way.

**Regardless of what PX4 does internally, add an explicit ROS-side backstop** — `flight_envelope_guard.py` already
has exactly this pattern for vision (`vision_cb`/`vision_timeout`, §Layer 4 base plan). Mirror it for range:
```python
self.sub_range = rospy.Subscriber('/tfmini/range', Range, self.range_cb, queue_size=1)
# range_cb records self.last_range_time, self.last_range_value, same shape as vision_cb
```
and add a `range_age > range_timeout` check in `fuel_cb` alongside the existing vision-staleness check — same
consequence (reject, fall back to `timer_cb`'s safe hold) — since with `EKF2_HGT_REF=2`, a stale/bad range reading
is now exactly as serious a failure mode as the vision stall documented in `realtime_test_verification_2026-08-17.md`,
not a lesser one.

Add all three `mavparam` lines above explicitly to the airframe file
(`simulation/PX4-Autopilot-v1.14.3/ROMFS/px4fmu_common/init.d-posix/airframes/1023_gazebo-classic_iris_vlp16`)
for auditability, matching how every other EKF2 param there is already set explicitly rather than left at
firmware defaults — or leave them as `mavparam` runtime calls in `test_takeoff.sh` if the team would rather not
touch the vendored airframe file at all; either is fine, just pick one so the value isn't set in two places that
could drift apart.

### 14.1 Confirmed today: `MPC_Z_VEL_MAX_UP = 1.8`

Down from the PX4 default `3.0` (`mc_pos_control_params.c:230`) — a deliberate slowdown, consistent with the
already-conservative `max_vel=0.6` horizontal cruise speed. Set via the same `mavparam`/airframe-file mechanism
as above.

### 14.2 `MPC_Z_VEL_MAX_DN` — open, not yet derivable from this model

Asked to size this "per landing gear height." Checked `simulation/custom_models/iris_vlp16/iris_vlp16.sdf` —
it doesn't define its own frame, it `<include>`s `model://iris` (the stock PX4/3DR quadcopter). Read that stock
model (`Tools/.../models/iris/iris.sdf`) in full: **it has no separate landing-gear/leg/strut links at all** — the
poses found near `base_link` (`0.13 -0.22 0.023`, etc.) are the four rotor mounts, not legs. The frame's rest
height above ground is just its `base_link` collision geometry, which is consistent with the small `~0.1m`
ground-clearance constant already baked into `flight_envelope_guard.py`'s `zw = zc + 0.1` transform — there's no
distinct "leg height" figure to derive a velocity cap from in this specific model.

PX4 already has a **separate, more conservative parameter specifically for final touchdown**:
`MPC_LAND_SPEED = 0.7` (`mc_pos_control_params.c:448`), not currently overridden, which PX4's own landing
detector applies automatically as the vehicle nears the ground — independent of `MPC_Z_VEL_MAX_DN`, which mostly
governs ordinary in-flight descent rate, not the final few centimeters. Given that:

- **Recommend leaving `MPC_Z_VEL_MAX_DN` at the PX4 default (`1.5`)** unless you have an actual physical figure in
  mind (e.g. from real hardware this is meant to eventually match) — there's nothing in this simulated model to
  derive a tighter number from, and `MPC_LAND_SPEED` already covers the safety-critical final-approach case.
- **Needs your input to close**: either confirm the default is fine, or give the actual number/reasoning you had
  in mind for "per landing gear height," since I don't have a real landing-gear dimension to compute from here.

## 15. Gazebo: the simulated TFmini, and where the CAD model drops in later

Two separate things need to reach the sensor: PX4's EKF2 (via MAVLink `DISTANCE_SENSOR`, §3.2 of the base plan)
**and** the ROS-side pin in §13 (via a plain `sensor_msgs/Range` topic). The stock vendored `model://lidar`
(`Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/lidar/`) only carries the first — it has no
ROS-facing plugin. Rather than editing that file in the vendored PX4 tree (keeps the vendored bulk untouched, per
`repo_cleanup_and_mind_map_2026-09-03.md`'s recommendation not to hand-edit vendored code), copy it into a new,
project-owned model:

1. **New directory**: `simulation/custom_models/tfmini_downward/` — copy `models/lidar/model.sdf` and
   `model.config` from the vendored tree as a starting point.
2. **Model name must contain a matched substring** — the MAVLink bridge only auto-subscribes lidar/sonar models
   whose name matches `.*(lidar|sf10a).*` (`gazebo_mavlink_interface.h:78-79`, verified in the base plan's §3.2
   research). Name it e.g. `tfmini_lidar` (contains "lidar", satisfies the regex, and is honest about what it is).
3. **Add a second plugin to the same `<sensor>` block**, alongside the existing `libgazebo_lidar_plugin.so`
   (Gazebo sensors support multiple plugins on one sensor definition — standard pattern), publishing the plain ROS
   topic §13 subscribes to:
   ```xml
   <plugin name="tfmini_ros_range" filename="libgazebo_ros_range.so">
     <topicName>/tfmini/range</topicName>
     <frameName>tfmini_link</frameName>
     <fov>0.05</fov>
     <radiation>infrared</radiation>
     <minRange>0.1</minRange>
     <maxRange>12.0</maxRange>
   </plugin>
   ```
   `libgazebo_ros_range.so` ships as part of `gazebo_plugins`, which `ros-noetic-desktop-full`
   (`docker/Dockerfile:59`, already the installed base) pulls in as a standard dependency — no new apt package
   expected, but **verify the plugin actually loads** (no `Gazebo: couldn't find plugin` error at spawn) as the
   first smoke test before relying on it, since exact SDF tag names can vary slightly by distro version.
4. **Mount it on `iris_vlp16`** in `simulation/custom_models/iris_vlp16/iris_vlp16.sdf`, copying the exact,
   already-working stock pattern from `Tools/.../models/iris_opt_flow/iris_opt_flow.sdf:25-34` (verified in the
   base plan's research — safer to replicate a proven pattern than invent a new pose/rotation convention):
   ```xml
   <include>
     <uri>model://tfmini_downward</uri>
     <pose>0 0 -0.05 0 0 0</pose>
   </include>
   <joint name="tfmini_joint" type="fixed">
     <parent>iris_vlp16::base_link</parent>
     <child>tfmini_lidar::link</child>
   </joint>
   ```
   Insert this near the existing `imu_sensor` block (ends ~line 94 per the base plan's file read) and before the
   model-level `mavlink_interface` plugin (starts ~line 103) — position doesn't matter functionally, just keeps
   sensor definitions grouped together in the file.
5. **No `GAZEBO_MODEL_PATH` changes needed** — `simulation/custom_models/` is already on the path
   (`scripts/build_px4.sh`'s existing `GAZEBO_MODEL_PATH` export), so `model://tfmini_downward` resolves for free.

### 15.1 CAD model — received and checked (2026-09-03)

File: `simulation/TF_MINI.STL` (confirmed present, 81,884 bytes). Full thorough check before touching anything:

| Check | Result |
|---|---|
| Format validity | Valid **binary** STL — header text ("solid Sensor TF Mini PLUS...") is misleading (binary STLs commonly keep an ASCII-looking 80-byte header; `file` alone reports it as generic "data"). Verified structurally: `80-byte header + 4-byte triangle count + count×50 bytes` matches the actual file size **exactly** (declared 1636 triangles → 81,884 bytes expected → 81,884 bytes actual) — not corrupt, not truncated. |
| Mesh complexity | 1,636 triangles, 4,908 vertices. Lightweight — no performance concern whether used for visual or collision. |
| **Bounding box (raw units in file)** | X: 25.0, Y: 21.0, Z: 35.0 |
| **Unit scale** | **The file is authored in millimeters, not meters.** 25×21×35 as meters would be a 25-meter sensor; as millimeters it's a plausible TFmini Plus housing size. **Requires `<scale>0.001 0.001 0.001</scale>` on the mesh URI** — this is the one real, easy-to-miss issue a "thorough check" needed to catch, and it's now measured, not guessed. |
| Origin | All three axes start at 0 — the mesh's local origin sits at a bounding-box **corner**, not centered. Purely cosmetic (affects where the visual appears relative to the mount pose); does not affect sensing function at all (see below). |
| Sensing-axis orientation | Cannot be determined from geometry alone — the bounding box doesn't say which face is the lens. Will need a quick visual check after first spawn to confirm the housing looks right-side-up; **does not block or risk anything functional** (see next section for why). |

**Does this affect the working pipeline? No — provided three specific things are done, all now fully specified
from the measurements above, not left as guesses:**

1. **Mesh scale**: `<scale>0.001 0.001 0.001</scale>` in the `<visual><geometry><mesh>` block (per the table above).
   Getting this wrong is the one change here with real blast radius — an unscaled 25×21×35**m** block spawned on
   the vehicle would visually engulf the arena and, if it were also the collision shape, cause Gazebo to detect
   massive interpenetration with the ground at spawn (the classic "model launches on spawn" failure). Mitigated
   by (2):
2. **Mesh is visual-only.** `<collision>` gets a plain box primitive sized `0.025 0.021 0.035` (meters — the
   measured bounding box, mm→m converted directly), not the CAD mesh itself. This is standard Gazebo practice
   regardless (detailed concave meshes as collision geometry are expensive and prone to exactly the kind of
   instability above) and means the origin-offset/orientation uncertainty in the table above **cannot** cause a
   physics problem — only how the housing looks, not how it collides or senses.
3. **Explicit `<inertial>` on the new link**, small and realistic: `<mass>0.005</mass>` (~5g, a real TFmini Plus's
   rough weight) with a correspondingly tiny inertia tensor. Without this, SDF links with no `<inertial>` block
   commonly default to **1kg** — on a ~1.5kg `iris` airframe, a silently-added phantom 1kg would be a ~65% mass
   increase and would visibly degrade flight dynamics. This is the second real risk a careless drop-in would hit;
   now explicitly called out so it isn't missed during implementation.

**Why none of this can affect FAST-LIO, FUEL, PX4, or the guard**: the mesh is pure visual/collision geometry on
a *new* link, added via a fixed joint, touching zero existing links/joints/plugins/topics in
`iris_vlp16.sdf` (velodyne_link, imu_sensor, and the mavlink_interface plugin are all untouched). The sensor's
actual *function* — the `<sensor type="ray">` block and its two plugins from §15 — is a completely separate SDF
element with its own independent `<pose>`, unaffected by the visual mesh's scale, origin, or orientation. Even in
a worst case where the mesh is somehow still wrong after all three mitigations above, the failure mode is
contained and visible (a mis-sized/mis-placed visual, or a Gazebo console error on spawn) — not a silent
corruption of estimation, planning, or control elsewhere in the stack.

Save the file as `simulation/custom_models/tfmini_downward/meshes/tfmini.stl` (copied from `simulation/TF_MINI.STL`)
when step 1 in §15 is implemented.

## 16. Single-pass execution checklist

In dependency order — later steps assume earlier ones already landed:

1. **`traj_server.cpp`** (§12.1): remove the wiggle, add the Z-pin + `z_cruise_` param. `catkin build plan_manage`.
2. **`launch/nidar_fuel_upstream.launch`** (§12.2): `init_z` → `1.4`, add `traj_server/z_cruise` param.
3. **`algorithm.xml`** (base plan §4, §12.3): tighten `frontier/min_candidate_z`/`max_candidate_z`, `sdf_map/box_min_z`.
4. **`bspline_optimizer.{h,cpp}`** (base plan §5): add `calcAltitudeCost`, wire into `combineCost`, add
   `optimization/ld_alt`/`optimization/z_cruise` params. `catkin build bspline_opt`.
5. **`flight_envelope_guard.yaml` / `.py`** (base plan §6): tighten bounds, re-derive `boundary_margin_z`,
   implement the `CLAMPED_Z` clamp-not-reject path.
6. **`laserMapping.cpp`** (§13.1, or the §13.2 relay node — pick one): add the Z-pin + TFmini subscriber + staleness
   fallback. `catkin build fast_lio`.
7. **`simulation/custom_models/tfmini_downward/`** (§15): new model, copied from stock `lidar`, second ROS plugin
   added, placeholder mesh.
8. **`iris_vlp16.sdf`** (§15.4): mount the new sensor.
9. **PX4 params** (§14): set `EKF2_RNG_CTRL`/`EKF2_MIN_RNG`, via airframe file or `mavparam`.
10. **Verify, staged** (base plan §7): Gazebo spawn sanity (plugin loads, `/tfmini/range` publishes sane values at
    known heights) → headless run confirming `distance_sensor` uORB populates and EKF2 shows range fusion active →
    full `test_takeoff.sh` run confirming `/planning/pos_cmd` and `/mavros/setpoint_raw/local` never carry Z ≠ 1.4
    (camera_init) for the entire mission, checked directly from the guard's CSV log, not just visually.

## 17. What's still needed from you

- **The CAD model — path given doesn't resolve in this environment.** `/home/ayush/Desktop/NIDAR/Sensor TF Mini
  PLUS.STL` is a path on your own machine, not inside this dev container (`/home/developer/NIDAR`, confirmed via
  `whoami`/`$HOME`) — checked, the file isn't reachable from here. Needs to actually be copied/uploaded into this
  environment (e.g. `docker cp` into `ros_workspace`, or attach it directly in chat) before it can be used. Blocks
  nothing else in this plan either way (§15.1) — placeholder geometry ships first regardless.
- **`MPC_Z_VEL_MAX_DN`** (§14.2) — no landing-gear geometry exists in this model to derive a number from;
  recommend the PX4 default (`1.5`) unless you have an actual figure in mind.
- **`EKF2_HGT_REF=2` internal-fallback behavior** (§14) — needs verifying against the real v1.14.3 EKF2 estimator
  source before treating PX4 as having its own automatic backstop; the ROS-side `range_cb`/`range_timeout`
  watchdog in `flight_envelope_guard.py` is being added regardless, so this doesn't block anything, just changes
  how much redundancy exists below the ROS layer.
- **Confirmed, applied above**: `MPC_Z_VEL_MAX_UP=1.8` (§14.1); `EKF2_HGT_REF=2` — TFmini is PX4's sole height
  reference, not blended with Vision (§14, confirmed 2026-09-03).
- **Everything else in this addendum is ready to implement as specified** — no other external input needed.

## 18. Process note — `graphify`, per `/home/developer/NIDAR/CLAUDE.md`

This repo's `CLAUDE.md` designates `graphify` (installed at `~/.local/bin/graphify`) as the first stop for
codebase questions ahead of raw grep, and requires `graphify update .` after any code change to keep the graph
current. All research for this plan and its addendum was done by direct code reading rather than `graphify query`
— worth reconciling before implementation starts (query it for the traj_server/flight_envelope_guard/FAST-LIO
relationships this plan touches, to cross-check nothing here was missed). Once code changes actually land per
§16's checklist, `graphify update .` runs as the last step of each build, not just once at the end.

---

## 19. First live test: three defects found and fixed (2026-09-04)

The first end-to-end run after implementation armed successfully but never climbed. PX4 logged
`Ready for takeoff!`, `Armed by external command`, and even `Takeoff detected`, yet the vehicle
stayed on the ground. Root cause was a chain of three independent defects, all in this feature's
own implementation.

### 19.1 Defect 1 — the TFmini was added to an SDF nothing loads

`simulation/custom_models/iris_vlp16/iris_vlp16.sdf` is **not** the model Gazebo spawns.
`scripts/test_takeoff.sh` runs `roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_vlp16` with no
`sdf:=` argument, so the file resolves through that launch file's default:

```
$(find mavlink_sitl_gazebo)/models/$(arg vehicle)/$(arg vehicle).sdf
  -> simulation/PX4-Autopilot-v1.14.3/Tools/simulation/gazebo-classic/
     sitl_gazebo-classic/models/iris_vlp16/iris_vlp16.sdf
```

Two copies of `iris_vlp16.sdf` exist and had already drifted apart before this work. The unused
copy additionally declares its own `mavlink_interface` plugin *and* includes `model://iris` (which
carries one), so it would bind two MAVLink interfaces to the same ports if it were ever spawned.
The `<include>` went into the dead copy, so **no rangefinder ever spawned** and `/tfmini/range`
never published a single message. `test_takeoff.sh`'s `/tmp/bridge.log` showed the guard rejecting
every command with `RANGE_STALE: age=289.18s`, i.e. the age of the whole run.

Fixed by adding the sensor to the spawned copy, and by putting a header comment on the dead copy
pointing at the real one.

### 19.2 Defect 2 — the Z-pin fabricated an altitude when the sensor was silent

This is the defect that actually caused the reported symptom, and it would have been dangerous even
with the sensor present.

`g_pinned_height_camera_init` was seeded with a plain `1.4` default and `publish_odometry()` pinned
to it **unconditionally**. With no rangefinder in the world (§19.1), `rangeCb` never fired, so
FAST-LIO published a rock-steady `z = 1.4` while the vehicle sat on the ground. That reached EKF2
through `relay_odometry.py -> /mavros/vision_pose/pose` (`use_relative_origin` defaults to `False`,
so the value passed through absolute), so PX4 believed it was already at 1.4 m against a
`MIS_TAKEOFF_ALT` of 1.5 m. It armed, saw its altitude target as essentially met, and never climbed.

The failure mode generalises: *any* loss of the rangefinder would have made FAST-LIO assert a
confident, fixed, wrong altitude to the estimator. The fix gates the pin on a fresh, in-range
sample (`g_range_valid` + `g_range_timeout`, matching the guard's `range_timeout`) and otherwise
falls through to FAST-LIO's own Z with a throttled warning. `rangeCb` now also rejects non-finite
and out-of-band readings. **Degrading to a noisier real Z is always safer than publishing a
fabricated one.**

### 19.3 Defect 3 — PX4 was running a stale airframe file

PX4 SITL reads airframe scripts from `build/px4_sitl_default/etc/init.d-posix/airframes/`, not from
the `ROMFS/` source tree. The build copy still carried `EKF2_HGT_REF 3` and lacked `EKF2_RNG_CTRL`,
`EKF2_MIN_RNG` and `MPC_Z_VEL_MAX_UP`, so none of §14's parameter changes were in effect. Any edit
to `ROMFS/.../1023_gazebo-classic_iris_vlp16` needs a PX4 rebuild (or the build copy refreshed)
before it takes effect. Verify with a `diff` of the two paths, not by reading the source file.

### 19.4 Correction to §15 — the joint name drives PX4 discovery, not the model name

§15 stated that `gazebo_mavlink_interface` finds the rangefinder by matching the *nested model*
name against `.*(lidar|sf10a)(.*)`. That branch exists but **cannot fire for this vehicle**: it
only ever inspects `model_->NestedModels()[0]` (`gazebo_mavlink_interface.cpp:444-445`), which for
`iris_vlp16` is the base `iris` include. The mechanism that actually works is the *joint* scan
further down the same function (`:126-176`):

| side | source | resulting Gazebo topic |
|---|---|---|
| publisher | `gazebo_lidar_plugin.cpp:108-124` takes the second-to-last element of the sensor's scoped link name `iris_vlp16::tfmini_lidar::link` | `~/iris_vlp16/link/tfmini_lidar` |
| subscriber | `gazebo_mavlink_interface.cpp:126-176` matches joint `tfmini_lidar_joint` against the regex, strips `_joint` | `~/iris_vlp16/link/tfmini_lidar` |

Both the model name **and** the joint name are load-bearing, and they must agree. Dropping the
`_joint` suffix, or renaming either side, silently kills MAVLink `DISTANCE_SENSOR` while ROS
`/tfmini/range` keeps working normally — leaving `EKF2_HGT_REF=2` with no height source at all.
This also confirms Gazebo hands the plugin the **top-level** model, which is why the identical
stock `iris_opt_flow` + `model://lidar` pairing works upstream.

### 19.5 Verified results

Headless run, `./scripts/test_takeoff.sh false 0.0 -6.5 0.1 1.5708`:

* `/tfmini/range` streaming at 20 Hz (ROS path).
* `/mavros/distance_sensor/hrlv_ez4_pub` streaming with `min_range=0.09, max_range=12.0` — proves
  the MAVLink `DISTANCE_SENSOR` path reaches PX4 and that `EKF2_HGT_REF=2` has a real height source.
* Takeoff to cruise altitude, then autonomous FUEL exploration.

Mount-offset calibration at hover (the §17 open item), 39 near-stationary samples:

| quantity | measured |
|---|---|
| true mount offset (`gt_z - range`) | **0.0477 m** vs 0.05 configured — 2.3 mm error |
| FAST-LIO z error vs `gt_z - 0.1` | **0.0018 m** |

`g_tfmini_mount_offset = 0.05` needs **no** retune; the frame math in §13 is correct as written.

Altitude hold across the exploration flight:

| signal | min | max | mean | sd |
|---|---|---|---|---|
| FUEL commanded z (camera_init) | 1.400 | 1.400 | 1.400 | **0.0000** |
| guard → MAVROS world z | 1.500 | 1.500 | 1.500 | **0.0000** |
| EKF2 z (camera_init) | 1.370 | 1.420 | 1.399 | 0.0130 |
| FAST-LIO published z | 1.400 | 1.470 | 1.427 | 0.0208 |
| FAST-LIO/EKF2 divergence | 0.020 | 0.060 | 0.038 | 0.0129 |

The commanded altitude took exactly **one distinct value for the entire flight**, which is the
structural property §12.1 was aiming for. Guard verdict was `ACCEPT` on every sample: no
`CLAMPED_Z`, no `RANGE_STALE`, and FAST-LIO logged zero unpinned-fallback warnings. Max commanded
world altitude **1.500 m against the 2 m arena limit**.

---

## 20. Second live test: crash at t=175 s — a pre-existing thrust-margin defect

The first successful run held altitude perfectly for 145 s and then fell out of the sky at
t≈175 s. The `mission_telemetry_logger` output made this look like an altitude-estimation
runaway (`EKF z` jumping 1.40 → 0.79 → 16.83 → 9.98, `diverge=24.97m`). **That reading was wrong**,
and the correction matters: the telemetry logger's `t+N` stamps are relative to its own start,
which is ~45 s after PX4 boots, so they do not line up with the flight log. PX4's own
`vehicle_local_position_groundtruth` is the authority.

### 20.1 What actually happened

| PX4 log time | ground-truth z | rangefinder | motors | note |
|---|---|---|---|---|
| 30–175 s | **1.38–1.50 m** | 1.44–1.54 m | ~0.901 | textbook altitude hold |
| 175.5 s | 1.42 m | 1.47 m | 0.90 | still nominal |
| 176.0 s | 1.34 m | 1.46 m | 0.940 / **1.000** / 0.583 / 0.650 | motor saturation |
| 176.5 s | 0.80 m | 1.06 m | 0.786 / **1.000** / 0.303 / 0.600 | falling, thrust setpoint pinned at −1.000 |
| 177.0 s | **−0.03 m** | 0.00 m | — | on the ground |

Ground-truth altitude peaked at **1.50 m across the entire flight**, so the 2 m arena rule was never
approached. Attitude stayed level throughout — max roll 9.7°, max pitch 10.3° — so this was not a
tumble or a wall strike. The rangefinder was reading *correctly* all the way down (1.47, 1.46,
1.06), and the estimator tracked it faithfully (1.40, 1.33, 0.81). **Nothing in the height pipeline
misbehaved.** The vehicle simply ran out of thrust.

### 20.2 Root cause: the airframe hovers at 90 % throttle

| metric | value |
|---|---|
| mean motor output, hover phase (40–175 s) | **0.901** |
| samples with at least one motor ≥ 0.99 | **4.0 %** |
| PX4's own `hover_thrust_estimate` | **0.85** |
| `MPC_THR_HOVER` configured in the airframe | **0.65** |
| battery at crash | 16.2 V, 100 % remaining |

With hover already at ~90 %, only ~10 % of thrust remains for attitude control. A moderate
maneuver drives one motor to 1.000; the mixer must then reduce the others to hold attitude, total
lift drops below weight, and the vehicle descends. It is a matter of time before any given flight
loses that toss — this one lasted 175 s.

**This is not caused by the TFmini work.** A flight log from 2026-09-03, predating any of it, shows
an identical hover-phase mean motor output of **0.901** with the same 1.000 peak. The added
rangefinder is 5 g on a 2.33 kg vehicle (0.2 %).

Two candidate remedies, both of which change flight characteristics and so are left as an explicit
decision rather than applied here:

1. **Correct `MPC_THR_HOVER` 0.65 → ~0.85** to match PX4's own converged estimate. Cheap, and it
   improves the controller's feedforward, but it does not create thrust margin.
2. **Restore real margin** — reduce modelled mass (the VLP-16 link is 0.83 kg, which is accurate
   for the real sensor) or raise the iris motor `motorConstant`. This is the only fix that
   addresses the underlying deficit.

### 20.3 What the height pipeline did wrong — and it is fixed

The crash was not caused by this feature, but the feature handled the *aftermath* badly, and that
part was a genuine defect.

`libgazebo_ros_range.so` fills `min_range`/`max_range` from the `<ray><range>` element (0.06 / 35 m),
**not** from the plugin's own `<minRange>`/`<maxRange>` (0.1 / 12 m). 35 m is also exactly what the
ray sensor reports when the beam hits nothing. The §19.2 validity gate tested
`msg->range > msg->max_range`, i.e. `35.0 > 35.0`, which is false — so the no-return sentinel was
accepted as a real measurement. After touchdown the beam pointed at nothing, FAST-LIO published
`z = 34.95` for the remaining 80 s, `relay_odometry.py` clamped it to its 10 m ceiling, and PX4
settled at a confident 9.99 m while sitting on the ground. That is why the vehicle never recovered.

Fixed by gating on the rangefinder's **own operating band** (`mapping/tfmini_range_min` = 0.10,
`mapping/tfmini_range_max` = 12.0) instead of the message's advertised limits, which rejects both
the 35 m no-return and the 0 m underflow.

Note the two Gazebo plugins disagree on how they signal "no return": the MAVLink path reported
`0.00` while the ROS path reported `35.0`. Gating on a band catches both; gating on either
endpoint alone would not.

### 20.4 `EKF2_EV_CTRL`: tried 9, measured it, kept 11

`EKF2_EV_CTRL` is 11 = `HPOS|VPOS|YAW` (`EKF/common.h:151`), so vision supplies X/Y, yaw **and**
vertical position. Because FAST-LIO's published Z is itself the TFmini reading (§13), that means
the same TFmini-derived height reaches EKF2 through two channels: the direct MAVLink
`DISTANCE_SENSOR` path and the vision path. That redundancy was the channel which carried the
bogus 34.95 m into PX4 in §20.3, so dropping the `VPOS` bit to **9** (`HPOS|YAW`) was tried, on the
reasoning that it would make height literally rangefinder-only.

**Measurement rejected it.** With `EV_CTRL=9`, EKF2 lost its absolute height anchor and its
range-derived origin drifted:

| | `EV_CTRL=11` | `EV_CTRL=9` |
|---|---|---|
| ground-truth hover altitude | **1.50 m** (on target) | **1.69 m** (+0.19 m) |
| EKF2 z error vs ground truth | −0.03 m | **−0.28 m** |
| measured mount offset | 0.0477 m | 0.0469 m (unchanged, so not a sensor effect) |

Because the position controller flies to its *own* estimate, a 0.28 m low bias makes the vehicle
cruise 0.19 m high, eating that much of the 2 m arena margin — the exact failure this whole feature
exists to prevent. Reverted to 11.

The concern that motivated the change is instead fixed at its source (§20.3): FAST-LIO now rejects
out-of-band rangefinder samples rather than forwarding the ray sensor's 35 m no-return sentinel as
a real altitude, so the vision channel can no longer carry a fabricated height. Height still
*originates* from the TFmini alone; the vision path simply delivers it in a form that also anchors
the estimator's absolute origin.

### 20.5 Method note

Diagnose altitude behaviour from the PX4 `.ulg` (`vehicle_local_position_groundtruth`,
`distance_sensor`, `actuator_motors`), not from `mission_telemetry_logger` output. The logger reads
post-`relay_odometry` topics, so it shows values that have already been clamped and re-framed, on a
time base offset from the flight log. In this incident it turned a thrust failure into what looked
like an estimator runaway.

---

## 21. Verification runs 3 and 4, and the remaining blocker (2026-09-04)

### 21.1 Run 3 was invalid — incomplete inter-run cleanup

Run 3 showed FAST-LIO's X/Y diverging to (−26, 56) within 30 s and PX4 dropping OFFBOARD→ALTCTL.
This was **not** a code defect. A process census during the run found **3 gzserver, 3 px4 and
2 fastlio_mapping** processes alive simultaneously, plus an orphaned `rviz` consuming ~290 % CPU.
Two FAST-LIO nodes were publishing competing SLAM solutions onto the same `/Fast_LIO/odometry`
topic, so consumers saw the estimate jump between two unrelated answers — which reads exactly like
a tracking divergence.

`scripts/test_takeoff.sh`'s "Ensure clean slate" block never killed `rviz` (it is started by
`nidar_mapping.launch` and is not among the `killall` names) or `mission_telemetry_logger.py`
(which runs in the previous invocation's foreground), and it spawned replacements immediately
without waiting for the SIGKILLs to land. Both gaps are now fixed in that script. **Any back-to-back
run before this fix is suspect**, which is worth keeping in mind when reading older test results.

### 21.2 Run 4 — clean environment, final configuration

Stable cruise window, t = 35–111 s (76 s), all figures from the PX4 ulog:

| quantity | value |
|---|---|
| ground-truth altitude | min 1.136, max 1.423, mean **1.356**, sd 0.037 |
| ARENA RULE (< 2.0 m) | **PASS**, 0.577 m margin |
| EKF2 bias vs ground truth | **+0.041 m** |
| rangefinder out-of-band samples | **0 / 765** |
| FUEL commanded z | 1.400, sd 0.0000 |
| guard rejects / clamps | **0** |

The height pipeline is correct end to end. The residual −0.144 m mean error against the 1.50 m
target is not an estimation error (EKF2 is within 4 cm of truth) — it is the vehicle sagging
because it cannot hold the setpoint, which is §20.2.

### 21.3 The §20.3 gate fix, proven

Both runs ended with the vehicle on the ground and the rangefinder emitting its no-return sentinel.
The two runs differ only in the validity gate:

| | run 1 (gate tested `> msg->max_range`) | run 4 (gate tests the operating band) |
|---|---|---|
| EKF2 z while sitting on the ground | **9.99 m** | **−0.12 m** |
| FAST-LIO published z | 34.95 m | raw SLAM Z, with a throttled warning |

The sentinel also appears in **normal** operation, not just after a crash: while the vehicle is
landed the sensor sits ~0.01 m above the floor, below the ray's `<min>0.06</min>`, so Gazebo
reports 35.00 m every cycle before takeoff. The old gate accepted every one of those.

### 21.4 Remaining blocker: the airframe cannot hold itself up

This is the only thing still stopping a full-duration mission, and it is **not** part of this
feature. Runs 1 and 4 both ended the same way — motors saturating and the vehicle descending —
at t ≈ 175 s and t ≈ 112 s respectively. At t = 117 s onwards in run 4 the motors sit pinned at
**1.000 (100 % thrust) while the vehicle remains on the ground**, i.e. it cannot lift itself at all.

| | run 1 | run 4 | 2026-09-03 (pre-TFmini) |
|---|---|---|---|
| hover-phase mean motor output | 0.901 | 0.901 | **0.901** |
| samples with a motor ≥ 0.99 | 4.0 % | 5.1 % | — |

Identical before and after this work, so the TFmini (5 g on 2.33 kg) is not the cause. See §20.2
for the two candidate remedies; both change flight characteristics and need an explicit decision.

---

## 22. Run 5 — thrust fix applied, feature verified end to end (2026-09-04)

`motorConstant` raised 5.84e-06 → 9.6e-06 in the shared `iris.sdf` **and** its `iris.sdf.jinja`
template (the `.sdf` is generated from the `.jinja`, so editing only one lets a PX4 rebuild silently
revert the change). Thrust-to-weight goes 1.22 → **2.00**.

| | run 4 (T/W 1.22) | run 5 (T/W 2.00) |
|---|---|---|
| airborne duration | 87 s, ended in a thrust-limited crash | **338 s, still flying when stopped** |
| mean motor output | 0.901 | **0.680** |
| samples with a motor ≥ 0.99 | 5.1 % | **0.3 %** |
| ground-truth altitude sd | 0.037 | **0.025** |
| rangefinder out-of-band while airborne | 0 / 765 | **0 / 3317** |

Measured hover throttle 0.680 matches the 0.678 predicted from
`ω = 100 + control·1000`, `thrust = motorConstant·ω²` — and is now consistent with the
`MPC_THR_HOVER 0.65` the airframe already declared, so that parameter needs no change after all.

Final verification, run 5:

* absolute altitude (Gazebo ground truth) **1.4161 m**, sd 0.0103
* mount offset measured **0.0513 m** vs 0.05 configured — 1.3 mm error
* FAST-LIO Z frame error vs `gt_z − 0.1`: **0.0002 m**
* FUEL commanded z: 1.400, sd 0.0000
* guard: **zero altitude events**; the 5 rejects logged were all `OUT_OF_BOUNDS_X_MIN`, i.e. the
  horizontal envelope correctly catching FUEL at the arena's X edge
* every FAST-LIO range warning occurred at t = 4.7–27.4 s, all pre-takeoff, none in flight
* ARENA RULE: max 1.383 m (PX4 frame) — **PASS**

### 22.1 Residual: 8.5 cm absolute offset, erring safe

The vehicle cruises at **1.416 m absolute against a 1.50 m intent**. This is not jitter — hold is
tight (sd 0.010) — it is a constant bias in EKF2's height origin: EKF2 reads 1.400 where truth is
1.316, i.e. **+0.085 m**, and the position controller flies to its own estimate.

Cause: while landed, the rangefinder sits ~0.009 m above the floor, below both the ray sensor's
`<min>0.06</min>` and `EKF2_MIN_RNG 0.1`. EKF2 therefore initialises its height origin assuming
0.1 m of standoff that is not really there, and carries that ~0.09 m offset for the whole flight.

Left as-is deliberately: the error points **away** from the 2 m ceiling (the vehicle flies lower
than commanded, not higher), so it widens the safety margin rather than eating it. If exact
absolute altitude is ever wanted, lower `EKF2_MIN_RNG` toward the true on-ground standoff, or trim
`traj_server/z_cruise` up by the measured bias — but re-measure after any change to landing-gear
height or sensor mounting, since the bias is a function of both.
