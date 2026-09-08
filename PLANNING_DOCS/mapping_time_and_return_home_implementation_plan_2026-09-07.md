# Mapping-time reduction and return-home implementation plan (2026-09-07)

Status: **implementation plan only. This document changes no production code or configuration.**

## Objective

Map the reachable arena in the least safe elapsed time, then return to the launch pad and land
autonomously. The priority is to eliminate wasted target switching and dead-end work before
raising vehicle speed. Collision clearance, FAST-LIO health, LiDAR coverage, and landing
reliability remain hard constraints.

## Repository state reviewed

Branch: `fix/livox-sdk-cmake-hardening`.

The current uncommitted changes are the correct first part of this work and must be preserved
and validated before additional tuning:

| Change already present | Purpose | Required validation |
|---|---|---|
| `FrontierFinder::hasViewpointNear()` supports target holding | A valid committed target is no longer released merely because the ranked representative viewpoint moves | Target-change rate and distance-to-target reduce across repeatable runs |
| Deferred `requestRetireFrontierNear()` retirement | A repeatedly unreachable frontier cannot be selected forever; cost-matrix updates remain safe | No stale list/matrix index; a map change can revive a dormant frontier |
| EDM uses `cruise_z_camera_init` at handoff | Removes the world-AGL versus `camera_init` altitude mismatch | Reliable `WAIT_TRIGGER` to exploration transition without overshoot |

The completion/return control chain already exists:

```
FUEL FINISH -> latched /exploration_completed -> EDM RETURN -> PX4 AUTO.LAND
```

The gaps are that FUEL normally finishes only when no live frontiers remain, and the EDM returns
by reversing exploration breadcrumbs. Breadcrumbs are a safe fallback but may replay a long
frontier tour rather than use the shortest known route home.

## Baseline and instrumentation first

Run the current branch at least three times from a clean, identical simulation reset with flight
logging enabled. Record takeoff-to-trigger time, trigger-to-first-trajectory time, coverage
growth, total path length, revisited distance, longest coverage plateau, selected-target changes,
A* successes/failures, each frontier retirement, estimator resets, guard clamps, clearance, and
landing result. Use `scripts/analyze_exploration.py`, `scripts/verify_flight.py`,
`scripts/verify_full_flight.py`, and `tools/flightlog` as the reporting base.

Add structured events before tuning more parameters: `target_selected`, `target_held`,
`target_released` (with reason), `frontier_retired`, `completion_reason`, `return_route_choice`,
and `return_complete`. This separates target churn from controller oscillation or estimator drift.

## Planned implementation

### 1. Complete the anti-oscillation fix

1. Associate each A* failure streak with the committed target/frontier identity. Reset it when
   the target changes materially, a route succeeds, or the frontier is no longer active. The
   threshold must mean consecutive failures for one target, never failures across candidates.
2. Retire a frontier only through the existing deferred `searchFrontiers()` lifecycle. It alone
   records `removed_ids_` and keeps per-frontier cost/path lists aligned; do not erase a frontier
   from `planExploreMotion()`.
3. Start from the configured `exploration/target_fail_limit = 5`. Log frontier ID, target pose,
   vehicle pose, and failure reason for every retirement. Keep the existing map-change mechanism
   as the only way a dormant frontier becomes live again.
4. Retain `yaw_follows_path=true` and the 60 deg/s yaw limit. The VLP-16 is omnidirectional, so
   yawing toward each frontier adds risk to FAST-LIO without adding coverage.

Exit criterion: in three comparable runs, a retired unreachable viewpoint is never selected
again, target churn materially decreases, and coverage does not remain flat while targets repeat.

### 2. Address doorway coverage only with evidence

The current normal viewpoint minimum radius is 1.0 m, which was selected to retain real VLP-16
vertical visibility at the cruise band. Do not globally lower it to 0.3--0.4 m merely because
doors are narrow: that can create a reachable but vertically blind viewpoint and cause repeat
scans.

If logs prove a doorway has no valid normal viewpoint, add a constrained fallback:

1. Keep normal 1.0--2.6 m ring sampling for open-room frontiers.
2. If every normal candidate fails and the candidate region is a narrow known-free corridor,
   sample a small centreline-biased doorway set.
3. Require the existing inflated-occupancy and unknown-space checks plus an explicit LiDAR
   elevation/visible-cell check. Log collision, unknown-clearance, and visibility rejections.
4. Validate this scenario before increasing radial/angular sample density; measure replan latency
   and CPU before accepting denser sampling.

Obstacle inflation remains a safety contract, not a time-saving value to reduce without measured
airframe geometry and clearance validation.

### 3. Make completion explicit and trustworthy

Add a completion arbiter in `FastExplorationFSM`, retaining the existing FINISH re-checks.

1. Normal completion remains `NO_FRONTIER`.
2. Near-complete plateau: observable coverage is above a high-water mark and grows less than an
   epsilon over a window while no successful plan is being executed to a live frontier. Initial
   values to test: 90% observable coverage, under 1.0 m2 growth for 45 s.
3. Frontier exhaustion: every live candidate has been covered, safely retired after its
   identity-aware A* failures, or repeatedly fails trajectory generation; perform bounded
   re-checks before completion.

Record `no_frontier`, `coverage_plateau`, or `frontier_exhausted` as the completion reason. Only
after FINISH re-checks exhaust may FUEL publish the existing latched `/exploration_completed`.
Use `/sdf_map/coverage`'s observable-area definition; do not compare against floor that inflated
clearance makes physically unreachable.

Configuration will be added under `nidar/exploration_completion` in `mission_config.yaml` and
propagated by `apply_mission_config.py`: `min_coverage_pct`, `plateau_window_s`,
`plateau_growth_m2`, and `frontier_exhaustion_rechecks`.

### 4. Add a direct, map-aware route home

The map box ends just inside the arena door; the pad is outside it. Return therefore remains a
two-stage route:

```
final FUEL pose -- A* in known free map --> inside-door waypoint
inside door -> door centre -> outside door -> pad -- EDM corridor follower --> PX4 AUTO.LAND
```

1. After completion is confirmed, FUEL plans A* from latest odometry to configurable
   `return/door_inside` in `camera_init`. Require `REACH_END`; any shortcut must be collision
   checked against the inflated map.
2. FUEL publishes the result as a latched `nav_msgs/Path` on `/exploration/return_path`, frame
   `camera_init`, after the FINISH re-checks. Publish only a verified path; never encode A*
   failure as an empty successful route.
3. EDM subscribes before acting on completion. On RETURN, validate route frame, freshness, and
   pose count; follow the direct path, then append its proven inside-door, door-centre,
   outside-door, and pad points. Retain its lead-limited follower, cruise altitude, fixed yaw,
   timeout, and `AUTO.LAND` handoff.
4. If the direct route is missing, stale, invalid, or failed, log the reason and use the current
   breadcrumb return unchanged. A later follower timeout retains the existing land-in-place
   behavior.
5. Derive route coordinates from the active arena-entry and launch-pad configuration, including
   the documented world-to-`camera_init` conversion. Do not hardcode arena coordinates in C++.

### 5. Increase speed only after stability is proven

First establish the 0.6 m/s baseline and direct route. Then test return-only speed at 0.8 m/s,
then 1.0 m/s if tracking, clearance, guard behavior, and FAST-LIO warnings remain no worse than
baseline. Keep FUEL limits, EDM lead distance, envelope guard, and PX4 limits consistent at each
step. Consider exploration-speed changes only after anti-oscillation and doorway metrics are
clean. No yaw-rate increase is planned.

## Intended file changes

| File or area | Planned change |
|---|---|
| `fuel_planner/active_perception/{include,src}/frontier_finder.*` | Lifecycle-safe frontier identity/retirement query and candidate/fallback diagnostics. |
| `fuel_planner/exploration_manager/{include,src}/fast_exploration_manager.*` | Identity-aware A* failure policy, retirement call, and collision-checked return-path planning. |
| `fuel_planner/exploration_manager/{include,src}/fast_exploration_fsm.*` | Completion arbiter, FINISH-preserving handoff, latched return-path publication, completion telemetry. |
| `fuel_planner/exploration_manager/launch/algorithm.xml` | Explicit completion/return settings; no normal viewpoint-geometry change until doorway evidence supports it. |
| `nidar_mission/scripts/entry_detection_module.py` | Validate and follow direct path, append outside-map corridor, deterministic breadcrumb fallback. |
| `nidar_mission/config/mission_config.yaml`, `scripts/apply_mission_config.py` | Completion thresholds, door route, path freshness, staged return speed. |
| Verification and flight-log scripts | Report churn, retirement, completion reason, route choice/distance/time, and landing result. |

## Delivery sequence

1. Isolate/commit the current uncommitted fixes and collect baseline runs.
2. Implement identity-aware failure accounting and diagnostics; test matrix consistency in
   simulation.
3. Implement the completion arbiter and prove it cannot publish before FINISH re-checks finish.
4. Publish and inspect FUEL's direct path without changing EDM behavior.
5. Integrate EDM consumption plus breadcrumb fallback; run full simulation through landing.
6. Add doorway fallback only if logs show it is still the coverage blocker.
7. Tune return speed one measured step at a time, then consider mapping speed.

## Acceptance criteria

- Entry reliably reaches FUEL; no altitude-frame overshoot or prolonged `WAIT_TRIGGER`.
- No retired frontier is reselected; no frontier-cost-matrix crash or stale index occurs.
- Target churn and longest zero-growth plateau improve versus the baseline without reducing
  observable coverage.
- Completion has a reason and publishes `/exploration_completed` exactly once after its debounce.
- A direct route has the right frame, starts near the final pose, ends inside the door, and is
  collision-free in the inflated map.
- Invalid direct routes demonstrably fall back to breadcrumbs.
- Full mission transitions RETURN to LAND, lands within 1.0 m of pad centre, disarms upright,
  and has no wall contact or guard violation.
- Direct return is materially shorter/faster than breadcrumbs without worse FAST-LIO health,
  tracking error, or clearance; every speed step is compared with the preceding proven step.

## Out of scope until this is proven

Extending the map box to the pad, reducing obstacle inflation, increasing yaw rate, replacing the
EDM follower, and replacing PX4 landing are deliberately excluded. They are not needed for the
two-stage return and would enlarge the safety and regression surface.
