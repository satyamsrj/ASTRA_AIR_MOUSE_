# Minimum-time return to the launch pad after mapping completes (2026-09-06)

Status: **plan, not yet implemented.**

Goal: the moment the arena is mapped, fly to the launch pad by the shortest route the known
map allows, and land. Today the vehicle never leaves exploration at all.

Evidence base: run 3 (`~/.ros/log/2026-09-06/16_05_50.ulg`, 615 s airborne, 102.7% coverage,
0.0 m2 left at sim t=550, no crash), `/tmp/fuel.log` from the same run.

---

## 1. What already exists and works

The entire handoff chain is built and none of it has ever run:

```
FastExplorationFSM  FINISH state -> publishes /exploration_completed (latched Bool)
                       fast_exploration_fsm.cpp:107-133
traj_server         explorationCompletedCallback -> exploration_done_ = true
                       -> cmdCallback returns early, yielding /planning/pos_cmd
                       traj_server.cpp:269-281
EntryDetectionModule completed_cb -> EXPLORATION -> RETURN      (line 524)
                     run_return(dt)    breadcrumb pursuit        (line 834)
                     run_land()        PX4 AUTO.LAND             (line 899)
```

So the plan is not "build a return leg". It is: **make it trigger, and make the path it flies
short.**

## 2. Why it never triggers, and why that will not fix itself

`FINISH` is reached only via `NO_FRONTIER`, which the manager returns only when
`ed_->frontiers_` is empty (`fast_exploration_manager.cpp:114-118`).

In run 3, with the arena fully mapped:

```
sim t=550   coverage 101.4%, 0.0 m2 left
sim t=638   [FUEL DIAG] Frontiers detected: 4 | Visitable: 25 | Dormant: 0 | Viewpoints: 763
            grep -c "state: FINISH" /tmp/fuel.log  ->  0
```

Frontiers never empty. A cluster is moved to the dormant list in **one** place, when no
viewpoint can be sampled for it at all (`frontier_finder.cpp:411-412`). A cluster whose
viewpoint *is* sampled but cannot be **reached** stays in `frontiers_` forever.

This is now structural, not incidental: at `obstacles_inflation` 0.40 the vehicle can only
reach 60.6% of the arena floor, so there will *always* be frontier clusters with valid
viewpoints sitting in the 39% it cannot enter. **`NO_FRONTIER` can never fire in this arena.**
`Dormant: 0` across the whole run is the direct confirmation.

That same defect also wastes exploration time: the ATSP keeps costing and sometimes selecting
targets the A* cannot reach. It is the likely source of the 96 s coverage stall at t=259 and
of the 97.5 churn ratio at t=578.

## 3. Why the current return is not minimum time

`run_return` flies `_home_waypoints()` = **reversed breadcrumbs** plus the door exit. The
breadcrumb trail is a record of the *exploration* path, which is a frontier tour, not a route.
Loop-closure pruning (`_record_breadcrumb`) removes closed loops but not detours.

Measured at the instant mapping finished in run 3:

```
vehicle at ENU (-5.60, 2.55);  door at (0, -7.5);  pad at (0, -9.5)
distance actually flown while exploring     265.0 m
A* distance from there to just inside door   15.0 m
straight line (blocked by walls, lower bound) 11.3 m
door -> pad, outside the arena                2.7 m
DIRECT RETURN TOTAL                          17.7 m
```

The retrace is bounded below by 17.7 m and above by 265 m. The direct path is the whole win.

## 4. The one hard constraint: the map box stops at the door

`launch/nidar_fuel_upstream.launch` sets the SDFMap box in **camera_init**, where
`cx = world_y + 9.5`, `cy = -world_x`:

```
box_min_x 2.2   -> world y = -7.3      <- 0.2 m INSIDE the door line (y = -7.5)
box_max_x 16.3  -> world y =  6.8
box_min_y -6.8  -> world x =  6.8
box_max_y  6.8  -> world x = -6.8
```

The launch pad at world (0, -9.5) is **outside the map entirely**. So A* can plan to just
inside the door and no further. The return must be two-stage, and the outside-the-map stage
must stay on the proven open-loop waypoints. This is a constraint, not a preference.

---

## 5. Design

### Part A - make completion actually happen (the blocker)

**A1. Mark unreachable frontiers dormant.** In `FastExplorationManager::planExploreMotion`,
the A* to the chosen viewpoint already runs and can fail (`fast_exploration_manager.cpp:289`).
Count consecutive failures per frontier cluster; after N (propose 3), move the cluster to
`dormant_frontiers_` through the existing `FrontierFinder` path. Dormant clusters are already
re-checked when the map changes (`frontier_finder.cpp:89-92`), so a frontier that becomes
reachable later comes back on its own.

This is the principled fix: it makes `NO_FRONTIER` reachable, and it stops the ATSP costing
targets that cannot be flown. Expect it to reduce the exploration stalls as a side effect.

**A2. Coverage-stall backstop.** A1 depends on A* failing, and a frontier can also be
"reachable but never clearable" (the viewpoint is flyable, the cells stay unknown). So add an
independent trigger in the FSM: if `/sdf_map/coverage` reports `< 1.0 m2` of progress for
`return.completion_stall_s` (propose 45 s) **and** coverage is above
`return.completion_min_pct` (propose 90%), declare completion.

Both triggers feed the existing `FINISH` state, which already has the `finish_recheck_*`
debounce so a momentary frontier gap does not end the mission. Do not bypass it.

### Part B - a direct path home instead of the breadcrumb trail

**B1.** On entering `FINISH`, the exploration node plans one A* from the current position to
the door-side box edge, using the machinery already proven in `planExploreMotion`:

```
planner_manager_->path_finder_->reset();
path_finder_->search(pos, door_inside);      // door_inside = camera_init (2.3, 0.0, cruise_z)
path = path_finder_->getPath();
```

Publish it as a `nav_msgs/Path` on `/exploration/return_path` (latched).

**B2.** `EntryDetectionModuleNode` subscribes. In `run_return`, if a return path has been
received, use it in place of `reversed(self.breadcrumbs)`; otherwise fall back to the
breadcrumbs exactly as today. The door exit and pad waypoints from `_home_waypoints()` are
appended unchanged in both cases — they are outside the map and already proven.

The fallback matters: if A* fails (vehicle wedged somewhere odd), the mission must still come
home rather than have no path at all.

**B3.** Keep EDM's existing lead-limited pursuit and `return_accept` logic. It is proven, and
the path is what was wrong, not the follower.

### Part C - return speed

The return crosses a **fully mapped** arena with no frontiers to observe and no yaw changes
(`run_return` already holds heading — keep that; yaw rate is the one input that has broken
FAST-LIO in this stack).

Add `return.cruise_speed` (propose 1.0 m/s, from 0.6) applied as a larger `lead_limit` in
`run_return`. Raise in one step and **measure**, do not assume:

```
17.7 m at 0.6 m/s = 29.5 s
17.7 m at 0.8 m/s = 22.1 s
17.7 m at 1.0 m/s = 17.7 s
17.7 m at 1.2 m/s = 14.7 s
```

The airframe now has T/W 2.37 (was 1.90) so thrust is not the limit. FAST-LIO tracking is.
If localisation degrades at 1.0, fall back to 0.8 — the direct path is worth far more than
the speed increase and must not be risked for it.

`flight_envelope_guard.yaml max_speed` and FUEL's `max_vel` must be checked against whatever
is chosen; the guard is the authority and will clamp silently otherwise.

### Part D - landing

Unchanged. `run_land()` hands the descent to PX4 `AUTO.LAND`. `return.max_duration` (240 s)
stays as the timeout that lands in place rather than wandering.

---

## 6. Files to touch

| file | change |
|---|---|
| `fuel_planner/exploration_manager/src/fast_exploration_manager.cpp` | A1 dormancy on repeated A* failure; B1 `planReturnPath()` |
| `fuel_planner/active_perception/src/frontier_finder.cpp` | expose a way to retire one cluster to dormant |
| `fuel_planner/exploration_manager/src/fast_exploration_fsm.cpp` | A2 coverage-stall trigger; publish `/exploration/return_path` on FINISH |
| `nidar_mission/scripts/entry_detection_module.py` | B2 consume the path; C `return.cruise_speed` |
| `nidar_mission/config/mission_config.yaml` | `return.cruise_speed`, `completion_stall_s`, `completion_min_pct` |

## 7. Verification criteria

Extends `scripts/verify_flight.py`. All measured from ground truth.

1. FSM reaches `FINISH` and `/exploration_completed` publishes exactly once.
2. EDM transitions `EXPLORATION -> RETURN -> LAND`.
3. Return path length within **1.5x** of the A* optimum for the position where mapping ended
   (the retrace today would be many times that).
4. Return leg completes in under **45 s** of sim time.
5. Vehicle touches down within **1.0 m** of the pad centre, disarmed, upright.
6. Zero wall contact during the return (blade-tip clearance > 0 throughout).
7. Criteria 1-7 of the airframe plan still hold, i.e. no regression in the mapping phase.

## 8. Risks

- **A1 could retire a frontier that was only temporarily unreachable.** Mitigated by the
  existing dormant re-check on map change, and by requiring N consecutive failures rather
  than one. Do not set N = 1.
- **A2 could fire during a legitimate long transit.** Mitigated by the coverage floor
  (90%) and by `finish_recheck_*`, which retries planning before accepting the finish.
- **Higher return speed may degrade FAST-LIO.** Measured localisation error is already
  101-160 mm median with a per-sample noise floor of 92 mm. Raise speed in one step, measure,
  and revert if the error grows; the speed is the smallest part of the time saving.
- **A* may fail from a wedged start.** B2's breadcrumb fallback covers it.
- **The pad is outside the map.** By design the last 2.7 m is open-loop on proven waypoints;
  do not attempt to extend the map box to cover the pad as part of this work — that changes
  the frontier set and the coverage denominator, and would invalidate the mapping baseline.
