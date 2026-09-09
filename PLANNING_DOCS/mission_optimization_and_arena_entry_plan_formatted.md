# Mission Optimization & Autonomous Arena Entry Implementation Plan

I have a complete picture of the repo. Let me now write the response.

## Answers, then the implementation plan

### Your doubt first — camera coverage and drone orientation

You're not wrong, but the reasoning needs one correction and one caveat.

**Correction on "the drone always moves front-first":** That's a software choice, not a hardware one. The X500 is yaw-symmetric — nothing about the flight controller forces "front" to lead. What actually makes it fly nose-first today is a single FUEL parameter:

```
catkin_ws/src/fuel/fuel_planner/exploration_manager/launch/algorithm.xml:182
    <param name="exploration/yaw_follows_path" value="true" type="bool"/>
```

With that on, `traj_server` yaws the vehicle to align with the trajectory tangent. The SIYI A8 mini is rigidly bolted at body-x = +0.05 m, pitched −15° down, no gimbal — so the camera FOV always points along the direction of motion (81° H-FOV, per `/nidar/camera/horizontal_fov_rad = 1.4137`).

**On "entering from a different face":** The arena mesh (`arina_nidar`) is a closed box with a single 1.90 m opening at world y = −7.5, x = 0. There is no other face to enter through — you can *only* get in via the south door. So the risk isn't "wrong door", it's "we approach the south wall but arrive off-centre and clip a jamb", which is exactly what happened before `align_tolerance/corridor_half_width` were added.

**The real caveat you should worry about:** even though the drone enters and yaws-toward-path, **FUEL's viewpoints are LiDAR-visibility-driven, not camera-visibility-driven.** FUEL picks a viewpoint that clears frontier *cells*, then yaws toward the frontier centroid. That yaw is approximate; a survivor tucked into a corner ±90° off the flight tangent is inside LiDAR FOV (360°) but outside the camera's 81° cone. Two ways to close that gap:

1. **Cheap:** insert a 360° yaw sweep after FUEL declares FINISH, before RETURN starts — 6 s at 60°/s. Guarantees every wall the drone has been near gets photographed.
2. **Right:** run the survivor detector continuously from takeoff to landing (already planned), so any pass at any range that puts a person inside the 81° cone counts.

We already do (2) as long as `detection.enabled: true` (currently `false` in `mission_config.yaml:503`). Turn it on and add (1) at the end of exploration. I'll fold both into the change list below.

### Where the 13 minutes actually go

From the analysis docs and the FSM structure, the split for a successful 13-min run is roughly:

| Phase | Time | Bottleneck |
| ----------------------------- | ------------ | -------------------------------------------------------------------------------------------------------------- |
| Arm + climb to 1.5 m + settle | \~15 s       | EKF2 external-vision alignment resets (`settle_time = 3.0 s`)                                                  |
| ENTRY\_SEARCH → confirm       | \~30 s       | `search_forward_speed = 0.40 m/s`, 2 m corridor + alignment                                                    |
| **FUEL exploration**          | **\~11 min** | `max_vel = 0.6 m/s`, `target_switch_margin = 3.0 s`, coverage completion via `NO_FRONTIER` + 15 rechecks × 2 s |
| RETURN (breadcrumb retrace)   | \~60–90 s    | Retrace of frontier-tour path, not the shortest route home                                                     |
| AUTO.LAND                     | \~15 s       | PX4 land detector                                                                                              |

**All 5 min of headroom to hit ≤10 min sit inside exploration and return.** Faster takeoff or faster door approach saves seconds; faster exploration and a direct return leg save minutes.

### Change plan — ranked by ROI

Ordered from biggest wall-clock win downward. Every item lists its sign-off requirement so you can slot the FUEL-source ones into the team's review queue and start the no-sign-off ones today.

#### Tier 1 — expected 4–5 min saved

##### 1. Direct-path return (replaces breadcrumb retrace) — **\~60–150 s**

This is already spec'd in `PLANNING_DOCS/mapping_time_and_return_home_implementation_plan_2026-09-07.md` and `PLANNING_DOCS/minimum_time_return_leg_plan_2026-09-06.md` but not implemented. It's the single biggest win because the retrace is bounded below by the direct A\* distance (\~15 m) and above by the total flown path (\~265 m in one recorded run) — same 17.7 m end-to-end route, but the breadcrumb version can be 10× longer.

**FUEL source (sign-off needed):**

- `catkin_ws/src/fuel/fuel_planner/exploration_manager/src/fast_exploration_manager.cpp` — add `planReturnPath(start, door_inside)` reusing the existing `path_finder_` A\* that `planExploreMotion()` already uses at line 289. Publish the returned path.
- `catkin_ws/src/fuel/fuel_planner/exploration_manager/src/fast_exploration_fsm.cpp` — on entering FINISH, after the existing `finish_recheck_*` debounce (line 112), call `planReturnPath()` with target = `camera_init (2.3, 0.0, cruise_z)` (this is world `(0, −7.2)` — 0.3 m inside the south door, well inside the SDF map box which stops at 2.2). Publish as `nav_msgs/Path` on `/exploration/return_path`, **latched**. Only publish if `path_finder_->search()` returns `REACH_END` — never publish an empty path as success.

**Python (no sign-off):**

- `catkin_ws/src/nidar_mission/scripts/entry_detection_module.py` — subscribe to `/exploration/return_path` (frame `camera_init`) before RETURN transitions. In `_home_waypoints()` (line 806), replace `list(reversed(self.breadcrumbs))` with the received path's poses when the message is fresh (< 5 s old) and non-empty; else fall back to breadcrumbs exactly as today. The door-exit and pad points (lines 831–835) still append unchanged — they're outside FUEL's map box and must stay open-loop.

Acceptance: return leg length within 1.5× A\* optimum and completion < 45 s of sim time (already in the doc).

##### 2. Raise exploration speed 0.6 → 0.9 m/s — **\~3–4 min**

`max_vel` at 0.6 m/s is the FUEL config default and matches what `flight_envelope_guard.py` was tuned against. Exploration path length is roughly the same whatever speed you fly it, so time scales inversely.

**No FUEL-source change needed** — everything is driven from `/nidar/flight/max_vel` via `apply_mission_config.py`.

Config-only edit in `catkin_ws/src/nidar_mission/config/mission_config.yaml:260`:


```yaml
flight:
  max_vel: 0.9        # was 0.6
  max_acc: 0.45       # was 0.3 — keep the acc/vel ratio at 0.5 so B-spline optimisation stays feasible
```

Then run `rosrun nidar_mission apply_mission_config.py` — it re-writes `max_vel` in `nidar_fuel_upstream.launch` and every `search/max_vel`, `optimization/max_vel`, `bspline/limit_vel` in `algorithm.xml`.

**Do not raise** **`yaw_rate_deg_s`** **past 60**. The memory (and the `flight:` comment block) is explicit: 158°/s cost a flight to FAST-LIO degeneracy on 2026-09-05. That constraint isn't the vehicle, it's scan-matching against sparse VLP-16 correspondences inside a single 100 ms scan.

Validation: 3 back-to-back runs; watch for FAST-LIO "No Effective Points" warnings and any rise in the `pose_jump_max` telemetry. If either regresses, drop to 0.8.

##### 3. Frontier retirement — verify already-committed fix and tighten it

Commit `01e147b4` retires unreachable frontiers via progress watchdogs — this is what let completion actually fire. But `target_fail_limit = 5` at \~2 Hz replan means \~2.5 s before a doomed target is dropped. Combined with `target_switch_margin = 3.0 s`, the planner can spend \~5 s per bad frontier.

**FUEL launch (no source change, sign-off still recommended):**

`catkin_ws/src/fuel/fuel_planner/exploration_manager/launch/algorithm.xml:165` — no numerical change to `target_fail_limit`; instead, ensure `frontier/retire_min_interval = 5.0` and `frontier/retire_cooldown = 60.0` stay in place. These prevent the 2026-09-07 pathology where 19 frontiers retired in 0.9 s.

The material win here is confirming this survives at `max_vel = 0.9`.

#### Tier 2 — expected 30–90 s saved

##### 4. Return speed 0.6 → 1.0 m/s — **\~15–30 s**

Return crosses fully-mapped free space with heading held (`run_return` line 883 pins yaw). Applied as a larger `lead_limit` in `entry_detection_module.py:868`:


```python
# In run_return, around line 868:
lead_limit = rospy.get_param('/nidar/return/lead_limit', 1.6)  # was 1.0
```

And add to `mission_config.yaml` under `return:`:


```yaml
return:
  max_duration: 240.0
  lead_limit: 1.6    # NEW — 1.6 m lead at 20 Hz control = ~1.0 m/s tracking
```

Do NOT change `flight.max_vel` for this; the guard doesn't have an explicit velocity clamp (checked — `flight_envelope_guard.py` clamps position, not velocity). PX4's own MPC will follow the lead.

##### 5. Coverage-plateau completion trigger — **\~30–60 s**

Currently FINISH fires only when `frontiers_` is empty and after `finish_recheck_max × finish_recheck_interval = 15 × 2 = 30 s` of rechecking. A frontier that is reachable-but-not-clearable (viewpoint valid, cells stay unknown due to sensor grazing angles) keeps the mission alive indefinitely.

**FUEL source (sign-off needed):**

`fast_exploration_fsm.cpp` — add a plateau tracker: sample `/sdf_map/coverage` at 1 Hz, and if `growth_m2_last_45s < 1.0` **and** `coverage_pct > 90`, force FINISH (still passes through the `finish_recheck_*` debounce). Config lives under a new `mission_config.yaml:nidar/exploration_completion` block:


```yaml
exploration_completion:
  min_coverage_pct: 90.0
  plateau_window_s: 30.0    # was 45 in the plan; tighten
  plateau_growth_m2: 1.0
```

##### 6. Trim takeoff and door approach — **\~10–15 s**

- `entry.search_forward_speed: 0.40 → 0.60` (`mission_config.yaml:463`). Once aligned (the hysteresis gate is already there), 0.6 m/s inside the corridor is the same as the FUEL cruise it hands over to; the door is 1.90 m wide and the corridor is only 2 m of transit. Saves \~2 s at the approach.
- `entry.max_search_duration` stays at 90 s — that's a safety timeout, not a normal path.
- Move the survivor-detector startup from EXPLORATION-only to takeoff-onwards so the camera window is fully used (this is a mission-rule thing, no time saving but no cost either).

#### Tier 3 — cheap, worth doing

##### 7. Turn on the camera detector

`mission_config.yaml:503` — `detection.enabled: false → true`. The `nidar_mission.launch` should conditionally start a detector node reading `/camera/image_raw` and publishing to `/detections` + `/survivor_markers`. This is a required competition deliverable that is currently gated off. No time cost, but you can't complete the mission without survivor tagging.

##### 8. 360° camera sweep at end of exploration (before RETURN)

In `entry_detection_module.py`, add a new short state `POST_EXPLORE_SWEEP` between EXPLORATION and RETURN: publish a `PositionCommand` holding position with `yaw_dot = 1.047 rad/s` for 6.28 s, then transition. Guarantees any wall previously flown past gets a camera pass.

Cost: 6 s. Saves at least one missed survivor.

### The LiDAR range-discontinuity entry algorithm — implementation

Your attached doc (`autonomous_arena_entry_plan_2026-09-05.md`) proposes replacing the current door-prior-driven `ENTRY_SEARCH` with a 4-substate SCAN\_ROTATE → APPROACH → GAP\_SEARCH → TRAVERSE flow. Here is the concrete way to slot that into `entry_detection_module.py` without touching FUEL or any C++ code.

#### Why the current approach won't work at competition

Today's EDM reads the door line from `mission_config.yaml` (`/nidar/arenas/arina_nidar/entry/line = -7.5`, `center.x = 0`). That's a hardcoded prior that only exists because you measured the sim arena. On competition day you won't know the door's world position — you'll know approximately where the pad is relative to the arena wall, and nothing else.

The current code's fallback when perception doesn't match the prior (`detector_refine_limit = 0.10`, `mission_config.yaml:481`) rejects any lidar-detected gap more than 10 cm off the config. That will fail hard on an unknown arena — either the config is wrong and the drone flies into the wall, or the config is a placeholder and the 10 cm clamp is meaningless.

#### Adapted 4-substate architecture

Add to `entry_detection_module.py` after line 42 (the `MissionState` class):


```python
class EntrySubState:
    SCAN_ROTATE = "SCAN_ROTATE"
    APPROACH    = "APPROACH"
    GAP_SEARCH  = "GAP_SEARCH"
    TRAVERSE    = "TRAVERSE"
```

Then in `MultiCueEntryDetector.__init__`, add:


```python
# 360° polar range scan (1° bins), extracted per cloud_cb from the body-frame band
self.range_scan = np.full(360, np.inf)
self.range_scan_stamp = None

# Arena/gap bookkeeping
self.arena_bearing_body = None    # radians, body frame, from SCAN_ROTATE
self.arena_nearest_range = None   # metres
self.gap_center_body = None       # (angle_rad, range_m) in body frame from GAP_SEARCH
self.gap_width = 0.0
self.gap_inward_normal_body = None
```

Then a new method — this is the core algorithm from the doc, directly translated to work off the existing `process_point_cloud()` height-band filter that's already at line 154:


```python
def extract_range_scan(self, x_band, y_band):
    """Convert the height-band body-frame cloud into a 360-bin polar range scan.

    Reuses the existing (x_band, y_band) arrays produced by process_point_cloud()
    at line 154-160. Zero extra cloud iteration.
    """
    if len(x_band) == 0:
        self.range_scan[:] = np.inf
        return

    angles = np.arctan2(y_band, x_band)          # -pi .. +pi
    ranges = np.hypot(x_band, y_band)

    # Discard returns closer than the LiDAR blind radius (matches FAST-LIO's blind: 0.5)
    valid = ranges >= 0.5
    angles = angles[valid]
    ranges = ranges[valid]

    if len(ranges) == 0:
        self.range_scan[:] = np.inf
        return

    bins = ((angles + np.pi) / (2 * np.pi) * 360).astype(np.int32) % 360
    scan = np.full(360, np.inf)
    # Vectorised min-per-bin via np.minimum.at is O(N)
    np.minimum.at(scan, bins, ranges)
    self.range_scan = scan
    self.range_scan_stamp = rospy.Time.now()

def detect_arena_bearing(self):
    """Find the dominant obstacle cluster in the range scan — this is the arena.

    Called during SCAN_ROTATE. Returns (bearing_body_rad, nearest_range_m) or (None, None).
    The arena in a 15x15 m indoor space is by far the largest continuous block of sub-10 m
    returns; nothing else in the launch area comes close.
    """
    valid = self.range_scan < 10.0
    if valid.sum() < 30:
        return None, None
    # Weighted circular mean of the valid bearings, weighted by 1/range so nearer returns
    # dominate (avoids being pulled by a distant tree line).
    valid_bins = np.where(valid)[0]
    bearings = (valid_bins + 0.5) * (2 * np.pi / 360) - np.pi
    weights = 1.0 / self.range_scan[valid_bins]
    sx = np.sum(weights * np.cos(bearings))
    sy = np.sum(weights * np.sin(bearings))
    bearing = np.arctan2(sy, sx)
    nearest = float(np.min(self.range_scan[valid_bins]))
    return bearing, nearest

def detect_gap(self, min_width=0.80, max_width=3.50,
               jump_threshold=1.5, max_wall_range=15.0):
    """Range-discontinuity gap detector — Schleich et al. IEEE SSRR 2025 § III-B.3.

    Returns list of (center_bearing_body_rad, width_m, inward_normal_body_rad).
    The width filter here is deliberately wider than the current EDM's door_max_width
    of 3.0 because at first sight a doorway across the room can look wider due to
    partial occlusion; the caller filters again against the arena bearing.
    """
    scan = self.range_scan
    gaps = []
    edge_starts = []  # (bin_index, range_before, range_after)

    for i in range(360):
        j = (i + 1) % 360
        r_i, r_j = scan[i], scan[j]
        if r_i < max_wall_range and (r_j - r_i) > jump_threshold:
            # Wall ends: short -> long. Opening starts here.
            edge_starts.append((i, r_i, r_j))
        elif r_j < max_wall_range and (r_i - r_j) > jump_threshold and edge_starts:
            # Wall begins: long -> short. Opening ends here.
            i0, r0_before, _r0_after = edge_starts.pop()
            # Angular span, wrapping-safe
            span_bins = (j - i0) % 360
            if span_bins == 0:
                continue
            # Width from the mean gap depth
            gap_depth = max(r0_before, r_j)  # short-side wall ranges bracket the opening
            width = 2.0 * gap_depth * np.sin(span_bins * np.pi / 360)
            if not (min_width <= width <= max_width):
                continue
            center_bin = (i0 + span_bins / 2.0) % 360
            center_bearing = (center_bin + 0.5) * (2 * np.pi / 360) - np.pi
            # Inward normal is the radial direction at the gap centre
            inward = center_bearing
            gaps.append((center_bearing, width, inward))

    return gaps
```

#### FSM wiring in `EntryDetectionModuleNode.control_loop()`

Replace the `elif self.state == MissionState.ENTRY_SEARCH:` block (line 985) with:


```python
elif self.state == MissionState.ENTRY_SEARCH:
    if self.entry_substate is None:
        self.entry_substate = EntrySubState.SCAN_ROTATE
        self.substate_start = rospy.Time.now()
        rospy.loginfo("[EDM] ENTRY_SEARCH -> SCAN_ROTATE (mapping surroundings)")

    if self.entry_substate == EntrySubState.SCAN_ROTATE:
        # Hover on the pad, yaw at 60 deg/s for one full rotation (6 s).
        cmd = self.create_position_cmd(0.0, 0.0, dt)
        cmd.yaw_dot = 1.047                              # rad/s = 60 deg/s
        cmd.yaw = self.uav_yaw + 1.047 * dt              # integrate for setpoint
        self.pub_pos_cmd.publish(cmd)

        if (rospy.Time.now() - self.substate_start).to_sec() > 6.5:
            bearing, nearest = self.detector.detect_arena_bearing()
            if bearing is None:
                rospy.logerr("[EDM] SCAN_ROTATE: no arena-like structure in range. Retrying.")
                self.substate_start = rospy.Time.now()
                return
            self.detector.arena_bearing_body = bearing
            self.detector.arena_nearest_range = nearest
            # Turn to face the arena centre and enter APPROACH.
            self.entry_yaw = self.uav_yaw + bearing
            rospy.loginfo("[EDM] SCAN_ROTATE done: arena at bearing %+.0f deg, %.2f m. "
                          "-> APPROACH", math.degrees(bearing), nearest)
            self.entry_substate = EntrySubState.APPROACH
            self.substate_start = rospy.Time.now()

    elif self.entry_substate == EntrySubState.APPROACH:
        # Fly toward the arena with heading locked to arena bearing. Slow down
        # as nearest wall closes.
        nearest = self.detector.min_obstacle_dist
        target_speed = np.clip(0.2 + 0.1 * (nearest - 2.0), 0.2, 0.6)
        cmd = self.create_position_cmd(target_speed, 0.0, dt)
        cmd.yaw = self.entry_yaw
        self.pub_pos_cmd.publish(cmd)

        if nearest < 3.0:
            rospy.loginfo("[EDM] APPROACH: within %.2f m of wall. -> GAP_SEARCH", nearest)
            self.entry_substate = EntrySubState.GAP_SEARCH
            self.substate_start = rospy.Time.now()

    elif self.entry_substate == EntrySubState.GAP_SEARCH:
        # Wall-follow at 2 m offset while running range-discontinuity gap detection.
        gaps = self.detector.detect_gap()
        # Filter to gaps roughly in the direction of the arena — reject gaps between
        # unrelated distant returns.
        arena_dir = self.detector.arena_bearing_body or 0.0
        good = [g for g in gaps
                if abs(math.atan2(math.sin(g[0] - arena_dir),
                                  math.cos(g[0] - arena_dir))) < math.radians(90)]

        if good:
            # Pick the gap whose inward normal is most aligned with our current heading.
            best = min(good, key=lambda g: abs(g[2]))
            self.detector.gap_center_body = best[0]
            self.detector.gap_width = best[1]
            self.detector.gap_inward_normal_body = best[2]
            rospy.loginfo("[EDM] GAP_SEARCH: opening at %+.0f deg, %.2f m wide. "
                          "-> TRAVERSE", math.degrees(best[0]), best[1])
            self.entry_substate = EntrySubState.TRAVERSE
            self.substate_start = rospy.Time.now()
        else:
            # Simple proportional wall-follow, wall on port side. See doc § 3.5.
            desired_wall_dist = 2.0
            wall_error = desired_wall_dist - self.detector.min_obstacle_dist
            v_fwd = 0.30
            v_lat = np.clip(-0.4 * wall_error, -0.3, 0.3)
            self.pub_pos_cmd.publish(self.create_position_cmd(v_fwd, v_lat, dt))
            # Timeout: one full arena perimeter is ~60 m at 0.3 m/s = 200 s. Cap at 120 s.
            if (rospy.Time.now() - self.substate_start).to_sec() > 120.0:
                rospy.logerr("[EDM] GAP_SEARCH timed out — no opening found in one circuit.")
                self.pub_pos_cmd.publish(self.create_position_cmd(0.0, 0.0, dt))

    elif self.entry_substate == EntrySubState.TRAVERSE:
        # Fly through the opening along its inward normal.
        gap_ang = self.detector.gap_center_body
        # Reproject gap bearing every scan (it will slide as we approach)
        self.entry_yaw = self.uav_yaw + gap_ang
        cmd = self.create_position_cmd(self.search_forward_speed, 0.0, dt)
        cmd.yaw = self.entry_yaw
        self.pub_pos_cmd.publish(cmd)

        # Traversal complete when range-behind exceeds range-ahead by a margin —
        # this is the "the wall you came in through is farther than the wall in
        # front of you" test, exactly as in the doc.
        scan = self.detector.range_scan
        ahead_bin = int(((0.0 + np.pi) / (2 * np.pi)) * 360) % 360
        behind_bin = (ahead_bin + 180) % 360
        ahead = np.min(scan[max(0, ahead_bin-15):ahead_bin+16])
        behind = np.min(scan[max(0, behind_bin-15):behind_bin+16])
        if behind > ahead + 1.5:
            rospy.loginfo("[EDM] TRAVERSE complete: behind %.2f m > ahead %.2f m. "
                          "-> ENTRY_CONFIRMATION", behind, ahead)
            self.transition_to(MissionState.ENTRY_CONFIRMATION)
            self.entry_substate = None

        elif (rospy.Time.now() - self.substate_start).to_sec() > 15.0:
            rospy.logerr("[EDM] TRAVERSE timeout. Aborting.")
            self.pub_pos_cmd.publish(self.create_position_cmd(0.0, 0.0, dt))
```

#### What can stay and what must go

- **Keep** the `create_position_cmd()` integrator with lead-limit and corridor clamp (lines 651–712). It's what makes commands survive the guard.
- **Keep** the geometric door-crossing test (line 1076) as a **secondary** completion condition when the arena config is populated. Belt and braces.
- **Remove** the hard dependency on `_resolve_door_geometry()` (line 793) — the substate machine above never reads `self.door_camera_x/y`. Leave the method for the fallback case where the config has real numbers.
- **Remove** `detector_refine_limit` clamping (line 481, and the logic at line 1030) — it exists to keep the LiDAR from overriding the prior, but with no reliable prior the LiDAR IS the primary.

#### What breaks and how to catch it

Two of your invariants are safe:

1. **The guard still owns the envelope.** All commands go through `/planning/pos_cmd`. If the SCAN\_ROTATE bearing turns the drone toward the negative-y direction (away from the arena), the guard's `mission` envelope (`y_min = -10.5`) still catches the drift.
2. **FAST-LIO stability is unchanged.** Yaw rate stays at 60°/s in SCAN\_ROTATE. No new stress on the localiser.

Two new failure modes to instrument:

- **False arena bearing:** if the pad is placed with a tree line to its west and the arena to its east, `detect_arena_bearing()` might pick the wrong cluster. Mitigation: after SCAN\_ROTATE, publish the detected bearing and expected range to a diagnostic topic. If a manual sanity check disagrees, add a `entry.arena_hint_bearing_deg` config that biases the weighted mean.
- **Gap-detection false positive on interior wall segment:** the drone could find a gap between two arena walls (e.g., a corner viewed at an angle). Mitigation is already in the code — filter gaps to those within ±90° of the arena bearing, and reject widths < 0.80 m or > 3.5 m.

### Full change list, ordered for execution

Sign-off column: **NO** = you can implement today; **YES** = requires team review of FUEL/FAST-LIO source. This matches the gate in your memory.

| # | Change | Files | Sign-off | Est. saving |                                                                                             |                                                                                         |            |                                                                                |
| ------------------------------- | ------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------ |
| 1                               | Direct-path return (A\* to inside-door, publish latched Path)                               | `fast_exploration_manager.cpp`, `fast_exploration_fsm.cpp`, `entry_detection_module.py` | YES (FUEL) | 60–150 s                                                                       |
| 2                               | Raise `flight.max_vel` 0.6 → 0.9, `max_acc` 0.3 → 0.45                                      | `mission_config.yaml`, run generator                                                    | NO         | 3–4 min                                                                        |
| 3                               | Return leg 0.6 → 1.0 m/s (larger `lead_limit`)                                              | `mission_config.yaml`, `entry_detection_module.py` line 868                             | NO         | 15–30 s                                                                        |
| 4                               | Coverage-plateau completion trigger                                                         | `fast_exploration_fsm.cpp`, `mission_config.yaml`                                       | YES (FUEL) | 30–60 s                                                                        |
| 5                               | Replace `ENTRY_SEARCH` with SCAN\_ROTATE → APPROACH → GAP\_SEARCH → TRAVERSE (LiDAR-driven) | `entry_detection_module.py` (add \~200 lines, remove \~80)                              | NO         | Enables unknown-arena capability; \~0 s time saving vs current known-prior EDM |
| 6                               | Enable survivor detector                                                                    | `mission_config.yaml:503`, `nidar_mission.launch` (add YOLO node)                       | NO         | Deliverable, not time                                                          |
| 7                               | 360° yaw sweep before RETURN transition                                                     | `entry_detection_module.py` (new POST\_EXPLORE\_SWEEP state)                            | NO         | Costs 6 s, buys survivor coverage                                              |
| 8                               | `entry.search_forward_speed` 0.40 → 0.60                                                    | `mission_config.yaml:463`                                                               | NO         | 2–5 s                                                                          |

**Do items 2, 3, 6, 7, 8 first** — they are Python/YAML only and give you the largest guaranteed win. Then integrate item 5 (the new entry algorithm) once you have telemetry from a "known-prior" 0.9 m/s run to compare against. Items 1 and 4 are the FUEL-source changes that need team review — queue those now so they're not the critical path.

**Expected end state:** with items 1–3 alone (direct return + 0.9 m/s explore + 1.0 m/s return), a 13-min mission drops to roughly 8–9 min. Items 4 and 7 give another 30–90 s margin. Item 5 is the algorithmic robustness upgrade, not the time win.
