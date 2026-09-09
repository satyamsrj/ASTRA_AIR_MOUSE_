# Phase 1 — Coverage-Plateau Completion: Implementation Plan

Date: 2026-09-09
Parent: `PLANNING_DOCS/mapping_time_optimisation_review_and_plan_2026-09-09.md`
Reviews: `PLANNING_DOCS/drone_planner_stability_and_coverage_completion_plan.md`
Status: plan only — no code changed in this session.

---

## 0. Scope and verdict on the reviewed document

Phase 1 is **Python/YAML only, no FUEL sign-off, and saves 60–137 s.** It consists of one change:
the coverage-plateau completion trigger, taken from §4 of the reviewed document with corrections.

Verdict on that document's four proposals:

| # | Proposal | Verdict |
|---|---|---|
| 1 | Wall-standoff / admissibility margin | **Idea accepted, implementation rejected** → Phase 2 |
| 2 | Retirement escalation with retry memory | **Rejected** — cannot work as specified |
| 3 | Config-only cooldown/hysteresis stopgap | **Rejected** — harmful before Phase 0 |
| 4 | Coverage-plateau completion | **Accepted** — this is Phase 1, with tuned thresholds and 3 fixes |

Everything below §1 is the accepted item. §5 records why the other three were rejected, with the
evidence, so they are not re-proposed unchanged.

---

## 1. What Phase 1 does

Add a second, independent completion condition to the mission controller: when mapped free area
stops growing while coverage is already high, stop exploring and return.

It is **additive**. FUEL's existing `/exploration_completed` path stays exactly as it is and
remains the primary trigger. The plateau test only fires when FUEL has not yet declared finish.

### Measured value

Calibrated against all three verified successful runs (ground truth, landed on pad):

| run | touchdown | final cov | W=60 G=0.5 P=97 | W=30 G=0.5 P=97 |
|---|---|---|---|---|
| 20260908_141709 | 582 s | 98.70 % | fires t=492 @ 98.10 % → **−60 s** | fires t=462 @ 98.08 % → **−90 s** |
| 20260908_143514 | 495 s | 99.07 % | fires t=400 @ 98.85 % → **−65 s** | fires t=328 @ 98.12 % → **−137 s** |
| 20260909_040942 | 612 s | 99.61 % | fires t=520 @ 99.19 % → **−61 s** | fires t=492 @ 99.19 % → **−90 s** |

(saving allows 30 s to fly home from the trigger point, which is the measured return-leg duration)

Coverage given up: 0.22–0.60 % at W=60, 0.42–0.95 % at W=30. **Both keep final coverage ≥ 98.0 %.**

The reviewed document proposed W=60/G=0.5/P=97. That works and is the right place to start
observing; W=45 or W=30 is worth roughly another 30–70 s once the trigger is trusted.

---

## 2. Configuration

Add to `catkin_ws/src/nidar_mission/config/mission_config.yaml` under `nidar:`:

```yaml
  exploration_completion:
    # Additional completion trigger: mapped free area has stopped growing while coverage is
    # already high. FUEL's own /exploration_completed remains the primary path; this only
    # fires when FUEL has not yet declared finish.
    #
    # Calibrated 2026-09-09 against runs 20260908_141709 / 20260908_143514 / 20260909_040942
    # (all three ground-truth verified landings on the pad). At the values below the trigger
    # fires at 98.10 / 98.85 / 99.19 % coverage, saving 60 / 65 / 61 s.
    plateau_window_s:   60.0
    plateau_growth_m2:   0.5
    # 97.0 is a real safety gate, not decoration. Run 20260908_160711 published
    # exploration_completed = True at 95.3 % after the retirement machinery emptied the
    # frontier list; a floor of 97 blocks the plateau trigger from doing the same thing
    # faster. Lowering it to 95 changes NOTHING on healthy runs -- the plateau only occurs
    # above 98 % -- so the floor costs no time and buys protection. Do not lower it.
    min_coverage_pct:   97.0
    # Observe-only until the trigger has been validated on >=3 runs. See the rollout in
    # PLANNING_DOCS/phase1_coverage_plateau_completion_implementation_2026-09-09.md
    force_enabled:      false
```

`apply_mission_config.py` manages named params by regex and does not touch keys it does not know,
so adding a new block is safe and needs no generator change.

---

## 3. Code changes — `catkin_ws/src/nidar_mission/scripts/entry_detection_module.py`

Three edits. All Python. No FUEL, no C++, no rebuild.

### 3.1 Imports and state

Add `Float64MultiArray` to the `std_msgs.msg` import and `deque` from `collections`.

In `EntryDetectionModuleNode.__init__`, alongside the existing `/exploration_completed`
subscriber (line 442):

```python
self.coverage_hist = deque()          # (t, free_area_m2, pct)
self.plateau_fired = False            # one-shot latch

self.plateau_window_s   = rospy.get_param('/nidar/exploration_completion/plateau_window_s', 60.0)
self.plateau_growth_m2  = rospy.get_param('/nidar/exploration_completion/plateau_growth_m2', 0.5)
self.min_coverage_pct   = rospy.get_param('/nidar/exploration_completion/min_coverage_pct', 97.0)
self.plateau_force      = rospy.get_param('/nidar/exploration_completion/force_enabled', False)

rospy.Subscriber('/sdf_map/coverage', Float64MultiArray, self.coverage_cb)
```

### 3.2 The callback

Message layout is **verified** against `map_ros.cpp:207-210`:

```
data = [free_n, occ_n, unk_n, free_area, known_area, unknown_area,
        denom, pct, left, layers, elapsed_s]
```

so index **3 = free_area_m2** and index **7 = pct_observable**. Published every
`map_ros/coverage_interval` = 2.0 s.

```python
def coverage_cb(self, msg):
    if len(msg.data) < 8:
        return
    now  = rospy.Time.now().to_sec()
    free = msg.data[3]
    pct  = msg.data[7]

    # History is only meaningful within one exploration episode. Reset on entry so a
    # window carried over from TAKEOFF/ENTRY cannot satisfy the plateau test.
    if self.state != MissionState.EXPLORATION:
        self.coverage_hist.clear()
        return

    self.coverage_hist.append((now, free, pct))
    while self.coverage_hist and now - self.coverage_hist[0][0] > self.plateau_window_s:
        self.coverage_hist.popleft()

    if self.plateau_fired or len(self.coverage_hist) < 2:
        return
    # Require a FULL window before judging, or the test passes trivially at episode start.
    if now - self.coverage_hist[0][0] < self.plateau_window_s:
        return

    growth = free - self.coverage_hist[0][1]
    would  = (pct >= self.min_coverage_pct) and (growth < self.plateau_growth_m2)

    # Always logged, acted on only when armed. This line IS the calibration dataset.
    rospy.loginfo_throttle(
        10.0, "[EDM] plateau: pct=%.2f growth=%.2f m2 / %.0f s would_trigger=%s force=%s",
        pct, growth, self.plateau_window_s, would, self.plateau_force)

    if would and self.plateau_force:
        self.plateau_fired = True
        rospy.logwarn("[EDM] coverage plateau (%.2f%%, +%.2f m2 / %.0f s) -- ending "
                      "exploration and returning.", pct, growth, self.plateau_window_s)
        self.transition_to(MissionState.RETURN)
```

### 3.3 Nothing else

No new publisher. See §5.4 — the reviewed document publishes to `/exploration_completed`, which
is wrong and must not be copied.

---

## 4. Rollout

Do not arm on the first run. The trigger ends the mission; a bad threshold ends it early and
that is worse than a slow mission.

**Stage A — observe (`force_enabled: false`).** Fly ≥ 3 runs. Collect the
`[EDM] plateau:` lines and confirm `would_trigger` first goes true at ≥ 98 % coverage and
50–150 s before the run actually ended. Cross-check each against `summary.json`
`coverage.final_pct_observable`.

**Stage B — arm (`force_enabled: true`).** Fly ≥ 3 runs. Accept only if all of:

- `exploration_completed` true and `flight_terminated` false on every run
- `coverage.final_pct_observable` ≥ 98.0 % on every run
- landing within 1.0 m of the pad, **verified against ulog ground truth**, not
  `/mavros/local_position/pose`
- airborne time reduced by ≥ 45 s against the 543 / 457 / 574 s baseline

**Stage C — tighten.** Only after Stage B passes, try `plateau_window_s: 45.0` then `30.0`,
re-running Stage B acceptance at each. Expected additional saving 30–70 s. Stop at the first
value that drops any run below 98 % final coverage.

**Stage D — `finish_recheck_max` 15 → 8** (`algorithm.xml`, ~14 s). Redundant belt-and-braces
once the plateau trigger is trusted. This one is a FUEL launch param, so it is config-only but
should still be flown separately so its effect is attributable.

### Rollback

Set `force_enabled: false`. The observe-only path has no effect on the mission, so rollback is a
config edit and a restart — no rebuild, no revert.

---

## 5. Why the other three proposals were rejected

### 5.1 Wall-standoff margin (§1 of the reviewed doc) — right idea, wrong implementation

The diagnosis is correct and the claim that it is independent of A* traversal is correct:
`findViewpoints` and `sampleViewpoints` gate viewpoint *placement*, not the A* graph, so
connectivity for flight is genuinely unaffected. Four problems with the implementation:

**(a) It re-introduces the exact quantisation cliff it claims to avoid.** The probe loop uses
`round(margin / resolution)` integer voxels. `0.15 / 0.1 = 1.4999999999999998` in floating point,
which rounds to **1** — so the recommended 0.15 m silently becomes 0.10 m. 0.20 gives 2. The
margin is only settable in 0.10 m steps, the same cliff that locks `obstacles_inflation`. The
document's central argument — that this margin is free of the severance risk because it is
"query-time" — does not follow from being query-time; it follows from being viewpoint-only, and
the quantisation problem is untouched.

**(b) Measured, the recommended value is too aggressive.** Sweeping the final map of the two most
recent successful runs at the viewpoint altitude (z = 1.39, viewpoints are clamped to 1.35–1.45):

| extra margin | probe vox | admissible area | vs current | components | largest island |
|---|---|---|---|---|---|
| 0.00 (current) | 0 | 79.0 m² | 100 % | 2 | 78.9 m² |
| 0.10 / 0.15 | 1 | 53.8 m² | **68 %** | **14** | **24.0 m²** |
| 0.20 / 0.25 | 2 | 35.2 m² | 45 % | 26 | 8.9 m² |
| 0.30 | 3 | 22.3 m² | 28 % | 22 | 6.4 m² |

(run 20260908_143514 gives the same shape: 83.5 → 58.2 → 39.0 → 26.0 m².)

At the recommended setting a third of viewpoint space disappears and what remains fragments from
one region into 14 islands. Frontiers whose only admissible viewpoints fall in the removed third
become unclearable — they get retired, which is the failure this change is meant to prevent.

**(c) One of the three named call sites is not an admissibility check.** Line ~694 is the
**information-gain raycast break condition** inside `findViewpoints`. Applying a margin there makes
every gain ray stop short of walls, systematically under-counting the unknown cells visible near
walls, biasing viewpoint selection *away* from wall-adjacent frontiers — the opposite of the
intent, and a likely coverage regression. Only lines 669 and 757 are genuine admissibility tests.

**(d) Two code defects.** `static const int margin_vox` inside a member function is computed once
on first call and never again, making the parameter immutable and shared across instances. And the
helper takes `Vector3i` while two of the three call sites pass `Vector3d`.

**The fix, for Phase 2:** use the continuous ESDF (`SDFMap::getDistance()`), which is already
computed by `updateESDF3d()` and already used this way by `astar.cpp`. That gives a continuous
margin with no quantisation cliff, at two call sites only (669, 757). Caveat to handle: the ESDF
is refreshed only over `local_bound_`, so values outside the recently-updated region are stale and
need a validity test. Sweep offline before flying; on this evidence start below 0.10 m effective,
not at 0.15.

### 5.2 Retirement escalation with retry memory (§2) — cannot work as specified

`resetFlag` (`frontier_finder.cpp:67-74`) **erases** the `Frontier` from the list and clears
`frontier_flag_` for its cells. Those cells are then re-seeded by `expandFrontier` into a
brand-new `Frontier` object. Every field on the old object, including a proposed `retire_count_`,
is destroyed on revival.

The lifecycle is therefore: retired (count = 1) → dormant → cooldown expires → revived by
`resetFlag`, object destroyed → re-detected as a new object (count = 0) → retired again (count = 1).
**The counter can never exceed 1.** The 60 → 120 → 240 → 480 s ladder and the
`retire_count_ >= 4` blacklist are both unreachable.

To work, the counter must be keyed to something that survives revival — e.g. a voxel-hashed
frontier centroid in a map owned by `FrontierFinder`, not a field on `Frontier`.

Two further gaps: `current_coverage_pct_` does not exist in `FrontierFinder` and would need
plumbing from `MapROS`; and while `dormant_frontiers_.erase()` is safe with respect to the
cost-matrix invariant (only `frontiers_` feeds `removed_ids_`), erasing without clearing
`frontier_flag_` permanently poisons those cells so they can never be re-detected — coverage can
then never reach 100 %. That may be the intent of a blacklist, but it must be deliberate.

### 5.3 Config-only stopgap (§3) — harmful before Phase 0

`retire_cooldown` 60 → 90 makes the 2026-09-09 failure worse. In run 20260908_160711 A* failed
because the *start* was blocked, so it failed against every target at once and the machinery
retired good frontiers one after another; `final_visitable` reached 0 and
`exploration_completed: True` was published at **95.3 %**. A longer cooldown keeps those frontiers
dormant longer and reaches the false completion *sooner*.

`target_switch_margin` 3.0 → 4.0 is neutral-to-harmful for the same reason: it holds an
unreachable target longer.

The thrash this targets is also already improving without it —
`targets.max_episodes_one_location` across the three successes runs 28 → 23 → **12**.

Revisit after Phase 0 (blocked-start / unreachable-goal separation) lands, not before.

### 5.4 One defect carried into the accepted item

The document's callback publishes `Bool(True)` on `/exploration_completed`. That topic is already
published **latched** by the FUEL FSM (`fast_exploration_fsm.cpp:60`). Two latched publishers on
one topic leaves subscribers with whichever published last, and a later FSM publish can contradict
EDM. Since EDM already owns the RETURN transition, the trigger should stay internal and publish
nothing. §3.2 above does this.

Its `coverage_cb` also appends to history before checking mission state, so a window built during
TAKEOFF/ENTRY can satisfy the plateau test on the first exploration sample. §3.2 clears the deque
outside EXPLORATION instead.

---

## 6. Measurement

Per run, from the existing flightlog summary — no new tooling:

- `exploration_completed`, `flight_terminated` — a false completion is a failed run, not a fast one
- `coverage.final_pct_observable` — hard floor 98.0 %
- `airborne_s` — baseline 543 / 457 / 574 s
- `localisation.max_m` — leading indicator for both known failure modes
- `frontiers.max_dormant` — a spike means retirement is eating good frontiers
- `No path to next viewpoint` count from `raw/fuel.log.gz` — 124 / 237 / 936 on the successes,
  1167–7942 on the failures. Cleanest discriminator found so far.
- final landing position **from ulog ground truth**, never from `/mavros/local_position/pose`
#In the terminal make sure to publish the the percentage of area covered also .