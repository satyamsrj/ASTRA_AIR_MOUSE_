# Stall root cause and movement optimisation — implementation plan

Date: 2026-09-09. Evidence: `logs/runs/20260909_090542` (good, 99.5 %),
`20260909_101738` (585 s, handshake missing), `20260909_111512` (407 s, 92.7 %, the run
analysed below). All numbers measured from the bundles, not estimated.

---

## 1. What the stall actually is

Not wall-hugging. Not box-edge frontiers. Not localisation. Not A* failure.

**FUEL orders and commits to frontier targets by travel cost alone. There is no
information-gain term anywhere in the selection.**

### 1.1 The measurement

Run `20260909_111512`, longest stall t=223–317 s (94 s). Unknown-space distribution at the
moment the stall began, from the map snapshots:

| at t=220 s | unknown columns |
|---|---|
| total | **3184** |
| in cx 6.5–9.0 (mid-arena blob) | **1188** |
| beyond cx 15.5 (far end) | **17** |

Targets the planner actually held during t=223–317:

```
(16.1, -1.9) x32   (16.1, 6.4) x19   (16.1, 6.0) x18
(16.3, -1.4) x16   (16.0, 6.6)  x7   (12.3, 1.8)  x9
```

92 of 101 target samples were beyond cx 16.0. **The vehicle flew to the far end of the arena
to chase 17 unknown cells while 1188 sat in the middle.**

### 1.2 Why the code does this

- `ViewNode::computeCost()` returns `max(path_len / v_max, yaw_diff / yaw_rate)` — seconds of
  travel. Nothing else.
- `FrontierFinder::getFullCostMatrix()` fills the ATSP matrix purely from those costs
  (`ftr.costs_`, and row 0 from `computeCost`). A frontier worth 17 cells and one worth 1188
  are indistinguishable to the tour.
- `refineLocalTour()` then picks among the top-N viewpoints, again by `computeCost`.
- The target hysteresis (`exploration/target_switch_margin` = 3.0 s) then **defends** whatever
  was chosen, so a bad pick is held rather than corrected.
- The only corrective is `exploration/global_stale_seconds` = 40.0. It works — it fired three
  times in this run, the first retiring exactly `(16.13, 6.28, 1.19)` — but the system's
  fastest possible recovery from a worthless target is **40 seconds**, and it recurs.

Three watchdog firings x 40 s accounts for the bulk of a 94 s stall almost exactly.

### 1.3 What is NOT the cause (checked and excluded)

| Hypothesis | Verdict |
|---|---|
| Localisation drift | **No.** Residual through the stall: median 0.107–0.283 m, max 0.338 m. The 6.4 m excursion is at t=420, after the fact. |
| A* / no-path failures | **No.** 3608 of 3612 no-path events occur at t=440+, during terminal divergence. **Zero** during the stall. |
| Box-edge frontiers being permanently unclearable | **No.** `getOccupancy` returns −1 (not UNKNOWN) outside the map, and the map region is larger than the planning box, so box-edge cells do clear. Only 7.9 % of unknown columns were within 0.3 m of a box edge at t=220. |
| Wall proximity (the Phase 2 standoff target) | **Not the driver.** The standoff works as designed — 71.7 rejections/cycle, pass 0 succeeding 96 of 111 times — but it measures obstacle clearance, which is not what selects these targets. |
| exploration_node segfault | **Fixed and verified.** Zero stack traces in this run. |

---

## 2. Changes

### 2.1 Gain-weighted ATSP cost (the fix for the stall)

`FrontierFinder::getFullCostMatrix()` — after the matrix is filled, scale each destination
column by a weight derived from that frontier's cell count:

```cpp
w(ftr) = 1.0 / (1.0 + gain_weight * min(1, cells / gain_ref_cells))
```

Column `j` (all rows, including row 0 = "from the vehicle") is multiplied by `w[j-1]`. Column 0
is untouched — it is the free return-to-depot column and `leftCols<1>().setZero()` owns it.

With `gain_weight = 0.5`, `gain_ref_cells = 200`:

| frontier size | weight | effect |
|---|---|---|
| 17 cells | 0.958 | ~unchanged |
| 200+ cells | 0.667 | 33 % cheaper to reach |

So the 1188-cell blob outranks the 17-cell far-end frontier by a wide margin, while the tour
remains a valid ATSP (already asymmetric; this is a bounded monotone reweighting).

**Safety:** `gain_weight = 0` reproduces the current matrix exactly, element for element. It is
a launch parameter, so reverting is one line and needs no rebuild. This mirrors the two-pass
pattern used for the viewpoint standoff.

### 2.2 Faster recovery from a bad target

`exploration/global_stale_seconds` 40.0 → 25.0.

Recovery from a worthless target drops from 40 s to 25 s. Safe because the clock resets every
time a cluster is covered, and in this run `Replan: cluster covered` fired **231 times over
407 s** — about once every 1.8 s during productive flight. 25 s is far outside that.

### 2.3 Not changed, deliberately

- `obstacles_inflation` stays 0.40. `inf_step = ceil(inflation/resolution)` quantises 0.45 to
  5 cells = 0.50 m, which severs this arena to 0.8 m² of free space.
- `target_switch_margin` stays 3.0. Lowering it re-opens the target-thrash the hysteresis was
  added to stop (12 of 12 consecutive 5 s samples changing target, 99° of yaw per change).
- `cluster_min` stays 30. Raising it to drop small frontiers would leave their cells unmapped
  and push final coverage below the 97 % plateau floor.
- The viewpoint standoff stays at 0.25 — it is working and is not implicated.

---

## 3. Verification

Pre-flight: `scripts/verify_fix_parity.sh` must print `PARITY OK`. `test_takeoff.sh` refuses to
launch otherwise.

Acceptance, against run `20260909_111512` (407 s, 92.7 %, 94 s stall):

1. Longest coverage stall **< 60 s**.
2. No target held beyond cx 15.5 while >200 unknown columns sit in cx 6.5–9.0.
3. `no frontier cluster covered for` fires **≤ 1** time.
4. Final coverage **≥ 98 %** (must not regress: the gain weighting must not abandon small
   frontiers, only defer them).
5. Zero stack traces; no `invalid setpoints` / `blind land`.
6. Touchdown within 1.0 m of the pad.

Rollback if any of 3–6 fails: set `exploration/gain_weight` to 0.0.
