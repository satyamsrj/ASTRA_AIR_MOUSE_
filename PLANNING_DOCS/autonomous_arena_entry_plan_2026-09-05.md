# Autonomous arena entry: what's actually broken and how to fix it

**Date:** 2026-09-05

---

## 1. What's actually wrong with `entry_detection_module.py`

I re-read the code line by line. Here's what it does vs. what it needs to do:

### What it actually does (the "forward-only" problem)

```
ENTRY_SEARCH state (line 501-530):
  → fly forward at 0.35 m/s (fixed heading, body-frame x-axis)
  → look for gap in a NARROW FORWARD CONE:
      x_body in [0.3, 4.0] m ahead
      y_body in [-2.5, 2.5] m lateral
  → need obstacles on BOTH sides (left_pts > 3 AND right_pts > 3)
  → gap must be 0.5 – 1.2 m wide
  → "crossing" declared when dot_product < -0.10 (line 214)
  → max 45 seconds, then force-transition regardless
```

### Why it doesn't work for arena entry

| Problem | Code location | Consequence |
|---|---|---|
| **No search pattern** | Line 504: `v_fwd = self.search_forward_speed` (constant) | Drone flies in a straight line. If the arena entry isn't directly ahead at takeoff, it never finds it |
| **Forward-only detection** | Line 178: `fwd_mask = (x_band >= 0.3) & (x_band <= 4.0)` | Only checks a 3.7m-deep, 5m-wide forward box. The VLP-16 gives you 360° × 100m — you're using <2% of the data |
| **Requires obstacles on BOTH sides** | Line 187: `if len(left_pts) > 3 and len(right_pts) > 3` | If the opening is wider than 5m (like your test arena's entire south face), there are no obstacle points on both sides within the forward window → gap never detected |
| **Width clamped to 1.2m** | Line 56/192: `door_max_width = 1.2` | Arena entry could be 1–15m wide. Any opening >1.2m is rejected |
| **Crossing threshold at 0.10m** | Line 214: `dot_prod < -0.10` | Essentially requires the gate to be behind the drone before declaring "crossed" — but this is the only trigger for FUEL handoff |
| **No wall detection** | Nowhere | The drone doesn't first detect the arena as a structure, so it has no bearing to approach |
| **No yaw control** | Line 521: `yaw_rate = 0.0` always | The drone never turns to face the opening — it just flies forward and steers laterally |
| **Baseline captured on first scan** | Line 226: `baseline_free_space = max(1.0, self.current_free_space)` | If first scan is in open air (outdoor pad), the "free space expansion" cue is permanently miscalibrated |

**In summary:** the EDM assumes the drone starts facing a narrow door at close range and flies
straight through it. It's a corridor-traversal detector bolted onto an arena-entry FSM. The
entire perception and control strategy inside ENTRY_SEARCH needs to be replaced.

---

## 2. The proven algorithm: range-discontinuity gap detection

From Schleich et al., "Remote Autonomy for Multiple Small Lowcost UAVs in GNSS-denied Search
and Rescue Operations" (IEEE SSRR 2025, Galway) — Section III-B.3, validated on real hardware
(DJI Mini 3 Pro/Mini 4 Pro):

**Core idea:** An opening in a wall shows up in a horizontal range scan as a **discontinuity** —
a sudden jump from a short distance (wall surface) to a long distance (empty space behind the
wall, or opposite wall). You don't look for "left edge and right edge with a gap between them" —
you look for **jumps in adjacent range readings**.

Their algorithm:
1. Take the horizontal obstacle scan (their DJI sensor provides 360 samples; your VLP-16
   provides 1800)
2. Search for jumps in adjacent distance measurements — a jump above a threshold (e.g., 1.5m
   between consecutive angular samples) indicates a wall ending or beginning
3. Pair adjacent jumps: one jump marks "wall ends" (range goes from short → long), the next
   marks "wall begins" (range goes from long → short) — the angular span between them is the
   opening
4. Filter candidates: check that the door's surface normal aligns with the radial direction
   (removes false positives from occlusion edges), verify sufficient width, verify clearance
   behind the opening
5. Select the best candidate (normal most aligned with current heading)
6. Compute a pre-traversal pose: midpoint of the opening, offset along the outward normal
7. Fly to pre-traversal pose (continuously updating from latest scan)
8. Once stable, fix the opening position and fly through along the inward normal

**Why this works for your arena:**
- Uses the full 360° scan, not a forward cone
- Detects openings of ANY width (not clamped to 1.2m)
- Works whether or not the drone is facing the opening — it finds it anywhere in the scan
- Doesn't require obstacles on both sides within a narrow window
- Proven on real UAV hardware in GNSS-denied environments

---

## 3. Adaptation for your stack (VLP-16 + FAST-LIO + ROS Noetic)

### 3.1 Extract a 2D horizontal range scan from VLP-16

The VLP-16 gives a 3D point cloud, not a 2D scan. But a 2D range ring at cruise altitude is
easy to extract — in fact, the existing EDM already does the height-band filtering (line 157-160):

```python
# Filter point cloud to a height band around cruise altitude
band_mask = (np.abs(z_body) <= 0.8)
```

Then convert the band to polar coordinates (angle, range) relative to the drone:

```python
angles = np.arctan2(y_band, x_band)          # -π to +π, full 360°
ranges = np.hypot(x_band, y_band)             # range in meters

# Bin into 360 angular sectors (1° resolution) — take minimum range per bin
num_bins = 360
bin_indices = ((angles + np.pi) / (2 * np.pi) * num_bins).astype(int) % num_bins
range_scan = np.full(num_bins, np.inf)
for i in range(len(ranges)):
    b = bin_indices[i]
    if ranges[i] < range_scan[b]:
        range_scan[b] = ranges[i]
```

This gives you a 360-element array of minimum ranges — equivalent to a 2D LiDAR scan at cruise
altitude. The VLP-16's 1800 horizontal samples are more than enough to populate 360 bins.

### 3.2 Gap detection: find range discontinuities

```python
JUMP_THRESHOLD = 1.5   # meters — a range jump bigger than this = wall edge
MIN_GAP_WIDTH = 0.8    # meters — competition corridors are ≥ 1m
MAX_GAP_WIDTH = 12.0   # meters — wider than this is probably open space, not an entry
MIN_WALL_RANGE = 0.5   # ignore anything closer than this (ground clutter)
MAX_WALL_RANGE = 15.0  # ignore returns farther than this (irrelevant)

gaps = []
for i in range(num_bins):
    r_curr = range_scan[i]
    r_next = range_scan[(i + 1) % num_bins]

    if r_curr < MAX_WALL_RANGE and r_next - r_curr > JUMP_THRESHOLD:
        # Wall ends here — range jumps from short to long
        # Mark as "opening start" at angle i
        gap_start_angle = (i + 0.5) * (2 * np.pi / num_bins) - np.pi

    elif r_curr - r_next > JUMP_THRESHOLD and r_next < MAX_WALL_RANGE:
        # Wall begins here — range jumps from long to short
        # Mark as "opening end" at angle i
        gap_end_angle = (i + 0.5) * (2 * np.pi / num_bins) - np.pi
```

Pair start/end edges → compute gap center angle, width (from geometry), and inward normal.

### 3.3 Arena detection and approach (the missing PHASE)

Before gap detection can work, the drone needs to be **close enough to the arena to see its walls**
in the range scan. At 10+ meters, the arena is a distant cluster of returns. At 2–3m, individual
walls are clear with sharp discontinuities.

**New ENTRY_SEARCH sub-states:**

```
ENTRY_SEARCH:
  ├─ SCAN_ROTATE        # Hover + rotate 360° to survey surroundings
  │   → builds a panoramic range image
  │   → identifies the dominant structure (arena) as the largest contiguous
  │     block of short-range returns
  │   → determines bearing to the structure's nearest face
  │
  ├─ APPROACH            # Fly toward the detected structure
  │   → heading locked to the structure bearing
  │   → slow down as nearest range decreases (e.g., 0.5 m/s at 5m, 0.2 m/s at 2m)
  │   → transition when nearest wall is within 3m
  │
  ├─ GAP_SEARCH          # Wall-following + gap detection
  │   → maintain 2m offset from nearest wall
  │   → follow wall contour (proportional heading control to keep wall at constant range)
  │   → run gap detection (§3.2) on every scan
  │   → if gap found: compute pre-traversal pose
  │   → timeout: if no gap after one full circuit → error/abort
  │
  └─ TRAVERSE            # Fly through the detected opening
      → align heading with inward normal of detected gap
      → fly to gap center
      → fly through (body-forward, lateral obstacle avoidance active)
      → once through (range-behind > range-ahead), transition to ENTRY_CONFIRMATION
```

### 3.4 Why SCAN_ROTATE matters

The drone spawns on a pad somewhere outside the arena. It doesn't know which direction the arena
is. But after takeoff and a 360° yaw rotation (5–10 seconds at 36°/s), the VLP-16 has seen
everything in range. The arena — a 15×15m walled structure — will be the **only** large obstacle
cluster in an otherwise open area. Identifying it is trivial: find the angular span with the
most sub-10m returns.

If the pad happens to be right at the entry (which the rules suggest: "The drone must begin the
mission from the designated entry point"), then SCAN_ROTATE might immediately see the gap, and
APPROACH + GAP_SEARCH are skipped entirely — the system degrades gracefully.

### 3.5 Wall-following during GAP_SEARCH

Classic proportional wall-following using the 2D range scan:

```python
# Maintain desired_wall_dist from nearest wall
nearest_range, nearest_angle = min((r, a) for a, r in enumerate(range_scan) if r < MAX_WALL_RANGE)

# Error: positive = too close, negative = too far
wall_error = desired_wall_dist - nearest_range

# Heading: perpendicular to the wall, with the wall on one side (e.g., left)
# Velocity: forward along the wall, lateral correction proportional to wall_error
v_fwd = wall_follow_speed  # 0.3 m/s
v_lat = -Kp * wall_error   # push away if too close, pull in if too far
yaw_rate = heading correction to keep wall at ~90° from heading
```

This follows the wall contour. The gap detection (§3.2) runs on every scan. The moment a gap is
found, wall-following stops and the drone transitions to TRAVERSE.

---

## 4. What doesn't change

- **FAST-LIO** — continues running throughout, providing odometry. The registered point cloud
  (`/cloud_registered`) is still the input for gap detection (same as current EDM).
- **FUEL** — still handles interior exploration after entry. The handoff trigger
  (`/waypoint_generator/waypoints`) is the same.
- **flight_envelope_guard.py** — still runs as a safety net, but with the MISSION envelope
  active during entry.
- **The FSM structure** — TAKEOFF → ENTRY_SEARCH → ENTRY_CONFIRMATION → EXPLORATION →
  RETURN → LAND is preserved. Only the internals of ENTRY_SEARCH change.

---

## 5. What about the camera during entry?

The camera runs in parallel from takeoff (mandatory per rules — live GCS feed). During entry
it is streaming video to the GCS and running the survivor detector. It does NOT drive entry
navigation — the VLP-16 handles that because:

- LiDAR gives precise range regardless of lighting (the arena is covered, possibly dark inside)
- 360° coverage vs. camera's 93° FOV
- Range discontinuities are a clean geometric signal — no training data, no model, no inference
  latency

The camera's role becomes primary during EXPLORE for survivor detection.

---

## 6. Build order

| Order | Task | Depends on | Sign-off needed? |
|---|---|---|---|
| 1 | Phase 0: collision-retreat fix | — | Yes (FUEL source) |
| 2 | Camera integration: `iris_vlp16_cam` model | — (parallelizable) | No |
| 3 | Rewrite ENTRY_SEARCH: SCAN_ROTATE → APPROACH → GAP_SEARCH → TRAVERSE | Phase 0 (so the drone survives after entry) | No (Python script, no FUEL source) |
| 4 | Phase-aware envelope guard | — | No |
| 5 | Survivor detection pipeline (YOLOv8-nano) | Camera integration | No |
| 6 | TRANSIT_OUT (reverse breadcrumb path) + termination | ENTRY_SEARCH rewrite | No |
| 7 | Landing accuracy | TRANSIT_OUT | No |

---

## 7. Reference implementations

| Source | What it does | Relevance |
|---|---|---|
| Schleich et al., IEEE SSRR 2025 (arxiv 2510.21357) §III-B.3 | Door traversal from horizontal obstacle scan: discontinuity detection, normal alignment filtering, pre/post-traversal poses | **Direct algorithm template** for GAP_SEARCH + TRAVERSE |
| US Patent 9,251,417 ("Fast open doorway detection for autonomous robot exploration") | RANSAC vertical plane extraction → horizontal stripe scan for gaps within doorway dimension range → clearance verification | Alternative approach using 3D plane extraction first |
| Quintana et al., IPIN 2016 ("Door Detection in 3D Colored Laser Scans") | Identifies gaps in detected wall planes as open doors | Confirms the "gap in wall plane" paradigm |
| `entry_detection_module.py` lines 125-280 (existing) | Height-band filtering, body-frame transform, opening width measurement | Reusable: the coordinate transforms and cloud-to-body conversion are correct; the detection logic inside that frame needs replacement |

---

## 8. What I need from you to proceed

1. **Team sign-off on Phase 0** — the collision-retreat fix. Without this, the drone will stall
   inside the arena after entry, making entry testing impossible to complete end-to-end.
2. **Push local changes** — `blind: 0.5` and anything else not in the repo. I'm working against
   `blind: 2` which changes the effective LiDAR range for gap detection.
3. **Telemetry from a stuck run** — upload the CSV or `.ulg` so I can confirm the oscillation
   pattern before and after fixing.
4. **Confirmation of pad-to-arena geometry** — how far is the launch pad from the arena wall in
   your test setup? This sizes the APPROACH phase parameters.
5. **SIYI A8 mini CAD (STL)** — for the camera integration (visual mesh only; sensor behavior
   comes from Gazebo plugins).
