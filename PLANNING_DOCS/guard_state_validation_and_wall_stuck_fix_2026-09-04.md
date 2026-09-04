# Why the drone got stuck at the arena walls, and the guard state-validation fix

**Date:** 2026-09-04
**Trigger:** user-reported "drone gets stuck at the walls of the arena," cross-checked against
the external audit `nidar_phase_plan_to_mission_complete.md` (commit `852482e5`).
**Status:** root-caused, fixed, verified live (two flights, 333 s and 400 s, zero regressions).

---

## 1. Two independent bugs, both in `scripts/flight_envelope_guard.py`

### 1.1 FUEL was allowed to plan into territory the guard always refuses

`launch/nidar_fuel_upstream.launch`'s exploration box was `[-0.5, 13.5] x [-7.0, 7.0]` in
camera_init. Converting through `camera_to_world` (`xw=-yc, yw=xc-6.5`), that is world
`[-7.0, 7.0] x [-7.0, 7.0]` — the raw arena size. But the guard's *effective* envelope is the arena
size **minus** `boundary_margin` (0.2 m): world `[-6.8, 6.8] x [-6.8, 6.8]`
(`config/flight_envelope_guard.yaml`).

FUEL therefore kept generating frontiers in the 0.2 m ring the guard would always reject. Every
approach to a wall ended in a rejected command.

**Fix:** tightened the box to `[-0.3, 13.3] x [-6.8, 6.8]` (the exact camera_init pre-image of the
guard's effective world envelope) in `launch/nidar_fuel_upstream.launch`.

### 1.2 The fallback-hold path double-rotated the vehicle's own pose

Three places in the guard converted `/mavros/local_position/pose` into camera_init before calling
`camera_to_world()`, using a fixed 90-degree pre-rotation (`xc = pose.y, yc = -pose.x`) on the
assumption that MAVROS reports standard ENU.

That assumption is wrong **in this stack specifically**. `scripts/relay_odometry.py` copies
FAST-LIO's camera_init pose into `/mavros/vision_pose/pose` **unmodified**
(`pose_msg.pose = msg.pose.pose`), and PX4's EKF2 fuses that as its position source
(`EKF2_EV_CTRL`). So `/mavros/local_position/pose` ends up expressed in the same frame as
camera_init already — confirmed empirically, repeatedly, this session:

- Same instant, run 6: `/Fast_LIO/odometry (6.242, 1.702)` vs `/mavros/local_position (6.248, 1.719)`
- Logged FASTLIO/EKF divergence stayed under 0.1 m for entire flights (would be impossible with an
  unaccounted 90-degree rotation between the two topics being differenced)

Pre-rotating and then calling `camera_to_world()` rotated the pose **twice**. Worked example, real
numbers from run 6 (vehicle actually at camera_init `(13.38, -0.62)`):

| | world computed | fallback-hold commanded camera_init | error |
|---|---|---|---|
| double-rotated (before) | `(13.38, -7.12)` | `(-0.30, -6.80)` | **15.0 m jump** |
| single-rotated (after) | `(0.62, 6.88)` | `(13.30, -0.62)` | 0.08 m (correct hold) |

This is the path that actually commands the vehicle whenever a command is rejected — which, per
§1.1, was constantly happening near every wall. Each rejection near a boundary triggered a large,
wrong jump instead of a graceful hold. That is what looked like the vehicle getting stuck / thrown
at the walls.

**Fix:** consolidated all three sites onto one corrected helper, `_current_pose_world()`, which
passes `/mavros/local_position/pose` straight into `camera_to_world()` with no pre-rotation.

## 2. A third, latent bug found by the external audit — fixed, then had to be partially reverted

`nidar_phase_plan_to_mission_complete.md` (§1.2/A4) identified that `validate_command()`'s state
check read the vehicle's actual pose, checked it against the envelope, and on failure only called
`rospy.logwarn_throttle(...)` before falling through to `return True, "ACCEPT", ...` regardless.
The log message even claimed "guiding to safe interior setpoint" — which nothing downstream
actually did. A vehicle that had already drifted outside the safe volume was never stopped.

**First fix:** made the check actually reject (`False, "FAULT_STATE_OUT_OF_ENVELOPE", ...`) when
the vehicle's real pose is outside `[eff_xw_min, eff_xw_max] x [eff_yw_min, eff_yw_max] x
[eff_zw_min, eff_zw_max]`.

**Live-tested consequence:** in a 410 s flight, this fired **33 times** — and all 33 were false
positives. `eff_zw_min/max` is a 4 cm band (`[1.48, 1.52]`), and normal EKF2 height noise
(measured sd ≈ 0.02-0.03 m) crosses a 4 cm band on ordinary hover jitter alone. X/Y was inside the
envelope in every single one of the 33 events. Unlike the position *command* (which `traj_server`
hard-pins to exactly 1.40 and which the existing `CLAMPED_Z_MIN/MAX` check backstops), the
*measured state* Z has real sensor noise centered on that setpoint — gating a hard fault on it was
always going to cry wolf continuously.

**Final fix:** state validation checks **X/Y only**. Z keeps its existing, separate protections
(traj_server pin + FAST-LIO range gating + command-level clamp) rather than gaining a new,
noise-prone one.

## 2a. A third bug — and the one actually responsible: arena wall friction 100x too high

After §1-2 landed and verified clean (0 guard events, 333 s), the user reported the drone was
**still** getting stuck at the walls. Re-investigated from scratch rather than assuming the guard
fixes were sufficient, since "stuck at a wall" is also consistent with a pure physics problem the
guard would never see (the guard only judges commanded/measured *position*, not contact forces).

`simulation/custom_models/nidar_arena/model.sdf` (the live arena, confirmed via
`nidar_competition.world`'s `model://nidar_arena` reference) set the wall collision surface to
`<mu>100</mu><mu2>50</mu2>`. Real materials run roughly 0.3-1.5; ODE/SDF's own unset default is
1.0, which is what the vehicle's own collision surfaces use. At mu=100, any contact between the
vehicle's collision box and a wall resists tangential sliding almost completely — instead of
deflecting and continuing past on contact (the normal, harmless outcome of a drone brushing a
wall), the vehicle would plant into it, with motor thrust fighting the contact rather than sliding
free. This is a pure Gazebo/ODE physics artifact, entirely independent of §1-2's guard logic, and
would produce zero guard-log evidence since the guard never inspects contact forces.

**Fix:** `mu`/`mu2` set to 1.0/1.0 (ODE's own default, matching the vehicle side), not removed
entirely, so the value stays visible and intentional in the file rather than reverting to
"whatever the engine defaults to".

Caught a real typo in the same edit before it ever reached Gazebo: the first attempt closed
`<mu2>` with `</mu>`, which XML-validated as a mismatched tag. Fixed and re-validated before testing.

## 3. Verification

| run | guard fix state | duration | guard events |
|---|---|---|---|
| 6 | none (baseline, pre-audit) | 152 s (GUI, ended by external kill) | 2 `REJECT [OUT_OF_BOUNDS_Y_*]` at the wall, no graceful recovery visible in the short window |
| 8 | double-rotation fixed + box tightened + state check (X/Y **and** Z) | 410 s | **33 `FAULT_STATE_OUT_OF_ENVELOPE`, all false positives (Z jitter)** |
| 9 | + state check narrowed to X/Y only | 333 s continuous flight (`flight_report.py` on the PX4 ulog) | **0** guard events of any kind |

Run 9 also confirms nothing else regressed: altitude 1.279-1.430 m (arena rule pass), rangefinder
0/3285 out-of-band, motors mean 0.680 / 0.8% saturated.

### 3a. Re-verification after the friction fix (run 10, GUI visible, per user request)

A custom `stuck_detector.py` watched `/mavros/local_position/pose` + `/velocity_local` +
`/planning/pos_cmd` live, flagging any sustained window where the vehicle was near the boundary,
commanded to keep moving, but actual speed stayed under 5 cm/s (the literal signature of "planted
against a wall, motors fighting it").

| window | duration | samples near wall (within 0.6m of 6.8 boundary) | worst stall detected | guard events |
|---|---|---|---|---|
| pass 1 | 180 s | 1/899 | 0.2 s | 0 |
| pass 2 | 300 s | 21/108 (checked at tighter margin) | 0.2 s | 0 |

Over the combined **480 s** flight, the vehicle reached **world Y = 6.70** against the 6.8 m
boundary — 10 cm of clearance, closer than any prior test run — and continued flying and
re-targeting normally (`[CHANGED]` on consecutive telemetry samples, not stalled). Zero guard
events of any kind across the entire flight.

## 4. Files changed

- `scripts/flight_envelope_guard.py` — `_current_pose_world()` fixed and made the single source of
  truth for pose-to-world conversion (was duplicated inline 3x); `validate_command()` block 4 now
  rejects on real X/Y excursions instead of logging and approving anyway.
- `launch/nidar_fuel_upstream.launch` — exploration box tightened to match the guard's effective
  envelope exactly.
- `simulation/custom_models/nidar_arena/model.sdf` — wall collision friction `mu/mu2` 100/50 -> 1.0/1.0.

## 5. What "how does the simulation end" actually means right now

There is no defined mission end. `scripts/mission_telemetry_logger.py` (the last thing
`test_takeoff.sh` runs) loops on `rospy.spin()`-equivalent forever; nothing in the stack declares
"exploration complete" or triggers return-to-home / land / shutdown. A run ends only when
something external kills it: the user's Ctrl+C, a `test_takeoff.sh` re-invocation's cleanup
sweep killing the previous run's leftovers (see the process-hygiene fixes in
`repo_cleanup_and_mind_map_2026-09-03.md` §6.2), or resource exhaustion (this session observed
2.9 GB of swap in use during a GUI run; the container's `dmesg` is not accessible to confirm OOM
directly, but every unattended run so far has ended in a `Killed` log line, not a clean shutdown).
This matches `nidar_phase_plan_to_mission_complete.md`'s Phase 7 finding: there is no mission state
machine (`ARM -> ... -> RETURN -> EXIT -> LAND -> COMPLETE`) anywhere in the tree yet.
