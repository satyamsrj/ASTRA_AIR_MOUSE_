# Replacing barometer height with a downward-facing TFmini rangefinder

## The problem being solved

Barometer-based altitude (`EKF2_HGT_REF = BARO` or baro used as the cross-check per
[[height_hold_baro_fastlio_analysis_2026-08-17]]) is noisy indoors — pressure fluctuates with HVAC drafts, prop
wash recirculation near walls/floor, and doors opening/closing in an enclosed arena, so the drone's estimated Z
oscillates even when it is physically level. A downward-facing TFmini (TFmini-S / TFmini Plus, Benewake) gives a
direct, low-noise distance-to-ground reading (UART, ~0.1m resolution, 12m/30m range depending on variant) that
PX4's EKF2 can fuse as the primary height source instead of baro.

This is a hardware + PX4 parameter change, separate from the FAST-LIO/vision height path already relayed via
`relay_odometry.py` → `/mavros/vision_pose/pose`. TFmini height and vision height are two independent inputs
into the same onboard EKF2 — this doc only covers wiring in the TFmini and switching EKF2's height reference.

## Repo status check (2026-08-23)

- No committed PX4 `.params`/airframe file exists in this repo — parameters referenced in
  [[height_hold_baro_fastlio_analysis_2026-08-17]] (`EKF2_BARO_CTRL`) were set live via `mavparam`, not from a
  tracked file. Same pattern applies here: parameters below are applied via `mavparam set` or QGroundControl,
  matching `scripts/test_takeoff.sh:133`'s existing `MIS_TAKEOFF_ALT` pattern.
- The upstream PX4 TFmini driver already exists, unmodified, at
  `simulation/PX4-Autopilot-v1.14.3/src/drivers/distance_sensor/tfmini/` — no new driver code is needed, only
  enabling it and setting params.
- No mavros distance-sensor YAML or serial-port config exists in this repo yet.

## 1. Hardware: mounting and wiring

- **Mount**: TFmini facing straight down, lens/window unobstructed, away from prop wash turbulence (avoid
  mounting directly under a motor). Keep it rigid — vibration coupling into the reading shows up as range noise.
- **Interface**: TFmini-S supports both UART and I2C; **use UART** — it's the mode the PX4 driver
  (`drivers/distance_sensor/tfmini`) targets natively (`tfmini start -d <device>`).
- **Wiring** (TFmini-S 5-pin: RED=5V, BLACK=GND, WHITE=TX, GREEN=RX, BLUE=unused in UART mode):
  - TFmini RED → 5V (flight controller peripheral 5V rail or dedicated BEC — TFmini draws ~150mA, don't share
    a low-current 3.3V rail)
  - TFmini BLACK → GND (common ground with flight controller)
  - TFmini WHITE (TX) → flight controller UART **RX**
  - TFmini GREEN (RX) → flight controller UART **TX**
- Use a free UART (e.g. `TELEM2`/`UART4` depending on flight controller — whichever port isn't already claimed by
  MAVLink telemetry or GPS). Confirm the physical port-to-`/dev/ttyS*` mapping for your specific flight
  controller board before setting `SENS_TFMINI_CFG`.

## 2. PX4 parameters

Set these via QGroundControl → Parameters, or `mavparam set <PARAM> <VALUE>` (same tool used for
`MIS_TAKEOFF_ALT` in `scripts/test_takeoff.sh`):

| Parameter | Value | Purpose |
|---|---|---|
| `SENS_TFMINI_CFG` | `<UART port>` (e.g. `TELEM2`) | Tells PX4 which serial port the TFmini is wired to; enables the driver at boot |
| `EKF2_RNG_CTRL` | `1` (enabled) or `2` (enabled, terrain hold below `EKF2_MIN_RNG`) | Turns on rangefinder fusion in EKF2 |
| `EKF2_HGT_REF` | `2` (Range sensor) | Makes the rangefinder the **primary** height reference instead of baro (`0`) or GPS (`1`) |
| `EKF2_RNG_AID` | `1` | Allows range data to aid height estimate even outside terrain-following mode |
| `EKF2_MIN_RNG` | `0.1` (TFmini min range, meters) | Reject readings below sensor's valid range |
| `EKF2_RNG_A_VMAX` / `EKF2_RNG_A_HMAX` | leave default unless high-speed flight | Caps horizontal/vertical velocity where range aiding is trusted |
| `EKF2_BARO_CTRL` | `1` (keep enabled, don't disable) | Retain baro as a cross-check/fallback, same rationale already documented in [[height_hold_baro_fastlio_analysis_2026-08-17]] — don't remove redundancy, just stop it being primary |

**Important distinction carried over from [[height_hold_baro_fastlio_analysis_2026-08-17]]**: this changes height
*estimation* accuracy/noise only. It does not touch commanded altitude — FUEL's frontier Z-sampling
(`frontier/min_candidate_z` / `max_candidate_z` in `exploration_manager/launch/algorithm.xml`) still governs
where the drone is told to fly. If oscillation persists after this change, re-check the planner-side band before
assuming it's a sensing problem again.

## 3. Verification plan

1. After wiring, confirm the driver starts: `tfmini start -d /dev/ttyS<N>` in the PX4 shell (or check boot log
   for `tfmini` init success) — should report valid range values, no `WARN: no readings`.
2. In QGroundControl → MAVLink Inspector, confirm `DISTANCE_SENSOR` messages are streaming with sane values
   (matches known height above ground when hand-held).
3. Set the params above, reboot the flight controller, and check EKF2 status (`ekf2 status` or QGC EKF status
   page) — height source should report range fusion active, no `EKF2_RNG` innovation warnings.
4. Bench/tether test: hold the drone at a few known heights (e.g. 0.5m, 1.0m, 1.5m) and confirm estimated Z
   matches within TFmini's rated accuracy (~±5cm typical) and is materially less noisy than the current
   baro-only trace.
5. Live indoor flight test at the arena: repeat the ~1.5m hold test referenced in
   [[height_hold_baro_fastlio_analysis_2026-08-17]] and confirm Z trace noise floor drops relative to the
   0.9–1.76m baro-driven swing recorded there. Any remaining drift/oscillation at that point is planner-side, not
   sensor-side, per the note above.

## Status

Not yet implemented — this is the plan to review before wiring anything. Confirm UART port choice against the
actual flight controller pinout in use before proceeding, since it isn't committed anywhere in this repo.
