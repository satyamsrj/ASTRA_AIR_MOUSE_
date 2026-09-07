# flightlog — save only the log data that matters, in a format that survives the run

A finished NIDAR run used to leave almost nothing usable behind. `/tmp/fuel.log` is truncated
by the next launch, the occupancy map was published and thrown away, and the only surviving
statement about coverage was a line of console text nobody could recompute. `rosbag record -a`
fixes none of that well: it writes gigabytes, mostly `/cloud_registered`, and still cannot say
what the planner believed.

**Design principle: record what is unrecoverable, derive everything else.**

| source | what only it knows | who captures it |
|---|---|---|
| live ROS topics | the map, FUEL's commanded setpoints, the chosen viewpoint, coverage | `record.py` |
| PX4 `.ulg` | ground truth pose/velocity/attitude/rates, failsafe, battery | already written by PX4; merged by `pack.py` |
| `/tmp/fuel.log` | *why* — FSM transitions, frontier counts, A\* failures | archived + parsed by `pack.py` |

Ground truth is deliberately **not** re-recorded over ROS: PX4 already logs it at full rate,
and a lower-fidelity copy would be both bigger and worse.

## Use

```bash
# Automatic: test_takeoff.sh starts the recorder and packs the bundle when the run ends.
./scripts/test_takeoff.sh true              # NIDAR_FLIGHTLOG=0 to disable

# Manual
python3 tools/flightlog/record.py                     # during the run
python3 tools/flightlog/pack.py --fuel-log /tmp/fuel.log   # after it
python3 tools/flightlog/load.py                       # print the summary
```

Bundles land in `logs/runs/<timestamp>/` (gitignored).

## Format

Chosen for what this box actually has: **numpy, pyulog and yaml only** — no pandas, no pyarrow,
no h5py — so Parquet and HDF5 are not options. Compressed `.npz` is lossless, columnar,
self-describing alongside the manifest, and loads anywhere numpy exists.

```
manifest.json      frames, units, column names, ROS params, git SHA   <- read this first
streams.npz        live time series, one float64 array per stream
map.npz            uint8 voxel grids + origin/resolution/shape
groundtruth.npz    true pose/velocity/attitude/rates in WORLD ENU, + status/failure/battery
planner.npz        frontier diagnostics parsed from fuel.log
events.jsonl       FSM transitions, plan failures, mode changes, PX4 messages — sorted by time
summary.json       headline metrics
raw/               fuel.log.gz + the .ulg, so nothing is unrecoverable
```

Typical size is a few MB plus the archived `.ulg`. The map is stored as a dense voxel grid
(`0=free, 1=occupied, 2=unknown`), not a point cloud: a 141×136×5 grid compresses to ~10 kB per
snapshot, and coverage can be recomputed from it independently of anything FUEL reported.

```python
import numpy as np, json
m = json.load(open('logs/runs/<id>/manifest.json'))
z = np.load('logs/runs/<id>/streams.npz')
cov = z['coverage']                        # columns: m['streams']['coverage']['columns']
free_m2, unknown_m2 = cov[:, 4], cov[:, 6]
```

## Ending the run when the drone crashes

A PX4 SITL crash does not stop the simulation, it **deadlocks** it. PX4 and Gazebo run in
lockstep, so when the FailureDetector terminates the flight the controller stops producing
actuator output, gzserver blocks waiting for it, and both park in `futex_wait`. Measured on
2026-09-07: after `fd_pitch` tripped at t=475 s, the sim clock advanced **476.2 -> 479.4 s over
nine wall minutes** (RTF ~0.005) and would have sat there indefinitely.

The vehicle crashing and the simulation ending are therefore two different events, and
`watchdog.py` is what connects them. It starts automatically from `test_takeoff.sh`.

| detector | fires on | default |
|---|---|---|
| tilt | roll/pitch beyond threshold, held | 60 deg / 0.3 s (PX4 `FD_FAIL_P/R`) |
| statustext | PX4 announcing termination/failsafe on `/mavros/statustext/recv` | — |
| stall | `/clock` advancing less than *n* sim-s per *m* wall-s = lockstep deadlock | 0.5 s per 60 s |
| disarm | armed -> disarmed after having flown | — |

On firing it **flushes and packs the bundle before killing anything** — a recording is only
worth having if it survives the failure it documents — then tears down the whole stack using
the same process list as `test_takeoff.sh`, so no orphans are left to corrupt the next run.

```bash
NIDAR_WATCHDOG=0 ./scripts/test_takeoff.sh true              # disable
NIDAR_WATCHDOG_ARGS="--no-kill" ./scripts/test_takeoff.sh true   # detect + pack, keep sim up
python3 tools/flightlog/watchdog.py --assume-flown            # attach to an already-wedged sim
```

All of its timing is wall clock. Sim time is exactly what stops being trustworthy in the case
it exists to catch, so there is no `rospy.Timer` or `rospy.sleep` anywhere in that file. The
destructive detectors are gated on having actually seen the vehicle fly, so a stack still
starting up — or a Gazebo deliberately paused from the GUI — is never torn down.

## Getting the data after a run ends or crashes

Nothing needs to be done by hand in the normal case. The bundle is written continuously:
`streams.npz`, `map.npz`, `events.jsonl` and `manifest.json` are re-flushed every 30 s of
**wall** time and each is written atomically, so even `kill -9` loses at most 30 s and never
leaves a half-written file.

```bash
python3 tools/flightlog/load.py                    # summary of the newest bundle
ls -t logs/runs/ | head                            # find older ones
cat /tmp/watchdog_reason.txt                       # why the run was ended
```

If the run died in a way that skipped the automatic pack — you killed the terminal, or the
machine went down — recover it manually **before launching anything else**, because the next
launch truncates `/tmp/fuel.log`:

```bash
python3 tools/flightlog/pack.py --fuel-log /tmp/fuel.log
```

`pack.py` is offline and needs no running ROS. It defaults to the newest run directory and the
newest `.ulg`; pass `--run` / `--ulog` to pick a specific pair. It is safe to re-run — it
overwrites its own outputs and does not touch the live capture.

## Five traps this encodes, so nobody re-discovers them

1. **Three frames, never interchangeable.** `world` (Gazebo ENU), `camera_init`
   (`cx = world_y + 9.5`, `cy = -world_x`) and PX4 `NED`. Every column is tagged `_w` or `_c`.
   `pack.py` converts ground truth NED→world once so nothing downstream has to.

2. **Estimator error must be rigid-aligned before it means anything.** Differencing EKF2
   against ground truth raw gives ~16.8 m here — that is the frame offset, not error. `load.py`
   removes the best-fit rotation+translation first (it recovers yaw 95.2°, shift ≈9.8 m — the
   known camera_init rotation and pad offset) and reports a real 0.52 m, with the raw number
   kept alongside so the offset stays visible.

3. **Never difference the logged ground-truth position to get speed.** PX4 logs it with a
   zero-order hold: consecutive samples repeat, then step. Differencing gives median 0.00 m/s
   and a 31 m/s maximum on a vehicle whose logged velocity peaked at 2.22 m/s. Use the logged
   velocity; position is still fine for path length in aggregate.

4. **`/sdf_map/unknown` is advertised but silent upstream** — `publishUnknown()` is commented
   out at `map_ros.cpp:212`, and it scans only the local bound rather than the planning box.
   A recorder that treats "no unknown cloud" as "everything is free" reports every unmapped
   cell as mapped: one run showed 54.5 m² unknown on `/sdf_map/coverage` while the grid claimed
   zero. The box's unknown voxels are now published from inside the coverage sweep that already
   classifies them (`map_ros/publish_unknown`, on by default; roscpp does not serialise to zero
   subscribers, so it is free when unused). Where the cloud is genuinely absent the grid codes
   non-occupied voxels `UNSURVEYED (3)`, never free, and `manifest.map.unknown_cloud` says which
   case a bundle is.

5. **Record the `/sdf_map/coverage` topic, not the `[coverage]` log line.** The log line is
   emitted only when the known-cell count *changes*, so a stalled map produces silence. The
   topic publishes every 2 s regardless, and a stall reads as flat data. Flat data is
   measurable; missing data is not.

## Metrics `load.py` reports

`revisit fraction` (path flown over ground already left behind — separates re-treading the
same floor from flying new ground and earning nothing) · `coverage earned per metre flown`
(a collapsing curve is the honest measure of wasted flight) · `target re-selection` (viewpoints
chosen again in a later episode — planner thrash, not the map revealing something new) ·
`churn ratio` (path ÷ net displacement — separates a bad route from not flying) · `time
rotating >15 °/s` (with a 360° lidar, yaw buys no coverage, so it is time wasted outright) ·
`target switch rate and jump size` (a planner changing its mind faster than it can finish the
turn) · `longest coverage stall` · `rigid-aligned localisation error` · `which failure detector
tripped, and when`.

Wall clearance and the pass/fail acceptance criteria live in `scripts/verify_flight.py`, which
needs the arena mesh; this package reports only what the bundle itself contains.
