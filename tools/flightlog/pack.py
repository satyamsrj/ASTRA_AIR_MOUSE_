#!/usr/bin/env python3
"""Consolidate one run into a single self-contained, self-describing bundle.

    python3 tools/flightlog/pack.py [--run logs/runs/<id>] [--ulog x.ulg] [--fuel-log /tmp/fuel.log]

Merges the three sources that together describe a flight, none of which is sufficient alone:

  live capture (record.py)  what the PLANNER believed and commanded, and the map itself
  PX4 .ulg                  what the VEHICLE actually did -- ground truth, at full rate
  fuel.log                  WHY the planner did it -- FSM transitions, frontier counts, failures

Ground truth is converted from PX4 NED into Gazebo world ENU here, once, so that no downstream
analysis has to get that transform right again. Movement must always be judged against ground
truth rather than the estimator: the whole class of bug worth catching is "the vehicle is not
where the software thinks it is", and comparing the planner to the estimator hides exactly it.

Adds to the run directory:

    groundtruth.npz   true pose/velocity/attitude/body-rates in WORLD ENU + vehicle status
    planner.npz       frontier diagnostics time series parsed from fuel.log
    events.jsonl      recorder events merged with planner events (FSM, plan failures)
    raw/              the original fuel.log (gzipped) and .ulg, so nothing is unrecoverable
    summary.json      headline numbers, computed by load.py
"""
import argparse, glob, gzip, json, os, re, shutil, sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import schema

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SPAWN_XY, SPAWN_Z = (0.0, -9.5), 0.26   # launch pad centre in Gazebo world ENU

# "[LEVEL] [wall_stamp, sim_stamp]: message"
LOG_LINE = re.compile(r'\[(INFO|WARN|ERROR)\]\s*\[(\d+\.\d+),\s*(\d+\.\d+)\]:\s*(.*)')
DIAG = re.compile(r'\[FUEL DIAG\] Frontiers detected:\s*(\d+)\s*\|\s*Visitable:\s*(\d+)'
                  r'\s*\|\s*Dormant:\s*(\d+)\s*\|\s*Total Viewpoints:\s*(\d+)')
FSM = re.compile(r'\[FSM\]:\s*from (\w+) to (\w+)')
# Messages worth keeping as discrete events. Each is a thing that changes what the vehicle
# does next, so they are what you scan first when a run goes wrong.
NOTABLE = [
    ('no_path', re.compile(r'No path to next viewpoint')),
    ('plan_fail', re.compile(r'plan fail')),
    ('replan', re.compile(r'Replan:\s*(.*?)=', re.S)),
    ('collision', re.compile(r'collision detected|collision at')),
    ('no_frontier', re.compile(r'No frontier|NO_FRONTIER')),
    ('coverage_cfg', re.compile(r'does not describe this planning box')),
]


def strip_ansi(s):
    return re.sub(r'\x1b\[[0-9;]*m', '', s)


def newest(pattern):
    c = sorted(glob.glob(os.path.expanduser(pattern)), key=os.path.getmtime)
    return c[-1] if c else None


def pick(data, names):
    """Take the fields that exist, in order, and report which they were.

    ulog field names move between PX4 releases (v1.14's vehicle_status has no `system_status`
    at all -- that one is a MAVLink heartbeat field, not a logged topic field). Selecting what
    is actually present keeps the packer working across versions instead of dying on a KeyError
    at the end of a long run, when the recording is the only thing that cannot be redone.
    """
    have = [n for n in names if n in data]
    return have, np.stack([data['timestamp'] / 1e6] + [data[n] for n in have], 1)


def load_ulog(path):
    """PX4 ulog -> ground truth in WORLD ENU, plus vehicle status."""
    from pyulog import ULog
    u = ULog(path, schema.ULOG_TOPICS)
    D = {}
    for d in u.data_list:
        D.setdefault(d.name, d)          # first instance of each topic
    out = {}

    bat = D.get('battery_status')
    if bat is not None:
        have, arr = pick(bat.data, ['voltage_v', 'current_a', 'remaining', 'discharged_mah'])
        out['battery'], out['battery_columns'] = arr, ['t'] + have

    gt = D.get('vehicle_local_position_groundtruth')
    if gt is not None:
        g = gt.data
        t = g['timestamp'] / 1e6
        # PX4 logs NED relative to an origin that may carry a UTM offset; re-reference to the
        # first sample, which is the spawn point, then rotate NED -> ENU about the pad.
        n, e, d_ = g['x'] - g['x'][0], g['y'] - g['y'][0], g['z'] - g['z'][0]
        out['gt'] = np.stack([t, e + SPAWN_XY[0], n + SPAWN_XY[1], SPAWN_Z - d_,
                              g['vy'], g['vx'], -g['vz']], 1)
        out['gt_columns'] = ['t', 'x_w', 'y_w', 'z_w', 'vx_w', 'vy_w', 'vz_w']

    att = D.get('vehicle_attitude_groundtruth')
    if att is not None:
        a = att.data
        q = [a['q[%d]' % i] for i in range(4)]        # PX4 order: w, x, y, z
        yaw_ned = np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]),
                             1 - 2 * (q[2] ** 2 + q[3] ** 2))
        out['att'] = np.stack([a['timestamp'] / 1e6] + q + [yaw_ned], 1)
        out['att_columns'] = ['t', 'qw', 'qx', 'qy', 'qz', 'yaw_ned']

    rate = D.get('vehicle_angular_velocity_groundtruth')
    if rate is not None:
        r = rate.data
        out['rates'] = np.stack([r['timestamp'] / 1e6,
                                 r['xyz[0]'], r['xyz[1]'], r['xyz[2]']], 1)
        out['rates_columns'] = ['t', 'wx', 'wy', 'wz']   # body rates, rad/s

    est = D.get('vehicle_local_position')
    if est is not None:
        p = est.data
        out['est'] = np.stack([p['timestamp'] / 1e6,
                               p['y'] - p['y'][0] + SPAWN_XY[0],
                               p['x'] - p['x'][0] + SPAWN_XY[1],
                               SPAWN_Z - (p['z'] - p['z'][0])], 1)
        out['est_columns'] = ['t', 'x_w', 'y_w', 'z_w']

    st = D.get('vehicle_status')
    if st is not None:
        # failsafe + failure_detector_status are the two that actually flag a terminated
        # flight; nav_state says which mode PX4 fell back to.
        have, arr = pick(st.data, ['arming_state', 'nav_state', 'failsafe',
                                   'failure_detector_status', 'latest_disarming_reason'])
        out['status'], out['status_columns'] = arr, ['t'] + have

    fd = D.get('failure_detector_status')
    if fd is not None:
        # fd_roll / fd_pitch are the 60 deg tilt trips that end a flight in this stack.
        have, arr = pick(fd.data, ['fd_roll', 'fd_pitch', 'fd_alt', 'fd_ext',
                                   'fd_motor', 'fd_imbalanced_prop'])
        out['failure'], out['failure_columns'] = arr, ['t'] + have

    msgs = [dict(t=m.timestamp / 1e6, kind='px4_message', text=m.message)
            for m in u.logged_messages]
    return out, msgs


def parse_fuel_log(path):
    """fuel.log -> frontier diagnostics series + discrete planner events."""
    diag, events, fsm_state = [], [], None
    with open(path, errors='ignore') as fh:
        for raw in fh:
            line = strip_ansi(raw.rstrip('\n'))
            f = FSM.search(line)
            if f:
                # These are printed without a stamp, so they are anchored to the sim time of
                # the most recent stamped line rather than dropped.
                events.append(dict(t=fsm_state, kind='fsm', frm=f.group(1), to=f.group(2)))
                continue
            m = LOG_LINE.match(line)
            if not m:
                continue
            _, _, sim, msg = m.group(1), m.group(2), float(m.group(3)), m.group(4)
            fsm_state = sim
            d = DIAG.search(msg)
            if d:
                diag.append([sim] + [float(x) for x in d.groups()])
            for kind, pat in NOTABLE:
                if pat.search(msg):
                    events.append(dict(t=sim, kind=kind, text=msg[:200]))
                    break
    # FSM lines seen before any stamped line have no time; drop rather than invent one.
    events = [e for e in events if e.get('t') is not None]
    return (np.asarray(diag, dtype=np.float64) if diag else np.empty((0, 5))), events


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run', help='run dir from record.py (default: newest under logs/runs)')
    ap.add_argument('--ulog', help='PX4 .ulg (default: newest under ~/.ros/log)')
    ap.add_argument('--fuel-log', default='/tmp/fuel.log')
    ap.add_argument('--no-raw', action='store_true', help='skip archiving the raw sources')
    a = ap.parse_args()

    run = a.run or newest(os.path.join(REPO, 'logs', 'runs', '*'))
    if not run or not os.path.isdir(run):
        print('no run directory found; pass --run'); return 2
    ulog = a.ulog or newest('~/.ros/log/*/*.ulg')
    print('run      :', run)
    print('ulog     :', ulog or '(none)')
    print('fuel log :', a.fuel_log if os.path.exists(a.fuel_log) else '(none)')

    events = []
    ev_path = os.path.join(run, 'events.jsonl')
    if os.path.exists(ev_path):
        events = [json.loads(l) for l in open(ev_path) if l.strip()]

    if ulog and os.path.exists(ulog):
        arrays, msgs = load_ulog(ulog)
        cols = {k: v for k, v in arrays.items() if k.endswith('_columns')}
        data = {k: v for k, v in arrays.items() if not k.endswith('_columns')}
        np.savez_compressed(os.path.join(run, 'groundtruth.npz'), **data)
        with open(os.path.join(run, 'groundtruth_columns.json'), 'w') as f:
            json.dump(cols, f, indent=2)
        events += msgs
        print('  groundtruth.npz:', {k: v.shape for k, v in data.items()})

    if os.path.exists(a.fuel_log):
        diag, fev = parse_fuel_log(a.fuel_log)
        np.savez_compressed(os.path.join(run, 'planner.npz'), diag=diag,
                            diag_columns=np.array(['t', 'frontiers', 'visitable',
                                                   'dormant', 'viewpoints']))
        events += fev
        print('  planner.npz: diag %s, %d planner events' % (diag.shape, len(fev)))

    events.sort(key=lambda e: e.get('t') or 0.0)
    with open(ev_path, 'w') as f:
        for e in events:
            f.write(json.dumps(e) + '\n')
    print('  events.jsonl: %d events' % len(events))

    if not a.no_raw:
        rawdir = os.path.join(run, 'raw')
        os.makedirs(rawdir, exist_ok=True)
        if os.path.exists(a.fuel_log):
            # gzip: fuel.log is verbose text and is TRUNCATED at every sim launch, so a copy
            # taken here is the only surviving record of this run's planner reasoning.
            with open(a.fuel_log, 'rb') as fi, gzip.open(os.path.join(rawdir, 'fuel.log.gz'), 'wb') as fo:
                shutil.copyfileobj(fi, fo)
        if ulog and os.path.exists(ulog):
            shutil.copy2(ulog, os.path.join(rawdir, os.path.basename(ulog)))
        print('  raw/:', ', '.join(sorted(os.listdir(rawdir))))

    try:
        import load as loader
        summary = loader.summarise(run)
        with open(os.path.join(run, 'summary.json'), 'w') as f:
            json.dump(summary, f, indent=2, sort_keys=True)
        print('\n' + loader.format_summary(summary))
    except Exception as exc:
        print('  summary skipped:', exc)

    total = sum(os.path.getsize(os.path.join(dp, f))
                for dp, _, fs in os.walk(run) for f in fs)
    print('\nbundle: %s  (%.1f MB)' % (run, total / 1e6))
    return 0


if __name__ == '__main__':
    sys.exit(main())
