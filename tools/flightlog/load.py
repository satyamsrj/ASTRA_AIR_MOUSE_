#!/usr/bin/env python3
"""Read a flight bundle and compute the metrics that have actually diagnosed problems here.

    python3 tools/flightlog/load.py [run_dir]          # print the summary
    from load import load_run, summarise               # or use it as a library

`load_run` returns a plain dict of numpy arrays plus the manifest, so analysis needs nothing
beyond numpy. Every metric below exists because it distinguished two competing explanations at
some point in this project:

  churn ratio          path length / net displacement. Separates "flying a bad route" from
                       "not flying". A vehicle can look busy and go nowhere.
  time rotating        fraction of samples above 15 deg/s of yaw rate. This is what showed the
                       vehicle was TURNING rather than hovering -- with a 360 deg lidar, yaw
                       buys no coverage, so time spent turning is time wasted outright.
  target switch rate   how often the ATSP changed its mind, and by how far. A switch rate whose
                       period is shorter than the turn it provokes is a planner thrashing.
  coverage stall       longest window gaining < 1 m2. The honest measure of "stuck", and it
                       reads off the topic, which samples even when the log line goes quiet.
  localisation error   ground truth minus estimate. Bounds how much clearance the planner needs.

Wall clearance and the pass/fail acceptance criteria live in scripts/verify_flight.py, which
needs the arena mesh; this module deliberately reports only what the bundle itself contains.
"""
import json, os, sys

import numpy as np

DEG = 180.0 / np.pi
ROTATING_DEG_S = 15.0     # above this the vehicle is turning rather than translating
HOVER_M_S = 0.10
STALL_GAIN_M2 = 1.0       # a window gaining less than this is not progress


def load_run(run):
    """Load every artefact present in a bundle. Missing pieces are simply absent from the dict."""
    out = {'dir': run}
    man = os.path.join(run, 'manifest.json')
    if os.path.exists(man):
        out['manifest'] = json.load(open(man))
    for name in ('streams', 'map', 'groundtruth', 'planner'):
        p = os.path.join(run, name + '.npz')
        if os.path.exists(p):
            z = np.load(p, allow_pickle=False)
            out[name] = {k: z[k] for k in z.files}
    cols = os.path.join(run, 'groundtruth_columns.json')
    if os.path.exists(cols):
        out['columns'] = json.load(open(cols))
    ev = os.path.join(run, 'events.jsonl')
    if os.path.exists(ev):
        out['events'] = [json.loads(l) for l in open(ev) if l.strip()]
    return out


def _motion(t, x, y, speed=None, max_gap_s=0.5):
    """Path length, net displacement, churn and speed stats from a 2D track.

    `speed` is the DIRECTLY LOGGED speed and is used for the distribution whenever available.
    Differencing the logged position is not equivalent: PX4 logs ground-truth position with a
    zero-order hold, so consecutive samples repeat and then step. Differencing that gives a
    median speed of 0.00 m/s and a 31 m/s maximum on a vehicle whose logged velocity never
    exceeded 2.22 m/s. Path length and churn are still taken from position -- the held steps
    sum to the correct distance -- but every per-sample speed statistic comes from velocity.

    Samples separated by more than max_gap_s are treated as a discontinuity and excluded from
    both path length and the speed distribution. Without this, any gap in the track (a dropped
    logging window, or a boolean mask that removes interior samples) is differenced as though
    the vehicle teleported across it, which produced a 31 m/s "max speed" on a vehicle whose
    cruise limit is 0.6 m/s. A fabricated outlier in a debugging tool is worse than a gap.
    """
    d = np.hypot(np.diff(x), np.diff(y))
    dt = np.diff(t)
    ok = (dt > 1e-4) & (dt <= max_gap_s)
    d = d[ok]
    path, net = float(d.sum()), float(np.hypot(x[-1] - x[0], y[-1] - y[0]))
    span = float(t[-1] - t[0])
    spd = np.asarray(speed, dtype=np.float64) if speed is not None else d / dt[ok]
    spd = spd[np.isfinite(spd)]
    return dict(
        span_s=span, path_m=path, net_m=net,
        churn_ratio=path / max(net, 1e-3),
        effective_progress_m_s=net / max(span, 1e-3),
        speed_median=float(np.median(spd)), speed_p90=float(np.percentile(spd, 90)),
        speed_max=float(spd.max()),
        speed_source='logged velocity' if speed is not None else 'differenced position',
        frac_hovering=float((spd < HOVER_M_S).mean()),
        frac_below_0p3=float((spd < 0.30).mean()),
        dropped_gaps=int((~ok).sum()))


def _rigid_residual(est, gt):
    """Best-fit rotation+translation of `est` onto `gt` (Kabsch), and the residual per sample.

    The estimator's local frame is NOT the ground-truth frame in this stack: PX4's EKF2 origin
    and heading are set at initialisation and the vision pose arrives already in camera_init,
    so differencing the two raw gives a constant frame offset -- 16.8 m on a vehicle whose true
    tracking error is ~0.1 m. Removing the best rigid transform first leaves the part that is
    genuinely drift, which is the quantity that determines how much wall clearance the planner
    needs. Reported alongside the raw number so the offset itself stays visible.
    """
    ec, gc = est - est.mean(0), gt - gt.mean(0)
    u, _, vt = np.linalg.svd(ec.T @ gc)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    R = vt.T @ np.diag([1.0, d]) @ u.T
    res = np.linalg.norm(gc - ec @ R.T, axis=1)
    shift = gt.mean(0) - est.mean(0) @ R.T
    return res, float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))), [float(v) for v in shift]


def _revisit(t, x, y, cell_m=0.5, return_gap_s=15.0):
    """How much of the path was flown over ground the vehicle had already left behind.

    Distinguishes two failures that look identical from the outside. Re-flying the same floor
    is one thing; flying new ground while earning no coverage is a different thing entirely,
    and only the second is fixed by changing the planner's target logic. A cell counts as a
    RETURN only if the vehicle had left it more than return_gap_s ago, so ordinary loitering
    inside one cell is not miscounted as a revisit.
    """
    ix = np.floor(x / cell_m).astype(np.int64)
    iy = np.floor(y / cell_m).astype(np.int64)
    key = ix * 100000 + iy
    seg = np.hypot(np.diff(x), np.diff(y))
    first, revisit = {}, 0.0
    for i in range(len(seg)):
        k = int(key[i])
        if k in first:
            if t[i] - first[k] > return_gap_s:
                revisit += seg[i]
        else:
            first[k] = t[i]
    total = float(seg.sum())
    return dict(path_m=total, revisit_m=float(revisit),
                revisit_frac=float(revisit / max(total, 1e-3)),
                cells_occupied=len(first),
                floor_flown_m2=float(len(first) * cell_m * cell_m))


def _coverage_efficiency(ct, free, t, x, y, bin_s=20.0):
    """Square metres of new map earned per metre flown, in time bins.

    The decay of this number is the honest measure of wasted flight: a vehicle still covering
    new ground has a flat curve, one re-treading known space has a collapsing one.
    """
    cum = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    pl = np.interp(ct, t, cum)
    out = []
    for a in np.arange(ct[0], ct[-1], bin_s):
        m = (ct >= a) & (ct < a + bin_s)
        if m.sum() < 2:
            continue
        d, dm = pl[m][-1] - pl[m][0], free[m][-1] - free[m][0]
        out.append(dict(t=float(a - ct[0]), path_m=float(d), new_m2=float(dm),
                        m2_per_m=float(dm / max(d, 1e-3))))
    return out


def _coverage_stall(t, free):
    """Longest window over which free area grew by less than STALL_GAIN_M2.

    Two-pointer over a series that is essentially monotonic; this is the number that told us a
    run had gone dormant while the log was still scrolling replan messages past.
    """
    worst, worst_t, j = 0.0, 0.0, 0
    for i in range(len(t)):
        if j < i:
            j = i
        while j + 1 < len(t) and free[j + 1] - free[i] < STALL_GAIN_M2:
            j += 1
        if t[j] - t[i] > worst:
            worst, worst_t = float(t[j] - t[i]), float(t[i])
    return worst, worst_t


def summarise(run):
    R = load_run(run) if isinstance(run, str) else run
    s = {'dir': R.get('dir'), 'run_id': R.get('manifest', {}).get('run_id')}
    s['params'] = R.get('manifest', {}).get('params', {})
    st = R.get('streams', {})

    # --- movement, from GROUND TRUTH where available (never the estimator) -----------------
    gt = R.get('groundtruth', {})
    if 'gt' in gt and len(gt['gt']) > 20:
        g = gt['gt']
        t, x, y, z = g[:, 0], g[:, 1], g[:, 2], g[:, 3]
        # Select the airborne WINDOW rather than filtering by a per-sample altitude mask: a
        # mask drops interior samples whenever the vehicle dips, leaving holes that the
        # differencing above would have to guess across.
        air = np.flatnonzero(z > 0.26 + 0.5)
        lo, hi = (air[0], air[-1]) if len(air) > 10 else (0, len(t) - 1)
        t, x, y = t[lo:hi + 1], x[lo:hi + 1], y[lo:hi + 1]
        spd = np.linalg.norm(g[lo:hi + 1, 4:7], axis=1) if len(air) > 10 \
            else np.linalg.norm(g[:, 4:7], axis=1)
        s['motion'] = _motion(t, x, y, speed=spd)
        s['motion']['source'] = 'ulog ground truth (world ENU)'
        s['airborne_s'] = float(t[-1] - t[0])
    elif 'lio' in st and len(st['lio']) > 20:
        L = st['lio']
        s['motion'] = _motion(L[:, 0], L[:, 1], L[:, 2])
        s['motion']['source'] = 'FAST-LIO odometry (camera_init) -- no ulog in bundle'

    # --- rotation: the metric that separated "hovering" from "turning" ---------------------
    wz = None
    if 'rates' in gt and len(gt['rates']):
        wz = np.abs(gt['rates'][:, 3]) * DEG
    elif 'lio' in st and len(st['lio']):
        wz = np.abs(st['lio'][:, 11]) * DEG
    if wz is not None and len(wz):
        s['rotation'] = dict(yaw_rate_median_deg_s=float(np.median(wz)),
                             yaw_rate_p90_deg_s=float(np.percentile(wz, 90)),
                             yaw_rate_max_deg_s=float(wz.max()),
                             frac_rotating=float((wz > ROTATING_DEG_S).mean()))

    # --- target churn ---------------------------------------------------------------------
    if 'target' in st and len(st['target']) > 1:
        T = st['target']
        T = T[np.argsort(T[:, 0])]
        jump = np.hypot(np.diff(T[:, 1]), np.diff(T[:, 2]))
        big = jump > 0.5
        span = max(T[-1, 0] - T[0, 0], 1e-3)
        # Cluster the chosen viewpoints and count how many were selected in MORE THAN ONE
        # separate episode. A frontier that is targeted, abandoned, and targeted again later is
        # the planner changing its mind, not the map revealing something new -- and it is the
        # signature that distinguishes planner thrash from honest exploration.
        cent, lab = [], []
        for p in T[:, 1:3]:
            hit = [i for i, c in enumerate(cent) if np.hypot(*(p - c)) < 1.5]
            if hit:
                lab.append(hit[0])
            else:
                cent.append(p)
                lab.append(len(cent) - 1)
        lab = np.asarray(lab)
        eps = {}
        for i, l in enumerate(lab):
            if i == 0 or lab[i - 1] != l:
                eps[int(l)] = eps.get(int(l), 0) + 1
        repeat = {k: v for k, v in eps.items() if v > 1}
        s['targets'] = dict(updates=int(len(T)), switches=int(big.sum()),
                            switches_per_s=float(big.sum() / span),
                            jump_median_m=float(np.median(jump[big])) if big.any() else 0.0,
                            jump_max_m=float(jump.max()),
                            distinct_locations=len(cent),
                            reselected_locations=len(repeat),
                            max_episodes_one_location=int(max(eps.values())) if eps else 0)

    if 'motion' in s and 'gt' in gt and len(gt['gt']) > 20:
        s['revisit'] = _revisit(t, x, y)

    # --- coverage -------------------------------------------------------------------------
    if 'coverage' in st and len(st['coverage']) > 1:
        C = st['coverage']
        C = C[np.argsort(C[:, 0])]
        t_, free, unk, pct = C[:, 0], C[:, 4], C[:, 6], C[:, 8]
        stall, stall_t = _coverage_stall(t_, free)
        if 'gt' in gt and len(gt['gt']) > 20:
            s['coverage_efficiency'] = _coverage_efficiency(t_, free, t, x, y)
        s['coverage'] = dict(
            final_free_m2=float(free[-1]), final_unknown_m2=float(unk[-1]),
            final_pct_observable=float(pct[-1]), peak_pct_observable=float(pct.max()),
            mean_rate_m2_per_s=float((free[-1] - free[0]) / max(t_[-1] - t_[0], 1e-3)),
            longest_stall_s=stall, longest_stall_from_s=stall_t, samples=int(len(C)))

    # --- localisation error ---------------------------------------------------------------
    if 'gt' in gt and 'est' in gt and len(gt['est']) > 10:
        g, e = gt['gt'], gt['est']
        ex = np.interp(g[:, 0], e[:, 0], e[:, 1])
        ey = np.interp(g[:, 0], e[:, 0], e[:, 2])
        raw = np.hypot(g[:, 1] - ex, g[:, 2] - ey)
        res, yaw_deg, shift = _rigid_residual(np.stack([ex, ey], 1), g[:, 1:3])
        s['localisation'] = dict(
            median_m=float(np.median(res)), p90_m=float(np.percentile(res, 90)),
            max_m=float(res.max()), raw_median_m=float(np.median(raw)),
            frame_yaw_deg=yaw_deg, frame_shift_m=shift)

    # --- planner health -------------------------------------------------------------------
    pl = R.get('planner', {})
    if 'diag' in pl and len(pl['diag']):
        d = pl['diag']
        s['frontiers'] = dict(final_detected=int(d[-1, 1]), final_visitable=int(d[-1, 2]),
                              final_dormant=int(d[-1, 3]), max_dormant=int(d[:, 3].max()),
                              samples=int(len(d)))

    if 'failure' in gt and len(gt['failure']):
        f, cols = gt['failure'], R.get('columns', {}).get('failure_columns')
        trips = {}
        for k in range(1, f.shape[1]):
            hit = f[:, k] > 0
            if hit.any():
                name = cols[k] if cols and k < len(cols) else 'col%d' % k
                trips[name] = float(f[hit, 0][0])
        if trips:
            s['failure_trips'] = trips

    ev = R.get('events', [])
    if ev:
        kinds = {}
        for e in ev:
            kinds[e.get('kind', '?')] = kinds.get(e.get('kind', '?'), 0) + 1
        s['events'] = kinds
        txt = ' '.join(e.get('text', '') for e in ev)
        s['flight_terminated'] = ('Attitude failure' in txt) or ('triggering terminate' in txt)
        s['exploration_completed'] = any(e.get('kind') == 'exploration_completed' for e in ev)

    m = R.get('manifest', {}).get('map')
    if m:
        s['map'] = dict(snapshots=m.get('snapshots'), shape=m.get('shape'),
                        resolution_m=m.get('resolution_m'))
    return s


def format_summary(s):
    L = ['=' * 74, 'FLIGHT SUMMARY  %s' % (s.get('run_id') or s.get('dir')), '=' * 74]
    a = L.append
    if 'motion' in s:
        m = s['motion']
        a('movement   [%s]' % m['source'])
        a('  airborne %.0f s | path %.1f m | net %.1f m | CHURN %.1f'
          % (s.get('airborne_s', m['span_s']), m['path_m'], m['net_m'], m['churn_ratio']))
        a('  effective progress %.3f m/s | speed med %.2f p90 %.2f max %.2f m/s'
          % (m['effective_progress_m_s'], m['speed_median'], m['speed_p90'], m['speed_max']))
        a('  time hovering (<0.10 m/s) %.0f%% | time <0.30 m/s %.0f%%   [speed from %s]'
          % (100 * m['frac_hovering'], 100 * m['frac_below_0p3'],
             m.get('speed_source', '?')))
    if 'rotation' in s:
        r = s['rotation']
        a('rotation')
        a('  yaw rate med %.1f p90 %.1f max %.1f deg/s | TIME ROTATING >%g deg/s %.0f%%'
          % (r['yaw_rate_median_deg_s'], r['yaw_rate_p90_deg_s'], r['yaw_rate_max_deg_s'],
             ROTATING_DEG_S, 100 * r['frac_rotating']))
    if 'targets' in s:
        t = s['targets']
        a('targets')
        a('  %d updates, %d switches -> %.2f/s | jump median %.2f m max %.2f m'
          % (t['updates'], t['switches'], t['switches_per_s'], t['jump_median_m'], t['jump_max_m']))
        a('  %d distinct locations, %d RE-SELECTED in a later episode (worst: %d episodes)'
          % (t['distinct_locations'], t['reselected_locations'],
             t['max_episodes_one_location']))
    if 'revisit' in s:
        r = s['revisit']
        a('revisit')
        a('  %.1f m of %.1f m flown over ground already left behind = %.0f%% | floor covered '
          '%.1f m2' % (r['revisit_m'], r['path_m'], 100 * r['revisit_frac'], r['floor_flown_m2']))
    if 'coverage_efficiency' in s and s['coverage_efficiency']:
        e = s['coverage_efficiency']
        a('coverage earned per metre flown (20 s bins)')
        a('  ' + '  '.join('t%d:%.2f' % (b['t'], b['m2_per_m']) for b in e))
        first, last = e[0]['m2_per_m'], e[-1]['m2_per_m']
        best = max(b['m2_per_m'] for b in e)
        a('  peak %.2f m2/m -> final %.2f m2/m' % (best, last))
    if 'coverage' in s:
        c = s['coverage']
        a('coverage')
        a('  %.1f m2 free, %.1f m2 STILL UNKNOWN, %.1f%% of observable (peak %.1f%%)'
          % (c['final_free_m2'], c['final_unknown_m2'], c['final_pct_observable'],
             c['peak_pct_observable']))
        a('  mean rate %.3f m2/s | longest stall %.0f s from t=%.0f'
          % (c['mean_rate_m2_per_s'], c['longest_stall_s'], c['longest_stall_from_s']))
    if 'localisation' in s:
        l = s['localisation']
        a('localisation error (ground truth vs EKF2, after rigid alignment)')
        a('  median %.3f m | p90 %.3f m | max %.3f m' % (l['median_m'], l['p90_m'], l['max_m']))
        a('  frame offset removed: yaw %.1f deg, shift %.2f,%.2f m (raw diff median %.2f m'
          ' is that offset, NOT error)'
          % (l['frame_yaw_deg'], l['frame_shift_m'][0], l['frame_shift_m'][1],
             l['raw_median_m']))
    if 'frontiers' in s:
        f = s['frontiers']
        a('frontiers  detected %d | visitable %d | dormant %d (max %d)'
          % (f['final_detected'], f['final_visitable'], f['final_dormant'], f['max_dormant']))
    if 'events' in s:
        a('events     ' + ', '.join('%s=%d' % kv for kv in sorted(s['events'].items())))
    if s.get('failure_trips'):
        a('failure    ' + ', '.join('%s tripped at t=%.1f s' % (k, v)
                                    for k, v in sorted(s['failure_trips'].items())))
    a('outcome    exploration_completed=%s | flight_terminated=%s'
      % (s.get('exploration_completed'), s.get('flight_terminated')))
    a('=' * 74)
    return '\n'.join(L)


if __name__ == '__main__':
    import glob
    REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    run = sys.argv[1] if len(sys.argv) > 1 else None
    if not run:
        c = sorted(glob.glob(os.path.join(REPO, 'logs', 'runs', '*')), key=os.path.getmtime)
        run = c[-1] if c else None
    if not run:
        print('no run found'); sys.exit(2)
    print(format_summary(summarise(run)))
