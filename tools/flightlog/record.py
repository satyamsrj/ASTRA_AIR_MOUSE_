#!/usr/bin/env python3
"""Live flight recorder: capture ONLY what cannot be reconstructed after the run.

    python3 tools/flightlog/record.py [--out logs/runs] [--map-interval 5.0]

Design principle: RECORD WHAT IS UNRECOVERABLE, DERIVE EVERYTHING ELSE.

  * The occupancy map, FUEL's commanded setpoints, the selected viewpoint and the coverage
    series exist only while the sim is alive. They are recorded here.
  * Ground-truth pose, attitude, body rates, arming and failsafe state are already written at
    full rate by PX4 into the .ulg. Re-recording them over ROS would be lower fidelity and
    bigger, so this node does not. `pack.py` pulls them from the ulog afterwards.

That split is why this produces megabytes where `rosbag record -a` produces gigabytes, while
answering strictly more questions -- a raw bag of /cloud_registered cannot tell you what the
planner believed, and this can.

Output is a run directory, flushed atomically every --flush-interval seconds so a hard kill
(the usual way a sim ends) loses at most that much:

    <out>/<run_id>/streams.npz     time series, one 2D float64 array per stream
    <out>/<run_id>/map.npz         uint8 voxel-grid snapshots + grid geometry
    <out>/<run_id>/manifest.json   frames, units, params, git SHA, column names
    <out>/<run_id>/events.jsonl    discrete events (mode changes, completion)
"""
import argparse, json, os, subprocess, sys, threading, time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import schema

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def cloud_xyz(msg):
    """PointCloud2 -> (N,3) float64, parsed straight out of the buffer.

    sensor_msgs.point_cloud2.read_points is a Python generator and is far too slow for a
    ~100k-point map cloud arriving every few seconds; this is the same work in numpy.
    """
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 3))
    off = {f.name: f.offset for f in msg.fields}
    if not all(k in off for k in 'xyz'):
        return np.empty((0, 3))
    buf = np.frombuffer(msg.data, dtype=np.uint8)[:n * msg.point_step]
    buf = buf.reshape(n, msg.point_step)
    cols = [buf[:, off[k]:off[k] + 4].copy().view('<f4').ravel() for k in 'xyz']
    p = np.stack(cols, 1).astype(np.float64)
    return p[np.isfinite(p).all(1)]


def find_param(suffix, default=None):
    """Look up a param by its trailing path, whatever node namespace it was pushed into."""
    try:
        for name in rospy.get_param_names():
            if name.endswith(suffix):
                return rospy.get_param(name)
    except Exception:
        pass
    return default


def jsonable(o):
    """Recursively convert numpy scalars/arrays to built-in types.

    ROS params and numpy-derived shapes leak np.int64/np.float64 into the manifest, and
    json.dump raises TypeError on those. That exception used to escape flush() and kill the
    flush timer thread, so the recorder went silent while still appearing to run -- the one
    failure mode a logger must not have.
    """
    if isinstance(o, dict):
        return {str(k): jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [jsonable(v) for v in o]
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    return o


def git_sha():
    try:
        return subprocess.check_output(['git', '-C', REPO, 'rev-parse', 'HEAD'],
                                       stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return None


def atomic_write(path, write_fn, mode='wb'):
    """Write via a temp file + rename.

    Every file this node produces goes through here. A simulation almost always ends by being
    killed, and a SIGTERM landing in the middle of an ordinary open()/write() leaves a
    truncated file -- which for manifest.json means the run is unreadable even though the
    several megabytes of stream data next to it are perfectly intact. os.replace is atomic on
    POSIX, so a reader sees either the previous flush or the new one, never a partial one.
    """
    tmp = path + '.tmp'
    try:
        with open(tmp, mode) as f:
            write_fn(f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except Exception:
        # Leave the previous good file in place rather than a half-written temp beside it.
        try:
            os.remove(tmp)
        except OSError:
            pass
        raise


class Recorder(object):
    def __init__(self, out_root, map_interval, flush_interval, keep_3d):
        self.run_id = time.strftime('%Y%m%d_%H%M%S')
        self.dir = os.path.join(out_root, self.run_id)
        os.makedirs(self.dir, exist_ok=True)
        self.map_interval, self.flush_interval, self.keep_3d = map_interval, flush_interval, keep_3d

        self.rows = {k: [] for k in schema.STREAMS}
        self.events = []
        self.modes, self.last_mode, self.last_armed = [], None, None

        # Latest map clouds, rasterised together on a timer so the two topics need not be synced.
        self.cloud = {k: None for k in schema.MAP_TOPICS}
        self.snaps, self.snap_t = [], []
        self.saw_unknown_cloud = False

        self.res = float(find_param('sdf_map/resolution', 0.1))
        lo = [find_param('sdf_map/box_min_%s' % a) for a in 'xyz']
        hi = [find_param('sdf_map/box_max_%s' % a) for a in 'xyz']
        self.have_box = all(v is not None for v in lo + hi)
        if self.have_box:
            self.lo = np.array([float(v) for v in lo])
            self.hi = np.array([float(v) for v in hi])
            self.shape = tuple(np.maximum(1, np.ceil((self.hi - self.lo) / self.res)).astype(int))
            rospy.loginfo('[flightlog] map grid %s @ %.2f m, box_min=%s (camera_init)',
                          self.shape, self.res, self.lo.round(2).tolist())
        else:
            # Without the box we cannot rasterise into a fixed grid, and a grid whose origin
            # drifts between snapshots is not comparable across time. Say so rather than
            # silently writing something that cannot be differenced.
            self.lo = self.hi = None
            self.shape = None
            rospy.logwarn('[flightlog] sdf_map box params not found; map snapshots DISABLED. '
                          'Time series are unaffected.')

        self._subscribe()
        # Map snapshots are on SIM time: even spacing in sim seconds is what makes the
        # coverage series comparable across runs with different real-time factors.
        rospy.Timer(rospy.Duration(self.map_interval), self._snap_cb)
        # Flushing is on WALL time, deliberately. rospy.Timer fires on sim time, so a wedged
        # or crawling simulation (RTF measured as low as 0.005 in this stack) would stop the
        # flushes exactly when the recording matters most, and the run would end with an empty
        # directory. A daemon thread on time.sleep keeps writing regardless of the sim clock.
        self._stop = threading.Event()
        t = threading.Thread(target=self._flush_loop, name='flightlog-flush')
        t.daemon = True
        t.start()
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo('[flightlog] recording -> %s', self.dir)

    # ---------------------------------------------------------------- subscribers
    def _subscribe(self):
        S = schema.STREAMS
        sub = rospy.Subscriber
        sub(S['cmd']['topic'], rospy.AnyMsg, self._cmd_cb, queue_size=200)
        sub(S['target']['topic'], PoseStamped, self._target_cb, queue_size=100)
        sub(S['coverage']['topic'], Float64MultiArray, self._cov_cb, queue_size=50)
        sub(S['lio']['topic'], Odometry, lambda m: self._odom_cb('lio', m), queue_size=200)
        sub(S['ekf']['topic'], Odometry, lambda m: self._odom_cb('ekf', m), queue_size=200)
        sub(S['state']['topic'], State, self._state_cb, queue_size=50)
        sub('/exploration_completed', Bool, self._done_cb, queue_size=5)
        for key, topic in schema.MAP_TOPICS.items():
            sub(topic, PointCloud2, lambda m, k=key: self.cloud.__setitem__(k, m), queue_size=1)

    def _cmd_cb(self, raw):
        # AnyMsg + late import: quadrotor_msgs is a workspace package and may not be on the
        # PYTHONPATH of whatever interpreter launched this. Deserialising on first use keeps
        # the recorder from dying at import time when everything else would have worked.
        if not hasattr(self, '_PosCmd'):
            from quadrotor_msgs.msg import PositionCommand
            self._PosCmd = PositionCommand
        m = self._PosCmd().deserialize(raw._buff)
        p, v, a = m.position, m.velocity, m.acceleration
        self.rows['cmd'].append([m.header.stamp.to_sec(), p.x, p.y, p.z, v.x, v.y, v.z,
                                 a.x, a.y, a.z, m.yaw, m.yaw_dot, m.trajectory_id])

    def _target_cb(self, m):
        p, q = m.pose.position, m.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        self.rows['target'].append([m.header.stamp.to_sec(), p.x, p.y, p.z, yaw])

    def _cov_cb(self, m):
        d = list(m.data)
        n = len(schema.COVERAGE_FIELDS)
        d = (d + [np.nan] * n)[:n]
        self.rows['coverage'].append([rospy.Time.now().to_sec()] + d)

    def _odom_cb(self, key, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        v, w = m.twist.twist.linear, m.twist.twist.angular
        self.rows[key].append([m.header.stamp.to_sec(), p.x, p.y, p.z,
                               q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.z])

    def _state_cb(self, m):
        if m.mode not in self.modes:
            self.modes.append(m.mode)
        mid = self.modes.index(m.mode)
        t = rospy.Time.now().to_sec()
        self.rows['state'].append([t, float(m.armed), float(mid)])
        if m.mode != self.last_mode or m.armed != self.last_armed:
            self.events.append(dict(t=t, kind='state', mode=m.mode, armed=bool(m.armed)))
            self.last_mode, self.last_armed = m.mode, m.armed

    def _done_cb(self, m):
        self.events.append(dict(t=rospy.Time.now().to_sec(), kind='exploration_completed',
                                value=bool(m.data)))
        rospy.loginfo('[flightlog] /exploration_completed = %s', m.data)
        self.flush()

    # ---------------------------------------------------------------- map snapshot
    def _snap_cb(self, _):
        if self.shape is None:
            return
        occ, unk = self.cloud['occupied'], self.cloud['unknown']
        if occ is None and unk is None:
            return
        # Every voxel is exactly one of FREE/OCCUPIED/UNKNOWN (SDFMap::getOccupancy), and the
        # two topics carry only the latter two -- so FREE would be the correct base state IF
        # both clouds arrived. Upstream, publishUnknown() is commented out (map_ros.cpp:212),
        # so /sdf_map/unknown is advertised but silent unless map_ros/publish_unknown is set.
        # Filling FREE regardless would silently relabel every unknown voxel as mapped: this
        # run reported 54.5 m2 unknown on /sdf_map/coverage while the grid claimed zero, which
        # is exactly the inflated coverage figure an external evaluator must not be handed.
        # Without that cloud the honest base is UNSURVEYED -- "walls known, free/unknown split
        # not observable here" -- and the manifest says which case this grid is.
        base = schema.FREE if unk is not None else schema.UNSURVEYED
        self.saw_unknown_cloud = self.saw_unknown_cloud or unk is not None
        g = np.full(self.shape, base, dtype=np.uint8)
        for msg, code in ((unk, schema.UNKNOWN), (occ, schema.OCCUPIED)):
            if msg is None:
                continue
            p = cloud_xyz(msg)
            if not len(p):
                continue
            idx = np.floor((p - self.lo) / self.res).astype(np.int64)
            ok = np.all((idx >= 0) & (idx < np.array(self.shape)), axis=1)
            idx = idx[ok]
            if len(idx):
                g[idx[:, 0], idx[:, 1], idx[:, 2]] = code
        self.snaps.append(g if self.keep_3d else g.max(axis=2))
        self.snap_t.append(rospy.Time.now().to_sec())

    def _flush_loop(self):
        while not self._stop.wait(self.flush_interval):
            self.flush()

    def _shutdown(self):
        self._stop.set()
        # Force a final snapshot. Snapshots are otherwise on sim time, so a wedged or crawling
        # simulation takes none at all -- and the final map state is the single most valuable
        # artefact in the bundle, being the only thing coverage can be recomputed from
        # independently of what FUEL reported.
        try:
            self._snap_cb(None)
        except Exception as exc:
            rospy.logerr('[flightlog] final map snapshot failed: %s', exc)
        self.flush()

    # ---------------------------------------------------------------- output
    def flush(self):
        """Write every output. Each is independent and guarded: a failure in one must not cost
        the others, and must never propagate into the rospy Timer thread that calls this."""
        for name, fn in (('streams', self._write_streams), ('map', self._write_map),
                         ('events', self._write_events), ('manifest', self._write_manifest)):
            try:
                fn()
            except Exception as exc:
                rospy.logerr('[flightlog] failed writing %s: %s', name, exc)

    def _write_streams(self):
        arrays = {}
        for name, spec in schema.STREAMS.items():
            rows = self.rows[name]
            # An empty stream still gets an array with the right width, so a reader can rely on
            # the column count from the manifest without special-casing "this topic never came up".
            arrays[name] = (np.asarray(rows, dtype=np.float64) if rows
                            else np.empty((0, len(spec['columns'])), dtype=np.float64))
        atomic_write(os.path.join(self.dir, 'streams.npz'),
                     lambda f: np.savez_compressed(f, **arrays))

    def _write_map(self):
        if self.snaps:
            atomic_write(os.path.join(self.dir, 'map.npz'), lambda f: np.savez_compressed(
                f, grid=np.stack(self.snaps), t=np.asarray(self.snap_t, dtype=np.float64),
                origin=self.lo, resolution=np.array([self.res]),
                shape=np.array(self.shape), is_3d=np.array([int(self.keep_3d)])))

    def _write_events(self):
        atomic_write(os.path.join(self.dir, 'events.jsonl'),
                     lambda f: f.write(''.join(json.dumps(e) + '\n' for e in self.events)),
                     mode='w')

    def _write_manifest(self):
        man = dict(
            schema_version=schema.SCHEMA_VERSION, run_id=self.run_id, git_sha=git_sha(),
            created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            frames=dict(streams='camera_init (cx = world_y + 9.5, cy = -world_x)',
                        note='See tools/flightlog/schema.py for the full frame contract.'),
            time_base='ROS sim time, seconds',
            streams={k: dict(topic=s['topic'], type=s['type'], frame=s['frame'],
                             columns=s['columns'], rows=len(self.rows[k]), note=s['note'])
                     for k, s in schema.STREAMS.items()},
            coverage_fields=schema.COVERAGE_FIELDS,
            voxel_codes={str(k): v for k, v in schema.VOXEL_CODES.items()},
            mode_table=self.modes,
            map=dict(enabled=self.shape is not None, snapshots=len(self.snaps),
                     interval_s=self.map_interval, resolution_m=self.res,
                     origin_camera_init=(self.lo.tolist() if self.lo is not None else None),
                     shape=(list(self.shape) if self.shape else None), is_3d=self.keep_3d,
                     unknown_cloud=self.saw_unknown_cloud,
                     unknown_note=('grid free/unknown split is valid'
                                   if self.saw_unknown_cloud else
                                   'NO /sdf_map/unknown cloud: non-occupied voxels are coded '
                                   'UNSURVEYED(3), not free. Use the coverage stream for area.')),
            params=dict(obstacles_inflation=find_param('sdf_map/obstacles_inflation'),
                        max_vel=find_param('fsm/max_vel') or find_param('/max_vel'),
                        arena_area_m2=find_param('map_ros/arena_area_m2'),
                        target_switch_margin=find_param('exploration/target_switch_margin'),
                        target_match_dist=find_param('exploration/target_match_dist')),
        )
        # Written LAST and atomically: its presence and validity is the signal that every
        # other file in this directory is a complete, self-describing flush.
        atomic_write(os.path.join(self.dir, 'manifest.json'),
                     lambda f: json.dump(jsonable(man), f, indent=2, sort_keys=True),
                     mode='w')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.join(REPO, 'logs', 'runs'))
    ap.add_argument('--map-interval', type=float, default=5.0,
                    help='seconds between voxel-grid snapshots (default 5)')
    ap.add_argument('--flush-interval', type=float, default=30.0)
    ap.add_argument('--map-2d', action='store_true',
                    help='store the z-collapsed grid instead of the full 3D box')
    a = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node('nidar_flightlog', anonymous=True)
    r = Recorder(a.out, a.map_interval, a.flush_interval, keep_3d=not a.map_2d)
    rospy.spin()
    r.flush()
    print('[flightlog] wrote %s' % r.dir)


if __name__ == '__main__':
    main()
