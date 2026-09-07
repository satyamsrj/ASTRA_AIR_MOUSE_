#!/usr/bin/env python3
"""Read component poses back out of a running Gazebo and print them as SDF-ready values.

Workflow this exists for: drag a component to where you want it in the Gazebo GUI, run this,
and paste the printed <pose> lines into the SDF. It converts each link's WORLD pose into a
pose RELATIVE TO base_link, which is what an SDF <link>/<include> pose actually wants, so you
do not have to do that subtraction by hand (getting it wrong is how the camera ended up drawn
through the middle of the airframe).

    Terminal 1:  scripts/spawn_vehicle_only.sh 0.30 true x500_vlp16
                 then in the GUI: select a part, use the translate/rotate handles
    Terminal 2:  python3 scripts/dump_gazebo_poses.py

Add --watch to reprint whenever something moves, so you can nudge and read continuously.

CAVEAT worth knowing: Gazebo reports the pose of each LINK FRAME, not of the mesh surface.
If a mesh has its origin at a corner (the TFmini's does) the numbers here still refer to the
link origin, so the visual <pose> offset inside that model stays whatever it already is.
This tool is for placing links, not for re-centring meshes.
"""
import argparse
import math
import sys

import rospy
from gazebo_msgs.msg import LinkStates, ModelStates


def quat_to_rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    s = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def rpy_to_mat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def mat_t(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def mat_vec(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_to_rpy(m):
    pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    else:
        roll = math.atan2(-m[1][2], m[1][1])
        yaw = 0.0
    return roll, pitch, yaw


def report(msg):
    base = None
    for name, pose in zip(msg.name, msg.pose):
        if name.endswith('::base_link'):
            base = (name, pose)
            break
    if base is None:
        print("no ::base_link in /gazebo/link_states - is a vehicle spawned?")
        return

    bp = base[1].position
    br, bpi, by = quat_to_rpy(base[1].orientation)
    R = rpy_to_mat(br, bpi, by)
    Rt = mat_t(R)

    print("=" * 78)
    print("base_link world pose: %.4f %.4f %.4f  rpy %.4f %.4f %.4f"
          % (bp.x, bp.y, bp.z, br, bpi, by))
    print("poses below are RELATIVE TO base_link, ready to paste as <pose>x y z r p y</pose>")
    print("=" * 78)
    # Only this vehicle's links. /gazebo/link_states also carries the world (ground_plane,
    # arena, launch pad), which would otherwise print as metres-away "link" rows.
    model = base[0].split('::')[0]
    for name, pose in zip(msg.name, msg.pose):
        if name == base[0] or not name.startswith(model + '::'):
            continue
        # Nested includes appear as "<model>::<nested>::link"; keep the nested name, which is
        # what identifies the component (a bare "link" tells you nothing).
        parts = name.split('::')
        short = '::'.join(parts[1:]) if len(parts) > 2 else parts[-1]
        d = [pose.position.x - bp.x, pose.position.y - bp.y, pose.position.z - bp.z]
        rel = mat_vec(Rt, d)
        lr, lp, ly = quat_to_rpy(pose.orientation)
        Rl = rpy_to_mat(lr, lp, ly)
        rr, rp, ry = mat_to_rpy(mat_mul(Rt, Rl))
        print("  %-22s <pose>%.4f %.4f %.4f %.4f %.4f %.4f</pose>"
              % (short, rel[0], rel[1], rel[2], rr, rp, ry))
    print("")
    print("Reminder: a nested <include> (tfmini_lidar) takes its pose in the PARENT model's")
    print("frame, same as a plain <link>, so these values drop straight in either way.")


def report_rig(msg):
    """Rig layout: components are separate models, so read /gazebo/model_states instead."""
    poses = dict(zip(msg.name, msg.pose))
    if 'rig_frame' not in poses:
        print("no 'rig_frame' model - start the rig first:  scripts/pose_rig.sh")
        return
    base = poses['rig_frame']
    bp = base.position
    br, bpi, by = quat_to_rpy(base.orientation)
    Rt = mat_t(rpy_to_mat(br, bpi, by))

    # The rig spawns the bare frame, whose origin IS base_link, at its resting height.
    print("=" * 78)
    print("rig_frame world pose: %.4f %.4f %.4f  (pinned static)" % (bp.x, bp.y, bp.z))
    print("component poses RELATIVE TO the frame - paste straight into x500_vlp16.sdf")
    print("=" * 78)
    targets = [('rig_velodyne', 'velodyne_link  <link>'),
               ('rig_tfmini', 'tfmini_lidar   <include>'),
               ('rig_camera', 'camera_link    <link>')]
    for model, label in targets:
        if model not in poses:
            continue
        p = poses[model]
        d = [p.position.x - bp.x, p.position.y - bp.y, p.position.z - bp.z]
        rel = mat_vec(Rt, d)
        lr, lp, ly = quat_to_rpy(p.orientation)
        rr, rp, ry = mat_to_rpy(mat_mul(Rt, rpy_to_mat(lr, lp, ly)))
        print("  %-26s <pose>%.4f %.4f %.4f %.4f %.4f %.4f</pose>"
              % (label, rel[0], rel[1], rel[2], rr, rp, ry))
        # flag the constraints that have already cost flights
        if model == 'rig_velodyne' and rel[2] <= 0.163:
            print("      !! z %.4f is at or below 0.163: the -15 deg ring will be cut by the "
                  "prop tips" % rel[2])
        if model == 'rig_tfmini':
            lens = rel[2] - 0.0125
            parked = 0.2195 + lens
            if parked <= 0.10:
                print("      !! parked range would be %.4f m, under the sensor's 0.10 m floor: "
                      "FAST-LIO would reject every sample" % parked)
            else:
                print("      parked range about %.4f m (floor 0.10, margin %+.0f mm)"
                      % (parked, (parked - 0.10) * 1000))
    print("")
    print("Note: the rig spawns components as STATIC standalone models purely so the GUI")
    print("handles can move them. Nothing here is jointed or flyable; x500_vlp16.sdf is.")


def run_rig(watch):
    try:
        msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=10)
    except Exception as exc:
        print("no /gazebo/model_states (%s). Start the rig:  scripts/pose_rig.sh" % exc)
        return 1
    report_rig(msg)
    if watch:
        print("\nwatching - drag components in the GUI, Ctrl+C to stop\n")
        last = {}
        while not rospy.is_shutdown():
            try:
                m = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5)
            except Exception:
                continue
            cur = {n: (p.position.x, p.position.y, p.position.z)
                   for n, p in zip(m.name, m.pose)}
            if any(n not in last or
                   max(abs(cur[n][i] - last[n][i]) for i in range(3)) > 0.001 for n in cur):
                report_rig(m)
                last = cur
            rospy.sleep(0.5)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--watch', action='store_true',
                    help='reprint whenever something moves more than 1 mm')
    ap.add_argument('--rig', action='store_true',
                    help='read the pose_rig.sh layout, where each component is its OWN model '
                         '(that is the only way the Gazebo GUI handles can move them)')
    args = ap.parse_args()

    # init_node MUST come first: wait_for_message needs a live node, and calling run_rig
    # before this silently timed out on every topic.
    rospy.init_node('dump_gazebo_poses', anonymous=True, disable_signals=True)

    if args.rig:
        return run_rig(args.watch)
    try:
        msg = rospy.wait_for_message('/gazebo/link_states', LinkStates, timeout=10)
    except Exception as exc:
        print("no /gazebo/link_states (%s). Start Gazebo first:" % exc)
        print("  scripts/spawn_vehicle_only.sh 0.30 true x500_vlp16")
        return 1
    report(msg)

    if args.watch:
        print("\nwatching - move parts in the GUI, Ctrl+C to stop\n")
        last = {}
        while not rospy.is_shutdown():
            try:
                m = rospy.wait_for_message('/gazebo/link_states', LinkStates, timeout=5)
            except Exception:
                continue
            cur = {n: (p.position.x, p.position.y, p.position.z)
                   for n, p in zip(m.name, m.pose)}
            moved = any(n not in last or
                        max(abs(cur[n][i] - last[n][i]) for i in range(3)) > 0.001
                        for n in cur)
            if moved:
                report(m)
                last = cur
            rospy.sleep(0.5)
    return 0


if __name__ == '__main__':
    sys.exit(main())
