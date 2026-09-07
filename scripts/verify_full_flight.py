#!/usr/bin/env python3
"""Full-flight verification: component placement, takeoff, altitude, entry, mapping, yaw.

Runs alongside test_takeoff.sh and records everything needed to say whether each fix from
2026-09-05/06 actually holds in flight, instead of grepping logs by hand afterwards.

Stages, in the order they must pass:
  A  PLACEMENT   camera / TFmini / legs ground truth while parked, and the rangefinder reading
  B  TAKEOFF     the EDM climb gate (this is what silently blocked two runs)
  C  ALTITUDE    ground truth vs the guard band, now that EKF2_RNG_POS_D is compensated
  D  ENTRY       EDM hand-off to FUEL
  E  EXPLORE     yaw churn, plan success, mapping rate, arena coverage

Baseline to beat, measured 2026-09-05 over 130 s of exploration BEFORE the yaw fixes:
    yaw travelled 1668 deg for 74 deg net   -> ratio 22.4
    peak yaw rate 111 deg/s                 (limit 60)
    commanded-position jumps >0.30 m: 23
    path / net displacement: 5.9

Usage:  python3 scripts/verify_full_flight.py [seconds]
"""
import math
import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

BASELINE = {
    'yaw_ratio': 22.4,
    'peak_yaw_rate_deg': 111.0,
    'cmd_jumps': 23,
    'path_over_net': 5.9,
}
GUARD_Z = (1.45, 1.55)
RNG_FLOOR = 0.10
TFMINI_OFFSET = 0.0725     # below base_link, must match EKF2_RNG_POS_D
LEG_FOOT = 0.200           # below base_link
CAMERA_BOTTOM = 0.1435     # below base_link


class Verifier(object):
    def __init__(self):
        self.gt_z = None
        self.gt_xy = None
        self.rng = None
        self.mav_z = None
        self.armed = False
        self.mode = ''
        self.odom = []          # (t, x, y, yaw)
        self.cmd = []           # (t, x, y, yaw)
        self.placement = None
        self.pad_top = 0.0296   # measured 2026-09-06

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gz_cb, queue_size=1)
        rospy.Subscriber('/tfmini/range', Range, self.rng_cb, queue_size=1)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mav_cb, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=1)
        rospy.Subscriber('/Fast_LIO/odometry', Odometry, self.odom_cb, queue_size=50)

    def gz_cb(self, m):
        for n, p in zip(m.name, m.pose):
            if 'iris' in n:
                self.gt_z = p.position.z
                self.gt_xy = (p.position.x, p.position.y)

    def rng_cb(self, m):
        self.rng = m.range

    def mav_cb(self, m):
        self.mav_z = m.pose.position.z

    def state_cb(self, m):
        self.armed = m.armed
        self.mode = m.mode

    def odom_cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom.append((rospy.Time.now().to_sec(), p.x, p.y, yaw))

    # ---- Stage A ---------------------------------------------------------------
    def capture_placement(self):
        """Ground truth of every belly-mounted item while the vehicle is still parked."""
        if self.gt_z is None or self.rng is None:
            return False
        base = self.gt_z
        self.placement = {
            'base_z': base,
            'rng': self.rng,
            'tfmini_z': base - TFMINI_OFFSET,
            'camera_z': base - CAMERA_BOTTOM,
            'feet_z': base - LEG_FOOT,
            'rest_height': base - self.pad_top,
        }
        return True

    def report_placement(self):
        p = self.placement
        print("\n" + "=" * 74)
        print("STAGE A - COMPONENT PLACEMENT (ground truth, parked)")
        print("=" * 74)
        if not p:
            print("  NOT CAPTURED (no gazebo/range data before takeoff)")
            return False
        print("  base_link world z        %+.4f" % p['base_z'])
        print("  rest height above pad    %+.4f m" % p['rest_height'])
        print("  TFmini world z           %+.4f" % p['tfmini_z'])
        print("  camera bottom world z    %+.4f   %s" % (
            p['camera_z'], "OK" if p['camera_z'] > self.pad_top else "*** BELOW PAD ***"))
        print("  leg feet world z         %+.4f   %s" % (
            p['feet_z'], "OK" if p['feet_z'] >= self.pad_top - 0.005 else "*** SUNK INTO PAD ***"))
        ok_rng = p['rng'] > RNG_FLOOR
        print("  TFmini range             %.4f m  (floor %.2f, margin %+.1f mm)  %s" % (
            p['rng'], RNG_FLOOR, (p['rng'] - RNG_FLOOR) * 1000,
            "OK" if ok_rng else "*** BELOW FLOOR - Z PIN WILL BE LOST ***"))
        good = (p['camera_z'] > self.pad_top and p['feet_z'] >= self.pad_top - 0.005 and ok_rng)
        print("  -> STAGE A %s" % ("PASS" if good else "FAIL"))
        return good

    # ---- Stage C ---------------------------------------------------------------
    def report_altitude(self, samples):
        print("\n" + "=" * 74)
        print("STAGE C - CRUISE ALTITUDE (ground truth vs guard band)")
        print("=" * 74)
        if not samples:
            print("  no samples")
            return False
        lo, hi = min(samples), max(samples)
        mean = sum(samples) / len(samples)
        inband = sum(1 for s in samples if GUARD_Z[0] <= s <= GUARD_Z[1])
        print("  ground truth world z: min %.4f  mean %.4f  max %.4f  (n=%d)"
              % (lo, mean, hi, len(samples)))
        print("  guard band %.2f - %.2f" % GUARD_Z)
        print("  samples inside band: %d/%d (%.0f%%)" % (inband, len(samples),
                                                         100.0 * inband / len(samples)))
        good = inband > 0.8 * len(samples)
        print("  -> STAGE C %s%s" % ("PASS" if good else "FAIL",
              "" if good else "  (EKF2_RNG_POS_D compensation may need the band widened)"))
        return good

    # ---- Stage E ---------------------------------------------------------------
    def report_explore(self, t0, t1):
        print("\n" + "=" * 74)
        print("STAGE E - EXPLORATION: yaw churn and motion, vs the pre-fix baseline")
        print("=" * 74)
        seg = [s for s in self.odom if t0 <= s[0] <= t1]
        if len(seg) < 20:
            print("  too few odometry samples in the exploration window (%d)" % len(seg))
            return False
        path = 0.0
        yaw_travel = 0.0
        rates = []
        for i in range(1, len(seg)):
            dt = seg[i][0] - seg[i - 1][0]
            if dt <= 1e-3:
                continue
            path += math.hypot(seg[i][1] - seg[i - 1][1], seg[i][2] - seg[i - 1][2])
            dy = seg[i][3] - seg[i - 1][3]
            dy = (dy + math.pi) % (2 * math.pi) - math.pi
            yaw_travel += abs(dy)
            rates.append(abs(dy) / dt)
        net = math.hypot(seg[-1][1] - seg[0][1], seg[-1][2] - seg[0][2])
        net_yaw = abs((seg[-1][3] - seg[0][3] + math.pi) % (2 * math.pi) - math.pi)
        ratio = math.degrees(yaw_travel) / max(math.degrees(net_yaw), 1e-3)
        peak = math.degrees(max(rates)) if rates else 0.0
        pon = path / max(net, 1e-3)

        def verdict(now, base, lower_is_better=True):
            better = now < base if lower_is_better else now > base
            return "BETTER than baseline %.1f" % base if better else "WORSE than baseline %.1f" % base

        print("  window %.0f s, %d samples" % (t1 - t0, len(seg)))
        print("  yaw travelled %.0f deg for %.0f deg net" % (math.degrees(yaw_travel),
                                                            math.degrees(net_yaw)))
        print("  yaw churn ratio   %6.1f    %s" % (ratio, verdict(ratio, BASELINE['yaw_ratio'])))
        print("  peak yaw rate     %6.1f deg/s (limit 60)  %s"
              % (peak, verdict(peak, BASELINE['peak_yaw_rate_deg'])))
        print("  path %.1f m / net %.1f m = %.1f   %s"
              % (path, net, pon, verdict(pon, BASELINE['path_over_net'])))
        good = ratio < BASELINE['yaw_ratio'] and peak <= 75.0
        print("  -> STAGE E yaw %s" % ("PASS" if good else "FAIL"))
        return good


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 900.0
    rospy.init_node('verify_full_flight', anonymous=True, disable_signals=True)
    v = Verifier()
    print("verify_full_flight: waiting for topics...")
    t_start = time.time()
    while v.gt_z is None and time.time() - t_start < 120:
        time.sleep(1)
    if v.gt_z is None:
        print("FAIL: no /gazebo/model_states within 120 s - sim did not start")
        return 1

    # Stage A: capture while still parked (not armed / on the ground)
    while not v.armed and time.time() - t_start < 180:
        v.capture_placement()
        time.sleep(0.5)
    placement_ok = v.report_placement()

    print("\nARMED - watching takeoff, altitude, entry, exploration for %.0f s" % dur)
    alt_samples = []
    t_expl_start = None
    deadline = time.time() + dur
    last_note = 0
    while time.time() < deadline and not rospy.is_shutdown():
        if v.gt_z is not None and v.gt_z > 1.0:
            alt_samples.append(v.gt_z)
            if t_expl_start is None:
                t_expl_start = rospy.Time.now().to_sec()
        if time.time() - last_note > 60:
            last_note = time.time()
            print("  t+%4.0fs  gt_z=%.3f  mav_z=%s  rng=%s  mode=%s  odom=%d" % (
                time.time() - t_start,
                v.gt_z if v.gt_z else -1,
                ("%.3f" % v.mav_z) if v.mav_z is not None else "n/a",
                ("%.3f" % v.rng) if v.rng is not None else "n/a",
                v.mode, len(v.odom)))
        time.sleep(0.2)

    alt_ok = v.report_altitude(alt_samples)
    yaw_ok = False
    if t_expl_start:
        yaw_ok = v.report_explore(t_expl_start, rospy.Time.now().to_sec())

    print("\n" + "=" * 74)
    print("SUMMARY")
    print("=" * 74)
    for name, ok in (('A placement', placement_ok), ('C altitude', alt_ok), ('E yaw', yaw_ok)):
        print("  %-14s %s" % (name, "PASS" if ok else "FAIL"))
    print("(B takeoff / D entry come from the EDM log; E plan-success and mapping from rosout)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
