"""Canonical description of what a NIDAR flight recording contains.

This module is the SINGLE SOURCE OF TRUTH for stream names, columns, units and frames.
`record.py` builds its subscribers from it, `pack.py` writes it into the manifest, and
`load.py` uses it to name columns. Change a recorded field here and nowhere else.

FRAMES -- the three coordinate frames in this stack are NOT interchangeable and values from
different frames must never be differenced:

  world      Gazebo ENU. The arena is centred on the origin; the launch pad is (0, -9.5, 0.26).
             Ground truth from the PX4 ulog is converted into this frame by pack.py.
  camera_init FAST-LIO's map frame, yaw-rotated 90 deg from world:
                 cx = world_y + 9.5      cy = -world_x       cz = world_z
             FUEL plans entirely in this frame, so /planning/pos_cmd, /exploration/next_view,
             /Fast_LIO/odometry and every sdf_map topic are all camera_init.
  NED        PX4's internal frame inside the .ulg (x=North, y=East, z=Down).

Anything named *_w is world, *_c is camera_init. Untagged columns are frame-free (times,
counts, areas, flags).
"""

# Voxel classification, matching SDFMap's enum as reported by getOccupancy().
FREE, OCCUPIED, UNKNOWN, UNSURVEYED = 0, 1, 2, 3
VOXEL_CODES = {FREE: 'free', OCCUPIED: 'occupied', UNKNOWN: 'unknown',
               UNSURVEYED: 'unsurveyed (recorder had no /sdf_map/unknown cloud)'}

# Layout of /sdf_map/coverage (std_msgs/Float64MultiArray).
# MIRRORS map_ros.cpp MapROS::coverageCallback -- if that msg.data initialiser changes, change
# this list in the same commit. Recording the TOPIC rather than scraping the "[coverage]" log
# line is deliberate: the log line is only emitted when the known-cell count CHANGES, so a
# stalled map produces silence, whereas the topic keeps publishing every coverage_interval
# (2.0 s) and a stall shows up as flat data. Flat data is measurable; missing data is not.
COVERAGE_FIELDS = [
    'free_n',            # cells classified FREE in the planning box (count, whole 3D box)
    'occ_n',             # cells classified OCCUPIED
    'unk_n',             # cells classified UNKNOWN  <- this is what is "left to map"
    'free_area_m2',      # free_n / layers * res^2
    'known_area_m2',     # (free_n + occ_n) / layers * res^2
    'unknown_area_m2',   # unk_n / layers * res^2
    'config_denom_m2',   # map_ros/arena_area_m2 if set. REFERENCE ONLY -- not a denominator.
    'pct_observable',    # 100 * free / (free + unknown). The honest coverage number.
    'left_m2',           # == unknown_area_m2
    'layers',            # z-layers in the box; the divisor that turns cell counts into area
    't_map_s',           # seconds since MapROS start
]

# Live streams. Each entry: (topic, ros type, frame, column names produced by record.py).
# Column 't' is always the ROS header stamp in seconds (SIM time), or rospy.Time.now() for
# messages that carry no header.
STREAMS = {
    'cmd': dict(
        topic='/planning/pos_cmd', type='quadrotor_msgs/PositionCommand', frame='camera_init',
        note='What FUEL actually commanded. Differencing this against ground truth separates '
             '"the planner asked for the wrong thing" from "the vehicle did not follow".',
        columns=['t', 'x_c', 'y_c', 'z_c', 'vx_c', 'vy_c', 'vz_c',
                 'ax_c', 'ay_c', 'az_c', 'yaw_c', 'yaw_dot_c', 'traj_id']),
    'target': dict(
        topic='/exploration/next_view', type='geometry_msgs/PoseStamped', frame='camera_init',
        note='The viewpoint the ATSP selected this replan. Consecutive rows that jump are '
             'target churn -- the thing that makes the vehicle turn instead of translate.',
        columns=['t', 'x_c', 'y_c', 'z_c', 'yaw_c']),
    'coverage': dict(
        topic='/sdf_map/coverage', type='std_msgs/Float64MultiArray', frame='n/a',
        note='Mapped-area time series. See COVERAGE_FIELDS.',
        columns=['t'] + COVERAGE_FIELDS),
    'lio': dict(
        topic='/Fast_LIO/odometry', type='nav_msgs/Odometry', frame='camera_init',
        note='Raw LiDAR-only odometry, independent of EKF2 and MAVROS. Diverging from the EKF '
             'stream below means PX4 has desynced from what the mapper believes.',
        columns=['t', 'x_c', 'y_c', 'z_c', 'qx', 'qy', 'qz', 'qw',
                 'vx_c', 'vy_c', 'vz_c', 'wz']),
    'ekf': dict(
        topic='/mavros/local_position/odom', type='nav_msgs/Odometry', frame='camera_init',
        note='PX4 EKF2 fused estimate as seen over MAVROS.',
        columns=['t', 'x_c', 'y_c', 'z_c', 'qx', 'qy', 'qz', 'qw',
                 'vx_c', 'vy_c', 'vz_c', 'wz']),
    'state': dict(
        topic='/mavros/state', type='mavros_msgs/State', frame='n/a',
        note='Arm state and flight mode, recorded on every message; mode changes are also '
             'written to events.jsonl.',
        columns=['t', 'armed', 'mode_id']),
}

# Snapshotted map. Both clouds are cached and rasterised together on a timer.
MAP_TOPICS = {
    'occupied': '/sdf_map/occupancy_all',   # publishes ONLY occupied voxels
    'unknown': '/sdf_map/unknown',          # publishes ONLY unknown voxels
}

# ulog topics pulled by pack.py. Ground truth is recorded by PX4 at full rate already, so the
# live recorder deliberately does NOT duplicate it over ROS at lower fidelity.
ULOG_TOPICS = [
    'vehicle_local_position_groundtruth',   # TRUE position/velocity (NED) -- the movement source
    'vehicle_attitude_groundtruth',         # TRUE attitude quaternion
    'vehicle_angular_velocity_groundtruth', # TRUE body rates -- the yaw-rate/"time rotating" source
    'vehicle_local_position',               # EKF2 estimate; differenced vs GT = localisation error
    'vehicle_status',                       # arming_state, nav_state, failsafe (v1.14 has no system_status)
    'failure_detector_status',              # WHY it terminated (tilt/attitude)
    'battery_status',
    'esc_status',
]

SCHEMA_VERSION = 1
