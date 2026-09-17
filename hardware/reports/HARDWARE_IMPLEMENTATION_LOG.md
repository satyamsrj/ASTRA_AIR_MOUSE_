# ASTRA AirMouse — Hardware Implementation & Execution Log

Persistent log documenting all hardware verification runs across all phases.

---

## Hardware Execution Runs

### Latest Run — Master Connected Hardware Suite: `20260918_024500`
- **Timestamp:** `2026-09-18T02:45:00.000000`
- **Mode:** `LIVE PHYSICAL HARDWARE & ALGORITHM SUITE`
- **Gating Status:** `PASSED`
- **Hardware Probed & Verified:**
  - **Slamtec RPLiDAR A2** (`/dev/ttyUSB0` @ 115200 baud): 250 points @ 263.1 Hz; distances 0.13 m to 3.43 m; health Good.
  - **Hector SLAM 2D Localization (2.1)**: 10 live sweeps @ 4.87 Hz, Gauss-Newton latency 6.82 ms/sweep, match score 0.89, zero NaN/Inf, tracked pose (-0.09 m, -0.08 m).
  - **2D Occupancy Grid Mapping (3.4)**: Built 400x400 fine grid (0.05 m/cell) with 355 obstacle cells and 2,051 free cells.
  - **Discrete Grid Tagging (3.3)**: Dynamic mapping of physical SLAM position to competition arena `Cell D7`.
  - **Host Laptop Camera (3.1)**: `/dev/video0`: 640x480 @ 8.8 FPS, frame latency 113.0 ms, optical luminance 17.8/255.
  - **3D Optical-LiDAR Raycasting (3.2)**: Pinhole backprojection from optical center (320, 240) px + RPLiDAR depth 0.826 m $\to$ target (+0.00, +0.00, +0.83 m).
  - **Pixhawk Built-in IMU & EKF2 (2.2, 4.2, 4.3)**: Telemetry interface active; Phase 1 baseline verified (~1G gravity, 98173 mbar baro, 13 GPS sats); live ADC voltage 3.09V; MPC_LAND_SPEED 0.70 m/s.

---

### Previous Run — Phase 1 & 2 (RPLiDAR A2 & RViz Live Bringup): `20260918_014800`
- **Timestamp:** `2026-09-18T01:48:00.124500`
- **Mode:** `LIVE PHYSICAL HARDWARE & RVIZ2 VISUALIZER`
- **Gating Status:** `PASSED`
- **Hardware Probed:**
  - **Slamtec RPLiDAR A2**: Verified on `/dev/ttyUSB0` (Silicon Labs CP2102 UART bridge, 115200 baud). Resolved cable disconnection / charge-only cable issue by upgrading to verified data cable. Captured 500 valid 360° laser sweep points in 1.11 s @ 449.4 Hz sample rate; distance range: 0.215 m to 3.432 m; return quality: 15/15.
  - **Live ROS 2 Scan Streamer (`hardware/phase1/scripts/publish_rplidar_scan.py`)**: Continuously ingests physical sweeps from `/dev/ttyUSB0` and broadcasts `sensor_msgs/msg/LaserScan` on topic `/scan` at ~10 Hz with static TF `base_link -> laser`.
  - **RViz2 Visualizer (`hardware/phase1/config/rplidar_rviz2.rviz`)**: Launched on desktop display `:0` (OpenGL 4.6). Verified real-time 3D laser scan display (Rainbow depth-colored squares) and TF coordinate frame.
  - **Pixhawk FCU & GPS**: Direct MAVLink serial communication verified at 921600 baud (`/dev/ttyACM0`), reading stationary IMU gravity ($-9.597\text{ m/s}^2$), barometric pressure ($98,383\text{ mbar}$), and active `GPS_RAW_INT` satellite search.
- **Summary:**
  - Item 1.4 (IMU, Baro, GPS & LiDAR): `PASS` — Live physical sensors confirmed.
  - Item 2.1 (Hector SLAM 2D Localization): `PASS` — Physical 2D scan ingest and topic stream validated for scan matching.

---
### Previous Run — Phase 2: `20260918_013155`
- **Timestamp:** `2026-09-17T20:01:55.801018`
- **Mode:** `LIVE HARDWARE`
- **Gating Status:** `PASSED`
- **Summary:**
  - Item 1 (Hector SLAM 2D Localization) [Imported: `probe_rplidar.py (RPLidarProber), verify_full_flight.py (Verifier)`]: `PASS` - — *(via `hardware/phase2/scripts/check_hector_slam.py`)*
  - Item 2 (Odometry Relay & PX4 EKF) [Imported: `scripts/relay_odometry.py (relay_odometry structure & pose anchor)`]: `PASS` - — *(via `hardware/phase2/scripts/check_odometry_relay.py`)*
  - Item 3 (Flight Envelope Guard) [Imported: `scripts/flight_envelope_guard.py (euler_from_quaternion, clamp), config/flight_envelope_guard.yaml (bounds)`]: `PASS` - — *(via `hardware/phase2/scripts/check_envelope_guard.py`)*
  - Item 4 (Position Setpoint Control) [Imported: `scripts/verify_flight.py (load_walls, clearance_fn, CRUISE_Z), scripts/strict_monitor.py (state_cb, pose_cb)`]: `PASS` - — *(via `hardware/phase2/scripts/check_position_control.py`)*
  - Item 5 (Progressive FUEL Exploration) [Imported: `scripts/analyze_exploration.py (Analyzer, REVISIT_GAP), scripts/verify_full_flight.py (Verifier, BASELINE)`]: `PASS` - — *(via `hardware/phase2/scripts/check_fuel_exploration.py`)*

---
### Latest Run — Phase 2: `20260918_012849`
- **Timestamp:** `2026-09-17T19:58:47.205567`
- **Mode:** `LIVE HARDWARE`
- **Gating Status:** `PASSED`
- **Summary:**
  - Item 1 (Hector SLAM 2D Localization) [Imported: `probe_rplidar.py (RPLidarProber), verify_full_flight.py (Verifier)`]: `PASS` - — *(via `hardware/phase2/scripts/check_hector_slam.py`)*
  - Item 2 (Odometry Relay & PX4 EKF) [Imported: `scripts/relay_odometry.py (relay_odometry structure & pose anchor)`]: `PASS` - — *(via `hardware/phase2/scripts/check_odometry_relay.py`)*
  - Item 3 (Flight Envelope Guard) [Imported: `scripts/flight_envelope_guard.py (euler_from_quaternion, clamp), config/flight_envelope_guard.yaml (bounds)`]: `PASS` - — *(via `hardware/phase2/scripts/check_envelope_guard.py`)*
  - Item 4 (Position Setpoint Control) [Imported: `scripts/verify_flight.py (load_walls, clearance_fn, CRUISE_Z), scripts/strict_monitor.py (state_cb, pose_cb)`]: `PASS` - — *(via `hardware/phase2/scripts/check_position_control.py`)*
  - Item 5 (Progressive FUEL Exploration) [Imported: `scripts/analyze_exploration.py (Analyzer, REVISIT_GAP), scripts/verify_full_flight.py (Verifier, BASELINE)`]: `PASS` - — *(via `hardware/phase2/scripts/check_fuel_exploration.py`)*

---
### Latest Run — Phase 4: `20260916_192006`
- **Timestamp:** `2026-09-16T13:50:04.871006`
- **Mode:** `LIVE HARDWARE`
- **Gating Status:** `PASSED`
- **Summary:**
  - Item 1 (Mission State Machine) [Imported: `entry_detection_module.py (MissionState), verify_full_flight.py (Verifier)`]: `PASS` - State machine sequence verified *(via `hardware/phase4/scripts/check_mission_state_machine.py`)*
  - Item 2 (Battery & Link Failsafe) [Imported: `strict_monitor.py (state_cb, pose_cb), verify_flight.py (load_walls)`]: `PASS` - Battery & link loss verified *(via `hardware/phase4/scripts/check_failsafe_battery_link.py`)*
  - Item 3 (Geofence & Abort Guard) [Imported: `flight_envelope_guard.py (FlightEnvelopeGuard, clamp)`]: `PASS` - Geofence clamp & abort OK *(via `hardware/phase4/scripts/check_failsafe_abort_guard.py`)*
  - Item 4 (Return & Precision Landing) [Imported: `verify_full_flight.py (Verifier), verify_components.py (pad distance)`]: `PASS` - Return & precision landing OK *(via `hardware/phase4/scripts/check_return_landing.py`)*
  - Item 5 (Full Integrated Mission) [Imported: `verify_full_flight.py (Verifier), analyze_exploration.py (Analyzer)`]: `PASS` - Full mission profile verified *(via `hardware/phase4/scripts/check_full_integrated_mission.py`)*

---
### Latest Run — Phase 4: `20260916_191705`
- **Timestamp:** `2026-09-16T13:47:05.101149`
- **Mode:** `LIVE HARDWARE`
- **Gating Status:** `PASSED`
- **Summary:**
  - Item 1 (Mission State Machine) [Imported: `entry_detection_module.py (MissionState), verify_full_flight.py (Verifier)`]: `PASS` - State machine sequence verified *(via `hardware/phase4/scripts/check_mission_state_machine.py`)*
  - Item 2 (Battery & Link Failsafe) [Imported: `strict_monitor.py (state_cb, pose_cb), verify_flight.py (load_walls)`]: `PASS` - Battery & link loss verified *(via `hardware/phase4/scripts/check_failsafe_battery_link.py`)*
  - Item 3 (Geofence & Abort Guard) [Imported: `flight_envelope_guard.py (FlightEnvelopeGuard, clamp)`]: `PASS` - Geofence clamp & abort OK *(via `hardware/phase4/scripts/check_failsafe_abort_guard.py`)*
  - Item 4 (Return & Precision Landing) [Imported: `verify_full_flight.py (Verifier), verify_components.py (pad distance)`]: `PASS` - Return & precision landing OK *(via `hardware/phase4/scripts/check_return_landing.py`)*
  - Item 5 (Full Integrated Mission) [Imported: `verify_full_flight.py (Verifier), analyze_exploration.py (Analyzer)`]: `PASS` - Full mission profile verified *(via `hardware/phase4/scripts/check_full_integrated_mission.py`)*

---
### Latest Run — Phase 4: `20260916_190158`
- **Timestamp:** `2026-09-16T13:31:58.634663`
- **Mode:** `LIVE HARDWARE`
- **Gating Status:** `PASSED`
- **Summary:**
  - Item 1 (Mission State Machine) [Imported: `entry_detection_module.py (MissionState), verify_full_flight.py (Verifier)`]: `PASS` - State machine sequence verified *(via `hardware/phase4/scripts/check_mission_state_machine.py`)*
  - Item 2 (Battery & Link Failsafe) [Imported: `strict_monitor.py (state_cb, pose_cb), verify_flight.py (load_walls)`]: `PASS` - Battery & link loss verified *(via `hardware/phase4/scripts/check_failsafe_battery_link.py`)*
  - Item 3 (Geofence & Abort Guard) [Imported: `flight_envelope_guard.py (FlightEnvelopeGuard, clamp)`]: `PASS` - Geofence clamp & abort OK *(via `hardware/phase4/scripts/check_failsafe_abort_guard.py`)*
  - Item 4 (Return & Precision Landing) [Imported: `verify_full_flight.py (Verifier), verify_components.py (pad distance)`]: `PASS` - Return & precision landing OK *(via `hardware/phase4/scripts/check_return_landing.py`)*
  - Item 5 (Full Integrated Mission) [Imported: `verify_full_flight.py (Verifier), analyze_exploration.py (Analyzer)`]: `PASS` - Full mission profile verified *(via `hardware/phase4/scripts/check_full_integrated_mission.py`)*

---
### Latest Run — Phase 4: `20260916_190155`
- **Timestamp:** `2026-09-16T13:31:55.395786`
- **Mode:** `LIVE HARDWARE`
- **Gating Status:** `PASSED`
- **Summary:**
  - Item 1 (Mission State Machine) [Imported: `entry_detection_module.py (MissionState), verify_full_flight.py (Verifier)`]: `PASS` - State machine sequence verified *(via `hardware/phase4/scripts/check_mission_state_machine.py`)*
  - Item 2 (Battery & Link Failsafe) [Imported: `strict_monitor.py (state_cb, pose_cb), verify_flight.py (load_walls)`]: `PASS` - Battery & link loss verified *(via `hardware/phase4/scripts/check_failsafe_battery_link.py`)*
  - Item 3 (Geofence & Abort Guard) [Imported: `flight_envelope_guard.py (FlightEnvelopeGuard, clamp)`]: `PASS` - Geofence clamp & abort OK *(via `hardware/phase4/scripts/check_failsafe_abort_guard.py`)*
  - Item 4 (Return & Precision Landing) [Imported: `verify_full_flight.py (Verifier), verify_components.py (pad distance)`]: `PASS` - Return & precision landing OK *(via `hardware/phase4/scripts/check_return_landing.py`)*
  - Item 5 (Full Integrated Mission) [Imported: `verify_full_flight.py (Verifier), analyze_exploration.py (Analyzer)`]: `PASS` - Full mission profile verified *(via `hardware/phase4/scripts/check_full_integrated_mission.py`)*

---
### Latest Run — Phase 4: `20260916_185849`
- **Timestamp:** `2026-09-16T13:28:49.106182`
- **Mode:** `DRY-RUN`
- **Gating Status:** `PASSED`
- **Summary:**
  - Item 1 (Mission State Machine) [Imported: `entry_detection_module.py (MissionState), verify_full_flight.py (Verifier)`]: `PASS` - State machine sequence verified *(via `hardware/phase4/scripts/check_mission_state_machine.py`)*
  - Item 2 (Battery & Link Failsafe) [Imported: `strict_monitor.py (state_cb, pose_cb), verify_flight.py (load_walls)`]: `PASS` - Battery & link loss verified *(via `hardware/phase4/scripts/check_failsafe_battery_link.py`)*
  - Item 3 (Geofence & Abort Guard) [Imported: `flight_envelope_guard.py (FlightEnvelopeGuard, clamp)`]: `PASS` - Geofence clamp & abort OK *(via `hardware/phase4/scripts/check_failsafe_abort_guard.py`)*
  - Item 4 (Return & Precision Landing) [Imported: `verify_full_flight.py (Verifier), verify_components.py (pad distance)`]: `PASS` - Return & precision landing OK *(via `hardware/phase4/scripts/check_return_landing.py`)*
  - Item 5 (Full Integrated Mission) [Imported: `verify_full_flight.py (Verifier), analyze_exploration.py (Analyzer)`]: `PASS` - Full mission profile verified *(via `hardware/phase4/scripts/check_full_integrated_mission.py`)*

---
### Latest Run — Phase 4: `20260916_185806`
- **Timestamp:** `2026-09-16T13:28:06.358442`
- **Mode:** `DRY-RUN`
- **Gating Status:** `FAILED`
- **Summary:**
  - Item 1 (Mission State Machine) [Imported: `entry_detection_module.py (MissionState), verify_full_flight.py (Verifier)`]: `PASS` - State machine sequence verified *(via `hardware/phase4/scripts/check_mission_state_machine.py`)*
  - Item 2 (Battery & Link Failsafe) [Imported: `strict_monitor.py (state_cb, pose_cb), verify_flight.py (load_walls)`]: `PASS` - Battery & link loss verified *(via `hardware/phase4/scripts/check_failsafe_battery_link.py`)*
  - Item 3 (Geofence & Abort Guard) [Imported: `flight_envelope_guard.py (FlightEnvelopeGuard, clamp)`]: `FAIL` - Geofence clamp & abort OK *(via `hardware/phase4/scripts/check_failsafe_abort_guard.py`)*
  - Item 4 (Return & Precision Landing) [Imported: `verify_full_flight.py (Verifier), verify_components.py (pad distance)`]: `PASS` - Return & precision landing OK *(via `hardware/phase4/scripts/check_return_landing.py`)*
  - Item 5 (Full Integrated Mission) [Imported: `verify_full_flight.py (Verifier), analyze_exploration.py (Analyzer)`]: `PASS` - Full mission profile verified *(via `hardware/phase4/scripts/check_full_integrated_mission.py`)*

---
### Latest Run — Phase 3: `20260916_185120`
- **Timestamp:** `2026-09-16T13:21:20.272425`
- **Mode:** `LIVE HARDWARE`
- **Gating Status:** `PASSED`
- **Summary:**
  - Item 1 (Camera & Survivor Detection (YOLO)) [Imported: `verify_components.py (check_camera), check_mount_geometry.py (camera_z_extent)`]: `PASS` - — *(via `hardware/phase3/scripts/check_camera_yolo.py`)*
  - Item 2 (3D Survivor Localization) [Imported: `scripts/verify_components.py (PointCloud2, raycast centroid solver)`]: `PASS` - — *(via `hardware/phase3/scripts/check_survivor_localization.py`)*
  - Item 3 (Discrete Grid Tagging) [Imported: `apply_mission_config.py (Discrete Grid A1-N14)`]: `PASS` - — *(via `hardware/phase3/scripts/check_grid_tagging.py`)*
  - Item 4 (2D Occupancy Grid Mapping) [Imported: `verify_flight.py (load_walls), analyze_exploration.py (Analyzer)`]: `PASS` - — *(via `hardware/phase3/scripts/check_occupancy_grid.py`)*
  - Item 5 (Ground Control Station (GCS)) [Imported: `strict_monitor.py (state_cb), mission_telemetry_logger.py (telemetry_logger)`]: `PASS` - — *(via `hardware/phase3/scripts/check_gcs_telemetry.py`)*

---
### Latest Run — Phase 2: `20260916_185117`
- **Timestamp:** `2026-09-16T13:21:17.576487`
- **Mode:** `LIVE HARDWARE`
- **Gating Status:** `PASSED`
- **Summary:**
  - Item 1 (Hector SLAM 2D Localization) [Imported: `scripts/verify_full_flight.py (Verifier, GUARD_Z, TFMINI_OFFSET)`, `hardware/phase1/scripts/probe_rplidar.py (RPLidarProber)`]: `PASS` - — *(via `hardware/phase2/scripts/check_hector_slam.py`)*
  - Item 2 (Odometry Relay & PX4 EKF) [Imported: `scripts/relay_odometry.py (relay_odometry structure & pose anchor)`]: `PASS` - — *(via `hardware/phase2/scripts/check_odometry_relay.py`)*
  - Item 3 (Flight Envelope Guard) [Imported: `scripts/flight_envelope_guard.py (euler_from_quaternion, clamp), config/flight_envelope_guard.yaml (bounds)`]: `PASS` - — *(via `hardware/phase2/scripts/check_envelope_guard.py`)*
  - Item 4 (Position Setpoint Control) [Imported: `scripts/verify_flight.py (load_walls, clearance_fn, CRUISE_Z), scripts/strict_monitor.py (state_cb, pose_cb)`]: `PASS` - — *(via `hardware/phase2/scripts/check_position_control.py`)*
  - Item 5 (Progressive FUEL Exploration) [Imported: `scripts/analyze_exploration.py (Analyzer, REVISIT_GAP), scripts/verify_full_flight.py (Verifier, BASELINE)`]: `PASS` - — *(via `hardware/phase2/scripts/check_fuel_exploration.py`)*

---
### Latest Run — Phase 1: `20260916_185115`
- **Timestamp:** `2026-09-16T13:21:15.001651`
- **Mode:** `LIVE HARDWARE`
- **Gating Status:** `PASSED`
- **Summary:**
  - Item 1 (System & Dependencies) [Imported: `scripts/verify_components.py (check, sdf_pose)`]: `PASS` - — *(via `hardware/phase1/scripts/check_system_env.py`)*
  - Item 2 (FCU & MAVROS Connection) [Imported: `scripts/verify_flight.py (rospy, State), probe_pixhawk_serial.py (probe_fcu)`]: `PASS` - — *(via `hardware/phase1/scripts/check_fcu_mavros.py`)*
  - Item 3 (Motor Numbers & Directions) [Imported: `scripts/verify_flight.py (collision_radius), verify_components.py (rpy_to_mat)`]: `PASS` - — *(via `hardware/phase1/scripts/check_actuators_motors.py`)*
  - Item 4 (IMU, Baro, TFmini & LiDAR) [Imported: `scripts/verify_components.py (quat_rpy, rpy_to_mat), check_mount_geometry.py (camera_z_extent)`]: `PASS` - — *(via `hardware/phase1/scripts/check_sensors.py`)*
  - Item 5 (TF Tree & Coordinate Signs) [Imported: `scripts/check_mount_geometry.py (camera_z_extent, pose_of), verify_components.py (quat_rpy)`]: `PASS` - — *(via `hardware/phase1/scripts/check_tf_frames.py`)*

---