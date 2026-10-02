# Implementation Guide & Comparative Benchmark Protocol for 2D SLAM Algorithms

## 1. Scope & Objective
This guide outlines the practical implementation and empirical benchmarking protocol for the top alternative 2D SLAM algorithms within the **ASTRA AirMouse / NIDAR** architecture:
1. **Hector SLAM** (`hector_mapping` / `hector_slam_engine.py`) — *Implemented & Baseline Verified*
2. **Google Cartographer 2D** (`cartographer_ros`) — *Configured in `hardware/launch/2d_slam/cartographer_2d.launch`*
3. **SLAM Toolbox** (`slam_toolbox`) — *Configured in `hardware/launch/2d_slam/slam_toolbox_2d.launch`*
4. **Iris LaMA** (`iris_lama_ros`) — *Configured in `hardware/launch/2d_slam/iris_lama_2d.launch`*

---

## 2. Standardized Interface & Topics
All 2D SLAM algorithms interface with the ASTRA vehicle stack using standard ROS topics:
- **Laser Input**: `/scan` (`sensor_msgs/LaserScan` matching Slamtec RPLiDAR A2 planar specifications: 360°, 10 Hz, 12m range).
- **IMU Orientation**: `/mavros/imu/data` (`sensor_msgs/Imu` from Pixhawk 4 at 250 Hz).
- **Odometry Prior**: `/mavros/local_position/odom` (`nav_msgs/Odometry` from PX4 EKF2 or optical flow).
- **Pose Output**: `/slam_out_pose` or `/odom` (`geometry_msgs/PoseStamped` or `nav_msgs/Odometry`).
- **Map Output**: `/map` (`nav_msgs/OccupancyGrid`, standard 0.05m resolution).

---

## 3. Launching and Running Each Implementation

### 3.1 Hector SLAM (Reference Baseline)
```bash
# Launch Hector SLAM on physical hardware or simulation
roslaunch /home/developer/NIDAR/hardware/launch/hector_slam/hector_mapping.launch scan_topic:=/scan rviz:=false
```

### 3.2 Google Cartographer 2D (IMU Fused + Global Pose Graph)
```bash
# Launch Cartographer with 2D LiDAR + Pixhawk IMU fusion
roslaunch /home/developer/NIDAR/hardware/launch/2d_slam/cartographer_2d.launch scan_topic:=/scan imu_topic:=/mavros/imu/data
```

### 3.3 SLAM Toolbox (Modern Lifelong / Asynchronous Graph SLAM)
```bash
# Launch SLAM Toolbox Async Node
roslaunch /home/developer/NIDAR/hardware/launch/2d_slam/slam_toolbox_2d.launch scan_topic:=/scan
```

### 3.4 Iris LaMA (High-Speed Euclidean Distance Transform)
```bash
# Launch Iris LaMA 2D SLAM
roslaunch /home/developer/NIDAR/hardware/launch/2d_slam/iris_lama_2d.launch scan_topic:=/scan
```

---

## 4. Empirical Evaluation Protocol & Benchmark Metrics

To systematically determine the best 2D SLAM algorithm for the project, evaluate each algorithm against the following four quantitative criteria:

### Metric 1: Absolute Trajectory Error (ATE) & Drift
- **Measurement**: Compare estimated trajectory against Gazebo ground-truth `/gazebo/model_states` or physical motion capture rig.
  $$\text{RMSE}_{\text{trans}} = \sqrt{\frac{1}{N} \sum_{k=1}^N \| p_{k}^{\text{est}} - p_{k}^{\text{gt}} \|^2}$$
- **Target**: RMSE $< 0.15\text{ m}$ across full $158\text{ m}^2$ arena traverse.

### Metric 2: Loop Closure Consistency & Wall Thickness
- **Measurement**: After completing a circular traversal of multi-room corridors, measure the thickness of walls in the generated `/map` grid.
- **Target**: Wall thickness $\le 0.10\text{ m}$ (indicating 0 double-wall phantom artifacts).

### Metric 3: CPU & Memory Overhead
- **Measurement**: Log `%CPU` and Resident Set Size (RSS) during active exploration using `top` or `psrecord`.
- **Target**: Average single-core CPU $< 40\%$; RAM $< 300\text{ MB}$.

### Metric 4: Robustness to UAV Pitch/Roll & Fast Yaw
- **Measurement**: Introduce $8^\circ$ attitude tilt maneuvers and $75^\circ/\text{s}$ yaw rotations.
- **Target**: Zero estimator divergence resets and 0 NaN/Inf pose drops.

---

## 5. Next Steps for Hardware & Flight Trials
1. Run parallel bag file playbacks (`rosbag play arena_flight.bag`) through each launch file to generate side-by-side metric tables.
2. If Cartographer demonstrates superior drift resistance under attitude tilt, configure `relay_odometry.py` to bridge Cartographer poses into PX4 EKF2.
