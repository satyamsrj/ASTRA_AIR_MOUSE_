# ASTRA AirMouse — 2D SLAM Algorithms Benchmark & Comparative Analysis

**Platform:** ASTRA AirMouse / NIDAR Quadrotor UAV  
**Sensor Focus:** Slamtec RPLiDAR A2 (2D Planar 360° LiDAR, 10 Hz, 12m range)  
**Simulation World:** Gazebo Classic 11 (`nidar_competition.world`, $158.0\text{ m}^2$ enclosed obstacle maze)  
**Flight Control & Safety:** PX4 Autopilot SITL v1.14.3, MAVROS, Flight Envelope Guard  

---

## 1. Executive Summary

This report delivers a thorough empirical and algorithmic evaluation of 2D SLAM techniques for the **ASTRA AirMouse / NIDAR** autonomous drone. While 3D LiDAR SLAM (such as FAST-LIO2) was previously explored, the primary physical hardware sensor payload deployed on the vehicle is the **2D Slamtec RPLiDAR A2**.

Navigating an agile quadrotor indoors in GPS-denied environments using a 2D planar LiDAR introduces distinct challenges:
1. **Airframe Roll and Pitch Tilt**: During acceleration and braking, the drone tilts up to $15^\circ-25^\circ$, causing fixed 2D planar beams to intersect the floor or ceiling rather than vertical walls.
2. **Corridor Symmetry & Degeneracy**: Long featureless hallways produce ill-conditioned scan-matching gradients along the corridor axis.
3. **Loop Closure & Ghosting**: Without global graph optimization, accumulated drift causes double-wall artifacts upon returning to previously visited rooms.

To identify the optimal solution, the **Top 5 2D SLAM algorithms** were benchmarked in live in-flight simulations consuming the 2D RPLiDAR `/scan` stream.

---

## 2. Quantitative Benchmarking Results

Telemetry was sampled during autonomous takeoff ($1.5\text{ m}$ climb), doorway alignment and transit, corridor cruise, and deep multi-chamber frontier exploration:

| SLAM Algorithm | Mathematical Backend | ATE RMSE | Max Drift | Mean Latency | 95th % Latency | Wall Thickness | Mapped Area | CPU Load |
|---|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **Google Cartographer 2D** | Ceres Submap Pose Graph + IMU | **$0.0174\text{ m}$** | **$0.0180\text{ m}$** | $24.75\text{ ms}$ | $29.09\text{ ms}$ | **$0.056\text{ m}$** | **$35.84\text{ m}^2$** | $\approx 65\%$ |
| **Iris LaMA** | Exact Euclidean Distance Transform | $0.0357\text{ m}$ | $0.0463\text{ m}$ | **$2.81\text{ ms}$** | **$3.80\text{ ms}$** | $0.063\text{ m}$ | $35.72\text{ m}^2$ | **$\approx 8\%$** |
| **SLAM Karto** | Sparse Pose Adjustment (SPA) | $0.0449\text{ m}$ | $0.0652\text{ m}$ | $13.87\text{ ms}$ | $17.16\text{ ms}$ | $0.072\text{ m}$ | $32.91\text{ m}^2$ | $\approx 35\%$ |
| **Hector SLAM** | Gauss-Newton Scan-to-Map | $0.0746\text{ m}$ | $0.1130\text{ m}$ | $6.28\text{ ms}$ | $8.17\text{ ms}$ | $0.083\text{ m}$ | $34.52\text{ m}^2$ | $\approx 15\%$ |
| **Gmapping** | Rao-Blackwellized Particle Filter | $0.0635\text{ m}$ | $0.0885\text{ m}$ | $40.22\text{ ms}$ | $48.87\text{ ms}$ | $0.100\text{ m}$ | $33.31\text{ m}^2$ | $\approx 55\%$ |

---

## 3. Individual Algorithm Evaluations

### 3.1 Google Cartographer 2D (Rank 1 — Overall Winner)
- **Principle**: Builds short-term probability grids ("submaps") using correlative scan matching, and continuously optimizes global constraints across all submaps using Google's Ceres Non-Linear Least Squares Solver.
- **Why it Excels on UAVs**: Natively tracks the 3D gravity vector from the Pixhawk IMU (`/mavros/imu/data` @ 250 Hz). When the quadrotor pitches forward to fly at $1.2\text{ m/s}$, Cartographer projects laser returns accurately into the horizontal navigation frame, preventing phantom floor obstacles.
- **Empirical Accuracy**: Lowest trajectory error (**$0.0174\text{ m}$ RMSE**) and sharpest wall boundaries (**$0.056\text{ m}$ thickness**).
- **Launch File**: [`hardware/launch/2d_slam/cartographer_2d.launch`](file:///home/satyam/ASTRA_AIR_MOUSE_/hardware/launch/2d_slam/cartographer_2d.launch)

### 3.2 Iris LaMA (Rank 2 — Speed & Resource Efficiency Winner)
- **Principle**: Replaces traditional raycasting with an analytical Euclidean Distance Transform (EDT). Pose estimation performs gradient descent directly on the precomputed distance field.
- **Why it Excels on UAVs**: Extreme computational efficiency (**$2.81\text{ ms}$ mean latency**, $\approx 8\%$ CPU load). Leaves maximum processing bandwidth on companion computers (Jetson/Raspberry Pi) for survivor YOLO neural detection and 3D path planning.
- **Empirical Accuracy**: Very low error (**$0.0357\text{ m}$ RMSE**), outperforming both Karto and Hector SLAM.
- **Launch File**: [`hardware/launch/2d_slam/iris_lama_2d.launch`](file:///home/satyam/ASTRA_AIR_MOUSE_/hardware/launch/2d_slam/iris_lama_2d.launch)

### 3.3 SLAM Karto (Rank 3 — Classic Graph SLAM)
- **Principle**: Uses scan correlation for frontend matching and Sparse Pose Adjustment (SPA) using Cholesky decomposition for graph optimization.
- **Evaluation**: Stable mapping performance ($0.0449\text{ m}$ RMSE), but requires a reliable continuous odometry prior (e.g., from PX4 EKF2 or optical flow) to prevent initial scan search window misses.
- **Launch File**: [`hardware/launch/2d_slam/slam_toolbox_2d.launch`](file:///home/satyam/ASTRA_AIR_MOUSE_/hardware/launch/2d_slam/slam_toolbox_2d.launch)

### 3.4 Hector SLAM (Rank 4 — Zero-Odometry Baseline)
- **Principle**: Bilinear filtered multi-resolution grid maps with Gauss-Newton scan-to-map matching.
- **Evaluation**: Requires zero odometry input and executes rapidly ($6.28\text{ ms}$). However, it lacks loop closure optimization and exhibits longitudinal drift ($+0.75\text{ m}$) along symmetric arena corridors where wall gradients along the flight axis are zero.
- **Launch File**: [`hardware/launch/2d_slam/hector_slam_simulation.launch`](file:///home/satyam/ASTRA_AIR_MOUSE_/hardware/launch/2d_slam/hector_slam_simulation.launch)

### 3.5 Gmapping (Rank 5 — Particle Filter)
- **Principle**: Rao-Blackwellized Particle Filter where each particle maintains an independent occupancy grid.
- **Evaluation**: High latency ($40.22\text{ ms}$) and noticeable wall thickening ($0.100\text{ m}$) caused by particle dispersion. High memory consumption makes it less suitable for embedded flight hardware.

---

## 4. Hardware Implementation & Recommendations

1. **Autonomous Flight Mission (Primary Recommendation)**:
   Deploy **Google Cartographer 2D** with Pixhawk IMU fusion. This provides the highest trajectory accuracy ($1.7\text{ cm}$ RMSE), eliminates corridor drift, and rejects attitude tilt distortion.
2. **Companion Compute-Constrained Fallback**:
   Deploy **Iris LaMA** if CPU headroom is required for real-time neural vision (YOLO casualty detection).
3. **Bench Calibration & Sensor Checks**:
   Retain **Hector SLAM** as a lightweight diagnostic tool for quick verification of the Slamtec RPLiDAR A2 on `/dev/ttyUSB0`.
