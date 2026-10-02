# Hector SLAM in ASTRA AirMouse / NIDAR: Detailed Algorithmic & Empirical Report

## 1. Executive Summary
This report provides an in-depth empirical and theoretical analysis of **Hector SLAM** (`hector_mapping` and standalone `hector_slam_engine.py`) within the **ASTRA AirMouse / NIDAR** project.

Hector SLAM was tested and benchmarked on:
1. **Physical Hardware**: Slamtec RPLiDAR A2 (10 Hz, 360°, 12m range) on companion computer UART `/dev/ttyUSB0` paired with Pixhawk 4 EKF2.
2. **Simulation**: Gazebo Classic 11 quadrotor SITL (`x500_vlp16` and 2D planar laserscan extraction) exploring enclosed obstacle arenas (`nidar_competition.world`).

---

## 2. Theoretical Principles of Hector SLAM

Unlike particle filter algorithms (e.g., Gmapping) or graph-based SLAM with loop closure (e.g., Cartographer, Karto), Hector SLAM relies on **pure scan-to-map alignment**:
- **Multi-Resolution Occupancy Grid Representation**: Maintains hierarchical grid pyramids (typically 2 to 3 resolution layers, e.g., $0.05\text{ m}$ fine, $0.10\text{ m}$ coarse, $0.20\text{ m}$ global) to escape local minima during fast displacement.
- **Gauss-Newton Optimization**:
  At each time step $t$, given the latest laser scan endpoints $S = \{s_i\}_{i=1}^N$ in sensor frame, Hector SLAM minimizes the difference between scan points and the current map probability:
  $$\xi^* = \arg\min_{\xi} \sum_{i=1}^N \left[ 1 - M\left( S_i(\xi) \right) \right]^2$$
  where $\xi = [p_x, p_y, \psi]^T$ is the 2D rigid transformation, and $M(p)$ is the continuously interpolated occupancy value at coordinate $p$ obtained via bilinear filtering.
- **First-Order Taylor Expansion**:
  $$\Delta \xi = H^{-1} \sum_{i=1}^N \left[ \nabla M(S_i(\xi)) \frac{\partial S_i(\xi)}{\partial \xi} \right]^T \left[ 1 - M(S_i(\xi)) \right]$$
  where $H$ is the Gauss-Newton approximate Hessian:
  $$H = \sum_{i=1}^N \left[ \nabla M(S_i(\xi)) \frac{\partial S_i(\xi)}{\partial \xi} \right]^T \left[ \nabla M(S_i(\xi)) \frac{\partial S_i(\xi)}{\partial \xi} \right]$$
- **No Wheel/Rotor Odometry Dependency**: No motion prediction model is strictly required from wheel encoders or prop RPMs.

---

## 3. Empirical Test Results in ASTRA AirMouse

### 3.1 Hardware Benchmarking (Phase 2 Master Run `20260918_024500`)
- **LiDAR Sensor**: Slamtec RPLiDAR A2 connected via Silicon Labs CP2102 UART bridge (`/dev/ttyUSB0` @ 115200 baud).
- **Update Frequency**: $4.87\text{ Hz} - 10.0\text{ Hz}$ scan rate.
- **Optimization Latency**: Mean **$6.82\text{ ms}$** per scan match on an Intel Core i5/i7 host.
- **Match Score**: Normalized correlation score **$0.89$**.
- **Static Drift**: When held static on a test bench for 5 minutes, translation drift was under **$0.02\text{ m}$** with zero NaN/Inf occurrences.
- **2D Occupancy Grid**: Clean $400 \times 400$ grid ($0.05\text{ m}$ resolution) successfully separating free cells ($2,051$) and wall obstacles ($355$).

### 3.2 Gazebo Simulation In-Flight Test (`20261002_112000`)
During full mission autonomous flight integration inside Gazebo simulation:
- **Free Space Expansion**: Expanded from $7,192\text{ cells}$ ($17.98\text{ m}^2$) at takeoff to **$29,254\text{ cells}$ ($73.14\text{ m}^2$)** and $662\text{ wall cells}$ with $0.05\text{ m}$ cell resolution.
- **Corridor Degeneracy**: Observed $+0.75\text{ m}$ longitudinal slip along symmetric hallway axis ($Y_H = +1.85\text{ m}$ vs. $Y_{\text{px4}} = +1.10\text{ m}$) due to unconstrained scan-matching gradient.
- **High Yaw Rotations ($> 60^\circ/\text{s}$)**: The Gauss-Newton solver lost track, causing estimated heading to lag physical heading by up to $25^\circ$.
- **Dual Odometry Conflict Resolution**: By remapping Hector SLAM to `/hector_slam/*` and keeping FAST-LIO on `/Fast_LIO/odometry` $\to$ EKF2, zero flight instability or wall collisions occurred.

---

## 4. Advantages of Hector SLAM

1. **Zero Odometry Prerequisite**:
   Can map entirely without wheel encoders, optical flow, or motor telemetry.
2. **Minimal Computational Footprint**:
   Low CPU utilization ($\approx 15-20\%$ of a single core). Suitable for resource-constrained companion computers (Raspberry Pi 4, Jetson Nano).
3. **Low Latency & High Rate**:
   Gauss-Newton iterations converge within $3-8\text{ ms}$, delivering instantaneous 2D pose updates.
4. **Crisp Short-Range Metric Maps**:
   Bilinear interpolation produces smooth, sharp occupancy walls without the "fuzziness" of early filter iterations.
5. **Simple ROS Integration**:
   Minimal TF dependencies (`map -> odom -> base_link` or direct `map -> base_link`).

---

## 5. Disadvantages & Limitations for UAV Autonomous Exploration

1. **Lack of Loop Closure**:
   Hector SLAM has no pose-graph optimization or appearance-based loop closure. In large circular loops ($> 30\text{ m}$ perimeter), accumulated drift causes noticeable double-wall artifacts upon loop closure.
2. **Vulnerability to Long Corridors (Degeneracy)**:
   In straight, featureless hallways with parallel walls, the Hessian matrix becomes ill-conditioned along the hallway axis, resulting in unbounded longitudinal drift.
3. **Severe Sensitivity to Drone Roll & Pitch (3D Tilt)**:
   A standard 2D planar LiDAR tilted by quadrotor attitude maneuvers slices the ground plane or flies above short obstacles.
4. **No Direct Velocity Output**:
   Only outputs `geometry_msgs/PoseStamped`, meaning linear and angular velocities required by flight controllers (PX4 EKF2 / Trajectory Trackers) must be numerically differentiated, introducing noise.
5. **Lack of 3D Volumetric Mapping**:
   Cannot detect low obstacles (tables, wires) or overhangs, making it inadequate on its own for full 3D collision avoidance (such as the FUEL 3D B-spline planner).

---

## 6. Verdict and Role in ASTRA AirMouse

- **For Primary Flight Control**: Displaced in favor of **FAST-LIO2** (3D LiDAR + 200 Hz IMU) to ensure 6-DOF collision avoidance and rock-solid EKF2 tracking.
- **For Secondary Payloads**: Retained as an ultra-lightweight 2D situational mapping node and bench diagnostic tool for 2D RPLiDAR verification.
