# Top 10 2D SLAM Algorithms for Autonomous Robotic & UAV Missions

## 1. Overview
In GPS-denied environments (indoor mazes, collapsed structures, subterranean tunnels), selecting the optimal 2D SLAM algorithm is essential for state estimation, mapping accuracy, and real-time path planning.

This report evaluates and compares the **Top 10 2D SLAM algorithms** for our **ASTRA AirMouse / NIDAR** quadrotor and ground-support systems.

---

## 2. In-Depth Analysis of the Top 10 2D SLAM Algorithms

### 1. Hector SLAM (`hector_mapping`)
- **Category**: Filterless / Direct Scan-to-Map Optimization.
- **Backend Optimization**: Multi-resolution Gauss-Newton bilinear interpolation.
- **Odometry Dependency**: **None** (Can operate with 100% pure 2D LiDAR).
- **Loop Closure**: No explicit global loop closure or pose graph.
- **Compute Overhead**: Very Low (~15% CPU on single core).
- **Pros**: Zero odometry requirement; ultra-low CPU footprint; fast convergence ($<7\text{ ms}$).
- **Cons**: High drift in long featureless corridors; failure on rapid yaw turns; sensitive to quadrotor roll/pitch tilt.
- **Suitability for ASTRA**: Verified on RPLiDAR A2; excellent for rapid bench diagnostics and auxiliary 2D occupancy mapping.

---

### 2. Google Cartographer 2D (`cartographer_ros`)
- **Category**: Graph-Based SLAM (Submap Scan-Matching + Global Optimization).
- **Backend Optimization**: Ceres Solver non-linear optimization with branch-and-bound loop closure.
- **Odometry Dependency**: Optional (supports pure LiDAR, LiDAR + IMU, or LiDAR + Odometry + IMU).
- **Loop Closure**: Yes, continuous multi-resolution submap matching with Ceres pose graph.
- **Compute Overhead**: High (Multi-threaded Ceres solver; recommended 4+ cores).
- **Pros**: Industry gold standard in global map consistency; robust loop closures; natively fuses high-rate IMU orientations to compensate for UAV pitch/roll.
- **Cons**: High memory and CPU usage; complex configuration (hundreds of Lua parameters); latency spikes during large loop closures if unconstrained.
- **Suitability for ASTRA**: **Top Contender (Rank 1)** for full arena mapping when running on powerful host or onboard Jetson.

---

### 3. OpenKarto SLAM (`slam_karto`)
- **Category**: Graph-Based SLAM.
- **Backend Optimization**: Sparse Pose Adjustment (SPA) using `sba` / Cholesky decomposition or Ceres / g2o.
- **Odometry Dependency**: Strongly recommended (relies on odometry prior for initial scan registration).
- **Loop Closure**: Yes, scan-correlation over historical graph nodes.
- **Compute Overhead**: Moderate (~30-40% single core).
- **Pros**: Highly efficient linear graph solver; memory footprint scales with trajectory length rather than map area; very stable in office/maze layouts.
- **Cons**: Requires reliable odometry (pure laser mode can diverge if scan rate is low); loop closure detection can be conservative.
- **Suitability for ASTRA**: Strong candidate when paired with PX4 EKF2 or optical flow odometry.

---

### 4. Gmapping (`gmapping` / OpenSLAM Gmapping)
- **Category**: Particle Filter (Rao-Blackwellized Particle Filter - RBPF).
- **Backend Optimization**: Importance sampling particle filter (each particle maintains its own grid map).
- **Odometry Dependency**: **Mandatory** (requires continuous wheel/inertial odometry).
- **Loop Closure**: Implicit via particle weight resample and dispersion.
- **Compute Overhead**: Moderate to High (proportional to particle count $N$, typically $N=30-80$, and map resolution).
- **Pros**: Time-tested classic; excellent particle diversity in complex non-linear motions; crisp maps in small-to-medium areas.
- **Cons**: High memory usage (each particle copies the map); cannot recover if the true hypothesis drops out of all particles; particle depletion in large-scale maps.
- **Suitability for ASTRA**: Moderate; feasible only if PX4 EKF2 odometry is provided as the base frame.

---

### 5. SLAM Toolbox (`slam_toolbox` by Steve Macenski)
- **Category**: Modern Graph-Based SLAM (Karto derivative with Ceres backend).
- **Backend Optimization**: Ceres solver; asynchronous and synchronous modes; lifelong mapping capability.
- **Odometry Dependency**: Recommended, but features robust laser-scan matcher fallbacks.
- **Loop Closure**: Yes, multi-threaded scan-matcher-driven loop detection with interactive map merging and serialization.
- **Compute Overhead**: Moderate (optimized modern C++ with multithreading).
- **Pros**: Active maintenance; ROS 1 & ROS 2 native; dynamic map expansion, map saving/loading, and interactive loop manual adjustment; low memory usage.
- **Cons**: Needs careful tuning of search windows in fast dynamic flight.
- **Suitability for ASTRA**: **Top Contender (Rank 2)**; best modern drop-in replacement for Karto and Gmapping.

---

### 6. CoreSLAM (`coreslam` / tinySLAM)
- **Category**: Minimalist Particle Filter / Heuristic Scan Matcher.
- **Backend Optimization**: Simplified Monte-Carlo particle evaluation (< 200 lines of core math).
- **Odometry Dependency**: Optional.
- **Loop Closure**: No.
- **Compute Overhead**: Extremely Low (< 5% CPU).
- **Pros**: Runs on microcontrollers (STM32, ESP32) and bare minimal single-board computers; zero external heavy dependencies.
- **Cons**: Low map fidelity; easily drifts over medium-to-large distances; no recovery once lost.
- **Suitability for ASTRA**: Too simplistic for complex multi-room competition arenas, but valuable as a ultra-low-power failsafe.

---

### 7. FastSLAM 2.0
- **Category**: Feature-Based Rao-Blackwellized Particle Filter.
- **Backend Optimization**: Extended Kalman Filters (EKF) per landmark conditioned on particle trajectory.
- **Odometry Dependency**: Mandatory.
- **Loop Closure**: Implicit via landmark association.
- **Compute Overhead**: Moderate ($O(M \log K)$ where $M$ is particles and $K$ is landmark count).
- **Pros**: Incorporates current laser measurement into the proposal distribution (much lower particle count needed than FastSLAM 1.0).
- **Cons**: Requires geometric feature extraction (corners, line segments, poles); struggles in unstructured or smooth curved walls.
- **Suitability for ASTRA**: Moderate; our arena features orthogonal flat walls which suit line-feature FastSLAM, but modern dense grid approaches are generally superior.

---

### 8. IrisSLAM (`iris_lama` / LaMA - Large Map SLAM)
- **Category**: 2D Scan Matching with Distance Transform / Submap Graph.
- **Backend Optimization**: Exact Euclidean Distance Transform (EDT) optimization + pose graph backend.
- **Odometry Dependency**: Optional (supports pure LiDAR scan-matching).
- **Loop Closure**: Fast multi-scale submap matching.
- **Compute Overhead**: Low to Moderate (extremely fast distance transform lookup).
- **Pros**: Exceptional speed; up to 10x faster than Cartographer with comparable map quality; very low memory footprint.
- **Cons**: Less widespread adoption in ROS community; fewer debugging and tuning tutorials.
- **Suitability for ASTRA**: **Top Contender (Rank 3)** for high-speed exploration on embedded hardware.

---

### 9. SegMap 2D / 3D
- **Category**: Segment-Based Graph SLAM.
- **Backend Optimization**: 3D/2D pointcloud segmentation + neural network / geometric descriptor feature matching.
- **Odometry Dependency**: Recommended.
- **Loop Closure**: Segment descriptor retrieval and geometric verification.
- **Compute Overhead**: High (requires feature extraction and neural descriptor matching).
- **Pros**: Viewpoint-invariant loop closures; semantic understanding of objects in the scene.
- **Cons**: Significant compute requirement; typically requires GPU for fast descriptor inference.
- **Suitability for ASTRA**: Overkill for simple 2D maze mapping; better suited for 3D outdoor semantic navigation.

---

### 10. MRPT-SLAM (`mrpt_rbpf_slam` / `mrpt_graph_slam_2d`)
- **Category**: Multi-Algorithm Suite (Mobile Robot Programming Toolkit - RBPF and Graph-SLAM).
- **Backend Optimization**: Levenberg-Marquardt pose-graph optimization (`mrpt-graphslam`).
- **Odometry Dependency**: Flexible.
- **Loop Closure**: Yes, robust Mahalanobis distance metric and topological node closure.
- **Compute Overhead**: Moderate.
- **Pros**: Mature scientific library; comprehensive probabilistic motion models; supports multi-metric maps (grid + point + landmark).
- **Cons**: Steeper learning curve; non-standard ROS message wrapping compared to pure ROS packages.
- **Suitability for ASTRA**: Solid academic benchmarking tool.

---

## 3. Quantitative Comparison Matrix

| # | Algorithm | Backend Type | Requires Odometry? | Loop Closure? | CPU Load | Drift Resistance | Drone Roll/Pitch Tolerance | Best Use Case |
|---|---|---|:---:|:---:|:---:|:---:|:---:|---|
| **1** | **Hector SLAM** | Gauss-Newton Scan-to-Map | ❌ No | ❌ No | **Very Low** | Moderate | Low (Requires flat flight) | Bench check, quick setup, zero-odom |
| **2** | **Cartographer** | Submap + Ceres Graph | ❌ Optional (Best with IMU) |  Yes | High | **Very High** | **High** (IMU 3D tilt tracking) | Final mission mapping, multi-room loops |
| **3** | **SLAM Toolbox** | Ceres Graph-Based (Karto) | ⚠️ Preferred |  Yes | Moderate | **High** | Moderate | ROS standard, large arenas, dynamic maps |
| **4** | **OpenKarto** | Sparse Pose Adjustment |  Yes |  Yes | Moderate | Moderate-High | Low | Classic ground robots with wheel odom |
| **5** | **Gmapping** | Rao-Blackwellized Filter |  Yes |  Implicit | Moderate-High | Moderate | Low (Particles diverge on tilt) | Classic differential drive robots |
| **6** | **Iris LaMA** | Distance Transform Graph | ❌ Optional |  Yes | **Low** | **High** | Moderate | High-speed mapping on low-power SBCs |
| **7** | **CoreSLAM** | Minimalist Monte-Carlo | ❌ No | ❌ No | **Ultra Low** | Low | Very Low | Microcontrollers, emergency fallback |
| **8** | **FastSLAM 2.0** | Feature EKF Filter |  Yes |  Implicit | Moderate | Moderate | Low | Feature-rich environments (corners/poles) |
| **9** | **MRPT Graph** | Levenberg-Marquardt Graph | ⚠️ Preferred |  Yes | Moderate | High | Moderate | Research and multi-metric benchmarking |
| **10**| **SegMap 2D** | Segment Descriptors + Graph|  Yes |  Yes | Very High (GPU) | **Very High** | Moderate | Long-term lifelong semantic SLAM |

---

## 4. Selection Verdict for ASTRA AirMouse / NIDAR

For our autonomous UAV drone navigating the competition arena:

1. **First Choice for 2D High-Accuracy Mapping**: **Google Cartographer 2D**
   - *Reasoning*: Can fuse the 250 Hz Pixhawk IMU to continuously compensate for quadrotor roll and pitch tilt during forward flight, eliminating tilted-scan distortion. Unsurpassed loop closure ensures multi-room walls stay razor sharp.
2. **Second Choice for Embedded Efficiency**: **SLAM Toolbox** (Lifelong/Async Mode)
   - *Reasoning*: Modern C++ graph optimization, multi-threaded loop closure, lower CPU overhead than Cartographer, seamlessly integrates with ROS move_base and costmaps.
3. **Third Choice for High-Speed / Low-Compute**: **Iris LaMA (`iris_lama`)**
   - *Reasoning*: Fastest 2D scan-matcher available, operates with or without odometry, ideal if onboard CPU is saturated by survivor YOLO detection and FUEL path planning.
4. **Current Baseline**: **Hector SLAM**
   - *Reasoning*: Already fully implemented and verified in `hardware/phase2/` and `hardware/launch/`. Retained as reference baseline and 0-odometry diagnostic tool.
