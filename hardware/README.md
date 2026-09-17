# ASTRA Hardware Verification Checklist

Complete phase-by-phase hardware verification guide for the **ASTRA AirMouse** autonomous drone.

---

## The 1 Golden Rule

> **Do not move to the next phase until the current phase passes on real hardware.**

```text
INPUT ──► INTERFACE ──► HARDWARE RESPONSE ──► TELEMETRY CONFIRMATION ──► PASS / FAIL
```

---

## Phase 1: Aircraft Bringup (Bench / Props OFF)

### 1.1 System & Environment
- [x] Companion computer OS running (Ubuntu / Linux)
- [x] ROS environment active (Humble / Noetic)
- [x] Python dependencies present (`rospy`/`rclpy`, `mavros`, `numpy`, `scipy`, `yaml`, `cv2`)
- [x] Serial ports accessible (`/dev/ttyUSB0` for RPLiDAR A2, `/dev/ttyACM0` for Pixhawk FCU) with dialout permissions
- [x] LiDAR serial communication verified at 115200 baud

### 1.2 PX4 Flight Controller & MAVROS
- [x] FCU boots with normal status LED and PyMAVLink serial handshake
- [x] Telemetry reports `connected: true` and stable heartbeat at 921600 baud (`/dev/ttyACM0`)
- [x] Critical parameters match configuration:
  - `EKF2_EV_CTRL = 11` (external vision position + yaw fusion)
  - `EKF2_HGT_REF = 2` (rangefinder primary height)
  - `MPC_THR_HOVER` matched to physical vehicle mass
- [x] Arm and Disarm command structures verified

### 1.3 Actuators & Motors (Props OFF!)
- [x] Motor numbering (1, 2, 3, 4) matches physical Quad X frame geometry in software
- [ ] Motor rotation directions physically verified on spinning motors *(Pending Flight Battery)*
- [ ] ESC response physically tested from idle to full range *(Pending Flight Battery)*

### 1.4 Sensors
- [x] **IMU:** Pixhawk built-in InvenSense IMU registers stationary earth gravity ($Z = -9.937\text{ m/s}^2 \approx 1\text{G}$)
- [x] **Barometer:** Stable barometric pressure reading on bench ($98,173\text{ mbar}$)
- [x] **GPS:** U-Blox NEO M8N external module locked ($13\text{ satellites}$, 3D Fix)
- [x] **LiDAR:** Slamtec RPLiDAR A2 streams 360° points at $468.5\text{ Hz}$ sample rate, distances $0.13\text{ m} - 4.65\text{ m}$

### 1.5 TF & Coordinate Frames
- [x] Valid TF tree definitions: `world` $\to$ `camera_init` $\to$ `base_link` $\to$ `laser`, `camera`
- [x] Right-hand FLU/ENU coordinate frame transforms verified

---

## Phase 2: Autonomy & Flight Control

### 2.1 Hector SLAM 2D Localization (RPLiDAR A2)
- [x] Scan matching optimization latency averages $6.8\text{ – }7.8\text{ ms / sweep}$ (real-time budget: $< 50\text{ ms}$)
- [x] Zero NaN/Inf singularities across consecutive 360° laser sweeps
- [x] Scan matching alignment score reaches $0.89\text{ – }0.91$ ($89\text{ – }91\%$ spatial correlation)
- [x] Continuous trajectory displacement tracking verified on bench

### 2.2 Odometry Relay & PX4 EKF
- [x] `check_odometry_relay.py` interfaces with Pixhawk built-in IMU and PX4 EKF2 onboard estimator
- [x] Hector SLAM planar pose ($X, Y, \text{yaw}$) formats into MAVLink `VISION_POSITION_ESTIMATE`
- [x] Origin-anchoring and jump-rejection filtering verified in software

### 2.3 Flight Envelope Guard
- [x] In-envelope setpoint commands accepted and forwarded
- [x] Out-of-bounds commands rejected and clamped to boundary margins
- [x] Emergency HOLD pose streaming logic verified

### 2.4 Position Setpoint Control
- [x] Offboard setpoint generation code verified
- [ ] In-flight step command steady-state tracking *(Pending Flight Battery & Arena)*
- [ ] In-flight altitude hold stability *(Pending Flight Battery & Arena)*

### 2.5 Progressive FUEL Exploration
- [x] 2D occupancy grid frontier selection and coverage rate calculations verified
- [ ] In-flight multi-room exploration coverage *(Pending Flight Battery & Arena)*

---

## Phase 3: Perception, Mapping & GCS

### 3.1 Camera & Vision Pipeline
- [x] Host camera (`/dev/video0`) streams live video frames at 640x480 resolution
- [x] Video pipeline throughput sustained at $8.8\text{ – }15.4\text{ FPS}$ with latency $65\text{ – }113\text{ ms}$
- [x] Grayscale and edge detection pipeline verified under bench optical illumination

### 3.2 3D Survivor Localization
- [x] Pinhole optical backprojection algorithm verified
- [x] Live fusion of camera optical center with real-time RPLiDAR depth ($0.83\text{ m}$) computes 3D target coordinates $(+0.00, +0.00, +0.83\text{ m})$

### 3.3 Discrete Grid Tagging
- [x] 3D coordinates map correctly to Discrete Arena Grid format (A1–N14)
- [x] Real-time Hector SLAM pose $(-0.09\text{ m}, -0.08\text{ m})$ dynamically tagged to competition `Cell D7`

### 3.4 2D Occupancy Grid Mapping
- [x] Continuous 2D occupancy grid built live from RPLiDAR A2 sweeps on `/dev/ttyUSB0`
- [x] $20.0\text{ m} \times 20.0\text{ m}$ fine resolution grid ($0.05\text{ m/cell}$, $400\times 400$ matrix) mapped with 355 obstacle cells and 2,051 free cells

### 3.5 Ground Control Station (GCS)
- [x] Telemetry serialization schema conforms to GCS packet format
- [x] Real-time telemetry logging callback verified

---

## Phase 4: Full Mission & Failsafes

### 4.1 Mission State Machine
- [x] State transition ordering verified:
  $$\text{PREFLIGHT} \to \text{TAKEOFF} \to \text{ENTRY} \to \text{EXPLORE} \to \text{RETURN} \to \text{LAND}$$
- [x] Illegal reverse state transitions blocked by validation logic

### 4.2 Failsafe Injections (Live FCU & Software Audited)
- [x] **Low Battery:** Live Pixhawk ADC voltage read ($3.09\text{ V}$ on USB rail); triggers return logic on $V < 14.2\text{ V}$
- [x] **Link Loss:** Telemetry watchdog timeout triggers autonomous return
- [x] **Companion Software Halt:** PX4 offboard failsafe verified
- [x] **Manual Abort:** Single switch triggers immediate landing sequence
- [x] **30-Minute Timeout:** Mission clock budget estimator orders return before timeout

### 4.3 Full Integrated Competition Run
- [x] Scoring rubric checklist and full mission orchestration pipeline verified in software
- [ ] End-to-end physical flight run *(Deferred pending flight battery, motors, and arena)*
