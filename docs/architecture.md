
# वेग Drone – System Architecture

The surveillance quadcopter drone **वेग (Veg)** integrates multiple subsystems to support autonomous navigation, fault-tolerant control, and onboard intelligence. Here's a high-level breakdown of the system architecture:

## 1. Flight Control

- **Inner Loop (100 Hz):**  
  LQR or PID running on Arduino for fast attitude stabilization using IMU feedback.
  
- **Outer Loop (10 Hz):**  
  Trajectory controller on Raspberry Pi computes desired roll/pitch/yaw/throttle and streams them via UART.

## 2. Navigation

- **SLAM-based Odometry:**  
  - ORB-SLAM3 for real-time monocular-inertial visual SLAM
  - RTAB-Map (offline or lightweight real-time) for dense 3D reconstruction
  
- **Planner:**  
  Graph-based path planner (Dijkstra or A*) generates waypoint trajectories based on SLAM map

## 3. Vision

- **Object Detection:**  
  Lightweight YOLO model (via OpenCV DNN) detects persons, vehicles from Pi camera
  
- **Face Recognition:**  
  PCA-based recognition engine matches detected faces against a stored database

## 4. Fault Detection

- Onboard FDI module monitors motor outputs, orientation
- If fault is detected (e.g., rotor loss), trajectory is redirected to safe landing

## 5. Communication

- Serial UART: Pi ↔ Arduino (setpoints + telemetry)
- Wi-Fi or LTE: Ground station VNC access and SLAM monitoring

---

Refer to the accompanying system diagram in `docs/images/veg_drone_architecture.png`.
