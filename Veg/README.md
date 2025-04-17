
# Surveillance Quadcopter Drone – वेग (Veg)

**वेग** (meaning "speed" in Hindi) is a feature-rich surveillance quadcopter drone developed as a major B.Tech project and later upgraded with advanced autonomy, fault-tolerant control, and real-time computer vision.

## 🚀 Highlights

- **SLAM-Based Autonomous Navigation**  
  Uses ORB-SLAM3 for real-time 6-DoF pose estimation and RTAB-Map for post-mission dense 3D reconstruction.

- **Cascaded Control Architecture**  
  - Inner-loop: Linear Quadratic Regulator (LQR) for attitude stabilization  
  - Outer-loop: Proportional-Derivative (PD) control for trajectory tracking  

- **Fault Detection and Tolerant Control**  
  Real-time FDI module monitors rotor health. In case of failure, it switches to an emergency landing protocol.

- **Onboard Vision**  
  - Deep-learning-based object detection (~2 FPS on Raspberry Pi)  
  - PCA-based face recognition (>94% accuracy on standard datasets)

- **Modular ROS Architecture**  
  SLAM, control, perception, and decision modules are developed as ROS nodes. Runs fully onboard Raspberry Pi 4 with real-time response handled by a microcontroller.

- **Simulated + Real-World Testing**  
  MATLAB/Simulink models were used to simulate dynamics, compare control strategies (PID, FBL+PD, LQR), and plan trajectories in a VR environment.

---

## 📂 Repository Structure

```bash
veg-drone/
├── README.md                # This file
├── LICENSE                  # License info
├── docs/                    # Diagrams, images, paper references
│   └── images/
├── hardware/                # Frame designs, BOM, motor specs
│   ├── frame_design/
│   └── motor_specs/
├── firmware/                # Arduino attitude control (LQR, PID, FDI)
│   └── arduino/
├── software/                # ROS packages and vision modules
│   ├── slam/                # ORB-SLAM3, RTAB-Map configs
│   ├── control/             # LQR, PD, trajectory controllers
│   ├── vision/              # Object detection and face recognition
│   └── fdi/                 # Failure detection logic
├── simulations/             # MATLAB and VR models
│   └── matlab/
├── datasets/                # Image datasets for detection and face DB
│   ├── training_images/
│   └── face_db/
├── results/                 # Plots, response graphs, comparison tables
│   └── plots/
└── config/                  # YAML/launch files and controller params
    └── params.yaml
```

---

## 📷 Demo

> Autonomous navigation and emergency landing demo (add .gif in `docs/images/veg_drone_demo.gif`)

<div align="center">
  <img src="docs/images/veg_drone_demo.gif" alt="Veg drone in simulation and real test" width="600">
</div>

---

## 🛠️ Requirements

- **Hardware**
  - Raspberry Pi 4 (4GB)
  - Arduino Nano / Uno
  - Pi Camera (5MP)
  - 9-DoF IMU (MPU-9250 / BNO055)
  - BLDC motors + ESCs (30A)
  - Power distribution board + LiPo battery
  - Optional: GPS, Lidar, Wi-Fi/4G module

- **Software Stack**
  - Ubuntu 20.04 + ROS Noetic
  - Python 3.7+, OpenCV, TensorFlow Lite
  - ORB-SLAM3, RTAB-Map
  - MATLAB R2020a + Simulink VR Toolbox (for simulation)

---

## 📈 Control Performance (Simulation)

| Controller     | Rise Time | Overshoot | Settling Time |
|----------------|-----------|-----------|----------------|
| PID            | 2.8 s     | ~0%       | 3.0 s          |
| FBL + PD       | 1.1 s     | 5.8%      | 1.5 s          |
| LQR (proposed) | 0.06 s    | ~0.5%     | 0.1 s          |

---

## 📜 License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details.

---

## 🔗 References

- ORB-SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3  
- RTAB-Map: https://github.com/introlab/rtabmap  
- PID vs LQR Comparison – Simulink: `/simulations/matlab/control_response.mdl`  
- Darknet Object Detection on Pi: `/software/vision/yolo_pi_detect.py`
