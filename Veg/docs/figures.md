
# Figures and Diagrams – Veg Drone

## 1. veg_drone_architecture.png

A block-level architecture diagram of the drone showing:
- SLAM stack (ORB-SLAM3, RTAB-Map)
- Control stack (Arduino LQR/PID)
- Vision pipeline (Object detection + PCA face recognition)
- Communication with ground station

## 2. control_response_plot.png

A comparative step-response plot showing:
- PID (sluggish, stable)
- FBL + PD (moderate, slight overshoot)
- LQR (sharp response, minimal overshoot)

Generated from MATLAB Simulink simulations.

## 3. demo_frame.png

An annotated frame from the drone’s onboard camera feed during a test:
- Detected object bounding boxes
- Face recognition label
- Timestamp and orientation overlay
