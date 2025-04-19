
# 🛸 Veg SLAM Drone - MATLAB Simulation

This repository contains all Simulink model generators, control scripts, and benchmark logs for the **Veg surveillance quadcopter** project. The drone features SLAM-based autonomous navigation, fault-resilient control, onboard computer vision, and real-time simulation tools.

---

## 📂 File Structure

### ✅ Control System Generators

| File | Description |
|------|-------------|
| `generate_quadplant_model.m` | Builds `quadplant.slx`: 12-state Newton-Euler quadcopter dynamics model |
| `generate_attitude_controller.m` | Builds `attitude_controller.slx`: LQR + PD cascaded controller |
| `generate_trajectory_follower.m` | Builds `trajectory_follower.slx`: Converts XY error to φ, θ setpoints |
| `pathcr.m` | Maze-based trajectory planner using Dijkstra's algorithm |

### 🧠 Model Parameters

- `veg_plant_parameters.mat`: Mass, inertia, rotor constants, and drag coefficients used across models.

---

### 📊 Benchmarking Logs

| File | Description |
|------|-------------|
| `veg_mission_01.csv` | Flight log with trajectory and velocity for a 30s mission |
| `emergency_landing_stats.csv` | Real-time CPU load stats on Raspberry Pi 4 during onboard inference |

---

### 🖼️ Visualizations (Plots)

| Image | Description |
|-------|-------------|
| `veg_mission_01_plot.png` | Drone trajectory in X-Y space ![XY Path](https://github.com/AbhishekTyagi404/veg-slam-drone/blob/main/veg-slam-drone-sim/veg_mission_01_plot.png) |
| `cpu_load_plot.png` | CPU Load during vision tasks ![CPU Load](https://github.com/AbhishekTyagi404/veg-slam-drone/blob/main/veg-slam-drone-sim/cpu_load_plot.png) |
| `slam_vs_gt_plot.png` | SLAM vs Ground Truth ![SLAM vs GT](https://github.com/AbhishekTyagi404/veg-slam-drone/blob/main/veg-slam-drone-sim/slam_vs_gt_plot.png) |

---

### 📌 How to Use

1. Open MATLAB.
2. Load parameters:
   ```matlab
   load('veg_plant_parameters.mat');
   ```
3. Run generation scripts:
   ```matlab
   generate_quadplant_model
   generate_attitude_controller
   generate_trajectory_follower
   ```
4. Use `pathcr.m` to generate XY waypoints for SLAM or manual mode.

---

**Author**: Abhishek Tyagi  
**Version**: 2.3  
**GitHub**: [AbhishekTyagi404](https://github.com/AbhishekTyagi404)
