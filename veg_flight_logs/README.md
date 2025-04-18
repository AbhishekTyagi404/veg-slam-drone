
# Veg Drone Flight Logs

This repository contains logged data and visual outputs from a simulated mission of the **Veg** UAV platform, featuring SLAM-based navigation, onboard vision, and fault resilience.

## 📁 Contents

| File                          | Description                                                                 |
|-------------------------------|-----------------------------------------------------------------------------|
| `veg_mission_01.csv`          | Time-stamped telemetry data: GPS, orientation, and PWM signals.            |
| `veg_mission_01_plot.png`     | 2D plot of the UAV trajectory captured via SLAM and visual odometry.       |
| `slam_trajectory.json`        | ORB-SLAM3 estimated pose data in JSON format.                              |
| `emergency_landing_stats.csv` | Stats on fault-triggered emergency landing: deviation and landing success. |
| `cpu_load_plot.png`           | Graph showing CPU usage during full onboard mission run.                   |

## 🛸 Usage

These logs were generated as part of the experimental evaluation of the research paper:

**Title**: *SLAM-Based Navigation and Fault Resilience in a Surveillance Quadcopter with Embedded Vision Systems*

Use these datasets and plots to:

- Validate SLAM trajectory estimation
- Reproduce controller benchmarking and fault handling
- Evaluate CPU performance under vision + SLAM workloads

## 📜 Citation

If you use this data in your work, please cite:

```
Abhishek Tyagi, "SLAM-Based Navigation and Fault Resilience in a Surveillance Quadcopter with Embedded Vision Systems", arXiv, 2025.
```

## 🧠 Notes

- All files were generated synthetically to simulate onboard processes from Veg drone.
- Actual hardware-in-loop or real drone trials may yield slightly varied data distributions.

---

Maintained by [Kritrim Intelligence](https://github.com/AbhishekTyagi404)
