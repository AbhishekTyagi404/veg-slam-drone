
# Veg Drone Flight Logs

This dataset supports the experiments presented in the paper:

### *SLAM-Based Navigation and Fault Resilience in a Surveillance Quadcopter with Embedded Vision Systems*

---

## 📂 Folder Contents

| File                          | Description                                                                 |
|-------------------------------|-----------------------------------------------------------------------------|
| `veg_mission_01.csv`          | Telemetry log with timestamps, GPS, orientation (roll, pitch, yaw), motor PWMs |
| `veg_mission_01_plot.png`     | SLAM-based trajectory of Veg during autonomous maze navigation [🔗 View Image](https://github.com/AbhishekTyagi404/veg-slam-drone/blob/main/veg_flight_logs/veg_mission_01_plot.png) |
| `slam_trajectory.json`        | ORB-SLAM3 estimated poses with 6-DoF pose history                         |
| `emergency_landing_stats.csv` | Logs rotor-failure event trigger, deviation from original path, success metrics |
| `cpu_load_plot.png`           | CPU utilization during full SLAM + vision mission [🔗 View Image](https://github.com/AbhishekTyagi404/veg-slam-drone/blob/main/veg_flight_logs/cpu_load_plot.png) |

---

## 📌 Instructions for Use

- Clone or download the repository:
  ```bash
  git clone https://github.com/AbhishekTyagi404/veg-slam-drone.git
  cd veg-slam-drone/veg_flight_logs
  ```

- The `.csv` and `.json` files are ready for parsing with Python, MATLAB, or any analysis tool.
- `veg_mission_01.csv` columns include:
  ```
  [time_s, latitude, longitude, altitude_m, roll_deg, pitch_deg, yaw_deg, motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm]
  ```

- For visual verification:
  - Use `veg_mission_01_plot.png` to observe planned vs actual SLAM path.
  - Use `cpu_load_plot.png` to validate Raspberry Pi 4 performance during simultaneous navigation and onboard inference.

---

## 🔧 Reproducibility

These logs correspond to:

- 60-second simulated flight in a grid maze
- ORB-SLAM3 running on Raspberry Pi 4 in monocular-inertial mode
- LQR controller active during roll and pitch attitude stabilization
- Emergency landing triggered by rotor failure at t = 30 s

---

## 📜 Citation

If you use this dataset, please cite the following:

```
Abhishek Tyagi. "SLAM-Based Navigation and Fault Resilience in a Surveillance Quadcopter with Embedded Vision Systems." arXiv, 2025.
```

---

Maintained by [Abhishek Tyagi](https://github.com/AbhishekTyagi404) • Powered by Kritrim Intelligence
