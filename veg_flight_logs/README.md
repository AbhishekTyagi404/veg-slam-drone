
# Veg Drone Flight Logs

This dataset contains telemetry, mapping, and performance benchmarks from Veg — a SLAM-enabled quadcopter with onboard vision and fault resilience.

---

## 📂 Flight Log Contents

| File                          | Description                                                                 |
|-------------------------------|-----------------------------------------------------------------------------|
| `veg_mission_01.csv`          | Full telemetry log (timestamp, GPS, altitude, orientation, motor PWMs)      |
| `veg_mission_01_plot.png`     | SLAM-estimated drone trajectory during autonomous navigation                |
| `slam_trajectory.json`        | ORB-SLAM3 pose log with timestamped 6-DoF motion data                       |
| `emergency_landing_stats.csv` | Fault-triggered landing log: deviation, landing site, and success status    |
| `cpu_load_plot.png`           | CPU usage profile during concurrent SLAM and object detection              |

---

## 🔍 Visual Previews

### 🛰️ SLAM Trajectory Output (`veg_mission_01_plot.png`)

![SLAM Trajectory](https://github.com/AbhishekTyagi404/veg-slam-drone/blob/main/veg_flight_logs/veg_mission_01_plot.png)

*Autonomous path reconstruction using ORB-SLAM3 visual-inertial odometry. The plot overlays real-time estimated positions during a maze navigation test.*

---

### 💻 CPU Load During Mission (`cpu_load_plot.png`)

![CPU Load Plot](https://github.com/AbhishekTyagi404/veg-slam-drone/blob/main/veg_flight_logs/cpu_load_plot.png)

*Resource usage graph from Raspberry Pi 4 showing system load while running SLAM, LQR control, and object detection concurrently.*

---

## 📘 How to Use

1. Clone or download the repository:
   ```bash
   git clone https://github.com/AbhishekTyagi404/veg-slam-drone.git
   cd veg-slam-drone/veg_flight_logs
   ```

2. Open `veg_mission_01.csv` to analyze raw telemetry:
   - Includes: timestamp, latitude, longitude, altitude, roll, pitch, yaw, and PWM signals for all four rotors.

3. Use `slam_trajectory.json` to visualize 3D trajectory or validate pose estimation.

4. Parse `emergency_landing_stats.csv` to evaluate deviation from planned trajectory during rotor failure event.

---

## 🔧 Configuration Notes

- ORB-SLAM3 in Visual-Inertial Monocular Mode
- LQR used for roll/pitch stabilization, PID for yaw and altitude
- 60 seconds of simulated indoor maze navigation
- Rotor failure at 30s with automatic safe landing reroute

---

Maintained by [Abhishek Tyagi](https://github.com/AbhishekTyagi404) · Part of [Kritrim Intelligence](https://kritrimintelligence.com) Research Logs
