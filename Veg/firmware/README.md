
# वेग Drone Firmware

This folder contains the Arduino-based firmware for the **वेग (Veg)** Surveillance Quadcopter. The firmware is written to support both PID and LQR control strategies, along with a modular structure for Fault Detection and Identification (FDI).

---

## 🔧 Files Overview

- `veg_attitude_controller_lqr.ino`  
  Inner-loop LQR controller for stabilizing roll and pitch using a linearized model. Suitable for fast, precise control.
  
- `veg_attitude_controller_pid.ino`  
  Classical PID controller version for roll, pitch, yaw, and altitude. Simple and reliable.

- `veg_main_flight_loop.ino`  
  Unified flight loop that allows switching between PID and LQR. It includes basic FDI hooks and control execution logic.

---

## 🧠 Switching Control Modes

To switch between **LQR** and **PID** control in `veg_main_flight_loop.ino`, locate the following line:

```cpp
int control_mode = CONTROL_MODE_LQR; // or CONTROL_MODE_PID
```

Change `CONTROL_MODE_LQR` to `CONTROL_MODE_PID` if you want to use the PID controller.

---

## 📲 Communication with Raspberry Pi

- The Arduino receives setpoints over serial via a formatted line:  
  ```
  SET <roll> <pitch> <yaw> <altitude>
  ```
  Example: `SET 5.0 -3.0 0.0 1.2`

- It sends back logs and FDI alerts (e.g., “FDI: Potential motor anomaly detected.”) via Serial.println for debugging.

---

## ⚙️ Hardware Setup

- MPU6050 IMU via I2C
- 4 PWM-capable motor outputs:
  - Motor 1 → D3
  - Motor 2 → D5
  - Motor 3 → D6
  - Motor 4 → D9

Ensure the PWM frequency and ESC range is compatible with your motor controllers.

---

## 🧪 Notes

- LQR gains are pre-tuned and may need adjustment per frame/inertia.
- PID gains are conservative to avoid overshoot.
- Altitude control is basic (constant placeholder). Consider adding a barometer or ultrasonic sensor for height feedback.

---

## 📜 License

MIT License — Use freely with attribution.
