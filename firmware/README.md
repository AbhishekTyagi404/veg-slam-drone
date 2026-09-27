
# वेग Drone Firmware

Inner-loop attitude controller for the **वेग (Veg)** quadcopter, running on an Arduino Uno/Nano at 250 Hz.
PID and LQR are both built into a single sketch. You can pick one at compile time or switch at runtime.

> **Safety first:** do every bench test with the **props removed**. Before the first flight, check the sign flags and the failsafe as described below.

---

## 🔧 Files

```
firmware/
├── veg_flight_controller/          ← open THIS folder in the Arduino IDE
│   ├── veg_flight_controller.ino   main loop, IMU fusion, PID + LQR, FDI, failsafe, serial protocol
│   └── config.h                    every gain, limit, pin and sign convention
└── test/                           host-side behaviour tests (no hardware): `cd firmware/test && make`
```

Version 2.3 had four `.ino` files in one folder, and the Arduino IDE merges every `.ino` in a sketch folder into one program. That caused duplicate `setup()`/`loop()` definitions, so none of them compiled. They are replaced by the single sketch above.

**Libraries:** Wire and Servo (both built in), plus **"MPU6050" by Electronic Cats** (from the Library Manager). You also need Arduino AVR Boards core ≥ 1.8.3.

---

## 📲 Serial protocol (115200 baud, one command per line)

| Pi → Arduino | Meaning |
|---|---|
| `SET <roll_deg> <pitch_deg> <yaw_rate_dps> <throttle>` | Setpoint **and heartbeat**. Throttle is 0…1. Send it at ≥ 10 Hz. |
| `ARM` | Arms only if the IMU is OK, the drone is level (< 10°), throttle < 0.05, and a `SET` has been received recently. |
| `DISARM` | Stops the motors immediately. |
| `MODE PID` / `MODE LQR` | Switches the controller. Only allowed while disarmed. |
| `CAL` | Re-calibrates the gyro. The drone must be disarmed and kept still. |

| Arduino → Pi | Meaning |
|---|---|
| `TEL <state> <roll> <pitch> <yaw_rate> <throttle> <m1> <m2> <m3> <m4> <loop_us>` | Telemetry at 10 Hz. |
| `ACK …` / `ERR …` / `FDI …` / `INFO …` | Replies and alerts. |

⚠️ **The 4th field changed from altitude to throttle.** The Arduino has no altitude sensor, so altitude hold now runs on the Pi from the SLAM z estimate (see `software/control/trajectory_controller.py`). The firmware rejects values outside 0…1, so an old-style `SET 5 -3 0 1.2` can never command full throttle.

`software/utils/serial_comm.py` implements this protocol.

---

## 🛡️ Safety behaviour

| Condition | Response |
|---|---|
| Power-up | ESCs receive 1000 µs from the very first pulse. The Servo library defaults to 1500 µs, so this is set explicitly. |
| No `SET` for 500 ms while flying | **FAILSAFE**: the drone levels, yaw stops, and throttle ramps *down* to `FS_THROTTLE`. After `FS_DESCENT_MS` the motors stop. |
| No `SET` for 500 ms while on the ground (throttle ≤ 0.15) | Disarms immediately. |
| Attitude can't follow the setpoint (> 25° error for 0.5 s) | FAILSAFE. This is the rotor/prop-loss detector. |
| Tilt > 60° | Motors are cut (crash or flip). |
| I²C / IMU read failure | Disarms. |
| Motor saturated for > 1 s | `FDI WARN` message only, rate-limited. |

---

## 🧭 First-time setup (props OFF)

1. **ESC calibration.** Calibrate each ESC to the 1000–2000 µs range, following your ESC's procedure.
2. **Upload** and open the Serial Monitor (115200, newline). You should see `INFO gyro calibrated` and `INFO Veg FC ready`.
3. **Check sensor signs** using the `TEL` output:
   - Tilt the right side down → `roll` goes **positive**.
   - Lift the nose → `pitch` goes **positive**.
   - Rotate the nose to the right → `yaw_rate` goes **positive**.

   If one is wrong, flip `IMU_ROLL_SIGN`, `IMU_PITCH_SIGN` or `IMU_YAW_SIGN` in `config.h`.
4. **Check motor response.** Send `SET 0 0 0 0`, then `ARM`, then keep sending `SET 0 0 0 0.3`. Tilt the frame right-side-down: the **right** motors (M2, M3) must speed up. If the left motors speed up instead, the motor layout or mixer doesn't match the diagram in `config.h`.
5. **Check the failsafe.** While armed at `SET … 0.3`, stop sending. You should see `FDI FAILSAFE link lost`, and the motors should stop after `FS_DESCENT_MS`.
6. **Set the hover throttle.** Measure it (tethered) and set `FS_THROTTLE` a little **below** it. Pass the measured hover value to `trajectory_controller.py --hover`.

---

## ⚙️ Hardware

- MPU6050 on I²C (A4/A5), with its X arrow pointing forward and the chip facing up. For any other mounting, change the sign flags.
- ESC signal wires: M1 → D3 (rear-left), M2 → D5 (rear-right), M3 → D6 (front-right), M4 → D9 (front-left).
- The ESCs are driven with standard 50 Hz servo pulses via `Servo.writeMicroseconds()`. `analogWrite()` produced ~490/980 Hz PWM that ESCs can't read. A faster protocol such as OneShot125 would improve control bandwidth later.

## 🎛️ Gains

- The LQR gains in `config.h` come from `veg-slam-drone-sim/compute_lqr_gains.m`, which uses `veg_plant_parameters.mat`. Re-run it after you measure your motors' thrust-per-µs (`kT_us`) on a thrust stand.
- The PID gains are the original v2.3 values. The D term now acts on the gyro rate, and every integrator has anti-windup.

## 📜 License

MIT License — use freely with attribution.
