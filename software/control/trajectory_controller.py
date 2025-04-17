
# trajectory_controller.py
# Sends dynamic setpoints from a planned trajectory to Arduino via serial

import time
import numpy as np
from serial_comm import ArduinoComm

class TrajectoryController:
    def __init__(self, serial_port='/dev/ttyUSB0'):
        self.arduino = ArduinoComm(port=serial_port)
        self.dt = 0.1  # 10 Hz update rate
        self.t = 0.0

    def generate_trajectory(self, duration=10):
        # Circle trajectory as an example
        t_vals = np.arange(0, duration, self.dt)
        traj = []
        for t in t_vals:
            x = 1.5 * np.cos(0.5 * t)
            y = 1.5 * np.sin(0.5 * t)
            z = 1.2 + 0.1 * np.sin(0.25 * t)
            yaw = 0.0
            traj.append((x, y, z, yaw))
        return traj

    def run(self):
        trajectory = self.generate_trajectory()
        print("[INFO] Starting trajectory execution...")
        for point in trajectory:
            x, y, z, yaw = point
            # Convert position commands to attitude setpoints (simple linear mapping)
            roll_cmd = y  # positive y -> roll right
            pitch_cmd = -x  # positive x -> pitch forward
            alt_cmd = z
            self.arduino.send_setpoint(roll_cmd, pitch_cmd, yaw, alt_cmd)
            response = self.arduino.read_response()
            if response:
                print("[ARDUINO]", response)
            time.sleep(self.dt)

        print("[INFO] Trajectory complete.")
        self.arduino.close()

if __name__ == '__main__':
    controller = TrajectoryController()
    controller.run()
