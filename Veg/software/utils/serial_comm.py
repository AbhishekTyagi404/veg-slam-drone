
# serial_comm.py
# Sends setpoints to Arduino for attitude and altitude control

import serial
import time

class ArduinoComm:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # wait for Arduino to initialize
            print(f"[INFO] Connected to Arduino on {port} at {baudrate} baud.")
        except serial.SerialException:
            print(f"[ERROR] Could not open serial port {port}.")
            self.ser = None

    def send_setpoint(self, roll, pitch, yaw, altitude):
        if self.ser:
            cmd = f"SET {roll:.2f} {pitch:.2f} {yaw:.2f} {altitude:.2f}\n"
            self.ser.write(cmd.encode('utf-8'))

    def read_response(self):
        if self.ser and self.ser.in_waiting:
            return self.ser.readline().decode('utf-8').strip()
        return None

    def close(self):
        if self.ser:
            self.ser.close()
            print("[INFO] Serial connection closed.")

if __name__ == '__main__':
    comm = ArduinoComm()
    try:
        while True:
            comm.send_setpoint(roll=5.0, pitch=-3.0, yaw=0.0, altitude=1.5)
            response = comm.read_response()
            if response:
                print("[ARDUINO]", response)
            time.sleep(0.1)
    except KeyboardInterrupt:
        comm.close()
