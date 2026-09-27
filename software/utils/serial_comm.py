"""
serial_comm.py — Raspberry Pi <-> Arduino link for the Veg flight controller.

Protocol (must match firmware/veg_flight_controller):
    Pi -> Arduino : SET <roll_deg> <pitch_deg> <yaw_rate_dps> <throttle 0..1>
                    ARM | DISARM | MODE PID | MODE LQR | CAL
    Arduino -> Pi : TEL <state> <roll> <pitch> <yaw_rate> <throttle> <m1> <m2> <m3> <m4> <loop_us>
                    ACK ... | ERR ... | FDI ... | INFO ...

The Arduino disarms / enters failsafe if it gets no SET for 500 ms, so the
caller's control loop must call send_setpoint() at >= 10 Hz. There is
deliberately no background heartbeat thread: if the Pi's control logic hangs,
the Arduino should notice.
"""

import threading
import time
from dataclasses import dataclass
from typing import Optional

import serial


@dataclass
class Telemetry:
    state: str
    roll: float
    pitch: float
    yaw_rate: float
    throttle: float
    motors: tuple
    loop_us: int
    stamp: float


class ArduinoComm:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, ready_timeout=6.0, verbose=True):
        # Raise instead of silently continuing: flying code must know the link is down.
        self.ser = serial.Serial(port, baudrate, timeout=0.05)
        self.verbose = verbose
        self._lock = threading.Lock()
        self._acks = []
        self.telemetry: Optional[Telemetry] = None
        self.messages = []          # FDI / INFO / ERR lines, newest last
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        # Opening the port resets the Arduino; wait for it to boot and calibrate the gyro.
        if not self._wait_for(lambda: any('Veg FC ready' in m for m in self.messages), ready_timeout):
            raise TimeoutError(f"No 'Veg FC ready' from {port} — check firmware/baud rate")
        if verbose:
            print(f"[INFO] Connected to Veg FC on {port} at {baudrate} baud.")

    # ---------------- commands ----------------
    def send_setpoint(self, roll_deg, pitch_deg, yaw_rate_dps, throttle):
        if not 0.0 <= throttle <= 1.0:
            raise ValueError("throttle must be in [0, 1] (4th field is throttle, not altitude)")
        self._write(f"SET {roll_deg:.2f} {pitch_deg:.2f} {yaw_rate_dps:.2f} {throttle:.3f}")

    def arm(self, timeout=1.0):
        """Send a zero-throttle heartbeat, then ARM. Returns True on ACK."""
        self.send_setpoint(0, 0, 0, 0)
        time.sleep(0.05)
        return self._command('ARM', timeout)

    def disarm(self, timeout=0.5):
        return self._command('DISARM', timeout)

    def set_mode(self, mode, timeout=0.5):
        assert mode in ('PID', 'LQR')
        return self._command(f'MODE {mode}', timeout)

    def close(self):
        try:
            self.disarm()
        except Exception:
            pass
        self._running = False
        self._reader.join(timeout=0.5)
        self.ser.close()
        if self.verbose:
            print("[INFO] Serial connection closed.")

    # ---------------- internals ----------------
    def _write(self, line):
        with self._lock:
            self.ser.write((line + '\n').encode('ascii'))

    def _command(self, cmd, timeout):
        word = cmd.split()[0]
        with self._lock:
            self._acks.clear()
        self._write(cmd)
        ok = self._wait_for(lambda: any(a.startswith('ACK ' + word) or a.startswith('ERR') for a in self._acks), timeout)
        with self._lock:
            acks = list(self._acks)
        if ok and any(a.startswith('ACK ' + word) for a in acks):
            return True
        if self.verbose:
            print(f"[WARN] {cmd} failed: {acks[-1] if acks else 'no reply'}")
        return False

    @staticmethod
    def _wait_for(pred, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if pred():
                return True
            time.sleep(0.01)
        return False

    def _read_loop(self):
        while self._running:
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                break
            if not raw:
                continue
            line = raw.decode('ascii', errors='replace').strip()
            self._handle_line(line)

    def _handle_line(self, line):
        if line.startswith('TEL '):
            p = line.split()
            if len(p) == 11:
                try:
                    self.telemetry = Telemetry(p[1], float(p[2]), float(p[3]), float(p[4]), float(p[5]),
                                               tuple(int(x) for x in p[6:10]), int(p[10]), time.time())
                except ValueError:
                    pass
            return
        with self._lock:
            if line.startswith(('ACK', 'ERR')):
                self._acks.append(line)
            self.messages.append(line)
            del self.messages[:-200]
        if self.verbose and not line.startswith('ACK'):
            print('[ARDUINO]', line)


if __name__ == '__main__':
    # Bench test with PROPS OFF: arms, idles for 3 s, disarms.
    comm = ArduinoComm()
    try:
        if comm.arm():
            t0 = time.time()
            while time.time() - t0 < 3.0:
                comm.send_setpoint(0.0, 0.0, 0.0, 0.0)
                if comm.telemetry:
                    print('[TEL]', comm.telemetry)
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        comm.close()
