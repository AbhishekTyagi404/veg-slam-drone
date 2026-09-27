"""
trajectory_controller.py — outer-loop position controller for Veg (runs on the Pi, 20 Hz).

Closes the loop on a pose estimate (ORB-SLAM3) and streams attitude + throttle
setpoints to the Arduino inner loop:

    position error --PD--> desired acceleration --> roll/pitch (deg)
    altitude error --PID-> throttle (0..1, around the hover throttle)
    yaw error      --P---> yaw rate (deg/s)

Frames: pose is x forward, y left, z up (ROS FLU/ENU), yaw CCW-positive, metres/radians.
Firmware conventions: roll + = right side down, pitch + = nose up, yaw rate + = nose right.

Usage
    python3 trajectory_controller.py --sim                  # no hardware: closed-loop simulation
    python3 trajectory_controller.py --port /dev/ttyUSB0    # real drone, pose from ROS topic
"""

import argparse
import math
import os
import sys
import time

import numpy as np

# utils/ is a sibling folder — make the import work no matter where this is launched from
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'utils'))

G = 9.81


# ============================ Trajectory ============================
def reference(t, start, alt=1.2, radius=1.5, omega=0.5, takeoff_s=4.0, circle_s=12.0, land_s=5.0):
    """Take off above `start`, fly one circle that begins and ends there, then land.
    Returns (x, y, z, yaw) or None when finished."""
    x0, y0, z0 = start
    if t < takeoff_s:
        return x0, y0, z0 + (alt - z0) * (t / takeoff_s), 0.0
    t -= takeoff_s
    if t < circle_s:
        a = omega * t
        # circle centred at (x0 - r, y0) so it starts at (x0, y0)
        return x0 - radius + radius * math.cos(a), y0 + radius * math.sin(a), alt + 0.1 * math.sin(0.25 * t), 0.0
    t -= circle_s
    xe = x0 - radius + radius * math.cos(omega * circle_s)
    ye = y0 + radius * math.sin(omega * circle_s)
    if t < land_s:
        return xe, ye, alt - (alt - z0) * (t / land_s), 0.0
    return None


# ============================ Controller ============================
class PositionController:
    def __init__(self, hover_throttle=0.5, kp_xy=1.2, kd_xy=1.6, kp_z=0.25, ki_z=0.08, kd_z=0.2,
                 kp_yaw=1.5, max_tilt_deg=15.0, max_yaw_rate=60.0):
        self.hover = hover_throttle
        self.kp_xy, self.kd_xy = kp_xy, kd_xy
        self.kp_z, self.ki_z, self.kd_z = kp_z, ki_z, kd_z
        self.kp_yaw = kp_yaw
        self.max_tilt = max_tilt_deg
        self.max_yaw_rate = max_yaw_rate
        self.iz = 0.0

    def update(self, ref, pose, vel, dt, ref_vel=(0.0, 0.0, 0.0), ref_acc=(0.0, 0.0, 0.0)):
        """ref=(x,y,z,yaw), pose=(x,y,z,yaw), vel=(vx,vy,vz) in world frame.
        ref_vel / ref_acc are feed-forward terms from the trajectory (world frame).
        Returns (roll_deg, pitch_deg, yaw_rate_dps, throttle)."""
        ex, ey, ez = ref[0] - pose[0], ref[1] - pose[1], ref[2] - pose[2]
        vx, vy, vz = vel

        # Desired horizontal acceleration (world) -> body frame
        ax_w = ref_acc[0] + self.kp_xy * ex + self.kd_xy * (ref_vel[0] - vx)
        ay_w = ref_acc[1] + self.kp_xy * ey + self.kd_xy * (ref_vel[1] - vy)
        c, s = math.cos(pose[3]), math.sin(pose[3])
        ax_b = c * ax_w + s * ay_w
        ay_b = -s * ax_w + c * ay_w

        # Forward accel needs nose DOWN (negative pitch); leftward accel needs roll LEFT (negative roll)
        pitch = -math.degrees(math.atan2(ax_b, G))
        roll = -math.degrees(math.atan2(ay_b, G))
        pitch = float(np.clip(pitch, -self.max_tilt, self.max_tilt))
        roll = float(np.clip(roll, -self.max_tilt, self.max_tilt))

        # Altitude PID -> throttle, with tilt compensation and anti-windup
        self.iz = float(np.clip(self.iz + ez * dt, -1.0, 1.0))
        tilt_comp = 1.0 / max(0.7, math.cos(math.radians(roll)) * math.cos(math.radians(pitch)))
        thr = (self.hover + self.kp_z * ez + self.ki_z * self.iz + self.kd_z * (ref_vel[2] - vz)) * tilt_comp
        thr = float(np.clip(thr, 0.0, 0.9))

        # Yaw: pose yaw is CCW+, firmware yaw rate is nose-right (CW)+
        eyaw = math.atan2(math.sin(ref[3] - pose[3]), math.cos(ref[3] - pose[3]))
        yaw_rate = float(np.clip(-math.degrees(self.kp_yaw * eyaw), -self.max_yaw_rate, self.max_yaw_rate))
        return roll, pitch, yaw_rate, thr


# ============================ Pose sources ============================
class RosPoseSource:
    """Latest pose from a geometry_msgs/PoseStamped topic (e.g. an ORB-SLAM3 ROS wrapper).
    Velocity is estimated by low-pass-filtered differentiation."""

    def __init__(self, topic='/orb_slam3/camera_pose', stale_s=0.3):
        import rospy
        from geometry_msgs.msg import PoseStamped
        self.rospy = rospy
        rospy.init_node('veg_trajectory_controller', anonymous=True, disable_signals=True)
        self.stale_s = stale_s
        self.pose, self.vel, self._last = None, (0.0, 0.0, 0.0), None
        rospy.Subscriber(topic, PoseStamped, self._cb, queue_size=1)

    def _cb(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = msg.header.stamp.to_sec() or time.time()
        if self._last is not None and t > self._last[0]:
            dt = t - self._last[0]
            raw = [(p.x - self._last[1]) / dt, (p.y - self._last[2]) / dt, (p.z - self._last[3]) / dt]
            self.vel = tuple(0.7 * v + 0.3 * r for v, r in zip(self.vel, raw))
        self._last = (t, p.x, p.y, p.z)
        self.pose = (p.x, p.y, p.z, yaw)
        self._stamp = time.time()

    def get(self):
        """Returns (pose, vel) or None if no fresh pose (tracking lost)."""
        if self.pose is None or time.time() - self._stamp > self.stale_s:
            return None
        return self.pose, self.vel


class SimulatedDrone:
    """Point-mass quad + first-order attitude response. Stands in for BOTH the
    Arduino link and the SLAM pose so the outer loop can be tested without hardware."""

    def __init__(self, hover_throttle=0.5, tau=0.12):
        self.hover, self.tau = hover_throttle, tau
        self.p = np.zeros(3); self.v = np.zeros(3)
        self.roll = self.pitch = self.yaw = 0.0
        self.cmd = (0.0, 0.0, 0.0, 0.0)
        self.armed = False
        self.telemetry = None

    # --- ArduinoComm-compatible interface ---
    def send_setpoint(self, roll, pitch, yaw_rate, throttle):
        assert 0.0 <= throttle <= 1.0
        self.cmd = (roll, pitch, yaw_rate, throttle)

    def arm(self):
        self.armed = True; return True

    def disarm(self):
        self.armed = False; return True

    def close(self):
        pass

    # --- pose-source interface ---
    def get(self):
        return (*self.p, self.yaw), tuple(self.v)

    def step(self, dt):
        if not self.armed:
            return
        r_cmd, p_cmd, yr_cmd, thr = self.cmd
        a = dt / self.tau
        self.roll += a * (math.radians(r_cmd) - self.roll)
        self.pitch += a * (math.radians(p_cmd) - self.pitch)
        self.yaw -= math.radians(yr_cmd) * dt          # nose-right rate decreases CCW yaw
        thrust = G * thr / self.hover
        ax_b = -thrust * math.sin(self.pitch)            # nose down -> forward
        ay_b = -thrust * math.sin(self.roll)             # right side down -> rightwards (-y)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        acc = np.array([c * ax_b - s * ay_b, s * ax_b + c * ay_b,
                        thrust * math.cos(self.roll) * math.cos(self.pitch) - G])
        self.v += acc * dt
        self.p += self.v * dt
        if self.p[2] < 0:
            self.p[2] = 0.0; self.v[2] = max(0.0, self.v[2])


# ============================ Runner ============================
class TrajectoryController:
    def __init__(self, fc, pose_source, rate_hz=20.0, hover_throttle=0.5, sim=None):
        self.fc, self.pose_source, self.sim = fc, pose_source, sim
        self.dt = 1.0 / rate_hz
        self.ctrl = PositionController(hover_throttle=hover_throttle)
        self.log = []

    def _pose(self):
        got = self.pose_source.get()
        if got is None:
            return None
        return got

    def run(self):
        got = self._pose()
        if got is None:
            raise RuntimeError("No pose available — is SLAM tracking?")
        start = got[0][:3]
        if not self.fc.arm():
            raise RuntimeError("Arming refused by flight controller")
        print("[INFO] Armed. Starting trajectory.")
        t = 0.0
        try:
            while True:
                ref = reference(t, start)
                if ref is None:
                    break
                got = self._pose()
                if got is None:
                    # SLAM lost: stop sending -> Arduino link watchdog triggers its failsafe descent
                    print("[WARN] Pose lost — handing over to flight-controller failsafe.")
                    return False
                pose, vel = got
                # feed-forward velocity/acceleration from the reference (central differences)
                h = self.dt
                rm, rp = reference(max(t - h, 0.0), start), reference(t + h, start) or ref
                ref_vel = [(rp[i] - rm[i]) / (2 * h) for i in range(3)]
                ref_acc = [(rp[i] - 2 * ref[i] + rm[i]) / (h * h) for i in range(3)]
                cmd = self.ctrl.update(ref, pose, vel, self.dt, ref_vel, ref_acc)
                self.fc.send_setpoint(*cmd)
                self.log.append((t, *ref[:3], *pose[:3], *cmd))
                if self.sim is not None:
                    for _ in range(10):
                        self.sim.step(self.dt / 10)
                else:
                    time.sleep(self.dt)
                t += self.dt
            # landed: idle briefly then disarm
            for _ in range(10):
                self.fc.send_setpoint(0.0, 0.0, 0.0, 0.0)
                if self.sim is None:
                    time.sleep(self.dt)
            print("[INFO] Trajectory complete.")
            return True
        finally:
            self.fc.disarm()
            self.fc.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sim', action='store_true', help='closed-loop simulation, no hardware')
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--pose-topic', default='/orb_slam3/camera_pose')
    ap.add_argument('--hover', type=float, default=0.5, help='throttle (0..1) that holds a hover — measure it!')
    args = ap.parse_args()

    if args.sim:
        drone = SimulatedDrone(hover_throttle=args.hover)
        tc = TrajectoryController(drone, drone, hover_throttle=args.hover, sim=drone)
        tc.run()
        log = np.array(tc.log)
        err = np.linalg.norm(log[:, 1:4] - log[:, 4:7], axis=1)
        print(f"[SIM] tracking error: RMS {np.sqrt(np.mean(err**2)):.3f} m, max {err.max():.3f} m, final z {drone.p[2]:.2f} m")
        return

    from serial_comm import ArduinoComm
    fc = ArduinoComm(port=args.port)
    TrajectoryController(fc, RosPoseSource(args.pose_topic), hover_throttle=args.hover).run()


if __name__ == '__main__':
    main()
