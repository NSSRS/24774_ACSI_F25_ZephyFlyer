# -*- coding: utf-8 -*-
"""
file: crazyflie_circle.py

Mode:
Automatic circular trajectory tracking.
- Fixed circle center at (0, 0.5)
- Radius = 0.5 m
- Period = 10 s per lap
- Total = 3 laps
- Counter-clockwise
- Vertical takeoff and landing
- No keyboard input

CSV format:
t, mode, px, py, pz, ref_px, ref_py, ref_pz, pos_error, yaw
"""

from controller import Robot
import math
import numpy as np
import csv

from pid_controller import pid_velocity_fixed_height_controller

# ======================================================
# ------------------- PARAMETERS -----------------------
# ======================================================

FLYING_ATTITUDE = 0.5          # Target altitude (m)

CIRCLE_CENTER_X = 0.0          # Fixed circle center (world frame)
CIRCLE_CENTER_Y = 0.5
CIRCLE_RADIUS = 0.5            # Circle radius (m)

CIRCLE_PERIOD = 10.0           # Seconds per lap
TARGET_LAPS = 3                # Number of laps

POS_P_GAIN = 2.0               # Position -> velocity gain

IMU_DEVICE_NAME = "inertial_unit"


# ======================================================
# ------------------- DATA LOGGER ----------------------
# ======================================================

class DataLogger:
    def __init__(self, filename="crazyflie_circle_log.csv"):
        self.filename = filename
        self.header = [
            't', 'mode',
            'px', 'py', 'pz',
            'ref_px', 'ref_py', 'ref_pz',
            'pos_error', 'yaw'
        ]

        with open(self.filename, 'w', newline='') as f:
            csv.writer(f).writerow(self.header)

        print(f"[LOGGER] Logging to {self.filename}")

    def log_data(self, time, mode, actual_pos, ref_pos, yaw):
        error = np.linalg.norm(np.array(actual_pos) - np.array(ref_pos))

        row = [
            time, mode,
            actual_pos[0], actual_pos[1], actual_pos[2],
            ref_pos[0], ref_pos[1], ref_pos[2],
            error, yaw
        ]

        with open(self.filename, 'a', newline='') as f:
            csv.writer(f).writerow(row)


# ======================================================
# ---------------- CIRCLE CONTROLLER -------------------
# ======================================================

class CrazyflieCircle:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.logger = DataLogger()

        # Sensors
        self.imu = self.robot.getDevice(IMU_DEVICE_NAME)
        self.imu.enable(self.timestep)

        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)

        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)

        # Motors
        self.motors = []
        for name in ["m1_motor", "m2_motor", "m3_motor", "m4_motor"]:
            m = self.robot.getDevice(name)
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
            self.motors.append(m)

        # PID controller
        self.controller = pid_velocity_fixed_height_controller()

        # State
        self.state = "TAKEOFF"
        self.prev_pos = [0.0, 0.0, 0.0]
        self.last_time = 0.0

        # Trajectory state
        self.setpoint_x = 0.0
        self.setpoint_y = 0.0
        self.setpoint_z = 0.0

        self.circle_angle = -math.pi / 2     # Start at (0, 0)
        self.angle_at_last_lap = self.circle_angle
        self.laps_completed = 0

        print("[AUTO MODE] Circle tracking started")
        print("[INFO] Center = (0, 0.5), Radius = 0.5, Period = 10 s, Laps = 3")


    # --------------------------------------------------
    # Sensor reading and velocity estimation
    # --------------------------------------------------
    def read_sensors(self, dt):
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        yaw_rate = self.gyro.getValues()[2]

        x, y, z = self.gps.getValues()

        vx_g = (x - self.prev_pos[0]) / dt
        vy_g = (y - self.prev_pos[1]) / dt

        cy = math.cos(yaw)
        sy = math.sin(yaw)

        vx = vx_g * cy + vy_g * sy
        vy = -vx_g * sy + vy_g * cy

        self.prev_pos = [x, y, z]

        return roll, pitch, yaw, yaw_rate, x, y, z, vx, vy


    # --------------------------------------------------
    # Main loop
    # --------------------------------------------------
    def run(self):
        while self.robot.step(self.timestep) != -1:

            t = self.robot.getTime()
            dt = t - self.last_time

            if self.last_time == 0.0:
                p = self.gps.getValues()
                self.prev_pos = [p[0], p[1], p[2]]
                self.last_time = t
                continue

            roll, pitch, yaw, yaw_rate, x, y, z, vx, vy = self.read_sensors(dt)

            desired_alt = 0.0
            desired_yaw_rate = 0.0

            # =======================
            # TAKEOFF
            # =======================
            if self.state == "TAKEOFF":
                desired_alt = FLYING_ATTITUDE
                self.setpoint_z = desired_alt
                self.setpoint_x = x
                self.setpoint_y = y

                if z >= 0.95 * FLYING_ATTITUDE:
                    self.state = "CIRCLE"
                    print("[STATE] TAKEOFF → CIRCLE")

            # =======================
            # CIRCLE
            # =======================
            elif self.state == "CIRCLE":

                if self.laps_completed >= TARGET_LAPS:
                    self.state = "LANDING"
                    print("[STATE] CIRCLE → LANDING")

                else:
                    desired_alt = FLYING_ATTITUDE
                    self.setpoint_z = desired_alt

                    omega = 2 * math.pi / CIRCLE_PERIOD
                    self.circle_angle += omega * dt

                    if self.circle_angle >= self.angle_at_last_lap + 2 * math.pi:
                        self.laps_completed += 1
                        self.angle_at_last_lap += 2 * math.pi
                        print(f"[INFO] Lap {self.laps_completed} completed at t = {t:.2f} s")

                    self.setpoint_x = CIRCLE_CENTER_X + CIRCLE_RADIUS * math.cos(self.circle_angle)
                    self.setpoint_y = CIRCLE_CENTER_Y + CIRCLE_RADIUS * math.sin(self.circle_angle)

            # =======================
            # LANDING
            # =======================
            elif self.state == "LANDING":
                desired_alt = 0.0
                self.setpoint_z = desired_alt
                self.setpoint_x = x
                self.setpoint_y = y

                if z < 0.05:
                    self.state = "STOP"
                    print("[STATE] LANDING → STOP")

            # =======================
            # STOP
            # =======================
            elif self.state == "STOP":
                for m in self.motors:
                    m.setVelocity(0.0)

                self.logger.log_data(
                    t, self.state,
                    [x, y, z],
                    [self.setpoint_x, self.setpoint_y, self.setpoint_z],
                    yaw
                )
                return

            # Position → velocity
            ex = self.setpoint_x - x
            ey = self.setpoint_y - y

            vx_d = POS_P_GAIN * ex
            vy_d = POS_P_GAIN * ey

            motor_cmd = self.controller.pid(
                dt,
                vx_d, vy_d,
                desired_yaw_rate,
                desired_alt,
                roll, pitch, yaw_rate,
                z, vx, vy
            )

            # Motor mapping (verified)
            self.motors[0].setVelocity(-motor_cmd[0])
            self.motors[1].setVelocity(+motor_cmd[1])
            self.motors[2].setVelocity(-motor_cmd[2])
            self.motors[3].setVelocity(+motor_cmd[3])

            self.logger.log_data(
                t, self.state,
                [x, y, z],
                [self.setpoint_x, self.setpoint_y, self.setpoint_z],
                yaw
            )

            self.last_time = t


# ======================================================
# ---------------------- MAIN --------------------------
# ======================================================

if __name__ == "__main__":
    CrazyflieCircle().run()
