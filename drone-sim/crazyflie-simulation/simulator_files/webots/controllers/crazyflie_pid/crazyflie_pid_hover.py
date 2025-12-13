# -*- coding: utf-8 -*-
"""
file: crazyflie_pid.py

Controller: PID (Baseline)
Scenario: HOVER_NO_WIND

Automatic takeoff and hover.
No keyboard, no landing.
Unified logging format for benchmark comparison.
"""

from controller import Robot
import math
import numpy as np
import csv

from pid_controller import pid_velocity_fixed_height_controller


# ======================================================
# ---------------- Benchmark Constants -----------------
# ======================================================
CONTROLLER_NAME = "PID"
SCENARIO_NAME   = "HOVER_NO_WIND"

FLYING_ATTITUDE = 0.5      # target hover height (m)
POS_P_GAIN      = 2.0      # position P gain (pos -> vel)

IMU_DEVICE_NAME = "inertial_unit"


# ======================================================
# -------------------- Data Logger ---------------------
# ======================================================
class DataLogger:
    def __init__(self,
                 filename="pid_hover_no_wind.csv",
                 controller=CONTROLLER_NAME,
                 scenario=SCENARIO_NAME):

        self.filename = filename
        self.controller = controller
        self.scenario = scenario

        self.header = [
            't', 'controller', 'scenario', 'mode',
            'px', 'py', 'pz',
            'vx', 'vy',
            'yaw',
            'ref_px', 'ref_py', 'ref_pz',
            'ex', 'ey', 'ez', 'pos_error',
            'wind_on'
        ]

        with open(self.filename, 'w', newline='') as f:
            csv.writer(f).writerow(self.header)

        print(f"[LOGGER] Logging to {self.filename}")

    def log(self, t, mode,
            actual_pos, actual_vel,
            ref_pos, yaw,
            wind_on=0):

        ex = actual_pos[0] - ref_pos[0]
        ey = actual_pos[1] - ref_pos[1]
        ez = actual_pos[2] - ref_pos[2]
        pos_error = math.sqrt(ex**2 + ey**2 + ez**2)

        row = [
            t,
            self.controller,
            self.scenario,
            mode,

            actual_pos[0], actual_pos[1], actual_pos[2],
            actual_vel[0], actual_vel[1],
            yaw,

            ref_pos[0], ref_pos[1], ref_pos[2],

            ex, ey, ez,
            pos_error,

            wind_on
        ]

        with open(self.filename, 'a', newline='') as f:
            csv.writer(f).writerow(row)


# ======================================================
# ---------------- PID Hover Controller ----------------
# ======================================================
class CrazyfliePID:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # logger
        self.logger = DataLogger()

        # sensors
        self.imu = self.robot.getDevice(IMU_DEVICE_NAME)
        self.imu.enable(self.timestep)

        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)

        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)

        # motors
        self.motors = []
        for name in ["m1_motor", "m2_motor", "m3_motor", "m4_motor"]:
            motor = self.robot.getDevice(name)
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
            self.motors.append(motor)

        # PID controller
        self.controller = pid_velocity_fixed_height_controller()

        # state
        self.mode = "TAKEOFF"
        self.prev_pos = None
        self.prev_time = None

        # reference
        self.ref_x = 0.0
        self.ref_y = 0.0
        self.ref_z = FLYING_ATTITUDE

        print("[PID] Automatic takeoff and hover started")


    # --------------------------------------------------
    def read_sensors(self, dt):
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        yaw_rate = self.gyro.getValues()[2]

        x, y, z = self.gps.getValues()

        if self.prev_pos is None:
            vx_global, vy_global = 0.0, 0.0
        else:
            vx_global = (x - self.prev_pos[0]) / dt
            vy_global = (y - self.prev_pos[1]) / dt

        # global -> body frame
        c = math.cos(yaw)
        s = math.sin(yaw)
        vx =  c * vx_global + s * vy_global
        vy = -s * vx_global + c * vy_global

        self.prev_pos = [x, y, z]

        return roll, pitch, yaw, yaw_rate, x, y, z, vx, vy


    # --------------------------------------------------
    def run(self):
        while self.robot.step(self.timestep) != -1:
            t = self.robot.getTime()

            if self.prev_time is None:
                self.prev_time = t
                self.prev_pos = self.gps.getValues()

                # lock hover reference to takeoff position
                self.ref_x = self.prev_pos[0]
                self.ref_y = self.prev_pos[1]
                continue

            dt = t - self.prev_time
            self.prev_time = t

            # sensors
            (roll, pitch, yaw, yaw_rate,
             x, y, z, vx, vy) = self.read_sensors(dt)

            # state machine
            if self.mode == "TAKEOFF" and z >= 0.95 * FLYING_ATTITUDE:
                self.mode = "HOVER"
                print("[PID] Hover stabilized")

            # position -> velocity
            ex = self.ref_x - x
            ey = self.ref_y - y
            des_vx = POS_P_GAIN * ex
            des_vy = POS_P_GAIN * ey

            # PID velocity + height control
            motor_cmd = self.controller.pid(
                dt,
                des_vx, des_vy,
                0.0,                 # desired yaw rate
                self.ref_z,
                roll, pitch, yaw_rate,
                z, vx, vy
            )

            # motor mapping (FINAL VERIFIED)
            self.motors[0].setVelocity(-motor_cmd[0])
            self.motors[1].setVelocity(+motor_cmd[1])
            self.motors[2].setVelocity(-motor_cmd[2])
            self.motors[3].setVelocity(+motor_cmd[3])

            # logging
            self.logger.log(
                t,
                self.mode,
                [x, y, z],
                [vx, vy],
                [self.ref_x, self.ref_y, self.ref_z],
                yaw,
                wind_on=0
            )


# ======================================================
# ------------------------- Main -----------------------
# ======================================================
if __name__ == "__main__":
    drone = CrazyfliePID()
    drone.run()
