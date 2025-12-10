#!/usr/bin/env python3
"""
FULL LQR-BASED WEBOTS CONTROLLER WITH SAFE THRUST PID
Trajectory: Hover at 0.5 m → after 3 sec → CCW circle radius 0.5 m, 10-sec period
Debug print interval: 0.5 s
"""

import numpy as np
from controller import Supervisor, InertialUnit, GPS, Gyro, Keyboard
from math import sin, cos, pi

# === ESO ADDED (imports) ===
import sys
sys.path.append("eso")          # path to your eso folder
from eso_interface import load_eso_interface
from eso import AttitudeESO
from design_L import compute_L

# =============================================================
# Crazyflie Physical Parameters
# =============================================================
m = 0.027
g = 9.81
Ix = 1.4e-5
Iy = 1.4e-5
Iz = 2.17e-5

# =============================================================
# Linearized Crazyflie Model (Full 12-state)
# =============================================================
def crazyflie_linear_model():
    A = np.zeros((12, 12))
    A[6, 9] = 1.0; A[7,10] = 1.0; A[8,11] = 1.0
    A[0,3] = 1.0; A[1,4] = 1.0; A[2,5] = 1.0
    A[9,1] = g; A[10,0] = -g

    B = np.zeros((12,4))
    B[11,0] = -1.0/m
    B[3,1]  = 1.0/Ix
    B[4,2]  = 1.0/Iy
    B[5,3]  = 1.0/Iz
    return A, B

# =============================================================
# Pure NumPy LQR Solver
# =============================================================
def lqr(A,B,Q,R,max_iter=200,eps=1e-9):
    P = Q.copy()
    for _ in range(max_iter):
        Pn = A.T@P + P@A - P@B@np.linalg.inv(R)@B.T@P + Q
        if np.max(np.abs(Pn-P)) < eps:
            break
        P = Pn
    return np.linalg.inv(R) @ (B.T @ P)

# =============================================================
# Baseline Q/R (active)
# =============================================================
Q = np.diag([
    50,50,20,
    10,10,5,
    30,30,60,
    10,10,20
])
R = np.diag([1e-3,1e-3,1e-3,1e-3])

# =============================================================
# Motor Mixer
# =============================================================
def motor_mixer(T, tx, ty, tz):
    u = np.array([T, tx, ty, tz], dtype=float)
    mix = np.array([
        [ 1,  1, -1, -1],
        [ 1, -1,  1, -1],
        [ 1, -1, -1,  1],
        [ 1,  1,  1,  1]
    ], dtype=float)
    cmds = mix @ u
    cmds = np.nan_to_num(cmds, nan=0.0)
    cmds = np.clip(cmds * 2500, -2000, 2000)
    return cmds

# =============================================================
# Trajectory Generator
# =============================================================
class Traj:
    def __init__(self):
        self.center = np.array([0,0,0.5])
        self.hover_time = 3.0
        self.R = 0.5
        self.omega = 2*pi/10

    def ref(self, t):
        if t < self.hover_time:
            return self.center.copy()
        dt = t - self.hover_time
        x = self.center[0] + self.R*cos(self.omega*dt)
        y = self.center[1] + self.R*sin(self.omega*dt)
        z = 0.5
        return np.array([x,y,z])

# =============================================================
# Main Controller
# =============================================================
if __name__ == '__main__':
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    imu = robot.getDevice("inertial_unit"); imu.enable(timestep)
    gps = robot.getDevice("gps"); gps.enable(timestep)
    gyro = robot.getDevice("gyro"); gyro.enable(timestep)

    m1 = robot.getDevice("m1_motor"); m1.setPosition(float('inf'))
    m2 = robot.getDevice("m2_motor"); m2.setPosition(float('inf'))
    m3 = robot.getDevice("m3_motor"); m3.setPosition(float('inf'))
    m4 = robot.getDevice("m4_motor"); m4.setPosition(float('inf'))

    traj = Traj()

    # LQR
    A,B = crazyflie_linear_model()
    K = lqr(A,B,Q,R)

    # === ESO INITIALIZATION ===
    Ts = timestep / 1000.0

    L_dummy = np.zeros((12, 6))
    eso = AttitudeESO(Ts, L_dummy, m=m, g=g)

    L = compute_L(eso, m)
    eso.L = L

    eso_interface = load_eso_interface(eso, mass=m)
    print("\n=== ESO Initialized (12-state AttitudeESO) ===\n")

    prev_t = robot.getTime()
    prev_x = prev_y = prev_z = 0
    print_t = 0.0
    PRINT_INTERVAL = 0.5

    while robot.step(timestep) != -1:
        t = robot.getTime()
        dt = t - prev_t
        if dt <= 0:
            continue

        # === Read sensors ===
        roll,pitch,yaw = imu.getRollPitchYaw()
        gx,gy,gz = gyro.getValues()
        xg,yg,zg = gps.getValues()

        vx = (xg - prev_x)/dt
        vy = (yg - prev_y)/dt
        vz = (zg - prev_z)/dt

        prev_x,prev_y,prev_z = xg,yg,zg
        prev_t = t

        # Replace NaNs
        roll,pitch,yaw = np.nan_to_num([roll,pitch,yaw])
        gx,gy,gz = np.nan_to_num([gx,gy,gz])

        # === ESO UPDATE ===
        y_meas = np.array([xg, yg, zg, roll, pitch, yaw])
        z_hat = eso.step(y_meas, m*g, np.array([gx,gy,gz]))
        eso_p   = z_hat[0:3]
        eso_v   = z_hat[3:6]
        eso_att = z_hat[6:9]

        # === Build LQR states (still using measured state here) ===
        p_ref = traj.ref(t)

        ref_state = np.array([
            p_ref[0],p_ref[1],p_ref[2],
            0,0,0,
            0,0,0,
            0,0,0
        ])

        x_state = np.array([
            xg,yg,zg,
            vx,vy,vz,
            roll,pitch,yaw,
            gx,gy,gz
        ])

        e = x_state - ref_state
        u_nominal = -K @ e   # [T, τx, τy, τz]

        # === ESO DISTURBANCE FEEDFORWARD (NO TORQUE LOGIC HERE) ===
        roll_est, pitch_est, yaw_est = eso_att
        u_cmd = eso_interface.apply_feedforward(
            u_nominal,
            roll_est,
            pitch_est,
            yaw_est,
            use_thrust_feedforward=False,   # thrust handled by PID
        )
        # u_cmd = [T_nom_corrected, tx_corrected, ty_corrected, tz_corrected]
        # We will discard T part here and use our safe thrust PID below.
        _, tx_raw, ty_raw, tz_raw = u_cmd

        # === SAFE THRUST PID (ALTITUDE ONLY) ===
        z_err = p_ref[2] - zg
        z_dot_err = -vz

        T = m*g + (5.0*z_err + 3.0*z_dot_err)
        T = np.clip(T, 0.0, 0.6)

        # Clamp torques
        tx = np.clip(tx_raw, -0.002, 0.002)
        ty = np.clip(ty_raw, -0.002, 0.002)
        tz = np.clip(tz_raw, -0.002, 0.002)

        motors = motor_mixer(T, tx, ty, tz)

        m1.setVelocity(motors[0])
        m2.setVelocity(motors[1])
        m3.setVelocity(motors[2])
        m4.setVelocity(motors[3])

        # Debug print
        if t - print_t >= PRINT_INTERVAL:
            print_t = t
            pos_str = f"({xg:+.3f},{yg:+.3f},{zg:+.3f})"
            vel_str = f"({vx:+.3f},{vy:+.3f},{vz:+.3f})"
            att_str = f"{roll*57.3:.1f}°, {pitch*57.3:.1f}°, {yaw*57.3:.1f}°"
            torque_str = f"({tx:+.4f},{ty:+.4f},{tz:+.4f})"
            print(f"\nTime {t:.2f}s")
            print(f"Position: {pos_str}")
            print(f"Velocity: {vel_str}")
            print(f"Attitude: {att_str}")
            print(f"T={T:.4f}, τ={torque_str}")
            print(f"Motors: {motors}\n")
