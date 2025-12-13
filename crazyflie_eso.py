"""
Crazyflie ESO Hover-Only Controller
— 只起飞、只悬停、不绕圈、不降落 —
"""

from controller import Robot, Keyboard
import numpy as np
import sys
import os
from datetime import datetime

# =========================================================
# DataLogger Class (Modified for CSV and Plotter Compatibility)
# =========================================================

class DataLogger:
    """
    Log data to a CSV file in the same directory as the script.
    Columns are formatted to be compatible with plot_lqr_trajectory.py.
    """
    def __init__(self, prefix="crazyflie_log"):
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
        except NameError:
            current_dir = os.getcwd()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(current_dir, f"{prefix}_{timestamp}.csv")
        self.file = open(self.filename, "w")

        # Header
        self.header = (
            "t,"
            "mode,"
            "px,py,pz,"
            "ref_px,ref_py,ref_pz,"
            "pos_error,"
            "yaw,"
            "eso_x,eso_y,eso_z,"
            "eso_vx,eso_vy,eso_vz,"
            "eso_roll,eso_pitch,eso_yaw,"
            "dist_x,dist_y,dist_z,"
            "imu_roll,imu_pitch,"
            "m1,m2,m3,m4,"
            "pid1,pid2,pid3,pid4,"
            "eso1,eso2,eso3,eso4\n"
        )
        self.file.write(self.header)

        print(f"[LOGGER] Logging to: {self.filename}")

    def log(self, t, mode, pos_error, gps_pos, eso_p, eso_v, eso_att, eso_d_f,
            imu_rpy, target, motor_power, pid_u, eso_u):

        imu_yaw = imu_rpy[2]
        imu_roll = imu_rpy[0]
        imu_pitch = imu_rpy[1]

        self.file.write(
            f"{t:.3f},"
            f"{mode},"
            f"{gps_pos[0]:.4f},{gps_pos[1]:.4f},{gps_pos[2]:.4f},"
            f"{target[0]:.4f},{target[1]:.4f},{target[2]:.4f},"
            f"{pos_error:.4f},"
            f"{imu_yaw:.4f},"
            f"{eso_p[0]:.4f},{eso_p[1]:.4f},{eso_p[2]:.4f},"
            f"{eso_v[0]:.4f},{eso_v[1]:.4f},{eso_v[2]:.4f},"
            f"{eso_att[0]:.4f},{eso_att[1]:.4f},{eso_att[2]:.4f},"
            f"{eso_d_f[0]:.4f},{eso_d_f[1]:.4f},{eso_d_f[2]:.4f},"
            f"{imu_roll:.4f},{imu_pitch:.4f},"
            f"{motor_power[0]:.1f},{motor_power[1]:.1f},{motor_power[2]:.1f},{motor_power[3]:.1f},"
            f"{pid_u[0]:.1f},{pid_u[1]:.1f},{pid_u[2]:.1f},{pid_u[3]:.1f},"
            f"{eso_u[0]:.2f},{eso_u[1]:.2f},{eso_u[2]:.2f},{eso_u[3]:.2f}\n"
        )

        self.file.flush()

    def close(self):
        self.file.close()


# Shared tools
sys.path.append('../../../../controllers_shared/python_based')
from pid_controller import pid_velocity_fixed_height_controller
from eso import AttitudeESO
from design_L import compute_L

# =========================
# Control Options
# =========================
USE_DISTURBANCE_COMPENSATION = True
K_DIST_MOTOR = 0.3
DIST_FF_RATIO = 0.40

# =========================
# Logger
# =========================
logger = DataLogger("eso_hover_only")

# =========================
# SIM SETUP
# =========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())
Ts = timestep / 1000.0

m = 0.031
g = 9.81

# =========================
# ESO initialization
# =========================
eso = AttitudeESO(Ts, np.zeros((12, 6)), m, g)
L = compute_L(eso, m)
eso.L = L

print("\n" + "="*60)
print("ESO MODE: ACTIVE (side estimator)")
print("="*60 + "\n")

# =========================
# Sensors
# =========================
keyboard = Keyboard()
keyboard.enable(timestep)

imu = robot.getDevice("inertial_unit")
imu.enable(timestep)
gps = robot.getDevice("gps")
gps.enable(timestep)
gyro = robot.getDevice("gyro")
gyro.enable(timestep)

motors = [
    robot.getDevice("m1_motor"),
    robot.getDevice("m2_motor"),
    robot.getDevice("m3_motor"),
    robot.getDevice("m4_motor"),
]

for mtr in motors:
    mtr.setPosition(float('inf'))
    mtr.setVelocity(0.0)

# =========================
# Wait for sensors
# =========================
print("Waiting for sensor initialization...")
init_steps = 0
while robot.step(timestep) != -1:
    pos0 = np.array(gps.getValues())
    rpy0 = imu.getRollPitchYaw()

    if np.isfinite(pos0).all() and np.linalg.norm(pos0) > 1e-6:
        break


    init_steps += 1
    if init_steps > 100:
        print("Warning: slow sensor startup")
        break

print(f"Sensors OK: pos={pos0}, rpy={rpy0}")

# =========================
# ESO Initial Alignment
# =========================
eso.initialize_from_measurement(
    np.array([pos0[0], pos0[1], pos0[2], rpy0[0], rpy0[1], rpy0[2]])
)
print("ESO initialized")

# =========================
# Helper functions
# =========================
def body_velocity_from_global(vx_g, vy_g, yaw):
    cy, sy = np.cos(yaw), np.sin(yaw)
    vx = vx_g * cy + vy_g * sy
    vy = -vx_g * sy + vy_g * cy
    return vx, vy

# =========================
# PID Controller
# =========================
PID_CF = pid_velocity_fixed_height_controller()
T_cmd = m * g

Kp_pos = 1.0
Kd_pos = 0.1

HOVER_Z = 0.5
TARGET_X = pos0[0]
TARGET_Y = pos0[1]

PRINT_INTERVAL = 0.5
last_print = 0.0

past_time = robot.getTime()
past_pos = pos0.copy()

ESO_READY = True

print("Starting HOVER-ONLY PID + ESO (side) control loop...")

# =========================
# MAIN LOOP
# =========================
while robot.step(timestep) != -1:

    # Time
    t = robot.getTime()
    dt = max(t - past_time, 1e-6)
    past_time = t

    # Sensors
    pos = np.array(gps.getValues())
    roll, pitch, yaw = imu.getRollPitchYaw()
    omega_body = np.array(gyro.getValues())

    gps_vel = (pos - past_pos) / dt
    past_pos = pos.copy()

    if not np.isfinite(pos).all():
        print("Sensor error, skipping")
        continue

    # ==========================================================
    # ESO UPDATE
    # ==========================================================
    y_meas = np.array([pos[0], pos[1], pos[2], roll, pitch, yaw])
    z_hat = eso.step(y_meas, T_cmd, omega_body)

    eso_p = z_hat[0:3]
    eso_v = z_hat[3:6]
    eso_att = z_hat[6:9]
    eso_d_f = np.clip(z_hat[9:12], -2.0, 2.0)

    eso_roll, eso_pitch, eso_yaw = eso_att

    # ==========================================================
    # Hover Target
    # ==========================================================
    target = np.array([TARGET_X, TARGET_Y, HOVER_Z])

    # ==========================================================
    # Position Control (PD)
    # ==========================================================
    ctrl_pos = eso_p
    ctrl_vx_w, ctrl_vy_w, ctrl_vz = eso_v

    pos_err = target - ctrl_pos

    vx_des = Kp_pos * pos_err[0] + Kd_pos * (-ctrl_vx_w)
    vy_des = Kp_pos * pos_err[1] + Kd_pos * (-ctrl_vy_w)
    vz_des = Kp_pos * pos_err[2] + Kd_pos * (-ctrl_vz)

    vbx_des, vby_des = body_velocity_from_global(vx_des, vy_des, eso_yaw)
    vbx, vby = body_velocity_from_global(ctrl_vx_w, ctrl_vy_w, eso_yaw)

    # ==========================================================
    # Inner PID
    # ==========================================================
    motor_power = PID_CF.pid(
        dt,
        vbx_des, vby_des,
        0.0,
        target[2],
        roll, pitch,
        omega_body[2],
        pos[2],
        vbx, vby
    )

    motor_power = np.array(motor_power)
    pid_u = motor_power.copy()

    # ==========================================================
    # Disturbance Feedforward
    # ==========================================================
    if USE_DISTURBANCE_COMPENSATION and ESO_READY:

        cy, sy = np.cos(eso_yaw), np.sin(eso_yaw)
        R_wb = np.array([
            [cy, -sy, 0],
            [sy,  cy, 0],
            [0,   0,  1]
        ])

        d_world = m * eso_d_f
        Fx_b, Fy_b, Fz_b = R_wb.T @ d_world

        dT = -Fz_b
        d_taux = -0.002 * Fy_b
        d_tauy = -0.002 * Fx_b
        d_tauz = 0.0

        U_dist = np.array([dT, d_taux, d_tauy, d_tauz])

        B_inv = np.array([
            [0.25,  0.0,  0.5, -0.25],
            [0.25, -0.5,  0.0,  0.25],
            [0.25,  0.0, -0.5, -0.25],
            [0.25,  0.5,  0.0,  0.25],
        ])

        delta_u = DIST_FF_RATIO * K_DIST_MOTOR * (B_inv @ U_dist)
        eso_u = delta_u.copy()
        motor_power = motor_power + delta_u

    else:
        eso_u = np.zeros(4)

    # ==========================================================
    # Motor safety
    # ==========================================================
    motor_power = np.clip(motor_power, 0, 600)

    motors[0].setVelocity(-motor_power[0])
    motors[1].setVelocity(+motor_power[1])
    motors[2].setVelocity(-motor_power[2])
    motors[3].setVelocity(+motor_power[3])

    # ----------------------------------------------------------
    # Log Calculation
    # ----------------------------------------------------------
    current_pos_global = pos
    reference_pos_global = target

    pos_err_vec = current_pos_global - reference_pos_global
    pos_error_val = np.linalg.norm(pos_err_vec)

    current_mode = "HOVER"

    # ==========================================================
    # Logging
    # ==========================================================
    logger.log(
        t,
        current_mode,
        pos_error_val,
        pos,
        eso_p,
        eso_v,
        eso_att,
        eso_d_f,
        [roll, pitch, yaw],
        target,
        motor_power,
        pid_u,
        eso_u
    )

    # ==========================================================
    # Print
    # ==========================================================
    if t - last_print > PRINT_INTERVAL:
        last_print = t
        print("\n" + "="*60)
        print(f"Time {t:.2f} | Hovering target ({TARGET_X:.2f},{TARGET_Y:.2f},0.50)")
        print(f"Position actual ({ctrl_pos[0]:+.3f},{ctrl_pos[1]:+.3f},{ctrl_pos[2]:+.3f})")
        print(f"ESO Disturbance ({eso_d_f[0]:+.3f},{eso_d_f[1]:+.3f},{eso_d_f[2]:+.3f})")
        print(f"Motors {motor_power}")

logger.close()
print("Log file closed.")
