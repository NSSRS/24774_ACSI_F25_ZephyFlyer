"""
Crazyflie ESO Hover-Only Controller
— 只起飞、只悬停、不绕圈、不降落 —
"""

from controller import Robot, Keyboard
import numpy as np
import sys
sys.path.append('../../../../controllers_shared/python_based')
from plotter import DataLogger
logger = DataLogger("eso_hover")

from pid_controller import pid_velocity_fixed_height_controller
from eso import AttitudeESO
from design_L import compute_L

# =========================
# ESO CONTROL MODE
# =========================
USE_ESO_FOR_CONTROL = True
USE_DISTURBANCE_COMPENSATION = True
K_DIST_MOTOR = 0.3          # 电机扰动补偿尺度 0.3
DIST_FF_RATIO = 0.40        # 只用 20% ESO 前馈，其余交给 PID

# =========================
# SIM SETUP
# =========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())
Ts = timestep / 1000.0

m = 0.031
g = 9.81

# =========================
# ESO initialization (12 states)
# =========================
eso = AttitudeESO(Ts, np.zeros((12, 6)), m, g)
L = compute_L(eso, m)
eso.L = L

print("\n" + "="*60)
print(f"ESO MODE: ACTIVE")
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
# Helper
# =========================
def body_velocity_from_global(vx_g, vy_g, yaw):
    cy, sy = np.cos(yaw), np.sin(yaw)
    vx = vx_g * cy + vy_g * sy
    vy = -vx_g * sy + vy_g * cy
    return vx, vy

# =========================
# PID + Hover Target
# =========================
PID_CF = pid_velocity_fixed_height_controller()
T_cmd = m * g

Kp_pos = 1.0
Kd_pos = 0.1

# ======== 固定悬停目标 ========
HOVER_Z = 0.5
TARGET_X = pos0[0]
TARGET_Y = pos0[1]

PRINT_INTERVAL = 0.5
last_print = 0.0

past_time = robot.getTime()
past_pos = pos0.copy()

ESO_READY = True     # 已经初始化，不需要延迟

# =========================
# MAIN LOOP
# =========================
print("Starting HOVER-ONLY control loop...")

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
    # Hover Target (ONLY)
    # ==========================================================
    target = np.array([TARGET_X, TARGET_Y, HOVER_Z])
    vel_ff = np.zeros(3)

    # ==========================================================
    # Position Control (PD)
    # ==========================================================
    ctrl_pos = eso_p
    ctrl_vx_w, ctrl_vy_w, ctrl_vz = eso_v

    pos_err = target - ctrl_pos

    vx_des = Kp_pos * pos_err[0] + Kd_pos * (-ctrl_vx_w)
    vy_des = Kp_pos * pos_err[1] + Kd_pos * (-ctrl_vy_w)
    vz_des    = Kp_pos * pos_err[2] + Kd_pos * (-ctrl_vz)

    # Body frame velocity
    vbx_des, vby_des = body_velocity_from_global(vx_des, vy_des, eso_yaw)
    vbx, vby = body_velocity_from_global(ctrl_vx_w, ctrl_vy_w, eso_yaw)

    # ==========================================================
    # Inner PID
    # ==========================================================
    motor_power = PID_CF.pid(
        dt,
        vbx_des, vby_des,
        0.0,                # desired yaw rate = 0
        target[2],          # desired altitude
        eso_roll, eso_pitch,
        omega_body[2],      # yaw rate
        ctrl_pos[2],
        vbx, vby
    )

    # ==========================================================
    # Disturbance Feedforward (ESO-based, 20% 比例)
    # ==========================================================
        # ==========================================================
    # Disturbance Feedforward (ESO-based)
    # ==========================================================
    if USE_DISTURBANCE_COMPENSATION and ESO_READY:

        # 只用 yaw 做旋转，减少耦合和数值抖动
        cy, sy = np.cos(eso_yaw), np.sin(eso_yaw)
        R_wb = np.array([
            [ cy, -sy, 0.0 ],
            [ sy,  cy, 0.0 ],
            [0.0, 0.0, 1.0 ]
        ])

        # ❗ 这里把“每单位质量的扰动”变回“力（N）”
        d_world = m * eso_d_f      # eso_d_f ~ 加速度，乘 m 变成 F (N)
        Fx_b, Fy_b, Fz_b = R_wb.T @ d_world

        # thrust + roll/pitch torque 补偿
        dT   = -Fz_b               # 垂直方向扰动 → thrust
        d_taux = -0.002 * Fy_b     # 经验系数：侧向力 → roll torque
        d_tauy = -0.002 * Fx_b     # 经验系数：前后力 → pitch torque
        d_tauz = 0.0               # 不做 yaw 扰动补偿

        U_dist = np.array([dT, d_taux, d_tauy, d_tauz])

        B_inv = np.array([
            [0.25,  0.0,  0.5, -0.25],
            [0.25, -0.5,  0.0,  0.25],
            [0.25,  0.0, -0.5, -0.25],
            [0.25,  0.5,  0.0,  0.25],
        ])

        delta_u = DIST_FF_RATIO * K_DIST_MOTOR * (B_inv @ U_dist)
        motor_power = np.array(motor_power) + delta_u


    # Motor safety
    motor_power = np.clip(motor_power, 0, 600)

    # Apply motors (注意方向映射)
    motors[0].setVelocity(-motor_power[0])
    motors[1].setVelocity( motor_power[1])
    motors[2].setVelocity(-motor_power[2])
    motors[3].setVelocity( motor_power[3])

    # Log
    logger.log(
        t, pos, eso_p, eso_v, eso_att,
        eso_d_f, [roll, pitch, yaw],
        target, motor_power
    )

    # Print
    if t - last_print > PRINT_INTERVAL:
        last_print = t
        print("\n" + "="*60)
        print(f"Time {t:.2f} | Hovering at ({TARGET_X:.2f},{TARGET_Y:.2f},0.50)")
        print(f"Position actual ({ctrl_pos[0]:+.3f},{ctrl_pos[1]:+.3f},{ctrl_pos[2]:+.3f})")
        print(f"ESO Disturbance ({eso_d_f[0]:+.3f},{eso_d_f[1]:+.3f},{eso_d_f[2]:+.3f}) N")
        print(f"Motors {motor_power}")

logger.close()
print("Log file closed.")
