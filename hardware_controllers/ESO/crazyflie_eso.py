"""
Crazyflie ESO Hover-Only Controller
— 只起飞、只悬停、不绕圈、不降落 —
"""

from controller import Robot, Keyboard
import numpy as np
import sys

# 共享工具
sys.path.append('../../../../controllers_shared/python_based')
from plotter import DataLogger
from pid_controller import pid_velocity_fixed_height_controller
from eso import AttitudeESO
from design_L import compute_L

# =========================
# Control Options
# =========================
USE_DISTURBANCE_COMPENSATION = True   # 先默认关掉扰动前馈，保证稳定
K_DIST_MOTOR = 0.3                    # 电机扰动补偿尺度
DIST_FF_RATIO = 0.40                  # ESO 前馈占比（开启时生效）

# =========================
# Logger
# =========================
logger = DataLogger("eso_hover")

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
print(f"ESO MODE: ACTIVE (side estimator)")
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
    robot.getDevice("m1_motor"),   # rear-right  (CCW)  → -cmd
    robot.getDevice("m2_motor"),   # rear-left   (CW)   → +cmd
    robot.getDevice("m3_motor"),   # front-left  (CCW)  → -cmd
    robot.getDevice("m4_motor"),   # front-right (CW)   → +cmd
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
    """把世界系速度投影到机体系（Crazyflie 经典做法）"""
    cy, sy = np.cos(yaw), np.sin(yaw)
    vx = vx_g * cy + vy_g * sy
    vy = -vx_g * sy + vy_g * cy
    return vx, vy

# =========================
# PID Controller (沿用你 9_1 的结构)
# =========================
PID_CF = pid_velocity_fixed_height_controller()
T_cmd = m * g   # 只作为 ESO 输入，不直接用在控制里

# 位置外环 PD（用 ESO 的 pos/vel）
Kp_pos = 1.0
Kd_pos = 0.1

# 固定悬停目标
HOVER_Z = 0.5
TARGET_X = pos0[0]
TARGET_Y = pos0[1]

PRINT_INTERVAL = 0.5
last_print = 0.0

past_time = robot.getTime()
past_pos = pos0.copy()

ESO_READY = True   # 已经初始化完，可以用

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
    # ESO UPDATE (只做估计，不主导姿态)
    # ==========================================================
    y_meas = np.array([pos[0], pos[1], pos[2], roll, pitch, yaw])
    z_hat = eso.step(y_meas, T_cmd, omega_body)

    eso_p   = z_hat[0:3]
    eso_v   = z_hat[3:6]
    eso_att = z_hat[6:9]
    eso_d_f = np.clip(z_hat[9:12], -2.0, 2.0)

    eso_roll, eso_pitch, eso_yaw = eso_att

    # ==========================================================
    # Hover Target
    # ==========================================================
    target = np.array([TARGET_X, TARGET_Y, HOVER_Z])

    # ==========================================================
    # Position Control (PD in world frame, 用 ESO 的 pos/vel)
    # ==========================================================
    ctrl_pos = eso_p
    ctrl_vx_w, ctrl_vy_w, ctrl_vz = eso_v

    pos_err = target - ctrl_pos

    vx_des = Kp_pos * pos_err[0] + Kd_pos * (-ctrl_vx_w)
    vy_des = Kp_pos * pos_err[1] + Kd_pos * (-ctrl_vy_w)
    vz_des = Kp_pos * pos_err[2] + Kd_pos * (-ctrl_vz)

    # Body frame velocity
    vbx_des, vby_des = body_velocity_from_global(vx_des, vy_des, eso_yaw)
    vbx,     vby     = body_velocity_from_global(ctrl_vx_w, ctrl_vy_w, eso_yaw)

    # ==========================================================
    # Inner PID （沿用你原来的 pid_velocity_fixed_height_controller）
    # ==========================================================
    motor_power = PID_CF.pid(
        dt,
        vbx_des, vby_des,
        0.0,                # desired yaw rate = 0
        target[2],          # desired altitude
        roll, pitch,        # 这里用 IMU 姿态更稳
        omega_body[2],      # yaw rate
        pos[2],             # 当前高度
        vbx, vby
    )

    motor_power = np.array(motor_power)
    pid_u = motor_power.copy()   # 单独记录 PID 输出

    # ==========================================================
    # Disturbance Feedforward (在 motor 空间的小修正，可关)
    # ==========================================================
    if USE_DISTURBANCE_COMPENSATION and ESO_READY:

        # 只用 yaw 做旋转，减少耦合和数值抖动
        cy, sy = np.cos(eso_yaw), np.sin(eso_yaw)
        R_wb = np.array([
            [ cy, -sy, 0.0 ],
            [ sy,  cy, 0.0 ],
            [0.0, 0.0, 1.0 ]
        ])

        # disturbance acceleration → 力(N)
        d_world = m * eso_d_f
        Fx_b, Fy_b, Fz_b = R_wb.T @ d_world

        dT    = -Fz_b               # 垂直方向扰动 → thrust 修正
        d_taux = -0.002 * Fy_b      # 经验系数
        d_tauy = -0.002 * Fx_b
        d_tauz = 0.0

        U_dist = np.array([dT, d_taux, d_tauy, d_tauz])

        # Crazyflie motor mixing 反矩阵（但现在只当作“经验分配矩阵”）
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

    # Apply motors (注意方向映射)
    motors[0].setVelocity(-motor_power[0])
    motors[1].setVelocity( motor_power[1])
    motors[2].setVelocity(-motor_power[2])
    motors[3].setVelocity( motor_power[3])

    # ==========================================================
    # Log: t, gps, eso_p, eso_v, eso_att, dist, imu_rpy, target, final motors, pid, eso
    # ==========================================================
    logger.log(
        t, pos, eso_p, eso_v, eso_att,
        eso_d_f, [roll, pitch, yaw],
        target, motor_power,
        pid_u, eso_u
    )

    # Print
    if t - last_print > PRINT_INTERVAL:
        last_print = t
        print("\n" + "="*60)
        print(f"Time {t:.2f} | Hovering target ({TARGET_X:.2f},{TARGET_Y:.2f},0.50)")
        print(f"Position actual ({ctrl_pos[0]:+.3f},{ctrl_pos[1]:+.3f},{ctrl_pos[2]:+.3f})")
        print(f"ESO Disturbance ({eso_d_f[0]:+.3f},{eso_d_f[1]:+.3f},{eso_d_f[2]:+.3f})")
        print(f"Motors {motor_power}")

logger.close()
print("Log file closed.")
