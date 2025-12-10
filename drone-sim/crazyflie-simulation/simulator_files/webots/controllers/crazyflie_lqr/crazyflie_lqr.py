#!/usr/bin/env python3
# FULL Webots-ready LQR controller for Crazyflie 2.1
# 12-state LQR (earth-frame dynamics, consistent with TinyMPC-style model)
#
# State x:
#   [ x, y, z,
#     vx, vy, vz,
#     roll, pitch, yaw,
#     p, q, r ]
#
# Input u:
#   [ T, tau_x, tau_y, tau_z ]
#
# Hover equilibrium:
#   x_ref = [0, 0, z_ref, 0,0,0, 0,0,0, 0,0,0]
#   u_eq  = [mass*g, 0, 0, 0]

import numpy as np
from controller import Robot
from scipy.linalg import solve_continuous_are

print("\n===== Crazyflie LQR Controller Started — TinyMPC-Consistent Model =====")

# ================================
# Crazyflie Physical Parameters
# (match TinyMPC)
# ================================
mass = 3.4e-02   # kg
g    = 9.81

Ix = 1.6e-5       # kg·m^2
Iy = 1.6e-5
Iz = 2.9e-5

L  = 0.031        # arm length (m)
kf = 4e-05        # thrust constant (N / (rad/s)^2)
km = 2.4e-06      # drag torque constant
max_vel = 600.0   # motor speed limit (rad/s, Webots units)

# ================================
# 12-State Linearized Earth-Frame Model
# ================================
# State order: [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
# Input: [T, tau_x, tau_y, tau_z]
#
# TinyMPC convention:
#   vx_dot =  g * theta
#   vy_dot = -g * roll
#   vz_dot = T / mass         (here T is absolute thrust)
def get_AB():
    A = np.zeros((12, 12))

    # Position derivatives
    A[0, 3] = 1.0   # x_dot = vx
    A[1, 4] = 1.0   # y_dot = vy
    A[2, 5] = 1.0   # z_dot = vz

    # Linearized at hover (T ≈ mass*g)
    # Horizontal accelerations from tilting thrust vector
    A[3, 7] =  g    # vx_dot =  g * theta (pitch)
    A[4, 6] = -g    # vy_dot = -g * phi   (roll)

    # Attitude kinematics
    A[6, 9]  = 1.0  # roll_dot  = p
    A[7, 10] = 1.0  # pitch_dot = q
    A[8, 11] = 1.0  # yaw_dot   = r

    # Input matrix
    B = np.zeros((12, 4))
    B[5, 0]  = 1.0 / mass  # vz_dot from T (absolute thrust)
    B[9, 1]  = 1.0 / Ix    # p_dot from tau_x
    B[10, 2] = 1.0 / Iy    # q_dot from tau_y
    B[11, 3] = 1.0 / Iz    # r_dot from tau_z

    return A, B

# ================================
# LQR Gain for 12 states
# (Q, R match TinyMPC cost structure)
# ================================
def compute_LQR(A, B):
    # Q matches state order: [x,y,z, vx,vy,vz, roll,pitch,yaw, p,q,r]
    Q = np.diag([
        5.0e+01, 5.0e+01, 8.0e+01,   # position (x, y, z)
        3.0e+01, 3.0e+01, 5.0e+01,   # velocity (vx, vy, vz)
        5.0e+01, 5.0e+01, 2.0e+01,   # angles (roll, pitch, yaw)
        1.0e+00, 1.0e+00, 1.0e+00    # angular rates (p, q, r)
    ])

    # Input cost: reduced thrust penalty to allow larger control authority
    R = np.diag([5, 10, 10, 5000])

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K

# ================================
# Motor Mixing (same as your original)
# ================================
# Mapping: f = M_inv @ u
#   f  = [f1, f2, f3, f4] rotor thrusts (N)
#   u  = [T, tau_x, tau_y, tau_z]
M = np.array([
    [ kf,      kf,      kf,      kf    ],  # Total thrust
    [-kf*L,    kf*L,    kf*L,   -kf*L  ],  # τx (roll)
    [-kf*L,   -kf*L,    kf*L,    kf*L  ],  # τy (pitch)
    [-km,      km,     -km,      km    ]   # τz (yaw)
])
M_inv = np.linalg.inv(M)

def mix(u):
    """Map virtual inputs [T, tau_x, tau_y, tau_z] to rotor thrusts [f1..f4]."""
    return M_inv @ u

# ================================
# Webots Setup
# ================================
robot = Robot()
dt = int(robot.getBasicTimeStep())
Ts = dt / 1000.0

# motors
motors = []
motor_names = ["m1_motor", "m2_motor", "m3_motor", "m4_motor"]
for name in motor_names:
    motor = robot.getDevice(name)
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)
    motors.append(motor)

# sensors
imu  = robot.getDevice("inertial_unit")
gyro = robot.getDevice("gyro")
gps  = robot.getDevice("gps")

imu.enable(dt)
gyro.enable(dt)
gps.enable(dt)

# ================================
# Init LQR
# ================================
A, B = get_AB()
K = compute_LQR(A, B)

hover_thrust = mass * g  # N

# ================================
# State Extraction (WORLD-FRAME VELOCITY)
# ================================
prev_pos = np.zeros(3)
prev_t = 0.0

def get_state():
    """
    Build state vector:
    x = [x,y,z, vx,vy,vz, roll,pitch,yaw, p,q,r]
    with velocities in WORLD frame.
    """
    global prev_pos, prev_t

    # Position (world frame)
    px, py, pz = gps.getValues()
    pos = np.array([px, py, pz])

    # Velocity via finite difference (world frame)
    t = robot.getTime()
    dt_local = max(t - prev_t, 1e-4)
    v_world = (pos - prev_pos) / dt_local
    prev_pos = pos.copy()
    prev_t = t

    vx, vy, vz = v_world

    # Attitude (roll, pitch, yaw)
    roll, pitch, yaw = imu.getRollPitchYaw()

    # Angular rates (body frame)
    p, q, r = gyro.getValues()

    return np.array([
        px, py, pz,
        vx, vy, vz,
        roll, pitch, yaw,
        p, q, r
    ])

# ================================
# Reference Hover (0.5 m)
# ================================
x_ref = np.zeros(12)
x_ref[2] = 0.5  # desired z in meters

u_eq = np.array([hover_thrust, 0.0, 0.0, 0.0])

# simple clamps for safety
T_min = 0.5 * hover_thrust
T_max = 1.5 * hover_thrust

# Torque limits
tau_max = 2e-4  # N·m

# ================================
# Main Control Loop
# ================================
while robot.step(dt) != -1:

    x = get_state()
    e = x - x_ref

    # LQR command (deviation from equilibrium)
    u_delta = -K @ e  # [ΔT_abs-ish, τx, τy, τz] but treated as correction

    # Add equilibrium (hover) input (absolute thrust)
    u = u_eq + u_delta

    # Thrust clamp to avoid rocket mode
    u[0] = np.clip(u[0], T_min, T_max)

    # Soft clamp torques (symmetric)
    u[1:] = np.clip(u[1:], -tau_max, tau_max)

    # Map [T, tau_x, tau_y, tau_z] -> rotor squared speeds
    omega_sq = mix(u)   # (rad/s)^2

    speeds = []
    for w2 in omega_sq:
        w2 = max(w2, 0.0)
        omega = np.sqrt(w2)      # rad/s
        omega = min(omega, max_vel)
        speeds.append(omega)

    # Apply speeds
    for motor, s in zip(motors, speeds):
        motor.setVelocity(float(s))

    # ============================
    # DEBUG PRINTS (every 0.05 s)
    # ============================
    print_timer = getattr(robot, "print_timer", 0.0)
    t = robot.getTime()
    if t - print_timer > 0.05:

        print("\n---------------- LQR DEBUG ----------------")
        print(f"Time: {t:.3f} s")

        # --- State ---
        print(f"State: pos=({x[0]:+.3f}, {x[1]:+.3f}, {x[2]:+.3f})  "
              f"att=({x[6]:+.3f}, {x[7]:+.3f}, {x[8]:+.3f})")

        print(f"Vel:   vx={x[3]:+.3f}, vy={x[4]:+.3f}, vz={x[5]:+.3f}")
        print(f"Rates: p={x[9]:+.3f}, q={x[10]:+.3f}, r={x[11]:+.3f}")

        # --- Error ---
        print(f"Err pos:  ex={e[0]:+.3f}, ey={e[1]:+.3f}, ez={e[2]:+.3f}")
        print(f"Err att:  eφ={e[6]:+.3f}, eθ={e[7]:+.3f}, eψ={e[8]:+.3f}")
        print(f"‖e‖ = {np.linalg.norm(e):.3f}")

        # --- Control before clamp ---
        print(f"u_delta (LQR raw): "
              f"T={u_delta[0]:+.3f}, "
              f"τx={u_delta[1]:+.3e}, τy={u_delta[2]:+.3e}, τz={u_delta[3]:+.3e}")

        # --- Control after clamp ---
        print(f"u final:  T={u[0]:+.3f}, "
              f"τx={u[1]:+.3e}, τy={u[2]:+.3e}, τz={u[3]:+.3e}")

        # --- Motor thrusts ---
        print(f"Rotor omega^2: "
            f"w1^2={omega_sq[0]:+.2f}, w2^2={omega_sq[1]:+.2f}, "
            f"w3^2={omega_sq[2]:+.2f}, w4^2={omega_sq[3]:+.2f}")

        # --- Speeds ---
        print("Motor speeds:", ", ".join([f"{s:.1f}" for s in speeds]))

        # --- Special yaw debug ---
        print(f"Yaw e={e[8]:+.3f}, yaw rate r={x[11]:+.3f}, τz={u[3]:+.3e}")
        print("-------------------------------------------\n")

        robot.print_timer = t
