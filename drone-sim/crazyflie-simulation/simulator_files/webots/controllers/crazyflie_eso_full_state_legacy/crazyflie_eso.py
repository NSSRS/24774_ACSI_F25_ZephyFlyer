"""
Crazyflie outer-loop controller with Full-State ESO structure.
"""

from controller import Robot, Keyboard
import numpy as np
import sys
sys.path.append('../../../../controllers_shared/python_based')

from pid_controller import pid_velocity_fixed_height_controller
from eso import FullStateESO
from design_L import compute_L

# =========================
# ESO CONTROL MODE
# =========================
USE_ESO_FOR_CONTROL = False  # Set to True to use ESO estimates in control loop
                              # Set to False to only monitor/log ESO (use raw sensors)

# =========================
# Sim
# =========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())
Ts = timestep / 1000.0

# =========================
# ESO initialization
# =========================
m = 0.031 # AMY: Need to hand measure this
J = np.diag([1.4e-5, 1.4e-5, 2.17e-5])

# 1) Create ESO with zero gains first
eso = FullStateESO(Ts, np.zeros((18,6)), m, J)

# 2) Design L using system model
L = compute_L(eso, m)

# 3) Assign
eso.L = L

print(f"\n{'='*60}")
print(f"ESO MODE: {'ACTIVE (using ESO for control)' if USE_ESO_FOR_CONTROL else 'MONITORING ONLY (raw sensors for control)'}")
print(f"{'='*60}\n")

# =========================
# Read devices
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
for motor in motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

# =========================
# Wait for GPS and IMU initialization
# =========================
print("Waiting for sensor initialization...")
initialization_steps = 0
while robot.step(timestep) != -1:
    pos0 = np.array(gps.getValues())
    rpy0 = imu.getRollPitchYaw()
    
    if np.isfinite(pos0).all() and np.isfinite(rpy0).all():
        if np.linalg.norm(pos0) > 1e-6:
            break
    
    initialization_steps += 1
    if initialization_steps > 100:
        print("Warning: Sensor initialization taking longer than expected")
        break

print(f"GPS initialized at position {pos0}")

# =========================
# Initialize ESO from first measurement
# =========================
roll0, pitch0, yaw0 = imu.getRollPitchYaw()
y_init = np.array([pos0[0], pos0[1], pos0[2], roll0, pitch0, yaw0])
eso.initialize_from_measurement(y_init)
print(f"ESO initialized with state: pos={pos0}, rpy=({roll0:.3f},{pitch0:.3f},{yaw0:.3f})")

# =========================
# Helper
# =========================
def body_velocity_from_global(vx_g, vy_g, yaw):
    """Transform global velocities to body frame."""
    cosy, siny = np.cos(yaw), np.sin(yaw)
    vx = vx_g * cosy + vy_g * siny
    vy = -vx_g * siny + vy_g * cosy
    return vx, vy

def global_velocity_from_body(vx_b, vy_b, yaw):
    """Transform body velocities to global frame."""
    cosy, siny = np.cos(yaw), np.sin(yaw)
    vx_g = vx_b * cosy - vy_b * siny
    vy_g = vx_b * siny + vy_b * cosy
    return vx_g, vy_g

# =========================
# Inner PID controller
# =========================
PID_CF = pid_velocity_fixed_height_controller()

# =========================
# Desired hover control input
# =========================
T_cmd = m * 9.81
tau_x_cmd = 0.0
tau_y_cmd = 0.0
tau_z_cmd = 0.0

# =========================
# Target position
# =========================
target = np.array([0.0, 0.0, 0.5])

# =========================
# PD gains
# =========================
Kp_pos = 2.0   # 1.5
Kd_pos = 0.8   # 0.4

past_time = robot.getTime()
past_pos = pos0.copy()

# =========================
# Velocity limits
# =========================
VEL_LIMIT = 0.5  # m/s

# =========================
# Print timing control
# =========================
last_print_time = 0.0
PRINT_INTERVAL = 0.1  # seconds

# =========================
# MAIN LOOP
# =========================
print("Starting main control loop...")
loop_count = 0

while robot.step(timestep) != -1:

    t = robot.getTime()
    dt = max(t - past_time, 1e-6)
    past_time = t
    loop_count += 1

    # ------------------------------
    # 1) Read sensors
    # ------------------------------
    pos = np.array(gps.getValues())
    roll, pitch, yaw = imu.getRollPitchYaw()  # Direct from IMU
    omega_body = np.array(gyro.getValues())
    yaw_rate = omega_body[2]

    # Sanity check for sensor readings
    if not (np.isfinite(pos).all() and np.isfinite([roll, pitch, yaw]).all()):
        print(f"Warning: Invalid sensor readings at t={t:.2f}")
        continue

    # Compute velocity from GPS (simple differentiation for baseline)
    gps_velocity = (pos - past_pos) / dt
    past_pos = pos.copy()

    # ------------------------------
    # 2) ESO update (ALWAYS runs for monitoring)
    # ------------------------------
    y_meas = np.array([pos[0], pos[1], pos[2],
                       roll, pitch, yaw])

    u_cmd = np.array([T_cmd, tau_x_cmd, tau_y_cmd, tau_z_cmd])
    z_hat = eso.step(y_meas, u_cmd)

    # Extract estimated states
    eso_p = z_hat[0:3]
    eso_v = z_hat[3:6]      # Velocity in WORLD frame
    eso_euler = z_hat[6:9]
    eso_omega = z_hat[9:12]
    eso_d_f   = z_hat[12:15]  # Force disturbances
    eso_d_tau = z_hat[15:18]  # Torque disturbances

    eso_roll, eso_pitch, eso_yaw = eso_euler
    eso_vx_world, eso_vy_world, eso_vz = eso_v

    # ------------------------------
    # 3) Choose control source based on USE_ESO_FOR_CONTROL
    # ------------------------------
    if USE_ESO_FOR_CONTROL:
        # Use ESO estimates for control
        ctrl_pos = eso_p
        ctrl_vx_world = eso_vx_world
        ctrl_vy_world = eso_vy_world
        ctrl_vz = eso_vz
        ctrl_yaw = yaw  # Still use IMU for yaw (more reliable)
        source_label = "ESO"
    else:
        # Use raw sensors for control (baseline)
        ctrl_pos = pos
        ctrl_vx_world = gps_velocity[0]
        ctrl_vy_world = gps_velocity[1]
        ctrl_vz = gps_velocity[2]
        ctrl_yaw = yaw
        source_label = "GPS"

    # ------------------------------
    # 4) Position → velocity command (in WORLD frame)
    # ------------------------------
    pos_err = target - ctrl_pos

    vx_des_world = Kp_pos * pos_err[0] + Kd_pos * (-ctrl_vx_world)
    vy_des_world = Kp_pos * pos_err[1] + Kd_pos * (-ctrl_vy_world)
    vz_des_world = Kp_pos * pos_err[2] + Kd_pos * (-ctrl_vz)

    vx_des_world = np.clip(vx_des_world, -VEL_LIMIT, VEL_LIMIT)
    vy_des_world = np.clip(vy_des_world, -VEL_LIMIT, VEL_LIMIT)
    vz_des_world = np.clip(vz_des_world, -VEL_LIMIT, VEL_LIMIT)

    # ------------------------------
    # 5) Transform to body frame for PID controller
    # ------------------------------
    v_body_x_des, v_body_y_des = body_velocity_from_global(
        vx_des_world, vy_des_world, ctrl_yaw
    )

    forward_desired = v_body_x_des
    sideways_desired = v_body_y_des
    yaw_desired = 0.0
    height_desired = target[2]

    # ------------------------------
    # 6) Inner PID controller
    # ------------------------------
    # Convert control velocities to body frame
    ctrl_vx_body, ctrl_vy_body = body_velocity_from_global(
        ctrl_vx_world, ctrl_vy_world, ctrl_yaw
    )

    # Use IMU attitude directly (always - most reliable)
    motor_power = PID_CF.pid(
        dt,
        forward_desired,      # Body-frame x velocity command
        sideways_desired,     # Body-frame y velocity command
        yaw_desired,          # Yaw angle command
        height_desired,       # Altitude command
        roll,                 # IMU roll
        pitch,                # IMU pitch
        yaw_rate,             # Gyro yaw rate
        ctrl_pos[2],          # Altitude (ESO or GPS)
        ctrl_vx_body,         # Body velocity x (ESO or GPS-derived)
        ctrl_vy_body          # Body velocity y (ESO or GPS-derived)
    )

    # ------------------------------
    # 7) Safety checks and motor application
    # ------------------------------
    # Clamp motor commands
    motor_power = np.clip(motor_power, 0, 600)

    # Sanity check
    if np.any(np.isnan(motor_power)) or np.any(np.isinf(motor_power)):
        print(f"ERROR: Invalid motor commands at t={t:.2f}")
        motor_power = np.array([100, 100, 100, 100])

    # Attitude safety check
    if abs(roll) > np.pi/3 or abs(pitch) > np.pi/3:  # > 60°
        print(f"WARNING: Extreme attitude at t={t:.2f}, roll={roll*180/np.pi:.1f}°, pitch={pitch*180/np.pi:.1f}°")

    # Apply to motors
    motors[0].setVelocity(-motor_power[0])
    motors[1].setVelocity( motor_power[1])
    motors[2].setVelocity(-motor_power[2])
    motors[3].setVelocity( motor_power[3])

    # ------------------------------
    # 8) Debug prints
    # ------------------------------
    if (t - last_print_time) >= PRINT_INTERVAL:
        last_print_time = t
        
        pos_error_norm = np.linalg.norm(pos_err)
        vel_world_norm = np.linalg.norm([eso_vx_world, eso_vy_world, eso_vz])
        gps_vel_norm = np.linalg.norm(gps_velocity)
        
        print(f"\n{'='*60}")
        print(f"Time: {t:.2f}s | Control Source: {source_label}")
        print(f"Position: GPS=({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}), "
              f"ESO=({eso_p[0]:+.3f}, {eso_p[1]:+.3f}, {eso_p[2]:+.3f})")
        print(f"Error: ({pos_err[0]:+.3f}, {pos_err[1]:+.3f}, {pos_err[2]:+.3f}), "
              f"norm={pos_error_norm:.3f}m")
        
        # Show both velocity sources for comparison
        print(f"Velocity ESO:  ({eso_vx_world:+.3f}, {eso_vy_world:+.3f}, {eso_vz:+.3f}), "
              f"norm={vel_world_norm:.3f}m/s")
        print(f"Velocity GPS:  ({gps_velocity[0]:+.3f}, {gps_velocity[1]:+.3f}, {gps_velocity[2]:+.3f}), "
              f"norm={gps_vel_norm:.3f}m/s")
        print(f"Velocity CTRL: ({ctrl_vx_world:+.3f}, {ctrl_vy_world:+.3f}, {ctrl_vz:+.3f}) "
              f"[using {source_label}]")
        
        print(f"Attitude (IMU): roll={roll*180/np.pi:+.1f}°, "
              f"pitch={pitch*180/np.pi:+.1f}°, "
              f"yaw={yaw*180/np.pi:+.1f}°")
        
        # ESO disturbances (always shown for monitoring)
        print(f"Disturbances: F=({eso_d_f[0]:+.4f}, {eso_d_f[1]:+.4f}, {eso_d_f[2]:+.4f})N, "
              f"τ=({eso_d_tau[0]:+.5f}, {eso_d_tau[1]:+.5f}, {eso_d_tau[2]:+.5f})N·m")
        print(f"Motor power: [{motor_power[0]:.1f}, {motor_power[1]:.1f}, "
              f"{motor_power[2]:.1f}, {motor_power[3]:.1f}]")