"""
Crazyflie outer-loop controller with 12-State ESO.
"""

from controller import Robot, Keyboard
import numpy as np
import sys
sys.path.append('../../../../controllers_shared/python_based')

from pid_controller import pid_velocity_fixed_height_controller
from eso import AttitudeESO
from design_L import compute_L

# =========================
# ESO CONTROL MODE
# =========================
USE_ESO_FOR_CONTROL = True

# =========================
# Sim
# =========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())
Ts = timestep / 1000.0

# =========================
# ESO initialization (12-state)
# =========================
m = 0.031
g = 9.81

# 1) Create ESO with zero gains first
eso = AttitudeESO(Ts, np.zeros((12,6)), m, g)

# 2) Design L
L = compute_L(eso, m)

# 3) Assign
eso.L = L

print(f"\n{'='*60}")
print(f"ESO MODE: {'ACTIVE' if USE_ESO_FOR_CONTROL else 'MONITORING ONLY'}")
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
# Wait for initialization
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

print(f"Sensors initialized: pos={pos0}, rpy={rpy0}")

# =========================
# Initialize ESO
# =========================
y_init = np.array([pos0[0], pos0[1], pos0[2], rpy0[0], rpy0[1], rpy0[2]])
eso.initialize_from_measurement(y_init)
print(f"ESO initialized")

# =========================
# Helper
# =========================
def body_velocity_from_global(vx_g, vy_g, yaw):
    cosy, siny = np.cos(yaw), np.sin(yaw)
    vx = vx_g * cosy + vy_g * siny
    vy = -vx_g * siny + vy_g * cosy
    return vx, vy

# =========================
# Controller
# =========================
PID_CF = pid_velocity_fixed_height_controller()
T_cmd = m * g
target = np.array([0.0, 0.0, 0.5])

Kp_pos = 0.5
Kd_pos = 0.3
VEL_LIMIT = 0.3

past_time = robot.getTime()
past_pos = pos0.copy()

PRINT_INTERVAL = 0.5
last_print_time = 0.0

# =========================
# MAIN LOOP
# =========================
print("Starting main control loop...")

while robot.step(timestep) != -1:
    t = robot.getTime()
    dt = max(t - past_time, 1e-6)
    past_time = t

    # Read sensors
    pos = np.array(gps.getValues())
    roll, pitch, yaw = imu.getRollPitchYaw()
    omega_body = np.array(gyro.getValues())
    yaw_rate = omega_body[2]

    if not (np.isfinite(pos).all() and np.isfinite([roll, pitch, yaw]).all()):
        print(f"Warning: Invalid sensor readings at t={t:.2f}")
        continue

    gps_velocity = (pos - past_pos) / dt
    past_pos = pos.copy()

    # ESO update - NOW with IMU measurements!
    y_meas = np.array([pos[0], pos[1], pos[2], roll, pitch, yaw])
    z_hat = eso.step(y_meas, T_cmd, omega_body)

    # Extract states
    eso_p = z_hat[0:3]
    eso_v = z_hat[3:6]
    eso_att = z_hat[6:9]    # Estimated attitude!
    eso_d_f = z_hat[9:12]
    
    eso_roll, eso_pitch, eso_yaw = eso_att

    # Choose control source
    if USE_ESO_FOR_CONTROL:
        ctrl_pos = eso_p
        ctrl_vx_world, ctrl_vy_world, ctrl_vz = eso_v
        ctrl_roll = eso_roll      # Use ESO attitude!
        ctrl_pitch = eso_pitch
        ctrl_yaw = eso_yaw
        source_label = "ESO"
    else:
        ctrl_pos = pos
        ctrl_vx_world, ctrl_vy_world, ctrl_vz = gps_velocity
        ctrl_roll = roll          # Use IMU attitude
        ctrl_pitch = pitch
        ctrl_yaw = yaw
        source_label = "GPS+IMU"

    # Position control
    pos_err = target - ctrl_pos
    vx_des = Kp_pos * pos_err[0] + Kd_pos * (-ctrl_vx_world)
    vy_des = Kp_pos * pos_err[1] + Kd_pos * (-ctrl_vy_world)
    vz_des = Kp_pos * pos_err[2] + Kd_pos * (-ctrl_vz)

    vx_des = np.clip(vx_des, -VEL_LIMIT, VEL_LIMIT)
    vy_des = np.clip(vy_des, -VEL_LIMIT, VEL_LIMIT)
    vz_des = np.clip(vz_des, -VEL_LIMIT, VEL_LIMIT)

    # Transform to body frame
    v_body_x_des, v_body_y_des = body_velocity_from_global(vx_des, vy_des, ctrl_yaw)
    ctrl_vx_body, ctrl_vy_body = body_velocity_from_global(ctrl_vx_world, ctrl_vy_world, ctrl_yaw)

    # Inner PID - USE ESO ATTITUDE if enabled!
    motor_power = PID_CF.pid(
        dt,
        v_body_x_des,
        v_body_y_des,
        0.0,
        target[2],
        ctrl_roll,       # ESO or IMU
        ctrl_pitch,      # ESO or IMU
        yaw_rate,
        ctrl_pos[2],
        ctrl_vx_body,
        ctrl_vy_body
    )

    # Safety
    motor_power = np.clip(motor_power, 0, 600)

    if np.any(np.isnan(motor_power)) or np.any(np.isinf(motor_power)):
        print(f"ERROR: Invalid motor commands at t={t:.2f}")
        motor_power = np.array([100, 100, 100, 100])

    if abs(ctrl_roll) > np.pi/4 or abs(ctrl_pitch) > np.pi/4:
        print(f"WARNING: Extreme attitude at t={t:.2f}")
        motor_power = np.array([20, 20, 20, 20])

    # Apply motors
    motors[0].setVelocity(-motor_power[0])
    motors[1].setVelocity( motor_power[1])
    motors[2].setVelocity(-motor_power[2])
    motors[3].setVelocity( motor_power[3])

    # Prints
    if (t - last_print_time) >= PRINT_INTERVAL:
        last_print_time = t
        
        print(f"\n{'='*60}")
        print(f"Time: {t:.2f}s | Control Source: {source_label}")
        print(f"Position: GPS=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}), ESO=({eso_p[0]:+.3f},{eso_p[1]:+.3f},{eso_p[2]:+.3f})")
        print(f"Velocity: GPS=({gps_velocity[0]:+.3f},{gps_velocity[1]:+.3f},{gps_velocity[2]:+.3f}), ESO=({eso_v[0]:+.3f},{eso_v[1]:+.3f},{eso_v[2]:+.3f})")
        print(f"Attitude: IMU=({roll*180/np.pi:+.1f}°,{pitch*180/np.pi:+.1f}°,{yaw*180/np.pi:+.1f}°), ESO=({eso_roll*180/np.pi:+.1f}°,{eso_pitch*180/np.pi:+.1f}°,{eso_yaw*180/np.pi:+.1f}°)")
        print(f"Force Dist: ({eso_d_f[0]:+.4f},{eso_d_f[1]:+.4f},{eso_d_f[2]:+.4f})N")
        print(f"Motors: [{motor_power[0]:.1f},{motor_power[1]:.1f},{motor_power[2]:.1f},{motor_power[3]:.1f}]")