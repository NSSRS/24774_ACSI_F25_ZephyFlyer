"""
Crazyflie outer-loop controller with 12-State ESO.
"""

from controller import Robot, Keyboard
import numpy as np
import sys
sys.path.append('../../../../controllers_shared/python_based')
from plotter import DataLogger
logger = DataLogger("eso_flight")

from pid_controller import pid_velocity_fixed_height_controller
from eso import AttitudeESO
from design_L import compute_L

# =========================
# ESO CONTROL MODE
# =========================
USE_ESO_FOR_CONTROL = True          # 必须是 True
USE_DISTURBANCE_COMPENSATION = True # 必须是 True，如果这行是 False，ESO 就算算准了风，飞机也不会动！
K_DIST_MOTOR = 0.3                    # motor gain (tune!)

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
# target = np.array([0.0, 0.0, 0.5])

Kp_pos = 1.0
Kd_pos = 0.1
VEL_LIMIT = 1.5   # hardware achieves ~0.6 easily

past_time = robot.getTime()
past_pos = pos0.copy()

PRINT_INTERVAL = 0.5
last_print_time = 0.0

# ===========================================================
# CIRCULAR TRAJECTORY GENERATOR (MODULAR, REUSABLE)
# ===========================================================
class CircleTrajectory:
    def __init__(self, radius=0.5, z_height=0.5, period=10.0, center=(0.0, 0.0), use_feedforward=True):
        self.radius = radius
        self.z = z_height
        self.T = period
        self.omega = 2 * np.pi / period
        self.cx, self.cy = center
        self.use_ff = use_feedforward

    def get(self, t):
        """Return desired position, velocity, and acceleration at time t."""
        r = self.radius
        w = self.omega
        cx, cy = self.cx, self.cy

        # Position
        x = cx + r * np.cos(w * t)
        y = cy + r * np.sin(w * t)
        z = self.z

        # Velocities (feedforward)
        vx = -r * w * np.sin(w * t)
        vy =  r * w * np.cos(w * t)
        vz = 0.0

        # Optional: Zero feedforward if not used
        if not self.use_ff:
            vx = vy = vz = 0.0

        return np.array([x, y, z]), np.array([vx, vy, vz])

trajectory = CircleTrajectory(
    radius=0.5,         # 1m diameter
    z_height=0.5,       # constant height
    period=10.0,        # one revolution per 10 seconds
    center=(0.0, 0.0),  # XY center
    use_feedforward=True
)

TAKEOFF_Z = 0.5
CIRCLE_STARTED = False
CIRCLE_COUNT = 0
MAX_CIRCLES = 5
circle_start_time = None
LANDING = False
HALF_LAP_ANNOUNCED = False

ESO_READY = False
ESO_INIT_TIME = 1.0     # seconds to wait before initializing ESO

imu_buffer = []
gps_buffer = []


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

    # ---------------------------------------------
    # ESO Initialization Delay
    # ---------------------------------------------
    if not ESO_READY:
        if t < ESO_INIT_TIME:
            imu_buffer.append([roll, pitch, yaw])
            gps_buffer.append(pos.tolist())
            continue  # skip control until ESO warmed up
        else:
            # compute stable averages
            avg_pos = np.mean(np.array(gps_buffer), axis=0)
            avg_rpy = np.mean(np.array(imu_buffer), axis=0)

            y_init = np.array([
                avg_pos[0], avg_pos[1], avg_pos[2],
                avg_rpy[0], avg_rpy[1], avg_rpy[2]
            ])

            eso.initialize_from_measurement(y_init)
            ESO_READY = True
            print(">>> ESO INITIALIZED AFTER DELAY. avg_pos=", avg_pos, " avg_rpy=", avg_rpy)

    # ---------------------------------------------
    # ESO UPDATE (ready or fallback, but always fill eso_* )
    # ---------------------------------------------
    if ESO_READY:
        y_meas = np.array([pos[0], pos[1], pos[2], roll, pitch, yaw])
        z_hat = eso.step(y_meas, T_cmd, omega_body)
    else:
        # fallback to IMU/GPS until ESO ready
        z_hat = np.zeros(12)
        z_hat[0:3] = pos
        z_hat[3:6] = gps_velocity
        z_hat[6:9] = [roll, pitch, yaw]

    # Extract states (common interface)
    eso_p = z_hat[0:3]
    eso_v = z_hat[3:6]
    eso_att = z_hat[6:9]
    eso_d_f = z_hat[9:12]
    # ✅【新增修改 1】添加限幅保护！
    # 如果不加这一行，一旦计算出错，电机直接满速疯转
    # 2.0 是经验值 (牛顿)，大概对应 200g 推力，足以抗风但不会炸机
    eso_d_f = np.clip(eso_d_f, -2.0, 2.0)
    eso_roll, eso_pitch, eso_yaw = eso_att

    # ---------------------------------------------
    # Choose control source (now ALWAYS defines ctrl_pos, etc.)
    # ---------------------------------------------
    if USE_ESO_FOR_CONTROL:
        ctrl_pos = eso_p
        ctrl_vx_world, ctrl_vy_world, ctrl_vz = eso_v
        ctrl_roll = eso_roll
        ctrl_pitch = eso_pitch
        ctrl_yaw = eso_yaw
        source_label = "ESO" if ESO_READY else "ESO (fallback from GPS)"
    else:
        ctrl_pos = pos
        ctrl_vx_world, ctrl_vy_world, ctrl_vz = gps_velocity
        ctrl_roll = roll
        ctrl_pitch = pitch
        ctrl_yaw = yaw
        source_label = "GPS+IMU"

    # ===============================
    # TRAJECTORY MANAGEMENT
    # ===============================

    # --- 1. TAKEOFF STRAIGHT UP ---
    if not CIRCLE_STARTED and not LANDING:
        # go straight up from starting XY
        target = np.array([pos0[0], pos0[1], TAKEOFF_Z])
        vel_ff = np.zeros(3)

        # when altitude reached, start circle
        if abs(ctrl_pos[2] - TAKEOFF_Z) < 0.05:
            CIRCLE_STARTED = True
            circle_start_time = t

            # compute circle center so CURRENT POINT is on circumference
            start_x, start_y = ctrl_pos[0], ctrl_pos[1]
            trajectory.cx = start_x - trajectory.radius
            trajectory.cy = start_y

            print(">>> TAKEOFF COMPLETE — Starting circle 1")
            print(f"    Start point = ({start_x:.3f}, {start_y:.3f})")
            print(f"    Circle center = ({trajectory.cx:.3f}, {trajectory.cy:.3f})")

    # --- 2. CIRCLE FLIGHT ---
    elif CIRCLE_STARTED and not LANDING:

        rel_time = t - circle_start_time

        # --------------------------------------------------
        # HALF-LAP ANNOUNCEMENT FOR SECOND LAP (12 O'CLOCK)
        # --------------------------------------------------
        if CIRCLE_COUNT == 1 and rel_time >= trajectory.T / 2 and not HALF_LAP_ANNOUNCED:
            print("\n" + "="*60)
            print("🔥🔥  SECOND LAP — HALF LAP REACHED (12 O'CLOCK)  🔥🔥")
            print("="*60 + "\n")
            HALF_LAP_ANNOUNCED = True
        # Still inside current circle
        if rel_time < trajectory.T:
            target, vel_ff = trajectory.get(rel_time)

        # Circle completed
        else:
            CIRCLE_COUNT += 1
            print(f">>> Completed circle {CIRCLE_COUNT}")

            # If finished 2 circles, begin landing
            if CIRCLE_COUNT >= 2:
                print(">>> Two circles completed — begin landing")

                LANDING = True

                # Land at start point, 6 o'clock
                target = np.array([
                    trajectory.cx + trajectory.radius,  # start_x
                    trajectory.cy,                      # start_y
                    TAKEOFF_Z
                ])
                vel_ff = np.zeros(3)

            else:
                # Start next circle
                circle_start_time = t
                target, vel_ff = trajectory.get(0.0)
                print(f">>> Starting circle {CIRCLE_COUNT + 1}")


    # --- 3. LANDING ---
    elif LANDING:
        descend_rate = -0.10
        final_z = 0.05

        next_z = ctrl_pos[2] + descend_rate * dt
        target = np.array([
            trajectory.cx + trajectory.radius,
            trajectory.cy,
            max(final_z, next_z)
        ])
        vel_ff = np.zeros(3)

        if ctrl_pos[2] <= final_z + 0.01:
            print(">>> LANDED — shutting down motors")
            break

    # Position control with feedforward + disturbance compensation
    pos_err = target - ctrl_pos

    # Base PD control + trajectory feedforward
    vx_des = Kp_pos * pos_err[0] + Kd_pos * (-ctrl_vx_world) + vel_ff[0]
    vy_des = Kp_pos * pos_err[1] + Kd_pos * (-ctrl_vy_world) + vel_ff[1]
    vz_des = Kp_pos * pos_err[2] + Kd_pos * (-ctrl_vz)       + vel_ff[2]

    # # Active disturbance rejection (ESO-based feedforward)
    # if USE_DISTURBANCE_COMPENSATION and ESO_READY:
    #     # Convert force disturbance to acceleration: a = F/m
    #     # Then integrate to velocity compensation: v_comp = a * gain
    #     dist_accel = eso_d_f / m
    #     v_comp_x = -K_dist_feedforward * dist_accel[0]  # Negative because we want to cancel the disturbance
    #     v_comp_y = -K_dist_feedforward * dist_accel[1]
    #     v_comp_z = -K_dist_feedforward * dist_accel[2]

    #     vx_des += v_comp_x
    #     vy_des += v_comp_y
    #     vz_des += v_comp_z

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

    # ======================================================
    # NEW: ESO disturbance → motor feedforward (u1..u4)
    # ======================================================
    if USE_DISTURBANCE_COMPENSATION and ESO_READY:
        # eso_d_f is in WORLD frame; convert to BODY frame
        cr, sr = np.cos(ctrl_roll), np.sin(ctrl_roll)
        cp, sp = np.cos(ctrl_pitch), np.sin(ctrl_pitch)
        cy, sy = np.cos(ctrl_yaw), np.sin(ctrl_yaw)

        R_wb = np.array([
            [ cy*cp,           cy*sp*sr - sy*cr,   cy*sp*cr + sy*sr ],
            [ sy*cp,           sy*sp*sr + cy*cr,   sy*sp*cr - cy*sr ],
            [   -sp,                     cp*sr,              cp*cr ]
        ])

        # Disturbance force in body frame
        d_f_body = R_wb.T @ eso_d_f
        Fx_b, Fy_b, Fz_b = d_f_body

        # === Vertical disturbance → thrust compensation ===
        dT = -Fz_b

        # === NEW: Lateral disturbance → roll/pitch torque compensation ===
        K_LAT_TORQUE = 0.002   # very small gain, tune!

        # Fy_b pushes drone to +Y → roll right torque needed (τx-)
        d_taux = -K_LAT_TORQUE * Fy_b

        # Fx_b pushes drone to +X → pitch-up torque needed (τy+)
        d_tauy = -K_LAT_TORQUE * Fx_b

        # No yaw compensation
        d_tauz = 0.0

        # Pack disturbance "input"
        U_dist = np.array([dT, d_taux, d_tauy, d_tauz])

        # Thrust/torque → motors allocation
        B_inv = np.array([
            [0.25,  0.0,  0.5, -0.25],
            [0.25, -0.5,  0.0,  0.25],
            [0.25,  0.0, -0.5, -0.25],
            [0.25,  0.5,  0.0,  0.25],
        ])

        # Motor-level bias
        delta_u = K_DIST_MOTOR * (B_inv @ U_dist)

        # Add to PID motor command
        motor_power = motor_power + delta_u


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

    # Log data
    logger.log(
        t,
        pos,
        eso_p, eso_v, eso_att,
        eso_d_f,                 # <<< ADD THIS
        [roll, pitch, yaw],
        target,
        motor_power
    )
    # Prints
    if (t - last_print_time) >= PRINT_INTERVAL:
        last_print_time = t
        print(f"\n{'='*60}")
        print(f"Time: {t:.2f}s | Phase: {'TAKEOFF' if not CIRCLE_STARTED else 'CIRCLE'} | ESO: {source_label}")
        print(f"Position: actual=({ctrl_pos[0]:+.3f},{ctrl_pos[1]:+.3f},{ctrl_pos[2]:+.3f}), target=({target[0]:+.3f},{target[1]:+.3f},{target[2]:+.3f})")
        print(f"Velocity: ({ctrl_vx_world:+.3f},{ctrl_vy_world:+.3f},{ctrl_vz:+.3f}) m/s")

        if ESO_READY:
            dist_norm = np.linalg.norm(eso_d_f)
            print(f"ESO Disturbance: ({eso_d_f[0]:+.4f},{eso_d_f[1]:+.4f},{eso_d_f[2]:+.4f})N, ||d||={dist_norm:.4f}N")

        print(f"Motors: [{motor_power[0]:.1f},{motor_power[1]:.1f},{motor_power[2]:.1f},{motor_power[3]:.1f}]")

# When loop ends (simulation reset or closed), close the log
logger.close()
print("Log file closed.")