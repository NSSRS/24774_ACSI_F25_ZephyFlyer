"""
TinyMPC Controller for CrazyFlie in Webots

State vector (12 states):
    x = [px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r]

Control input (4 inputs):
    u = [thrust_delta, roll_moment, pitch_moment, yaw_moment]
"""

import numpy as np
import tinympc
from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard
from math import cos, sin, pi
import csv
import os
import sys
import time as pytime
from datetime import datetime

THRUST_SCALE = 1.0
MOMENT_SCALE = 2.0e-4


class MPCDataLogger:
    """Data logger for MPC analysis."""

    def __init__(self, log_dir="logs", prefix="mpc", log_decimation=5):
        self.log_dir = log_dir
        self.prefix = prefix
        self.data = []
        self.start_time = None
        self.logging_enabled = False
        self.log_decimation = log_decimation
        self.log_counter = 0

        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            print(f"📁 Created log directory: {log_dir}")
        print(f"📊 Log decimation: recording 1/{log_decimation} samples")

    def start_logging(self):
        self.data = []
        self.start_time = pytime.time()
        self.logging_enabled = True
        self.log_counter = 0
        print("🔴 Logging STARTED")

    def stop_logging(self):
        self.logging_enabled = False
        print(f"⬛ Logging STOPPED ({len(self.data)} samples recorded)")

    def toggle_logging(self):
        if self.logging_enabled:
            self.stop_logging()
        else:
            self.start_logging()
        return self.logging_enabled

    def log(self, sim_time, state, x_ref, u, motors, solve_time_ms=0.0):
        if not self.logging_enabled:
            return

        self.log_counter += 1
        if self.log_counter % self.log_decimation != 0:
            return

        pos_error = np.sqrt((state[0]-x_ref[0])**2 +
                           (state[1]-x_ref[1])**2 +
                           (state[2]-x_ref[2])**2)
        vel_error = np.sqrt((state[3]-x_ref[3])**2 +
                           (state[4]-x_ref[4])**2 +
                           (state[5]-x_ref[5])**2)
        att_error = np.sqrt((state[6]-x_ref[6])**2 +
                           (state[7]-x_ref[7])**2 +
                           (state[8]-x_ref[8])**2)

        entry = {
            'sim_time': sim_time,
            'px': state[0], 'py': state[1], 'pz': state[2],
            'vx': state[3], 'vy': state[4], 'vz': state[5],
            'phi': state[6], 'theta': state[7], 'psi': state[8],
            'p': state[9], 'q': state[10], 'r': state[11],
            'ref_px': x_ref[0], 'ref_py': x_ref[1], 'ref_pz': x_ref[2],
            'ref_vx': x_ref[3], 'ref_vy': x_ref[4], 'ref_vz': x_ref[5],
            'u_thrust': u[0], 'u_roll': u[1], 'u_pitch': u[2], 'u_yaw': u[3],
            'm1': motors[0], 'm2': motors[1], 'm3': motors[2], 'm4': motors[3],
            'pos_error': pos_error,
            'vel_error': vel_error,
            'att_error': att_error,
            'err_x': state[0] - x_ref[0],
            'err_y': state[1] - x_ref[1],
            'err_z': state[2] - x_ref[2],
            'solve_time_ms': solve_time_ms,
        }
        self.data.append(entry)

    def save(self, filename=None):
        if not self.data:
            print("⚠️  No data to save!")
            return None

        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.prefix}_{timestamp}.csv"

        filepath = os.path.join(self.log_dir, filename)

        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.data[0].keys())
            writer.writeheader()
            writer.writerows(self.data)

        duration = self.data[-1]['sim_time'] - self.data[0]['sim_time']
        print(f"\n📊 Log saved: {filepath}")
        print(f"   Samples: {len(self.data)}")
        print(f"   Duration: {duration:.2f}s")
        print(f"   Avg sample rate: {len(self.data)/duration:.1f} Hz")

        return filepath

    def get_summary(self):
        if not self.data:
            return "No data logged"

        pos_errors = [d['pos_error'] for d in self.data]
        solve_times = [d['solve_time_ms'] for d in self.data]

        summary = f"""
📈 MPC Log Summary ({len(self.data)} samples)
{'='*50}
Position Error (m):
  Mean: {np.mean(pos_errors):.4f}
  Max:  {np.max(pos_errors):.4f}
  Min:  {np.min(pos_errors):.4f}
  Std:  {np.std(pos_errors):.4f}

MPC Solve Time (ms):
  Mean: {np.mean(solve_times):.3f}
  Max:  {np.max(solve_times):.3f}
  Min:  {np.min(solve_times):.3f}
{'='*50}
"""
        return summary


def get_linearized_model(dt=0.002, mass=0.027, g=9.81,
                         Ixx=1.6e-5, Iyy=1.6e-5, Izz=2.9e-5):
    """
    Linearized discrete-time quadrotor model near hover.
    
    Returns: A (12x12), B (12x4)
    """
    nx = 12
    nu = 4

    thrust_scale = THRUST_SCALE
    moment_scale = MOMENT_SCALE

    A_cont = np.zeros((nx, nx))

    # Position dynamics
    A_cont[0, 3] = 1.0
    A_cont[1, 4] = 1.0
    A_cont[2, 5] = 1.0

    # Velocity dynamics (linearized near hover)
    A_cont[3, 7] = g
    A_cont[4, 6] = -g

    # Attitude dynamics
    A_cont[6, 9] = 1.0
    A_cont[7, 10] = 1.0
    A_cont[8, 11] = 1.0

    # Damping
    kv_xy = 2.00e+00
    kv_z = 3.00e+00
    A_cont[3, 3] = -kv_xy
    A_cont[4, 4] = -kv_xy
    A_cont[5, 5] = -kv_z

    kp_att = 1.00e+01
    A_cont[9, 9] = -kp_att
    A_cont[10, 10] = -kp_att

    B_cont = np.zeros((nx, nu))
    B_cont[5, 0] = thrust_scale / mass
    B_cont[9, 1] = moment_scale / Ixx
    B_cont[10, 2] = moment_scale / Iyy
    B_cont[11, 3] = 0.0

    # Discretization
    A_disc = np.eye(nx) + A_cont * dt
    B_disc = B_cont * dt

    A_disc = np.asfortranarray(A_disc, dtype=np.float64)
    B_disc = np.asfortranarray(B_disc, dtype=np.float64)

    return A_disc, B_disc


def get_cost_matrices(nx=12, nu=4):
    Q = np.diag([
        8.00e+01, 8.00e+01, 1.50e+02,
        5.00e+00, 5.00e+00, 1.00e+01,
        2.00e+01, 2.00e+01, 1.00e+00,
        5.00e+00, 5.00e+00, 5.00e+00
    ])

    R = np.diag([
        8.00e+01,
        5.00e+01,
        5.00e+01,
        1.00e+04
    ])
    Q = np.asfortranarray(Q, dtype=np.float64)
    R = np.asfortranarray(R, dtype=np.float64)
    return Q, R


class TinyMPCController:
    """TinyMPC-based controller for CrazyFlie."""

    def __init__(self, dt=0.01, hover_height=0.5, horizon=80):
        self.dt = dt
        self.hover_height = hover_height
        self.horizon = horizon

        self.mass = 2.7E-02
        self.g = 9.81
        self.Ixx = 1.6e-5
        self.Iyy = 1.6e-5
        self.Izz = 2.9e-5
        
        self.vx_f = 0.0
        self.vy_f = 0.0
        self.vz_f = 0.0
        self.alpha_v = 0.4

        self.A, self.B = get_linearized_model(
            dt=dt, mass=self.mass, g=self.g,
            Ixx=self.Ixx, Iyy=self.Iyy, Izz=self.Izz
        )

        self.Q, self.R = get_cost_matrices()

        self.nx = 12
        self.nu = 4

        u_min = np.array([-0.05, -0.015, -0.015, -0.05])
        u_max = np.array([+0.05, +0.015, +0.015, +0.05])

        x_min = np.full(12, -np.inf, dtype=np.float64)
        x_max = np.full(12, np.inf, dtype=np.float64)
        x_min[3:6] = -2.0
        x_max[3:6] = 2.0
        x_min[6] = -1.50e-01
        x_max[6] = 1.50e-01
        x_min[7] = -1.50e-01
        x_max[7] = 1.50e-01

        rho = 250.0

        self.solver = tinympc.TinyMPC()
        self.solver.setup(self.A, self.B, self.Q, self.R, self.horizon,
                          u_min=u_min, u_max=u_max,
                          x_min=x_min, x_max=x_max,
                          rho=rho,
                          verbose=0)

        motor_to_thrust = 1.38e-3

        print(f"  Constraints (normalized inputs → physical units):")
        print(f"    - u_thrust: [{u_min[0]:.3f}, {u_max[0]:.3f}] → ±{THRUST_SCALE*u_max[0]:.3e} N")
        print(f"    - u_moment: [{u_min[1]:.3f}, {u_max[1]:.3f}] → ±{MOMENT_SCALE*u_max[1]:.3e} N·m")
        print(f"    - Velocity: ±{x_max[3]:.1f} m/s")
        print(f"    - Roll/Pitch: ±{x_max[6]:.2f} rad")
        print(f"    - ADMM rho: {rho}")

        print(f"  Force-Torque mixer:")
        print(f"    - Hover thrust: {self.mass * self.g:.4f} N")
        print(f"    - Arm length: 0.046 m")
        print(f"    - Motor-to-thrust: {motor_to_thrust:.4e} N/cmd")

        self.x_ref = np.zeros(self.nx, dtype=np.float64)
        self.x_ref[2] = hover_height

        self.u_ref = np.zeros(self.nu, dtype=np.float64)

        self.solver.set_x_ref(self.x_ref)
        self.solver.set_u_ref(self.u_ref)

        print(f"\nTinyMPC Controller initialized:")
        print(f"  - Timestep: {dt*1000:.0f}ms ({1/dt:.0f}Hz)")
        print(f"  - Horizon: {horizon} steps ({horizon*dt*1000:.0f}ms)")
        print(f"  - Hover height: {hover_height}m")
        print(f"  - State dim: {self.nx}, Input dim: {self.nu}")

    def set_target_position(self, x, y, z):
        self.x_ref[0] = float(x)
        self.x_ref[1] = float(y)
        self.x_ref[2] = float(z)
        self.x_ref_current = self.x_ref.copy()
        self.solver.set_x_ref(self.x_ref)
    
    def set_trajectory_reference(self, traj_func, t_now, dt):
        """Set reference trajectory for MPC horizon."""
        x_ref_traj = np.zeros((self.nx, self.horizon), dtype=np.float64)
        
        for k in range(self.horizon):
            tk = t_now + k * dt
            x_c, y_c, z_c = traj_func(tk)
            
            x_ref_traj[0, k] = x_c
            x_ref_traj[1, k] = y_c  
            x_ref_traj[2, k] = z_c
            
            if k > 0:
                x_ref_traj[3, k] = (x_ref_traj[0, k] - x_ref_traj[0, k-1]) / dt
                x_ref_traj[4, k] = (x_ref_traj[1, k] - x_ref_traj[1, k-1]) / dt
                x_ref_traj[5, k] = (x_ref_traj[2, k] - x_ref_traj[2, k-1]) / dt
        
        x_ref_traj = np.asfortranarray(x_ref_traj)
        self.x_ref_current = x_ref_traj[:, 0].copy()
        self.solver.set_x_ref(x_ref_traj)

    def get_state(self, gps_values, imu_values, gyro_values,
                  past_pos, past_time, current_time):
        """Construct state vector from sensor readings."""
        dt = current_time - past_time
        if dt < 1e-6:
            dt = self.dt

        px, py, pz = gps_values

        # Velocity with low-pass filter
        vx_raw = (px - past_pos[0]) / dt
        vy_raw = (py - past_pos[1]) / dt
        vz_raw = (pz - past_pos[2]) / dt

        self.vx_f = (1 - self.alpha_v) * self.vx_f + self.alpha_v * vx_raw
        self.vy_f = (1 - self.alpha_v) * self.vy_f + self.alpha_v * vy_raw
        self.vz_f = (1 - self.alpha_v) * self.vz_f + self.alpha_v * vz_raw

        vx, vy, vz = self.vx_f, self.vy_f, self.vz_f

        phi, theta, psi = imu_values
        p, q, r = gyro_values

        return np.array([px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r], dtype=np.float64)

    def compute_control(self, state):
        """Compute optimal control using TinyMPC."""
        self.solver.set_x0(state)

        # Suppress C-level stdout
        stdout_fd = sys.stdout.fileno()
        saved_stdout_fd = os.dup(stdout_fd)
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull_fd, stdout_fd)
        os.close(devnull_fd)
        try:
            solution = self.solver.solve()
        finally:
            os.dup2(saved_stdout_fd, stdout_fd)
            os.close(saved_stdout_fd)

        u = solution['controls'][:, 0] if solution['controls'].ndim > 1 else solution['controls']
        return u

    def control_to_motors(self, u, imu_angles=None):
        """Force-Torque mixer (CrazyFlie firmware)."""
        u_thrust, u_roll, u_pitch, u_yaw = u

        hover_thrust = self.mass * self.g
        thrust_scale = THRUST_SCALE
        moment_scale = MOMENT_SCALE

        total_thrust = max(0.0, hover_thrust + u_thrust * thrust_scale)
        tau_x = u_roll * moment_scale
        tau_y = u_pitch * moment_scale
        tau_z = 0.0
        
        ARM_LENGTH = 0.046
        THRUST2TORQUE = 0.005022
        arm = 0.707106781 * ARM_LENGTH

        thrust_part = 0.25 * total_thrust
        roll_part = 0.25 / arm * tau_x
        pitch_part = 0.25 / arm * tau_y
        yaw_part = 0.25 * tau_z / THRUST2TORQUE

        F1 = thrust_part - roll_part - pitch_part - yaw_part
        F2 = thrust_part - roll_part + pitch_part + yaw_part
        F3 = thrust_part + roll_part + pitch_part - yaw_part
        F4 = thrust_part + roll_part - pitch_part + yaw_part
        
        motor_to_thrust = 1.38e-3
        
        m1 = np.clip(F1 / motor_to_thrust, 0, 600)
        m2 = np.clip(F2 / motor_to_thrust, 0, 600)
        m3 = np.clip(F3 / motor_to_thrust, 0, 600)
        m4 = np.clip(F4 / motor_to_thrust, 0, 600)
        
        return [m1, m2, m3, m4]


def main():
    """Main control loop."""
    robot = Robot()
    timestep = 2
    control_dt = timestep / 1000.0

    print(f"\nWebots timestep: {timestep}ms")

    # Initialize motors
    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float('inf'))
    m1_motor.setVelocity(-1)

    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float('inf'))
    m2_motor.setVelocity(1)

    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float('inf'))
    m3_motor.setVelocity(-1)

    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float('inf'))
    m4_motor.setVelocity(1)

    # Initialize sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)

    gps = robot.getDevice("gps")
    gps.enable(timestep)

    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)

    keyboard = Keyboard()
    keyboard.enable(timestep)

    hover_height = 0.5
    horizon = 80

    mpc = TinyMPCController(
        dt=control_dt,
        hover_height=hover_height,
        horizon=horizon
    )

    target_x = 0.0
    target_y = 0.0
    target_z = hover_height

    # Trajectory parameters
    circle_mode = False
    circle_start_time = 0.0
    circle_radius = 0.3
    circle_period = 10.0
    circle_center_x = 0.0
    circle_center_y = 0.0

    line_mode = False
    line_start_time = 0.0
    line_distance = 0.5
    line_duration = 5.0
    line_axis = 'x'

    past_pos = [0.0, 0.0, 0.0]
    past_time = 0.0

    last_print_time = 0.0
    print_interval = 1.0

    logger = MPCDataLogger(log_dir="logs", prefix="tinympc", log_decimation=20)

    last_key_time = {}
    KEY_DEBOUNCE = 0.3

    print("\n" + "="*60)
    print("TinyMPC Hover Controller for CrazyFlie")
    print("="*60)
    print(f"\nTarget: hover at ({target_x}, {target_y}, {target_z})m")
    print("\nControls:")
    print("  Arrow keys: Move target X/Y")
    print("  W/S: Adjust target altitude")
    print("  R: Reset to origin")
    print("  SPACE: Print status")
    print("\nTrajectory:")
    print("  T: Start/Stop circular trajectory")
    print("  +/-: Adjust circle radius")
    print("\nData Logging:")
    print("  L: Toggle logging ON/OFF")
    print("  K: Save log to CSV file")
    print("  P: Print log summary")
    print("="*60 + "\n")

    for _ in range(5):
        robot.step(timestep)

    while robot.step(timestep) != -1:
        current_time = robot.getTime()

        gps_values = gps.getValues()
        imu_values = imu.getRollPitchYaw()
        gyro_values = gyro.getValues()

        # Keyboard input with debouncing
        key = keyboard.getKey()
        while key > 0:
            if key in last_key_time and (current_time - last_key_time[key]) < KEY_DEBOUNCE:
                key = keyboard.getKey()
                continue
            last_key_time[key] = current_time

            if key == Keyboard.UP:
                target_x += 0.05
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == Keyboard.DOWN:
                target_x -= 0.05
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == Keyboard.LEFT:
                target_y += 0.05
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == Keyboard.RIGHT:
                target_y -= 0.05
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == ord('W'):
                target_z += 0.1
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == ord('S'):
                target_z = max(0.1, target_z - 0.1)
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == ord('R') or key == ord('r'):
                target_x, target_y, target_z = 0.0, 0.0, hover_height
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Reset target to ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == ord(' '):
                pos_err = np.sqrt((gps_values[0]-target_x)**2 +
                                  (gps_values[1]-target_y)**2 +
                                  (gps_values[2]-target_z)**2)
                print(f"\nStatus at t={current_time:.2f}s:")
                print(f"  Position: ({gps_values[0]:.3f}, {gps_values[1]:.3f}, {gps_values[2]:.3f})")
                print(f"  Target:   ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
                print(f"  Error:    {pos_err:.4f}m")
                print(f"  Attitude: roll={imu_values[0]*180/pi:.1f}, pitch={imu_values[1]*180/pi:.1f}, yaw={imu_values[2]*180/pi:.1f} deg")
                print(f"  Logging:  {'ON' if logger.logging_enabled else 'OFF'} ({len(logger.data)} samples)\n")
            elif key == ord('L') or key == ord('l'):
                logger.toggle_logging()
            elif key == ord('K') or key == ord('k'):
                logger.save()
            elif key == ord('P') or key == ord('p'):
                print(logger.get_summary())
            elif key == ord('T') or key == ord('t'):
                circle_mode = not circle_mode
                if circle_mode:
                    circle_start_time = current_time
                    circle_center_x = gps_values[0]
                    circle_center_y = gps_values[1]
                    print(f"🔵 Circle trajectory STARTED: radius={circle_radius}m, period={circle_period}s")
                    print(f"   Center: ({circle_center_x:.2f}, {circle_center_y:.2f})")
                else:
                    print("⬛ Circle trajectory STOPPED")
            elif key == ord('+') or key == ord('='):
                circle_radius = min(1.0, circle_radius + 0.1)
                print(f"Circle radius: {circle_radius:.1f}m")
            elif key == ord('-') or key == ord('_'):
                circle_radius = max(0.1, circle_radius - 0.1)
                print(f"Circle radius: {circle_radius:.1f}m")
            elif key == ord('Y') or key == ord('y'):
                line_mode = not line_mode
                if line_mode:
                    circle_mode = False
                    line_start_time = current_time
                    print(f"📏 Line trajectory STARTED: {line_distance}m in {line_duration}s along {line_axis.upper()}-axis")
                else:
                    print("⬛ Line trajectory STOPPED")
            elif key == ord('['):
                line_distance = max(0.1, line_distance - 0.1)
                print(f"Line distance: {line_distance:.1f}m")
            elif key == ord(']'):
                line_distance = min(2.0, line_distance + 0.1)
                print(f"Line distance: {line_distance:.1f}m")
            key = keyboard.getKey()

        # Update target for trajectory tracking
        if line_mode:
            def line_trajectory(t):
                t_line = t - line_start_time
                progress = min(1.0, t_line / line_duration)
                displacement = progress * line_distance
                
                if line_axis == 'x':
                    return (displacement, 0.0, target_z)
                elif line_axis == 'y':
                    return (0.0, displacement, target_z)
                else:
                    return (0.0, 0.0, hover_height + displacement)
            
            mpc.set_trajectory_reference(line_trajectory, current_time, control_dt)
            
        elif circle_mode:
            def circle_trajectory(t):
                t_circle = t - circle_start_time
                omega = 2.0 * pi / circle_period
                x = circle_center_x + circle_radius * cos(omega * t_circle)
                y = circle_center_y + circle_radius * sin(omega * t_circle)
                z = target_z
                return (x, y, z)
            
            mpc.set_trajectory_reference(circle_trajectory, current_time, control_dt)
        else:
            mpc.set_target_position(target_x, target_y, target_z)

        state = mpc.get_state(
            gps_values, imu_values, gyro_values,
            past_pos, past_time, current_time
        )

        solve_start = pytime.perf_counter()
        u = mpc.compute_control(state)
        solve_time_ms = (pytime.perf_counter() - solve_start) * 1000.0

        motors = mpc.control_to_motors(u, imu_values)

        logger.log(current_time, state, mpc.x_ref_current, u, motors, solve_time_ms)

        m1_motor.setVelocity(-motors[0])
        m2_motor.setVelocity(motors[1])
        m3_motor.setVelocity(-motors[2])
        m4_motor.setVelocity(motors[3])

        if current_time - last_print_time >= print_interval:
            pos_err = np.sqrt((gps_values[0]-target_x)**2 +
                              (gps_values[1]-target_y)**2 +
                              (gps_values[2]-target_z)**2)
            print(f"[{current_time:6.1f}s] Pos:({gps_values[0]:+.3f},{gps_values[1]:+.3f},{gps_values[2]:+.3f}) | "
                  f"Err:{pos_err:.4f}m | u:[{u[0]:.4f},{u[1]:.6f},{u[2]:.6f},{u[3]:.6f}]")
            last_print_time = current_time

        past_pos = list(gps_values)
        past_time = current_time


if __name__ == '__main__':
    main()