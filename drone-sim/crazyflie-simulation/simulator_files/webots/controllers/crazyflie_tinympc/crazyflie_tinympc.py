"""
TinyMPC Hover Controller for CrazyFlie in Webots

This controller uses TinyMPC to stabilize the CrazyFlie at a fixed hover position.
The MPC formulation uses a linearized quadrotor model discretized at 100Hz.

State vector (12 states):
    x = [px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r]
    - px, py, pz: position in world frame (m)
    - vx, vy, vz: velocity in world frame (m/s)
    - phi, theta, psi: roll, pitch, yaw angles (rad)
    - p, q, r: angular rates (rad/s)

Control input (4 inputs):
    u = [thrust_delta, roll_moment, pitch_moment, yaw_moment]
    - thrust_delta: deviation from hover thrust
    - roll/pitch/yaw moments: body torques

Author: Claude Code
Date: 2025-11-23
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


class MPCDataLogger:
    """
    Data logger for MPC analysis.

    Records all relevant data for post-flight analysis:
    - Time stamps
    - Full state vector (12 states)
    - Reference state
    - MPC control outputs
    - Motor commands
    - Position/attitude errors
    - MPC solver timing
    """

    def __init__(self, log_dir="logs", prefix="mpc", log_decimation=5):
        """
        Initialize the data logger.

        Args:
            log_dir: Directory to save log files
            prefix: Prefix for log filename
            log_decimation: Log every Nth sample (1=all, 5=every 5th, 10=every 10th)
        """
        self.log_dir = log_dir
        self.prefix = prefix
        self.data = []
        self.start_time = None
        self.logging_enabled = False
        self.log_decimation = log_decimation
        self.log_counter = 0

        # Create log directory if needed
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            print(f"📁 Created log directory: {log_dir}")
        print(f"📊 Log decimation: recording 1/{log_decimation} samples")

    def start_logging(self):
        """Start recording data."""
        self.data = []
        self.start_time = pytime.time()
        self.logging_enabled = True
        self.log_counter = 0  # Reset counter for new logging session
        print("🔴 Logging STARTED")

    def stop_logging(self):
        """Stop recording data."""
        self.logging_enabled = False
        print(f"⬛ Logging STOPPED ({len(self.data)} samples recorded)")

    def toggle_logging(self):
        """Toggle logging on/off."""
        if self.logging_enabled:
            self.stop_logging()
        else:
            self.start_logging()
        return self.logging_enabled

    def log(self, sim_time, state, x_ref, u, motors, solve_time_ms=0.0):
        """
        Log one timestep of data.

        Args:
            sim_time: Simulation time (s)
            state: 12-element state vector [px,py,pz,vx,vy,vz,phi,theta,psi,p,q,r]
            x_ref: 12-element reference state
            u: 4-element control input [thrust_delta, roll_mom, pitch_mom, yaw_mom]
            motors: 4-element motor commands [m1,m2,m3,m4]
            solve_time_ms: MPC solve time in milliseconds
        """
        if not self.logging_enabled:
            return

        # Decimation: only log every Nth sample
        self.log_counter += 1
        if self.log_counter % self.log_decimation != 0:
            return

        # Compute errors
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
            # Time
            'sim_time': sim_time,

            # State - Position
            'px': state[0], 'py': state[1], 'pz': state[2],
            # State - Velocity
            'vx': state[3], 'vy': state[4], 'vz': state[5],
            # State - Attitude (rad)
            'phi': state[6], 'theta': state[7], 'psi': state[8],
            # State - Angular rates (rad/s)
            'p': state[9], 'q': state[10], 'r': state[11],

            # Reference - Position
            'ref_px': x_ref[0], 'ref_py': x_ref[1], 'ref_pz': x_ref[2],
            # Reference - Velocity (usually 0 for hover)
            'ref_vx': x_ref[3], 'ref_vy': x_ref[4], 'ref_vz': x_ref[5],

            # MPC Control Output
            'u_thrust': u[0], 'u_roll': u[1], 'u_pitch': u[2], 'u_yaw': u[3],

            # Motor Commands
            'm1': motors[0], 'm2': motors[1], 'm3': motors[2], 'm4': motors[3],

            # Errors
            'pos_error': pos_error,
            'vel_error': vel_error,
            'att_error': att_error,

            # Errors by axis (for detailed analysis)
            'err_x': state[0] - x_ref[0],
            'err_y': state[1] - x_ref[1],
            'err_z': state[2] - x_ref[2],

            # MPC Performance
            'solve_time_ms': solve_time_ms,
        }

        self.data.append(entry)

    def save(self, filename=None):
        """
        Save logged data to CSV file.

        Args:
            filename: Custom filename (optional). If None, auto-generates with timestamp.

        Returns:
            Path to saved file
        """
        if not self.data:
            print("⚠️  No data to save!")
            return None

        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.prefix}_{timestamp}.csv"

        filepath = os.path.join(self.log_dir, filename)

        # Write CSV
        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.data[0].keys())
            writer.writeheader()
            writer.writerows(self.data)

        # Print summary
        duration = self.data[-1]['sim_time'] - self.data[0]['sim_time']
        print(f"\n📊 Log saved: {filepath}")
        print(f"   Samples: {len(self.data)}")
        print(f"   Duration: {duration:.2f}s")
        print(f"   Avg sample rate: {len(self.data)/duration:.1f} Hz")

        return filepath

    def get_summary(self):
        """Get summary statistics of logged data."""
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


def get_linearized_model(dt=0.01, mass=0.027, g=9.81,
                         Ixx=1.6e-5, Iyy=1.6e-5, Izz=2.9e-5):
    """
    Get linearized discrete-time state-space model for quadrotor near hover.

    State: x = [px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r]
    Input: u = [thrust_delta, roll_moment, pitch_moment, yaw_moment]

    Continuous dynamics linearized at hover:
        ṗx = vx
        ṗy = vy
        ṗz = vz
        v̇x = g * theta  (small angle approx)
        v̇y = -g * phi
        v̇z = thrust_delta / mass
        φ̇ = p
        θ̇ = q
        ψ̇ = r
        ṗ = roll_moment / Ixx
        q̇ = pitch_moment / Iyy
        ṙ = yaw_moment / Izz

    Returns:
        A: 12x12 discrete state transition matrix
        B: 12x4 discrete input matrix
    """
    nx = 12  # number of states
    nu = 4   # number of inputs

    # Continuous-time A matrix
    A_cont = np.zeros((nx, nx))

    # Position dynamics: ṗ = v
    A_cont[0, 3] = 1.0  # ṗx = vx
    A_cont[1, 4] = 1.0  # ṗy = vy
    A_cont[2, 5] = 1.0  # ṗz = vz

    # Velocity dynamics (linearized): v̇ = g * angles
    A_cont[3, 7] = g     # v̇x = g * theta
    A_cont[4, 6] = -g    # v̇y = -g * phi

    # Attitude dynamics: θ̇ = ω
    A_cont[6, 9] = 1.0   # φ̇ = p
    A_cont[7, 10] = 1.0  # θ̇ = q
    A_cont[8, 11] = 1.0  # ψ̇ = r

    # Continuous-time B matrix
    B_cont = np.zeros((nx, nu))
    B_cont[5, 0] = 1.0 / mass      # v̇z from thrust
    B_cont[9, 1] = 1.0 / Ixx       # ṗ from roll moment
    B_cont[10, 2] = 1.0 / Iyy      # q̇ from pitch moment
    B_cont[11, 3] = 1.0 / Izz      # ṙ from yaw moment

    # Discretization using Euler method: A_d = I + A*dt, B_d = B*dt
    A_disc = np.eye(nx) + A_cont * dt
    B_disc = B_cont * dt

    # TinyMPC requires Fortran-contiguous arrays
    A_disc = np.asfortranarray(A_disc, dtype=np.float64)
    B_disc = np.asfortranarray(B_disc, dtype=np.float64)

    return A_disc, B_disc


def get_cost_matrices(nx=12, nu=4):
    """
    Get Q and R cost matrices for MPC.

    Q: state cost (penalizes deviation from reference)
    R: input cost (penalizes control effort)

    Returns:
        Q: 12x12 state cost matrix
        R: 4x4 input cost matrix
    """
    # State cost: Reduced velocity weights for faster convergence
    Q = np.diag([
        1.0, 1.0, 5.0,      # position x,y,z
        0.4, 0.4, 0.4,      # velocity
        0.4, 0.4, 0.4,      # angles
        0.2, 0.2, 0.2       # angular rates
    ])

    # Input cost: Penalize thrust saturation to prevent bang-bang control
    R = np.diag([
    5.0E+02,              # thrust: 从 1.0E+03 降到 5.0E+02
    1.0E+03, 1.0E+03, 1.0E+03
    ])




    # TinyMPC requires Fortran-contiguous arrays
    Q = np.asfortranarray(Q, dtype=np.float64)
    R = np.asfortranarray(R, dtype=np.float64)

    return Q, R


class TinyMPCController:
    """TinyMPC-based controller for CrazyFlie hover."""

    def __init__(self, dt=0.01, hover_height=0.5, horizon=10):
        """
        Initialize TinyMPC controller.

        Args:
            dt: control timestep (s), default 0.01 (100Hz)
            hover_height: target hover altitude (m)
            horizon: MPC prediction horizon (N)
        """
        self.dt = dt
        self.hover_height = hover_height
        self.horizon = horizon

        # CrazyFlie parameters (approximate values for Webots model)
        self.mass =  2.7E-02  # kg (CORRECTED: 27g = CrazyFlie 2.1 base weight)
        self.g = 9.81      # m/s^2
        self.Ixx = 1.6e-5  # kg*m^2
        self.Iyy = 1.6e-5
        self.Izz = 2.9e-5
        # 在 TinyMPCController.__init__ 里加：
        self.vx_f = 0.0
        self.vy_f = 0.0
        self.vz_f = 0.0
        self.alpha_v = 0.2  # 0~1，小一点更平滑

        # Get linearized model
        self.A, self.B = get_linearized_model(
            dt=dt, mass=self.mass, g=self.g,
            Ixx=self.Ixx, Iyy=self.Iyy, Izz=self.Izz
        )

        # Get cost matrices
        self.Q, self.R = get_cost_matrices()

        # State and input dimensions
        self.nx = 12
        self.nu = 4

        # ========== Input Constraints ==========
        # u = [Δthrust, roll_moment, pitch_moment, yaw_moment]
        # Δthrust 是相对于 hover 的增量，单位 N
        # Relaxed from ±0.20 to ±0.10 to prevent aggressive control and saturation
        u_min = np.array([-0.10, -5e-3, -5e-3, -1e-3], dtype=np.float64)
        u_max = np.array([+0.10, +5e-3, +5e-3, +1e-3], dtype=np.float64)

        # ========== State Constraints ==========
        # x = [px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r]
        # Attitude constraint: |phi|, |theta| <= 15 deg = 0.26 rad
        angle_limit = 0.26  # 15 degrees in radians
        x_min = np.full(self.nx, -1e6, dtype=np.float64)  # No constraint by default
        x_max = np.full(self.nx, +1e6, dtype=np.float64)
        # Roll (phi) constraint - index 6
        x_min[6] = -angle_limit
        x_max[6] = +angle_limit
        # Pitch (theta) constraint - index 7
        x_min[7] = -angle_limit
        x_max[7] = +angle_limit

        # Initialize TinyMPC solver with constraints in setup()
        self.solver = tinympc.TinyMPC()
        self.solver.setup(self.A, self.B, self.Q, self.R, self.horizon,
                          u_min=u_min, u_max=u_max,
                          x_min=x_min, x_max=x_max,
                          verbose=0)

        print(f"  Constraints:")
        print(f"    - Δthrust: [{u_min[0]:.2f}, {u_max[0]:.2f}] N")
        print(f"    - Roll/Pitch moment: [{u_min[1]*1000:.1f}, {u_max[1]*1000:.1f}] mN·m")
        print(f"    - Yaw moment: [{u_min[3]*1000:.1f}, {u_max[3]*1000:.1f}] mN·m")
        print(f"    - Roll/Pitch angle: [{-angle_limit*180/pi:.0f}, {angle_limit*180/pi:.0f}] deg")

        # For motor mixing (from PID/经验标定) - 需要先定义，后面 print 会用到
        # IMPORTANT: This should be calibrated experimentally by finding m_hover
        # where the drone hovers stably at the desired altitude with roll=pitch=0
        # Method: In Webots, manually test motor commands (40, 45, 50...) and find
        # the value where drone maintains altitude without drift.
        # Then use: hover_thrust_cmd = m_hover (not theoretical mass*g mapping)
        m_hover = 4.8E+01  # Experimentally determined hover motor command (TO BE CALIBRATED)
        self.hover_thrust_cmd = m_hover
        self.hover_thrust = self.mass * self.g  # Theoretical hover thrust (N)

        # Thrust scaling: maps thrust (N) to motor command
        # This ensures MPC's physical thrust maps correctly to motor units
        self.thrust_scale = self.hover_thrust_cmd / self.hover_thrust

        # Reference state (hover position)
        self.x_ref = np.zeros(self.nx, dtype=np.float64)
        self.x_ref[2] = hover_height  # z position

        # Reference input: Δthrust = 0 at hover (因为我们用增量控制)
        self.u_ref = np.zeros(self.nu, dtype=np.float64)
        # u_ref[0] = 0 (Δthrust = 0 at hover)

        # Set references
        self.solver.set_x_ref(self.x_ref)
        self.solver.set_u_ref(self.u_ref)

        print(f"TinyMPC Controller initialized:")
        print(f"  - Timestep: {dt*1000:.0f}ms ({1/dt:.0f}Hz)")
        print(f"  - Horizon: {horizon} steps ({horizon*dt*1000:.0f}ms)")
        print(f"  - Hover height: {hover_height}m")
        print(f"  - State dim: {self.nx}, Input dim: {self.nu}")
        print(f"  Hover calibration:")
        print(f"    - Theoretical hover thrust: {self.hover_thrust:.4f} N (mass × g)")
        print(f"    - Calibrated motor command: {m_hover:.1f}")
        print(f"    - Thrust scale: {self.thrust_scale:.2f} cmd/N")

    def set_target_position(self, x, y, z):
        """Update target hover position."""
        self.x_ref[0] = float(x)
        self.x_ref[1] = float(y)
        self.x_ref[2] = float(z)
        self.solver.set_x_ref(self.x_ref)

    def get_state(self, gps_values, imu_values, gyro_values,
                  past_pos, past_time, current_time):
        """
        Construct state vector from sensor readings.

        Args:
            gps_values: [x, y, z] position
            imu_values: [roll, pitch, yaw] angles
            gyro_values: [gx, gy, gz] angular rates
            past_pos: [x, y, z] position at previous timestep
            past_time: previous timestamp
            current_time: current timestamp

        Returns:
            x: 12-element state vector
        """
        # Calculate timestep
        dt = current_time - past_time
        if dt < 1e-6:
            dt = self.dt

        # Position
        px, py, pz = gps_values

        # Velocity (numerical differentiation with low-pass filter)
        vx_raw = (px - past_pos[0]) / dt
        vy_raw = (py - past_pos[1]) / dt
        vz_raw = (pz - past_pos[2]) / dt

        # First-order low-pass filter (exponential moving average)
        self.vx_f = (1 - self.alpha_v) * self.vx_f + self.alpha_v * vx_raw
        self.vy_f = (1 - self.alpha_v) * self.vy_f + self.alpha_v * vy_raw
        self.vz_f = (1 - self.alpha_v) * self.vz_f + self.alpha_v * vz_raw

        # Use filtered velocities
        vx, vy, vz = self.vx_f, self.vy_f, self.vz_f

        # Attitude
        phi, theta, psi = imu_values  # roll, pitch, yaw

        # Angular rates
        p, q, r = gyro_values

        return np.array([px, py, pz, vx, vy, vz, phi, theta, psi, p, q, r], dtype=np.float64)

    def compute_control(self, state):
        """
        Compute optimal control using TinyMPC.

        Args:
            state: 12-element state vector

        Returns:
            u: 4-element control vector [thrust_delta, roll_mom, pitch_mom, yaw_mom]
        """
        # Set current state
        self.solver.set_x0(state)

        # Solve MPC problem (suppress C-level stdout to hide iteration messages)
        # Save original file descriptors
        stdout_fd = sys.stdout.fileno()
        saved_stdout_fd = os.dup(stdout_fd)
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull_fd, stdout_fd)
        os.close(devnull_fd)
        try:
            solution = self.solver.solve()
        finally:
            # Restore original stdout
            os.dup2(saved_stdout_fd, stdout_fd)
            os.close(saved_stdout_fd)

        # Get first control input
        u = solution['controls'][:, 0] if solution['controls'].ndim > 1 else solution['controls']

        return u

    def control_to_motors(self, u):
        """
        Convert MPC control output to motor commands.

        Args:
            u: [Δthrust_normalized, roll_moment, pitch_moment, yaw_moment]
               Δthrust_normalized ∈ [-1, +1], 映射到 [-0.10, +0.10] N

        Returns:
            motors: [m1, m2, m3, m4] motor velocities
        """
        d_thrust_n, roll_mom, pitch_mom, yaw_mom = u

        # MPC outputs Δthrust directly in Newtons (not normalized)
        # u[0] ∈ [-0.10, +0.10] N (set by constraints in __init__)
        d_thrust = d_thrust_n  # Already in physical units (N)

        # 先把 Δthrust 加回 hover thrust
        thrust = self.hover_thrust + d_thrust  # N
        thrust = max(0.0, thrust)  # 避免负推力

        # Use pre-computed thrust_scale from calibration
        # This maps physical thrust (N) to motor command units
        moment_scale = 5000.0  # moment to motor units

        # 对应的高度通道指令
        alt_cmd = thrust * self.thrust_scale

        # Moment commands
        roll_cmd = roll_mom * moment_scale
        pitch_cmd = pitch_mom * moment_scale
        yaw_cmd = yaw_mom * moment_scale

        # Motor mixing (same as PID controller)
        m1 = alt_cmd - roll_cmd + pitch_cmd + yaw_cmd
        m2 = alt_cmd - roll_cmd - pitch_cmd - yaw_cmd
        m3 = alt_cmd + roll_cmd - pitch_cmd + yaw_cmd
        m4 = alt_cmd + roll_cmd + pitch_cmd - yaw_cmd

        # Clamp to valid range
        m1 = np.clip(m1, 0, 600)
        m2 = np.clip(m2, 0, 600)
        m3 = np.clip(m3, 0, 600)
        m4 = np.clip(m4, 0, 600)

        return [m1, m2, m3, m4]


def main():
    """Main control loop."""
    # Initialize Webots robot
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # For 100Hz control, we need timestep <= 10ms
    # Webots default is often 32ms, so we'll run MPC at simulation rate
    control_dt = timestep / 1000.0  # convert to seconds

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

    # Initialize keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # Initialize TinyMPC controller
    hover_height = 0.5  # Target hover height
    horizon = 20       # MPC horizon (N), 总预测时间 ≈ 0.2s

    mpc = TinyMPCController(
        dt=control_dt,
        hover_height=hover_height,
        horizon=horizon
    )

    # Target position
    target_x = 0.0
    target_y = 0.0
    target_z = hover_height

    # State tracking
    past_pos = [0.0, 0.0, 0.0]
    past_time = 0.0

    # Console logging
    last_print_time = 0.0
    print_interval = 1.0  # Print every 1 second

    # Data logger for MPC analysis
    # log_decimation=1 means record every sample (highest frequency)
    logger = MPCDataLogger(log_dir="logs", prefix="tinympc", log_decimation=1)

    # Keyboard debouncing
    last_key_time = {}  # Track last time each key was pressed
    KEY_DEBOUNCE = 0.3  # 300ms debounce

    print("\n" + "="*60)
    print("TinyMPC Hover Controller for CrazyFlie")
    print("="*60)
    print(f"\nTarget: hover at ({target_x}, {target_y}, {target_z})m")
    print("\nControls:")
    print("  Arrow keys: Move target X/Y")
    print("  W/S: Adjust target altitude")
    print("  R: Reset to origin")
    print("  SPACE: Print status")
    print("\nData Logging:")
    print("  L: Toggle logging ON/OFF")
    print("  K: Save log to CSV file")
    print("  P: Print log summary")
    print("="*60 + "\n")

    # Wait for sensors to initialize
    for _ in range(5):
        robot.step(timestep)

    # Main loop
    while robot.step(timestep) != -1:
        current_time = robot.getTime()

        # Get sensor readings
        gps_values = gps.getValues()
        imu_values = imu.getRollPitchYaw()
        gyro_values = gyro.getValues()

        # Handle keyboard input with debouncing
        key = keyboard.getKey()
        while key > 0:
            # Check debounce for this key
            if key in last_key_time and (current_time - last_key_time[key]) < KEY_DEBOUNCE:
                key = keyboard.getKey()
                continue
            last_key_time[key] = current_time

            if key == Keyboard.UP:
                target_x += 0.1
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == Keyboard.DOWN:
                target_x -= 0.1
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == Keyboard.LEFT:
                target_y += 0.1
                mpc.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == Keyboard.RIGHT:
                target_y -= 0.1
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
            # Data logging controls
            elif key == ord('L') or key == ord('l'):
                logger.toggle_logging()
            elif key == ord('K') or key == ord('k'):
                logger.save()
            elif key == ord('P') or key == ord('p'):
                print(logger.get_summary())
            key = keyboard.getKey()

        # Construct state vector
        state = mpc.get_state(
            gps_values, imu_values, gyro_values,
            past_pos, past_time, current_time
        )

        # Compute MPC control (with timing)
        solve_start = pytime.perf_counter()
        u = mpc.compute_control(state)
        solve_time_ms = (pytime.perf_counter() - solve_start) * 1000.0

        # Convert to motor commands
        motors = mpc.control_to_motors(u)

        # Log data
        logger.log(current_time, state, mpc.x_ref, u, motors, solve_time_ms)

        # Apply motor commands
        m1_motor.setVelocity(-motors[0])
        m2_motor.setVelocity(motors[1])
        m3_motor.setVelocity(-motors[2])
        m4_motor.setVelocity(motors[3])

        # Periodic status print
        if current_time - last_print_time >= print_interval:
            pos_err = np.sqrt((gps_values[0]-target_x)**2 +
                              (gps_values[1]-target_y)**2 +
                              (gps_values[2]-target_z)**2)
            print(f"[{current_time:6.1f}s] Pos:({gps_values[0]:+.3f},{gps_values[1]:+.3f},{gps_values[2]:+.3f}) | "
                  f"Err:{pos_err:.4f}m | u:[{u[0]:.4f},{u[1]:.6f},{u[2]:.6f},{u[3]:.6f}]")
            last_print_time = current_time

        # Update past state
        past_pos = list(gps_values)
        past_time = current_time


if __name__ == '__main__':
    main()
