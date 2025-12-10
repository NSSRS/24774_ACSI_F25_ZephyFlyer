"""
Trajectory Controller with External Force Disturbance Testing
Enhanced with XY Position Outer Loop (PD+I) for Wind Rejection

Based on Official Bitcraze PID Controller + Supervisor for force application
Added XY outer loop: position error → desired velocity (with integral for constant wind)

Author: Based on Bitcraze official code
Date: 2025-11-05
"""

from controller import Supervisor  # Use Supervisor instead of Robot
from controller import Motor, InertialUnit, GPS, Gyro, Keyboard
from math import cos, sin, pi, exp
import sys
sys.path.append('../../../../controllers_shared/python_based')
from pid_controller import pid_velocity_fixed_height_controller


class TrajectoryGenerator:
    """Generate simple trajectory patterns"""

    def __init__(self):
        self.trajectory_type = "hover"
        self.start_time = 0.0
        self.start_position = [0.0, 0.0, 1.0]
        self.target_position = [0.0, 0.0, 1.0]

    def set_trajectory(self, traj_type, start_time, current_pos):
        """Set trajectory type and starting position"""
        self.trajectory_type = traj_type
        self.start_time = start_time
        self.start_position = current_pos.copy()

        if traj_type == "hover":
            self.target_position = current_pos.copy()
            print(f"\n✅ Trajectory: HOVER at [{current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f}]")

        elif traj_type == "line_x":
            self.target_position = [current_pos[0] + 1.0, current_pos[1], current_pos[2]]
            print(f"\n✅ Trajectory: LINE +X (1.0m forward)")

        elif traj_type == "line_y":
            self.target_position = [current_pos[0], current_pos[1] + 1.0, current_pos[2]]
            print(f"\n✅ Trajectory: LINE +Y (1.0m left)")

        elif traj_type == "circle":
            self.target_position = current_pos.copy()
            print(f"\n✅ Trajectory: CIRCLE (0.3m radius, 20s period)")

    def get_target_position(self, current_time):
        """Get target position for current time"""
        t = current_time - self.start_time

        if self.trajectory_type == "hover":
            return self.target_position.copy()

        elif self.trajectory_type == "line_x":
            # Linear interpolation over 5 seconds
            progress = min(t / 5.0, 1.0)
            x = self.start_position[0] + progress * 1.0
            return [x, self.start_position[1], self.start_position[2]]

        elif self.trajectory_type == "line_y":
            # Linear interpolation over 5 seconds
            progress = min(t / 5.0, 1.0)
            y = self.start_position[1] + progress * 1.0
            return [self.start_position[0], y, self.start_position[2]]

        elif self.trajectory_type == "circle":
            # Circle: radius=0.3m, period=20s
            omega = 2 * pi / 20.0
            x = self.start_position[0] + 0.3 * cos(omega * t)
            y = self.start_position[1] + 0.3 * sin(omega * t)
            return [x, y, self.start_position[2]]

        return self.target_position.copy()


if __name__ == '__main__':
    # Use Supervisor instead of Robot
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    # Get reference to self (Crazyflie node)
    crazyflie_node = robot.getSelf()

    # Initialize motors (use official sign convention)
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

    # Get keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # Initialize trajectory generator
    trajectory = TrajectoryGenerator()

    # Initialize variables
    past_x_global = 0
    past_y_global = 0
    past_time = robot.getTime()

    # Use official Bitcraze PID controller
    PID_CF = pid_velocity_fixed_height_controller()

    height_desired = 1.0  # Default hover height

    # Control mode
    auto_mode = False  # False = manual, True = trajectory following

    # Force disturbance settings
    force_magnitude = 0.5  # Default: 0.5N (impulse force)
    last_force_time = -10.0  # Track last force application time

    # Continuous wind mode
    wind_enabled = False
    wind_magnitude = 0.01  # Default: 0.01N (much smaller for continuous force!)
    wind_force = [0.0, 0.0, 0.0]  # Continuous wind force [Fx, Fy, Fz]

    # XY Position Outer Loop Parameters (for wind/disturbance rejection)
    Kp_xy = 6.000E-01      # [1/s] Position proportional gain
    Kd_xy = 3.000E-01      # [-] Velocity damping gain
    Ki_xy = 5.000E-02      # [1/s^2] Position integral gain (for constant wind)
    v_xy_max = 3.000E-01   # [m/s] Maximum velocity command
    tau_v = 1.000E-01      # [s] Low-pass filter time constant for velocity

    # XY Outer Loop State Variables
    int_ep_x = 0.0  # Integral of body-frame X position error
    int_ep_y = 0.0  # Integral of body-frame Y position error
    vx_f = 0.0      # Filtered body-frame X velocity
    vy_f = 0.0      # Filtered body-frame Y velocity

    # Keyboard debouncing
    last_key_time = {}  # Track last time each key was pressed
    KEY_DEBOUNCE = 0.3  # 300ms debounce for sensitive keys

    print("\n" + "="*70)
    print("🚁 CrazyFlie Disturbance Testing (Enhanced XY Outer Loop)")
    print("="*70)
    print("\n✨ NEW: XY Position Outer Loop (PD+I control)")
    print(f"  Kp_xy={Kp_xy:.3f}, Kd_xy={Kd_xy:.3f}, Ki_xy={Ki_xy:.3f}")
    print(f"  v_xy_max={v_xy_max:.3f} m/s, tau_v={tau_v:.3f} s")
    print("  Features: Low-pass velocity filter + Integral for wind rejection")
    print("\n📋 MANUAL MODE Controls:")
    print("  Arrow Keys: Move horizontally (body frame)")
    print("  W/S: Altitude up/down")
    print("  Q/E: Yaw rotation")
    print("\n🎯 TRAJECTORY MODE:")
    print("  1: Hover at current position")
    print("  2: Line trajectory +X (1.0m forward)")
    print("  3: Line trajectory +Y (1.0m left)")
    print("  4: Circle trajectory (0.3m radius)")
    print("  M: Toggle Manual/Auto mode")
    print("  P: Print XY outer loop status (integrals, filtered velocity)")
    print("\n💥 IMPULSE FORCE (single push):")
    print("  F: Apply horizontal force +X (forward push)")
    print("  G: Apply horizontal force +Y (left push)")
    print("  H: Apply horizontal force -X (backward push)")
    print("  J: Apply horizontal force -Y (right push)")
    print("  U: Apply upward force +Z")
    print("  D: Apply downward force -Z")
    print("  [/]: Decrease/Increase impulse force (±0.5N step)")
    print(f"  Current impulse force: {force_magnitude:.1f} N (range: 0.5-50.0N)")
    print("\n🌬️  CONTINUOUS WIND MODE (applied every timestep!):")
    print("  V: Toggle wind ON/OFF")
    print("  7: Wind +X (forward)")
    print("  8: Wind +Y (left)")
    print("  9: Wind -X (backward)")
    print("  0: Wind -Y (right)")
    print("  -: Wind +Z (upward)")
    print("  =: Wind -Z (downward)")
    print("  ,/<: Decrease wind force (±0.005N step)")
    print("  ./>: Increase wind force (±0.005N step)")
    print(f"  Current wind force: {wind_magnitude:.3f} N (range: 0.005-0.5N)")
    print("  C: Clear all wind")
    print("="*70 + "\n")

    # Wait for sensors
    for _ in range(5):
        robot.step(timestep)

    # Main loop
    while robot.step(timestep) != -1:
        dt = robot.getTime() - past_time
        current_time = robot.getTime()

        if dt < 1e-6:
            continue

        # Get sensor data
        roll = imu.getRollPitchYaw()[0]
        pitch = imu.getRollPitchYaw()[1]
        yaw = imu.getRollPitchYaw()[2]
        yaw_rate = gyro.getValues()[2]
        altitude = gps.getValues()[2]
        x_global = gps.getValues()[0]
        v_x_global = (x_global - past_x_global) / dt
        y_global = gps.getValues()[1]
        v_y_global = (y_global - past_y_global) / dt

        current_pos = [x_global, y_global, altitude]

        # Get body-fixed velocities
        cosyaw = cos(yaw)
        sinyaw = sin(yaw)
        v_x = v_x_global * cosyaw + v_y_global * sinyaw
        v_y = -v_x_global * sinyaw + v_y_global * cosyaw

        # Initialize desired commands
        forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0
        height_diff_desired = 0

        # Keyboard input with debouncing
        key = keyboard.getKey()
        while key > 0:
            # Check if this key should be debounced
            key_needs_debounce = key in [
                ord('M'), ord('m'), ord('V'), ord('v'), ord('P'), ord('p'),
                ord('1'), ord('2'), ord('3'), ord('4'),
                ord('['), ord(']'), ord(','), ord('<'), ord('.'), ord('>'),
                ord('7'), ord('8'), ord('9'), ord('0'),
                ord('-'), ord('_'), ord('='), ord('+'),
                ord('C'), ord('c')
            ]

            # Debounce check
            if key_needs_debounce:
                if key in last_key_time and (current_time - last_key_time[key]) < KEY_DEBOUNCE:
                    key = keyboard.getKey()
                    continue  # Skip this key, too soon
                last_key_time[key] = current_time

            # Mode toggle
            if key == ord('M') or key == ord('m'):
                auto_mode = not auto_mode
                mode_str = "TRAJECTORY AUTO" if auto_mode else "MANUAL"
                print(f"\n🔄 Mode: {mode_str}")

            # Print XY outer loop status
            elif key == ord('P') or key == ord('p'):
                print(f"\n📊 XY Outer Loop Status (t={current_time:.2f}s):")
                print(f"  Position integrals: int_ep_x={int_ep_x:+.4f}, int_ep_y={int_ep_y:+.4f}")
                print(f"  Filtered velocity:  vx_f={vx_f:+.4f} m/s, vy_f={vy_f:+.4f} m/s")
                print(f"  Current position:   ({x_global:+.3f}, {y_global:+.3f}, {altitude:+.3f})")
                if auto_mode:
                    target_pos_debug = trajectory.get_target_position(current_time)
                    pos_err = ((target_pos_debug[0]-x_global)**2 + (target_pos_debug[1]-y_global)**2)**0.5
                    print(f"  Target position:    ({target_pos_debug[0]:+.3f}, {target_pos_debug[1]:+.3f}, {target_pos_debug[2]:+.3f})")
                    print(f"  XY position error:  {pos_err:.4f} m")
                print()

            # Trajectory selection
            elif key == ord('1'):
                trajectory.set_trajectory("hover", current_time, current_pos)
                auto_mode = True
            elif key == ord('2'):
                trajectory.set_trajectory("line_x", current_time, current_pos)
                auto_mode = True
            elif key == ord('3'):
                trajectory.set_trajectory("line_y", current_time, current_pos)
                auto_mode = True
            elif key == ord('4'):
                trajectory.set_trajectory("circle", current_time, current_pos)
                auto_mode = True

            # Force magnitude adjustment (impulse)
            elif key == ord('['):
                force_magnitude = max(0.5, force_magnitude - 0.5)
                print(f"💥 Impulse force: {force_magnitude:.1f} N (range: 0.5-50.0N)")
            elif key == ord(']'):
                force_magnitude = min(50.0, force_magnitude + 0.5)
                print(f"💥 Impulse force: {force_magnitude:.1f} N (range: 0.5-50.0N)")

            # Wind magnitude adjustment (continuous)
            elif key == ord(',') or key == ord('<'):
                wind_magnitude = max(0.005, wind_magnitude - 0.005)
                print(f"🌬️  Wind force: {wind_magnitude:.3f} N (range: 0.005-0.5N)")
            elif key == ord('.') or key == ord('>'):
                wind_magnitude = min(0.5, wind_magnitude + 0.005)
                print(f"🌬️  Wind force: {wind_magnitude:.3f} N (range: 0.005-0.5N)")

            # Wind mode toggle
            elif key == ord('V') or key == ord('v'):
                wind_enabled = not wind_enabled
                status = "ENABLED" if wind_enabled else "DISABLED"
                print(f"\n🌬️  Wind mode: {status}")
                if wind_enabled and (wind_force[0] != 0 or wind_force[1] != 0 or wind_force[2] != 0):
                    print(f"   Wind force: [{wind_force[0]:+.3f}, {wind_force[1]:+.3f}, {wind_force[2]:+.3f}] N")

            # Set continuous wind direction (uses wind_magnitude, not force_magnitude!)
            elif key == ord('7'):
                wind_force = [wind_magnitude, 0, 0]
                print(f"🌬️  Wind set: +X {wind_magnitude:.3f}N (forward)")
                if not wind_enabled:
                    print("   ⚠️  Press 'V' to enable wind")
            elif key == ord('8'):
                wind_force = [0, wind_magnitude, 0]
                print(f"🌬️  Wind set: +Y {wind_magnitude:.3f}N (left)")
                if not wind_enabled:
                    print("   ⚠️  Press 'V' to enable wind")
            elif key == ord('9'):
                wind_force = [-wind_magnitude, 0, 0]
                print(f"🌬️  Wind set: -X {wind_magnitude:.3f}N (backward)")
                if not wind_enabled:
                    print("   ⚠️  Press 'V' to enable wind")
            elif key == ord('0'):
                wind_force = [0, -wind_magnitude, 0]
                print(f"🌬️  Wind set: -Y {wind_magnitude:.3f}N (right)")
                if not wind_enabled:
                    print("   ⚠️  Press 'V' to enable wind")
            elif key == ord('-') or key == ord('_'):
                wind_force = [0, 0, wind_magnitude]
                print(f"🌬️  Wind set: +Z {wind_magnitude:.3f}N (upward)")
                if not wind_enabled:
                    print("   ⚠️  Press 'V' to enable wind")
            elif key == ord('=') or key == ord('+'):
                wind_force = [0, 0, -wind_magnitude]
                print(f"🌬️  Wind set: -Z {wind_magnitude:.3f}N (downward)")
                if not wind_enabled:
                    print("   ⚠️  Press 'V' to enable wind")
            elif key == ord('C') or key == ord('c'):
                wind_force = [0, 0, 0]
                print("🌬️  Wind cleared")
                if wind_enabled:
                    print("   Wind mode still enabled but force = 0")

            # Apply external forces (with cooldown to prevent spamming)
            elif current_time - last_force_time > 0.5:  # 0.5s cooldown
                force_applied = False

                if key == ord('F') or key == ord('f'):
                    # Forward push (+X in world frame)
                    crazyflie_node.addForce([force_magnitude, 0, 0], False)
                    print(f"💥 Applied force: +X {force_magnitude:.3f}N (forward)")
                    force_applied = True

                elif key == ord('G') or key == ord('g'):
                    # Left push (+Y in world frame)
                    crazyflie_node.addForce([0, force_magnitude, 0], False)
                    print(f"💥 Applied force: +Y {force_magnitude:.3f}N (left)")
                    force_applied = True

                elif key == ord('H') or key == ord('h'):
                    # Backward push (-X in world frame)
                    crazyflie_node.addForce([-force_magnitude, 0, 0], False)
                    print(f"💥 Applied force: -X {force_magnitude:.3f}N (backward)")
                    force_applied = True

                elif key == ord('J') or key == ord('j'):
                    # Right push (-Y in world frame)
                    crazyflie_node.addForce([0, -force_magnitude, 0], False)
                    print(f"💥 Applied force: -Y {force_magnitude:.3f}N (right)")
                    force_applied = True

                elif key == ord('U') or key == ord('u'):
                    # Upward push (+Z)
                    crazyflie_node.addForce([0, 0, force_magnitude], False)
                    print(f"💥 Applied force: +Z {force_magnitude:.3f}N (upward)")
                    force_applied = True

                elif key == ord('D') or key == ord('d'):
                    # Downward push (-Z)
                    crazyflie_node.addForce([0, 0, -force_magnitude], False)
                    print(f"💥 Applied force: -Z {force_magnitude:.3f}N (downward)")
                    force_applied = True

                if force_applied:
                    last_force_time = current_time

            # Manual controls (only in manual mode)
            elif not auto_mode:
                if key == Keyboard.UP:
                    forward_desired += 0.5
                elif key == Keyboard.DOWN:
                    forward_desired -= 0.5
                elif key == Keyboard.RIGHT:
                    sideways_desired -= 0.5
                elif key == Keyboard.LEFT:
                    sideways_desired += 0.5
                elif key == ord('Q'):
                    yaw_desired = +1
                elif key == ord('E'):
                    yaw_desired = -1
                elif key == ord('W'):
                    height_diff_desired = 0.1
                elif key == ord('S'):
                    height_diff_desired = -0.1

            key = keyboard.getKey()

        # Trajectory following (auto mode)
        if auto_mode:
            target_pos = trajectory.get_target_position(current_time)

            # Enhanced XY Position Outer Loop (PD+I control with anti-windup)
            # Step 1: Position error in world frame
            ep_x_w = target_pos[0] - x_global
            ep_y_w = target_pos[1] - y_global

            # Step 2: Rotate world error → body frame using yaw
            ep_x_b = ep_x_w * cosyaw + ep_y_w * sinyaw
            ep_y_b = -ep_x_w * sinyaw + ep_y_w * cosyaw

            # Step 3: Body-frame velocity (rotate world velocity)
            vxb = v_x_global * cosyaw + v_y_global * sinyaw
            vyb = -v_x_global * sinyaw + v_y_global * cosyaw

            # Step 4: Low-pass filter velocity (reduces noise amplification)
            alpha = exp(-dt / tau_v)
            vx_f = alpha * vx_f + (1.0 - alpha) * vxb
            vy_f = alpha * vy_f + (1.0 - alpha) * vyb

            # Step 5: Integrate position error (for constant-wind rejection)
            int_ep_x += ep_x_b * dt
            int_ep_y += ep_y_b * dt

            # Step 6: PD+I control: position error → desired body-frame velocity
            fwd_cmd = Kp_xy * ep_x_b - Kd_xy * vx_f + Ki_xy * int_ep_x
            side_cmd = Kp_xy * ep_y_b - Kd_xy * vy_f + Ki_xy * int_ep_y

            # Step 7: Saturate velocity commands
            def clip(u, m):
                return max(-m, min(m, u))
            fwd_cmd = clip(fwd_cmd, v_xy_max)
            side_cmd = clip(side_cmd, v_xy_max)

            # Step 8: Simple anti-windup: if saturated and pushing further, undo this step's integral
            if abs(fwd_cmd) >= v_xy_max and (fwd_cmd * ep_x_b) > 0.0:
                int_ep_x -= ep_x_b * dt
            if abs(side_cmd) >= v_xy_max and (side_cmd * ep_y_b) > 0.0:
                int_ep_y -= ep_y_b * dt

            # Pass enhanced commands to inner loop
            forward_desired = fwd_cmd
            sideways_desired = side_cmd

            # Height
            height_desired = target_pos[2]
        else:
            # Manual height adjustment
            height_desired += height_diff_desired * dt

        # Official PID controller (unchanged)
        motor_power = PID_CF.pid(dt, forward_desired, sideways_desired,
                                  yaw_desired, height_desired,
                                  roll, pitch, yaw_rate,
                                  altitude, v_x, v_y)

        # Apply motor commands (use official sign convention)
        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        # Apply continuous wind force (if enabled)
        if wind_enabled:
            crazyflie_node.addForce(wind_force, False)

        # Update state
        past_time = current_time
        past_x_global = x_global
        past_y_global = y_global
