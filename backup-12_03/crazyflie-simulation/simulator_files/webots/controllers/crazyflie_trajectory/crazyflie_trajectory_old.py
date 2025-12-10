"""
Trajectory Tracking Controller for Crazyflie in Webots

This controller extends the simple PID controller with trajectory tracking:
- Circle trajectory
- Figure-8 trajectory
- Square trajectory
- Custom waypoint following

Author: Claude Code & Hanzel
Date: 2025-11-04
"""

from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard
import numpy as np
from math import cos, sin, pi


class SimplePIDController:
    """Simple 3-term PID controller with anti-windup"""

    def __init__(self, kp, ki, kd, output_limits=None, integral_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits if output_limits else (-float('inf'), float('inf'))
        self.integral_limits = integral_limits if integral_limits else (-float('inf'), float('inf'))

        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        """Compute PID output"""
        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, self.integral_limits[0], self.integral_limits[1])
        i_term = self.ki * self.integral

        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        # Total output with limits
        output = p_term + i_term + d_term
        output = np.clip(output, self.output_limits[0], self.output_limits[1])

        return output

    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.prev_error = 0.0


class TrajectoryGenerator:
    """Generate various trajectory patterns"""

    def __init__(self):
        self.trajectory_type = "hover"  # hover, circle, figure8, square
        self.start_time = 0.0
        self.center = [0.0, 0.0, 1.0]

        # Circle parameters
        self.radius = 0.5  # meters
        self.period = 10.0  # seconds per circle

        # Figure-8 parameters
        self.fig8_scale = 0.5

        # Square parameters
        self.square_size = 1.0

    def set_trajectory(self, traj_type, start_time):
        """Set trajectory type and start time"""
        self.trajectory_type = traj_type
        self.start_time = start_time
        print(f"\n✈️  Trajectory set to: {traj_type}")

    def get_target_position(self, current_time):
        """Get target position at current time"""
        t = current_time - self.start_time

        if self.trajectory_type == "hover":
            return self.center.copy()

        elif self.trajectory_type == "circle":
            # Circular trajectory in XY plane
            omega = 2 * pi / self.period
            x = self.center[0] + self.radius * cos(omega * t)
            y = self.center[1] + self.radius * sin(omega * t)
            z = self.center[2]
            return [x, y, z]

        elif self.trajectory_type == "figure8":
            # Figure-8 trajectory (lemniscate)
            omega = 2 * pi / (self.period * 2)  # Double period for figure-8
            x = self.center[0] + self.fig8_scale * sin(omega * t)
            y = self.center[1] + self.fig8_scale * sin(2 * omega * t) / 2
            z = self.center[2]
            return [x, y, z]

        elif self.trajectory_type == "square":
            # Square trajectory
            side_time = self.period / 4
            progress = (t % self.period) / self.period  # 0 to 1
            half_size = self.square_size / 2

            if progress < 0.25:  # Side 1: +X
                x = self.center[0] + half_size * (progress * 4)
                y = self.center[1] - half_size
            elif progress < 0.5:  # Side 2: +Y
                x = self.center[0] + half_size
                y = self.center[1] - half_size + half_size * 2 * ((progress - 0.25) * 4)
            elif progress < 0.75:  # Side 3: -X
                x = self.center[0] + half_size - half_size * 2 * ((progress - 0.5) * 4)
                y = self.center[1] + half_size
            else:  # Side 4: -Y
                x = self.center[0] - half_size
                y = self.center[1] + half_size - half_size * 2 * ((progress - 0.75) * 4)

            z = self.center[2]
            return [x, y, z]

        else:
            return self.center.copy()


class CrazyfliePositionController:
    """Cascade PID controller for position control"""

    def __init__(self):
        # Position control (outer loop) - produces desired velocity
        self.pid_x = SimplePIDController(
            kp=0.5, ki=0.05, kd=0.3,
            output_limits=(-0.3, 0.3),
            integral_limits=(-0.1, 0.1)
        )
        self.pid_y = SimplePIDController(
            kp=0.5, ki=0.05, kd=0.3,
            output_limits=(-0.3, 0.3),
            integral_limits=(-0.1, 0.1)
        )

        # Altitude control - optimized to reduce overshoot
        self.pid_z = SimplePIDController(
            kp=2.0, ki=3.0, kd=8.0,
            output_limits=(-15, 15),
            integral_limits=(-2.5, 2.5)
        )

        # Velocity control (middle loop) - produces desired attitude
        self.pid_vx = SimplePIDController(
            kp=2.0, ki=0.0, kd=0.5,
            output_limits=(-0.3, 0.3),
            integral_limits=(-0.1, 0.1)
        )
        self.pid_vy = SimplePIDController(
            kp=2.0, ki=0.0, kd=0.5,
            output_limits=(-0.3, 0.3),
            integral_limits=(-0.1, 0.1)
        )

        # Attitude control (inner loop) - produces moment commands
        self.pid_roll = SimplePIDController(
            kp=0.5, ki=0.0, kd=0.1,
            output_limits=(-10, 10),
            integral_limits=(-1.0, 1.0)
        )
        self.pid_pitch = SimplePIDController(
            kp=0.5, ki=0.0, kd=0.1,
            output_limits=(-10, 10),
            integral_limits=(-1.0, 1.0)
        )
        self.pid_yaw = SimplePIDController(
            kp=1.0, ki=0.0, kd=0.5,
            output_limits=(-5, 5),
            integral_limits=(-1.0, 1.0)
        )

    def compute_control(self, target_pos, current_pos, current_vel, current_attitude,
                       current_yaw_rate, dt, target_yaw_rate=0.0):
        """
        Compute motor commands from position error

        Args:
            target_pos: [x, y, z] target position
            current_pos: [x, y, z] current position from GPS
            current_vel: [vx, vy] current velocity in body frame
            current_attitude: [roll, pitch, yaw] current attitude
            current_yaw_rate: current yaw rate
            dt: time step
            target_yaw_rate: desired yaw rate (default 0)

        Returns:
            [m1, m2, m3, m4] motor commands
        """
        # Outer loop: Position → Desired velocity
        desired_vx = self.pid_x.update(target_pos[0] - current_pos[0], dt)
        desired_vy = self.pid_y.update(target_pos[1] - current_pos[1], dt)

        # Middle loop: Velocity → Desired attitude
        desired_pitch = self.pid_vx.update(desired_vx - current_vel[0], dt)
        desired_roll = -self.pid_vy.update(desired_vy - current_vel[1], dt)

        # Altitude loop: Height → Thrust
        altitude_error = target_pos[2] - current_pos[2]
        thrust_adjustment = self.pid_z.update(altitude_error, dt)
        base_thrust = 48.0  # Hover thrust
        thrust_command = base_thrust + thrust_adjustment

        # Inner loop: Attitude → Moments
        roll_command = self.pid_roll.update(desired_roll - current_attitude[0], dt)
        pitch_command = -self.pid_pitch.update(desired_pitch - current_attitude[1], dt)
        yaw_command = self.pid_yaw.update(target_yaw_rate - current_yaw_rate, dt)

        # Motor mixing
        m1 = thrust_command - roll_command + pitch_command + yaw_command
        m2 = thrust_command - roll_command - pitch_command - yaw_command
        m3 = thrust_command + roll_command - pitch_command + yaw_command
        m4 = thrust_command + roll_command + pitch_command - yaw_command

        # Limit motor commands
        m1 = np.clip(m1, 0, 600)
        m2 = np.clip(m2, 0, 600)
        m3 = np.clip(m3, 0, 600)
        m4 = np.clip(m4, 0, 600)

        return [m1, m2, m3, m4]


def main():
    # Initialize robot
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Initialize motors
    motors = []
    for i in range(1, 5):
        motor = robot.getDevice(f"m{i}_motor")
        motor.setPosition(float('inf'))
        motor.setVelocity(1.0 if i % 2 == 0 else -1.0)
        motors.append(motor)

    # Initialize sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # Initialize controller and trajectory generator
    controller = CrazyfliePositionController()
    trajectory = TrajectoryGenerator()

    # State variables
    past_x = 0.0
    past_y = 0.0
    past_z = 0.0
    past_time = robot.getTime()

    # Manual control target (used when not following trajectory)
    manual_target = [0.0, 0.0, 1.0]

    print("\n" + "="*70)
    print("🚁 Trajectory Tracking Controller for Crazyflie")
    print("="*70)
    print("\n🎮 Manual Controls:")
    print("  Arrow Keys  - Move forward/backward/left/right [±0.2m]")
    print("  W/S         - Move up/down [±0.1m]")
    print("  A/D         - Alternative left/right [±0.2m]")
    print("  Q/E         - Rotate left/right (Yaw)")
    print("  R           - Reset to origin (0,0,1)")
    print("\n🛤️  Trajectory Commands:")
    print("  1           - Hover at current position")
    print("  2           - Circle trajectory (radius=0.5m, period=10s)")
    print("  3           - Figure-8 trajectory")
    print("  4           - Square trajectory (1m x 1m)")
    print("  +/-         - Increase/decrease trajectory speed")
    print("\n📊 Info:")
    print("  SPACE       - Print current status")
    print("\n💡 Tip: Click on 3D window to ensure keyboard focus!")
    print("="*70 + "\n")

    # Main control loop
    step_count = 0
    while robot.step(timestep) != -1:
        current_time = robot.getTime()
        dt = current_time - past_time

        if dt < 1e-6:  # Skip if dt too small
            continue

        # Read sensor data
        gps_values = gps.getValues()
        current_pos = [gps_values[0], gps_values[1], gps_values[2]]

        roll, pitch, yaw = imu.getRollPitchYaw()
        yaw_rate = gyro.getValues()[2]

        # Compute velocity in body frame
        x_global, y_global = gps_values[0], gps_values[1]
        vx_global = (x_global - past_x) / dt
        vy_global = (y_global - past_y) / dt

        cosyaw, sinyaw = cos(yaw), sin(yaw)
        vx_body = vx_global * cosyaw + vy_global * sinyaw
        vy_body = -vx_global * sinyaw + vy_global * cosyaw

        # Handle keyboard input
        target_yaw_rate = 0.0
        key = keyboard.getKey()

        # DEBUG: Print any key press
        if key > 0:
            print(f"🔍 DEBUG: Key pressed = {key} (char: {chr(key) if 32 <= key <= 126 else 'special'})")

        while key > 0:
            # Manual position control (override trajectory)
            if key == Keyboard.UP:
                manual_target[0] += 0.2
                trajectory.set_trajectory("hover", current_time)
                trajectory.center = manual_target.copy()
                print(f"⬆️  Forward: Target X = {manual_target[0]:.2f}m")
            elif key == Keyboard.DOWN:
                manual_target[0] -= 0.2
                trajectory.set_trajectory("hover", current_time)
                trajectory.center = manual_target.copy()
                print(f"⬇️  Backward: Target X = {manual_target[0]:.2f}m")
            elif key == Keyboard.RIGHT:
                manual_target[1] -= 0.2
                trajectory.set_trajectory("hover", current_time)
                trajectory.center = manual_target.copy()
                print(f"➡️  Right: Target Y = {manual_target[1]:.2f}m")
            elif key == Keyboard.LEFT:
                manual_target[1] += 0.2
                trajectory.set_trajectory("hover", current_time)
                trajectory.center = manual_target.copy()
                print(f"⬅️  Left: Target Y = {manual_target[1]:.2f}m")
            elif key == ord('W') or key == ord('w'):
                manual_target[2] += 0.1
                trajectory.set_trajectory("hover", current_time)
                trajectory.center = manual_target.copy()
                print(f"🔼 Up: Target Z = {manual_target[2]:.2f}m")
            elif key == ord('S') or key == ord('s'):
                manual_target[2] -= 0.1
                manual_target[2] = max(0.1, manual_target[2])
                trajectory.set_trajectory("hover", current_time)
                trajectory.center = manual_target.copy()
                print(f"🔽 Down: Target Z = {manual_target[2]:.2f}m")
            elif key == ord('A') or key == ord('a'):
                manual_target[1] += 0.2
                trajectory.set_trajectory("hover", current_time)
                trajectory.center = manual_target.copy()
                print(f"⬅️  Left (A): Target Y = {manual_target[1]:.2f}m")
            elif key == ord('D') or key == ord('d'):
                manual_target[1] -= 0.2
                trajectory.set_trajectory("hover", current_time)
                trajectory.center = manual_target.copy()
                print(f"➡️  Right (D): Target Y = {manual_target[1]:.2f}m")

            # Trajectory commands
            elif key == ord('1'):
                trajectory.center = current_pos.copy()
                trajectory.set_trajectory("hover", current_time)
            elif key == ord('2'):
                trajectory.center = current_pos.copy()
                trajectory.set_trajectory("circle", current_time)
                print(f"   Radius: {trajectory.radius}m, Period: {trajectory.period}s")
            elif key == ord('3'):
                trajectory.center = current_pos.copy()
                trajectory.set_trajectory("figure8", current_time)
                print(f"   Scale: {trajectory.fig8_scale}m, Period: {trajectory.period}s")
            elif key == ord('4'):
                trajectory.center = current_pos.copy()
                trajectory.set_trajectory("square", current_time)
                print(f"   Size: {trajectory.square_size}m, Period: {trajectory.period}s")
            elif key == ord('+') or key == ord('='):
                trajectory.period = max(5.0, trajectory.period - 1.0)
                print(f"⚡ Speed increased: Period = {trajectory.period}s")
            elif key == ord('-') or key == ord('_'):
                trajectory.period = min(30.0, trajectory.period + 1.0)
                print(f"🐢 Speed decreased: Period = {trajectory.period}s")

            # Other commands
            elif key == ord('Q') or key == ord('q'):
                target_yaw_rate = 1.0
                print("↺ Yaw Left")
            elif key == ord('E') or key == ord('e'):
                target_yaw_rate = -1.0
                print("↻ Yaw Right")
            elif key == ord('R') or key == ord('r'):
                manual_target = [0.0, 0.0, 1.0]
                trajectory.center = manual_target.copy()
                trajectory.set_trajectory("hover", current_time)
                print("🔄 Reset to origin (0, 0, 1)")
            elif key == ord(' '):
                target_pos = trajectory.get_target_position(current_time)
                pos_error = np.linalg.norm([target_pos[i] - current_pos[i] for i in range(3)])
                print(f"\n[t={current_time:.2f}s] Status:")
                print(f"  Mode:     {trajectory.trajectory_type}")
                print(f"  Position: ({current_pos[0]:+.3f}, {current_pos[1]:+.3f}, {current_pos[2]:.3f})")
                print(f"  Target:   ({target_pos[0]:+.3f}, {target_pos[1]:+.3f}, {target_pos[2]:.3f})")
                print(f"  Error:    {pos_error:.3f}m")
                print(f"  Attitude: Roll={roll*57.3:.1f}°, Pitch={pitch*57.3:.1f}°, Yaw={yaw*57.3:.1f}°\n")
            else:
                # DEBUG: Unhandled key
                if 32 <= key <= 126:
                    print(f"❌ Unhandled key: {chr(key)} (code: {key})")
                else:
                    print(f"❌ Unhandled special key: {key}")

            key = keyboard.getKey()

        # Get target position from trajectory
        target_position = trajectory.get_target_position(current_time)

        # Compute motor commands
        motor_commands = controller.compute_control(
            target_position,
            current_pos,
            [vx_body, vy_body],
            [roll, pitch, yaw],
            yaw_rate,
            dt,
            target_yaw_rate
        )

        # Apply motor commands
        motors[0].setVelocity(-motor_commands[0])  # M1
        motors[1].setVelocity(motor_commands[1])    # M2
        motors[2].setVelocity(-motor_commands[2])  # M3
        motors[3].setVelocity(motor_commands[3])    # M4

        # Print detailed status every 0.5 seconds (ALL MODES, including hover)
        if step_count % int(0.5 / (timestep / 1000.0)) == 0:
            pos_error = np.linalg.norm([target_position[i] - current_pos[i] for i in range(3)])
            z_error = target_position[2] - current_pos[2]
            print(f"[{current_time:6.2f}s] Mode:{trajectory.trajectory_type:8s} | "
                  f"Pos:[{current_pos[0]:+.2f},{current_pos[1]:+.2f},{current_pos[2]:.2f}] | "
                  f"Target:[{target_position[0]:+.2f},{target_position[1]:+.2f},{target_position[2]:.2f}] | "
                  f"Err:{pos_error:.3f}m | Z_err:{z_error:+.3f}m | "
                  f"Motors:[{int(motor_commands[0])},{int(motor_commands[1])},{int(motor_commands[2])},{int(motor_commands[3])}]")

        # Update state
        past_x = x_global
        past_y = y_global
        past_z = current_pos[2]
        past_time = current_time
        step_count += 1


if __name__ == "__main__":
    main()
