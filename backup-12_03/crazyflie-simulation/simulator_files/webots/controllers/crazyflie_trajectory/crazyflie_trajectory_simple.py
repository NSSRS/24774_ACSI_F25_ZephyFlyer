"""
Simplified Trajectory Controller - Fixed keyboard handling

Fixes:
1. Debounce keyboard input (only trigger once per key press)
2. Better handling of key repeat
3. Simpler trajectory switching
4. More stable position control

Author: Claude Code & Hanzel
Date: 2025-11-04
"""

from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard
import numpy as np
from math import cos, sin, pi


class SimplePIDController:
    """Advanced PID controller with D-on-measurement, low-pass filter, and back-calculation anti-windup"""

    def __init__(self, kp, ki, kd, output_limits=None, integral_limits=None, d_cut_hz=None, aw_gain=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits if output_limits else (-float('inf'), float('inf'))
        self.integral_limits = integral_limits if integral_limits else (-float('inf'), float('inf'))

        # D-on-measurement: derivative of measurement, not error
        self.d_cut_hz = d_cut_hz  # Low-pass filter cutoff frequency for D term
        self.aw_gain = aw_gain if aw_gain is not None else 1.0  # Back-calculation anti-windup gain

        self.integral = 0.0
        self.prev_measurement = None  # For D-on-measurement
        self.d_filtered = 0.0  # Low-pass filtered derivative

    def update(self, error, dt, meas=None):
        """
        Compute PID output with advanced features

        Args:
            error: Control error (setpoint - measurement)
            dt: Time step
            meas: Measurement value (for D-on-measurement). If None, falls back to D-on-error

        Returns:
            PID output (limited)
        """
        # Proportional term
        p_term = self.kp * error

        # Integral term (will apply back-calculation anti-windup later)
        self.integral += error * dt
        self.integral = np.clip(self.integral, self.integral_limits[0], self.integral_limits[1])
        i_term = self.ki * self.integral

        # Derivative term: D-on-measurement with low-pass filter
        if meas is not None and self.prev_measurement is not None and dt > 0:
            # D-on-measurement: negative derivative of measurement
            d_raw = -(meas - self.prev_measurement) / dt

            # Low-pass filter for D term
            if self.d_cut_hz is not None and self.d_cut_hz > 0:
                alpha = dt / (dt + 1.0 / (2.0 * 3.14159 * self.d_cut_hz))
                self.d_filtered = alpha * d_raw + (1 - alpha) * self.d_filtered
            else:
                self.d_filtered = d_raw

            d_term = self.kd * self.d_filtered
        else:
            d_term = 0.0

        if meas is not None:
            self.prev_measurement = meas

        # Total output BEFORE limiting
        output_unlim = p_term + i_term + d_term

        # Apply output limits
        output = np.clip(output_unlim, self.output_limits[0], self.output_limits[1])

        # Back-calculation anti-windup: reduce integral if output is saturated
        if self.aw_gain > 0 and dt > 0:
            saturation_error = output - output_unlim  # How much we clipped
            self.integral += (saturation_error / self.ki) * self.aw_gain if self.ki != 0 else 0
            self.integral = np.clip(self.integral, self.integral_limits[0], self.integral_limits[1])

        return output

    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.prev_measurement = None
        self.d_filtered = 0.0


class TrajectoryGenerator:
    """Generate various trajectory patterns"""

    def __init__(self):
        self.trajectory_type = "hover"
        self.start_time = 0.0
        self.center = [0.0, 0.0, 1.0]
        self.radius = 0.2  # REDUCED from 0.3m for very conservative trajectory
        self.period = 40.0  # INCREASED from 20s for very slow speed (v = 0.031 m/s)

    def set_trajectory(self, traj_type, start_time):
        """Set trajectory type"""
        self.trajectory_type = traj_type
        self.start_time = start_time
        print(f"\n✅ Trajectory: {traj_type}")
        if traj_type == "circle":
            speed = 2 * 3.14159 * self.radius / self.period
            print(f"   ⭕ Circle: radius={self.radius}m, period={self.period}s, speed={speed:.3f}m/s")
        elif traj_type == "figure8":
            print(f"   ∞ Figure-8: scale=0.5m")
        elif traj_type == "square":
            print(f"   ⬜ Square: size=1.0m")

    def get_target_position(self, current_time):
        """Get target position"""
        t = current_time - self.start_time

        if self.trajectory_type == "hover":
            return self.center.copy()

        elif self.trajectory_type == "circle":
            omega = 2 * pi / self.period
            x = self.center[0] + self.radius * cos(omega * t)
            y = self.center[1] + self.radius * sin(omega * t)
            z = self.center[2]
            return [x, y, z]

        elif self.trajectory_type == "figure8":
            omega = 2 * pi / (self.period * 2)
            x = self.center[0] + 0.5 * sin(omega * t)
            y = self.center[1] + 0.5 * sin(2 * omega * t) / 2
            z = self.center[2]
            return [x, y, z]

        elif self.trajectory_type == "square":
            progress = (t % self.period) / self.period
            half_size = 0.5

            if progress < 0.25:
                x = self.center[0] + half_size * (progress * 4)
                y = self.center[1] - half_size
            elif progress < 0.5:
                x = self.center[0] + half_size
                y = self.center[1] - half_size + half_size * 2 * ((progress - 0.25) * 4)
            elif progress < 0.75:
                x = self.center[0] + half_size - half_size * 2 * ((progress - 0.5) * 4)
                y = self.center[1] + half_size
            else:
                x = self.center[0] - half_size
                y = self.center[1] + half_size - half_size * 2 * ((progress - 0.75) * 4)

            z = self.center[2]
            return [x, y, z]

        return self.center.copy()


class CrazyfliePositionController:
    """Cascade PID controller"""

    def __init__(self):
        # Position control (outer loop) - CONSERVATIVE for stability
        self.pid_x = SimplePIDController(
            kp=0.3,   # REDUCED from 1.5 to prevent saturation (0.3*0.2m = 0.06 m/s << 0.2 limit)
            ki=0.02,  # REDUCED from 0.1 to prevent integral windup
            kd=0.15,  # REDUCED from 0.5 to reduce noise sensitivity
            output_limits=(-0.15, 0.15),  # REDUCED from 0.2 for more margin
            integral_limits=(-0.1, 0.1)   # REDUCED for safety
        )
        self.pid_y = SimplePIDController(
            kp=0.3, ki=0.02, kd=0.15,
            output_limits=(-0.15, 0.15),
            integral_limits=(-0.1, 0.1)
        )

        # Altitude control - ADVANCED: D-on-measurement, low-pass filter, back-calculation anti-windup
        self.pid_z = SimplePIDController(
            kp=1.2,    # REDUCED: prevent initial overshoot
            ki=0.20,   # VERY SMALL: prevent integral windup and steady-state error
            kd=0.15,   # SMALL: prevent derivative kick, will be low-pass filtered
            output_limits=(-8, 8),      # REDUCED: gentler thrust adjustment
            integral_limits=(-0.6, 0.6),  # SMALL: prevent integral accumulation
            d_cut_hz=20.0,  # Low-pass filter cutoff for D term
            aw_gain=3.0     # Back-calculation anti-windup gain
        )

        # Velocity control (middle loop) - VERY CONSERVATIVE ATTITUDE
        self.pid_vx = SimplePIDController(
            kp=1.0,   # REDUCED from 1.5 to prevent aggressive attitude
            ki=0.0,
            kd=0.2,   # REDUCED from 0.3 for stability
            output_limits=(-0.1, 0.1),  # REDUCED to ~5.7° max angle (was 8.6°)
            integral_limits=(-0.05, 0.05)
        )
        self.pid_vy = SimplePIDController(
            kp=1.0, ki=0.0, kd=0.2,
            output_limits=(-0.1, 0.1),  # Max ~5.7° attitude angle
            integral_limits=(-0.05, 0.05)
        )

        # Attitude control (inner loop) - REDUCE OUTPUT to prevent aggressive maneuvers
        self.pid_roll = SimplePIDController(
            kp=0.5, ki=0.0, kd=0.1,
            output_limits=(-3, 3),  # REDUCED from 10 to prevent fast rolling
            integral_limits=(-0.5, 0.5)
        )
        self.pid_pitch = SimplePIDController(
            kp=0.5, ki=0.0, kd=0.1,
            output_limits=(-3, 3),  # REDUCED from 10 to prevent fast pitching
            integral_limits=(-0.5, 0.5)
        )
        self.pid_yaw = SimplePIDController(
            kp=1.0, ki=0.0, kd=0.5,
            output_limits=(-2, 2),  # REDUCED from 5 for gentler yaw
            integral_limits=(-0.5, 0.5)
        )

    def compute_control(self, target_pos, current_pos, current_vel, current_attitude, current_yaw_rate, dt, target_yaw_rate=0.0):
        desired_vx = self.pid_x.update(target_pos[0] - current_pos[0], dt)
        desired_vy = self.pid_y.update(target_pos[1] - current_pos[1], dt)
        desired_pitch = self.pid_vx.update(desired_vx - current_vel[0], dt)
        desired_roll = -self.pid_vy.update(desired_vy - current_vel[1], dt)

        # Altitude control with D-on-measurement
        altitude_error = target_pos[2] - current_pos[2]
        thrust_adjustment = self.pid_z.update(altitude_error, dt, meas=current_pos[2])
        base_thrust = 48.0
        thrust_command = base_thrust + thrust_adjustment

        roll_command = self.pid_roll.update(desired_roll - current_attitude[0], dt)
        pitch_command = -self.pid_pitch.update(desired_pitch - current_attitude[1], dt)
        yaw_command = self.pid_yaw.update(target_yaw_rate - current_yaw_rate, dt)

        m1 = thrust_command - roll_command + pitch_command + yaw_command
        m2 = thrust_command - roll_command - pitch_command - yaw_command
        m3 = thrust_command + roll_command - pitch_command + yaw_command
        m4 = thrust_command + roll_command + pitch_command - yaw_command

        m1 = np.clip(m1, 0, 600)
        m2 = np.clip(m2, 0, 600)
        m3 = np.clip(m3, 0, 600)
        m4 = np.clip(m4, 0, 600)

        return [m1, m2, m3, m4]


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    motors = []
    for i in range(1, 5):
        motor = robot.getDevice(f"m{i}_motor")
        motor.setPosition(float('inf'))
        motor.setVelocity(1.0 if i % 2 == 0 else -1.0)
        motors.append(motor)

    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    keyboard = Keyboard()
    keyboard.enable(timestep)

    controller = CrazyfliePositionController()
    trajectory = TrajectoryGenerator()

    past_x = 0.0
    past_y = 0.0
    past_z = 0.0
    past_time = robot.getTime()

    # Z setpoint smoothing (2-second ramp)
    z_sp_smooth = 0.05  # Initialize to starting height
    z_sp_tau = 2.0  # Time constant for setpoint ramp (seconds)

    # Key debouncing
    last_key_time = {}
    KEY_DEBOUNCE = 0.3  # 300ms debounce

    print("\n" + "="*70)
    print("🚁 Simplified Trajectory Controller (Fixed Keyboard)")
    print("="*70)
    print("\n🎯 Press number keys for trajectories:")
    print("  1 - Hover at current position")
    print("  2 - Circle trajectory")
    print("  3 - Figure-8 trajectory")
    print("  4 - Square trajectory")
    print("\n⚠️  NOTE: Arrow keys disabled (too sensitive)")
    print("💡 Tip: Wait for drone to stabilize before starting trajectory!")
    print("="*70 + "\n")

    step_count = 0
    while robot.step(timestep) != -1:
        current_time = robot.getTime()
        dt = current_time - past_time

        if dt < 1e-6:
            continue

        gps_values = gps.getValues()
        current_pos = [gps_values[0], gps_values[1], gps_values[2]]
        roll, pitch, yaw = imu.getRollPitchYaw()
        yaw_rate = gyro.getValues()[2]

        x_global, y_global = gps_values[0], gps_values[1]
        vx_global = (x_global - past_x) / dt
        vy_global = (y_global - past_y) / dt
        cosyaw, sinyaw = cos(yaw), sin(yaw)
        vx_body = vx_global * cosyaw + vy_global * sinyaw
        vy_body = -vx_global * sinyaw + vy_global * cosyaw

        # Keyboard handling - CORRECT: Read all keys in queue
        key = keyboard.getKey()
        while key != -1:  # Loop until queue is empty (returns -1)
            # Check debounce
            if key not in last_key_time or (current_time - last_key_time[key]) > KEY_DEBOUNCE:
                last_key_time[key] = current_time

                # Print debug
                if 32 <= key <= 126:
                    print(f"\n🔍 Key: '{chr(key)}' (code: {key})")
                else:
                    print(f"\n🔍 Special key: {key}")

                # Handle trajectory commands
                if key == ord('1'):
                    trajectory.center = current_pos.copy()
                    trajectory.set_trajectory("hover", current_time)
                    controller.pid_z.reset()  # Reset Z PID on trajectory change
                    z_sp_smooth = current_pos[2]  # Reset Z setpoint to current position
                elif key == ord('2'):
                    trajectory.center = current_pos.copy()
                    trajectory.set_trajectory("circle", current_time)
                    controller.pid_z.reset()  # Reset Z PID on trajectory change
                    z_sp_smooth = current_pos[2]  # Reset Z setpoint to current position
                elif key == ord('3'):
                    trajectory.center = current_pos.copy()
                    trajectory.set_trajectory("figure8", current_time)
                    controller.pid_z.reset()  # Reset Z PID on trajectory change
                    z_sp_smooth = current_pos[2]  # Reset Z setpoint to current position
                elif key == ord('4'):
                    trajectory.center = current_pos.copy()
                    trajectory.set_trajectory("square", current_time)
                    controller.pid_z.reset()  # Reset Z PID on trajectory change
                    z_sp_smooth = current_pos[2]  # Reset Z setpoint to current position
                elif key == ord(' '):
                    target_pos = trajectory.get_target_position(current_time)
                    pos_error = np.linalg.norm([target_pos[i] - current_pos[i] for i in range(3)])
                    print(f"\n📊 Status at t={current_time:.1f}s:")
                    print(f"   Mode: {trajectory.trajectory_type}")
                    print(f"   Position: ({current_pos[0]:+.2f}, {current_pos[1]:+.2f}, {current_pos[2]:.2f})")
                    print(f"   Target:   ({target_pos[0]:+.2f}, {target_pos[1]:+.2f}, {target_pos[2]:.2f})")
                    print(f"   Error: {pos_error:.3f}m\n")
                else:
                    if 32 <= key <= 126:
                        print(f"   ⚠️  Key '{chr(key)}' not assigned")
                    else:
                        print(f"   ⚠️  Special key {key} not assigned")

            # CRITICAL: Read next key from queue
            key = keyboard.getKey()

        # Get target position
        target_position = trajectory.get_target_position(current_time)

        # Apply Z setpoint smoothing (2-second ramp to prevent jump)
        target_z_raw = target_position[2]
        z_sp_smooth += (target_z_raw - z_sp_smooth) * min(1.0, dt / z_sp_tau)
        target_position[2] = z_sp_smooth  # Replace Z with smoothed value

        # Compute control with smoothed target
        motor_commands = controller.compute_control(target_position, current_pos, [vx_body, vy_body],
                                                    [roll, pitch, yaw], yaw_rate, dt)

        motors[0].setVelocity(-motor_commands[0])
        motors[1].setVelocity(motor_commands[1])
        motors[2].setVelocity(-motor_commands[2])
        motors[3].setVelocity(motor_commands[3])

        # Print status every 0.5s
        if step_count % int(0.5 / (timestep / 1000.0)) == 0:
            pos_error = np.linalg.norm([target_position[i] - current_pos[i] for i in range(3)])
            z_error = target_position[2] - current_pos[2]

            # Only print when tracking trajectory (not hover at origin)
            if trajectory.trajectory_type != "hover" or pos_error > 0.05:
                print(f"[{current_time:6.2f}s] {trajectory.trajectory_type:8s} | "
                      f"Pos:[{current_pos[0]:+.2f},{current_pos[1]:+.2f},{current_pos[2]:.2f}] | "
                      f"Tgt:[{target_position[0]:+.2f},{target_position[1]:+.2f},{target_position[2]:.2f}] | "
                      f"Err:{pos_error:.3f}m")

        past_x = x_global
        past_y = y_global
        past_z = current_pos[2]
        past_time = current_time
        step_count += 1


if __name__ == "__main__":
    main()
