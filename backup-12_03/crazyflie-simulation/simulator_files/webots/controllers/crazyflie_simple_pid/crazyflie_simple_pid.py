"""
Simple PID Position Controller for Crazyflie in Webots

This controller uses a cascade PID architecture with higher-level control:
- Outer loop: Position error → Desired velocity
- Middle loop: Velocity error → Desired attitude (pitch/roll)
- Inner loop: Attitude error → Motor commands
- Altitude loop: Height error → Thrust command

Author: Claude Code & Hanzel
Date: 2025-11-03
"""

from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard
import numpy as np
from math import cos, sin


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


class CrazyfliePositionController:
    """Cascade PID controller for position control"""

    def __init__(self):
        # Position control (outer loop) - produces desired velocity
        self.pid_x = SimplePIDController(
            kp=0.5, ki=0.05, kd=0.3,  # More conservative gains
            output_limits=(-0.3, 0.3),  # Lower max velocity
            integral_limits=(-0.1, 0.1)
        )
        self.pid_y = SimplePIDController(
            kp=0.5, ki=0.05, kd=0.3,
            output_limits=(-0.3, 0.3),
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

        # Velocity control (middle loop) - produces desired attitude
        self.pid_vx = SimplePIDController(
            kp=2.0, ki=0.0, kd=0.5,
            output_limits=(-0.3, 0.3),  # Max pitch/roll angle (radians)
            integral_limits=(-0.1, 0.1)
        )
        self.pid_vy = SimplePIDController(
            kp=2.0, ki=0.0, kd=0.5,
            output_limits=(-0.3, 0.3),
            integral_limits=(-0.1, 0.1)
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
        desired_roll = -self.pid_vy.update(desired_vy - current_vel[1], dt)  # Negative for correct direction

        # Altitude loop: Height → Thrust (with D-on-measurement)
        altitude_error = target_pos[2] - current_pos[2]
        thrust_adjustment = self.pid_z.update(altitude_error, dt, meas=current_pos[2])
        base_thrust = 48.0  # Hover thrust (Bitcraze official, for thrustConstant=4e-05)
        thrust_command = base_thrust + thrust_adjustment

        # Inner loop: Attitude → Moments
        roll_command = self.pid_roll.update(desired_roll - current_attitude[0], dt)
        pitch_command = -self.pid_pitch.update(desired_pitch - current_attitude[1], dt)  # Negative for correct direction
        yaw_command = self.pid_yaw.update(target_yaw_rate - current_yaw_rate, dt)

        # Motor mixing (from Bitcraze implementation)
        # X configuration: M1 (FL), M2 (FR), M3 (BL), M4 (BR)
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
        motor.setVelocity(1.0 if i % 2 == 0 else -1.0)  # Alternating directions
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

    # Initialize controller
    controller = CrazyfliePositionController()

    # Target position
    target_position = [0.0, 0.0, 1.0]  # Hover at 1m height

    # State variables
    past_x = 0.0
    past_y = 0.0
    past_z = 0.0
    past_time = robot.getTime()

    # Key debouncing to prevent key repeat issues
    last_key_time = {}
    KEY_DEBOUNCE = 0.2  # 200ms debounce

    print("\n" + "="*60)
    print("Simple PID Position Controller for Crazyflie")
    print("="*60)
    print("\n🎮 Controls:")
    print("  Arrow Keys  - Move forward/backward/left/right (X/Y) [±0.2m]")
    print("  W/S         - Move up/down (Z) [±0.1m]")
    print("  A/D         - Alternative left/right (Y) [±0.2m]")
    print("  Q/E         - Rotate left/right (Yaw)")
    print("  R           - Reset to origin (0,0,1)")
    print("  SPACE       - Print current status")
    print("\n📍 Starting target: X=%.2f, Y=%.2f, Z=%.2f" % tuple(target_position))
    print("💡 Tip: Click on 3D window to ensure keyboard focus!")
    print("="*60 + "\n")

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

        # Handle keyboard input (CORRECT: Read all keys until -1)
        target_yaw_rate = 0.0
        key = keyboard.getKey()
        while key != -1:  # Loop until queue is empty
            # Debounce check - only process if enough time has passed
            if key not in last_key_time or (current_time - last_key_time[key]) > KEY_DEBOUNCE:
                last_key_time[key] = current_time

                if key == Keyboard.UP:
                    target_position[0] += 0.2
                    print(f"⬆️  Forward: Target X = {target_position[0]:.2f}m")
                elif key == Keyboard.DOWN:
                    target_position[0] -= 0.2
                    print(f"⬇️  Backward: Target X = {target_position[0]:.2f}m")
                elif key == Keyboard.RIGHT:
                    target_position[1] -= 0.2
                    print(f"➡️  Right: Target Y = {target_position[1]:.2f}m")
                elif key == Keyboard.LEFT:
                    target_position[1] += 0.2
                    print(f"⬅️  Left: Target Y = {target_position[1]:.2f}m")
                elif key == ord('W') or key == ord('w'):
                    target_position[2] += 0.1
                    print(f"🔼 Up: Target Z = {target_position[2]:.2f}m")
                elif key == ord('S') or key == ord('s'):
                    target_position[2] -= 0.1
                    target_position[2] = max(0.1, target_position[2])
                    print(f"🔽 Down: Target Z = {target_position[2]:.2f}m")
                elif key == ord('A') or key == ord('a'):
                    target_position[1] += 0.2
                    print(f"⬅️  Left (A): Target Y = {target_position[1]:.2f}m")
                elif key == ord('D') or key == ord('d'):
                    target_position[1] -= 0.2
                    print(f"➡️  Right (D): Target Y = {target_position[1]:.2f}m")
                elif key == ord('Q') or key == ord('q'):
                    target_yaw_rate = 1.0
                    print("↺ Yaw Left")
                elif key == ord('E') or key == ord('e'):
                    target_yaw_rate = -1.0
                    print("↻ Yaw Right")
                elif key == ord('R') or key == ord('r'):
                    target_position = [0.0, 0.0, 1.0]
                    print("🔄 Target reset to origin (0, 0, 1)")
                elif key == ord(' '):
                    pos_error = np.linalg.norm([target_position[i] - current_pos[i] for i in range(3)])
                    print(f"\n[t={current_time:.2f}s] Status:")
                    print(f"  Position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
                    print(f"  Target:   ({target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f})")
                    print(f"  Error:    {pos_error:.3f}m")
                    print(f"  Attitude: Roll={roll*57.3:.1f}°, Pitch={pitch*57.3:.1f}°, Yaw={yaw*57.3:.1f}°\n")

            key = keyboard.getKey()

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

        # Apply motor commands (M1 and M3 need negative velocity due to negative thrustConstants)
        motors[0].setVelocity(-motor_commands[0])  # M1
        motors[1].setVelocity(motor_commands[1])    # M2
        motors[2].setVelocity(-motor_commands[2])  # M3
        motors[3].setVelocity(motor_commands[3])    # M4

        # Print detailed status every 0.5 seconds
        if step_count % int(0.5 / (timestep / 1000.0)) == 0:
            pos_error = np.linalg.norm([target_position[i] - current_pos[i] for i in range(3)])
            z_error = target_position[2] - current_pos[2]
            print(f"[{current_time:6.2f}s] Z:{current_pos[2]:7.3f} (target:{target_position[2]:.1f}, err:{z_error:+6.3f}) | "
                  f"Motors:[{motor_commands[0]:.0f},{motor_commands[1]:.0f},{motor_commands[2]:.0f},{motor_commands[3]:.0f}] | "
                  f"Vel_Z:{(current_pos[2] - past_z) / dt if dt > 0 else 0:+6.2f}m/s")

        # Update state
        past_x = x_global
        past_y = y_global
        past_z = current_pos[2]
        past_time = current_time
        step_count += 1


if __name__ == "__main__":
    main()
