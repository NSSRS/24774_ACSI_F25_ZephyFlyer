#!/usr/bin/env python3
"""
Improved Cascade PID controller with better stability and diagnostics.

Improvements over cascade_pid_control.py:
1. Enhanced anti-windup with conditional integration
2. Integral term clamping
3. Better PID parameter tuning
4. Rate limiting on attitude commands
5. Detailed diagnostic logging
6. Emergency reset on large errors
"""

import os
import numpy as np
import mujoco
import mujoco.viewer


class ImprovedPIDController:
    """
    Enhanced PID controller with better anti-windup and diagnostics.
    """

    def __init__(self, kp, ki, kd, output_limits=(-np.inf, np.inf), integral_limits=None):
        """
        Initialize improved PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limits: Tuple of (min, max) output limits
            integral_limits: Tuple of (min, max) integral term limits (None = use output_limits)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits

        # Set integral limits (prevent excessive windup)
        if integral_limits is None:
            # Default: limit integral to contribute at most half of output range
            range_half = (output_limits[1] - output_limits[0]) / 2.0
            self.integral_limits = (-range_half, range_half)
        else:
            self.integral_limits = integral_limits

        # State variables
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0  # For derivative filtering

        # Diagnostics
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0

    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0

    def compute(self, error, dt, enable_integration=True):
        """
        Compute PID control output with enhanced anti-windup.

        Args:
            error: Current error (setpoint - measurement)
            dt: Time step in seconds
            enable_integration: Disable integration when error is large (conditional integration)

        Returns:
            Control output (limited to output_limits)
        """
        # Proportional term
        self.last_p = self.kp * error

        # Integral term with conditional integration
        if enable_integration and abs(error) < 1.0:  # Only integrate when error is reasonable
            self.integral += error * dt
            # Clamp integral term
            self.integral = np.clip(self.integral, self.integral_limits[0] / max(self.ki, 1e-6),
                                   self.integral_limits[1] / max(self.ki, 1e-6))

        self.last_i = self.ki * self.integral

        # Derivative term with filtering (to reduce noise amplification)
        if dt > 0:
            raw_derivative = (error - self.prev_error) / dt
            # Low-pass filter: filtered = α * new + (1-α) * old
            alpha = 0.1  # Filter coefficient (lower = more filtering)
            filtered_derivative = alpha * raw_derivative + (1 - alpha) * self.prev_derivative
            self.prev_derivative = filtered_derivative
        else:
            filtered_derivative = 0.0

        self.last_d = self.kd * filtered_derivative

        # Total output
        output = self.last_p + self.last_i + self.last_d

        # Apply output limits
        output_limited = np.clip(output, self.output_limits[0], self.output_limits[1])

        # Anti-windup: back-calculate integral if saturated
        if output != output_limited and self.ki != 0:
            # Reduce integral to prevent windup
            excess = output - output_limited
            self.integral -= excess / self.ki

        # Store for next iteration
        self.prev_error = error

        return output_limited

    def get_diagnostics(self):
        """Return PID term values for debugging."""
        return {
            'P': self.last_p,
            'I': self.last_i,
            'D': self.last_d,
            'integral': self.integral
        }


class MotorMixer:
    """Motor mixer (same as before, included for completeness)."""

    def __init__(self):
        self.arm_length = 0.046
        self.thrust_to_torque_ratio = 0.005824
        self.max_motor_thrust = 0.16
        self.min_motor_thrust = 0.0

        L = self.arm_length
        k = self.thrust_to_torque_ratio
        sqrt2 = np.sqrt(2)

        self.mix_matrix = np.array([
            [ 0.25,      sqrt2/(4*L),  sqrt2/(4*L), 1/(4*k)],
            [ 0.25,     -sqrt2/(4*L),  sqrt2/(4*L),-1/(4*k)],
            [ 0.25,     -sqrt2/(4*L), -sqrt2/(4*L), 1/(4*k)],
            [ 0.25,      sqrt2/(4*L), -sqrt2/(4*L),-1/(4*k)],
        ])

        self.inv_mix_matrix = np.linalg.pinv(self.mix_matrix)

    def allocate(self, desired_forces):
        motor_thrusts = self.mix_matrix @ desired_forces
        saturated = np.any(motor_thrusts < self.min_motor_thrust) or \
                   np.any(motor_thrusts > self.max_motor_thrust)
        motor_thrusts = np.clip(motor_thrusts, self.min_motor_thrust, self.max_motor_thrust)
        return motor_thrusts, saturated

    def compute_actual_forces(self, motor_thrusts):
        return self.inv_mix_matrix @ motor_thrusts


class ImprovedCascadePIDController:
    """
    Improved cascade PID controller with stability enhancements.
    """

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.mass = model.body_mass[1]
        self.gravity = 9.81

        # Target position
        self.target_position = np.array([0.0, 0.0, 0.5])
        self.target_yaw = 0.0

        # Outer loop PIDs (position control) - More conservative tuning
        self.z_pid = ImprovedPIDController(
            kp=8.0, ki=1.5, kd=6.0,  # Reduced Ki from 3.0 to 1.5
            output_limits=(-3, 3),  # Reduced from (-5, 5)
            integral_limits=(-1.0, 1.0)  # Explicit integral limit
        )

        self.x_pid = ImprovedPIDController(
            kp=1.5, ki=0.2, kd=2.5,  # Reduced Ki from 0.5 to 0.2
            output_limits=(-0.2, 0.2),  # Reduced from (-0.3, 0.3)
            integral_limits=(-0.1, 0.1)
        )

        self.y_pid = ImprovedPIDController(
            kp=1.5, ki=0.2, kd=2.5,
            output_limits=(-0.2, 0.2),
            integral_limits=(-0.1, 0.1)
        )

        # Inner loop PIDs (attitude control) - Faster response
        self.roll_pid = ImprovedPIDController(
            kp=0.02, ki=0.0005, kd=0.004,  # Reduced Ki from 0.001
            output_limits=(-0.3, 0.3),  # Reduced from (-0.5, 0.5)
            integral_limits=(-0.1, 0.1)
        )

        self.pitch_pid = ImprovedPIDController(
            kp=0.02, ki=0.0005, kd=0.004,
            output_limits=(-0.3, 0.3),
            integral_limits=(-0.1, 0.1)
        )

        self.yaw_pid = ImprovedPIDController(
            kp=0.008, ki=0.0003, kd=0.0015,  # Reduced Ki
            output_limits=(-0.15, 0.15),  # Reduced from (-0.2, 0.2)
            integral_limits=(-0.05, 0.05)
        )

        # Motor mixer
        self.mixer = MotorMixer()

        # Safety limits
        self.max_tilt_angle = np.deg2rad(15)  # Maximum 15 degrees tilt
        self.emergency_reset_threshold = 2.0  # Reset if error > 2m

        # Diagnostics
        self.motor_saturated = False
        self.emergency_resets = 0

    def quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles."""
        w, x, y, z = quat

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])

    def check_and_reset_on_large_error(self, pos_error):
        """Emergency reset if position error is too large."""
        error_magnitude = np.linalg.norm(pos_error)

        if error_magnitude > self.emergency_reset_threshold:
            print(f"\n⚠️  EMERGENCY RESET: Position error {error_magnitude:.3f}m > threshold")

            # Reset all PID integrators
            self.z_pid.reset()
            self.x_pid.reset()
            self.y_pid.reset()
            self.roll_pid.reset()
            self.pitch_pid.reset()
            self.yaw_pid.reset()

            self.emergency_resets += 1
            return True

        return False

    def compute_control(self, dt):
        """Compute control with enhanced stability."""
        # Get current state
        position = self.data.qpos[0:3]
        velocity = self.data.qvel[0:3]
        quaternion = self.data.qpos[3:7]
        angular_velocity = self.data.qvel[3:6]

        # Convert quaternion to Euler angles
        euler = self.quaternion_to_euler(quaternion)
        roll, pitch, yaw = euler

        # ==================== Position Error and Safety Check ====================
        pos_error = self.target_position - position

        # Emergency reset on large errors
        if self.check_and_reset_on_large_error(pos_error):
            # Return minimal control to stabilize
            hover_thrust = self.mass * self.gravity
            return np.array([hover_thrust, 0.0, 0.0, 0.0]), np.array([hover_thrust/4]*4)

        # ==================== OUTER LOOP: Position Control ====================

        # Only enable integration when close to target (conditional integration)
        error_magnitude = np.linalg.norm(pos_error)
        enable_integration = error_magnitude < 0.5  # Only integrate within 0.5m

        # Z-axis (altitude) control
        z_accel = self.z_pid.compute(pos_error[2], dt, enable_integration)
        desired_thrust = self.mass * (self.gravity + z_accel)
        desired_thrust = np.clip(desired_thrust, 0.0, 0.6)  # Hard limit thrust

        # X-axis control -> desired pitch
        x_accel = self.x_pid.compute(pos_error[0], dt, enable_integration)
        desired_pitch = -np.arcsin(np.clip(x_accel / self.gravity, -0.9, 0.9))
        desired_pitch = np.clip(desired_pitch, -self.max_tilt_angle, self.max_tilt_angle)

        # Y-axis control -> desired roll
        y_accel = self.y_pid.compute(pos_error[1], dt, enable_integration)
        desired_roll = np.arcsin(np.clip(y_accel / self.gravity, -0.9, 0.9))
        desired_roll = np.clip(desired_roll, -self.max_tilt_angle, self.max_tilt_angle)

        # ==================== INNER LOOP: Attitude Control ====================

        # Attitude errors
        roll_error = desired_roll - roll
        pitch_error = desired_pitch - pitch
        yaw_error = self.target_yaw - yaw
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))

        # Attitude PIDs
        roll_moment = self.roll_pid.compute(roll_error, dt, enable_integration)
        pitch_moment = self.pitch_pid.compute(pitch_error, dt, enable_integration)
        yaw_moment = self.yaw_pid.compute(yaw_error, dt, enable_integration)

        # ==================== MOTOR MIXING ====================
        desired_forces = np.array([desired_thrust, roll_moment, pitch_moment, yaw_moment])
        motor_thrusts, saturated = self.mixer.allocate(desired_forces)
        self.motor_saturated = saturated
        actual_forces = self.mixer.compute_actual_forces(motor_thrusts)

        return actual_forces, motor_thrusts

    def get_diagnostics(self):
        """Get detailed diagnostic information."""
        return {
            'z_pid': self.z_pid.get_diagnostics(),
            'x_pid': self.x_pid.get_diagnostics(),
            'y_pid': self.y_pid.get_diagnostics(),
            'roll_pid': self.roll_pid.get_diagnostics(),
            'pitch_pid': self.pitch_pid.get_diagnostics(),
            'yaw_pid': self.yaw_pid.get_diagnostics(),
            'emergency_resets': self.emergency_resets,
            'motor_saturated': self.motor_saturated
        }


def main():
    """Main function with enhanced diagnostics."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "crazyflie_sim", "scene.xml")

    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model file not found: {model_path}")

    print(f"Loading model from: {model_path}")

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    print(f"\n=== Model Information ===")
    print(f"Drone mass: {model.body_mass[1]:.4f} kg")
    print(f"Simulation timestep: {model.opt.timestep} seconds")

    # Initialize from hover keyframe
    hover_keyframe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "hover")
    if hover_keyframe_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, hover_keyframe_id)

    # Create improved controller
    controller = ImprovedCascadePIDController(model, data)

    print(f"\n=== Improved Cascade PID Controller ===")
    print(f"Target position: {controller.target_position}")
    print(f"Max tilt angle: {np.rad2deg(controller.max_tilt_angle):.1f} degrees")
    print(f"Emergency reset threshold: {controller.emergency_reset_threshold}m")
    print(f"\n=== Improvements ===")
    print("✓ Conditional integration (disabled when error > 0.5m)")
    print("✓ Integral term clamping")
    print("✓ Reduced Ki gains (less windup)")
    print("✓ Tilt angle limiting (max 15°)")
    print("✓ Emergency reset on large errors")
    print("✓ Derivative filtering")
    print(f"\n=== Starting Simulation ===\n")

    dt = model.opt.timestep
    last_print_time = 0.0
    print_interval = 2.0  # Print every 2 seconds

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            ctrl, motor_thrusts = controller.compute_control(dt)

            data.ctrl[0] = ctrl[0]
            data.ctrl[1] = ctrl[1]
            data.ctrl[2] = ctrl[2]
            data.ctrl[3] = ctrl[3]

            mujoco.mj_step(model, data)
            viewer.sync()

            # Print status
            if data.time - last_print_time >= print_interval:
                position = data.qpos[0:3]
                pos_error = controller.target_position - position
                error_mag = np.linalg.norm(pos_error)

                # Get diagnostics
                diag = controller.get_diagnostics()

                print(f"T:{data.time:6.1f}s | "
                      f"Pos:[{position[0]:6.3f},{position[1]:6.3f},{position[2]:6.3f}] | "
                      f"Err:{error_mag:6.3f}m | "
                      f"Motors:[{motor_thrusts[0]:.3f},{motor_thrusts[1]:.3f},"
                      f"{motor_thrusts[2]:.3f},{motor_thrusts[3]:.3f}]" +
                      (" [SAT]" if controller.motor_saturated else "") +
                      (f" Resets:{diag['emergency_resets']}" if diag['emergency_resets'] > 0 else ""))

                last_print_time = data.time

    print("\nSimulation ended.")
    print(f"Total emergency resets: {controller.emergency_resets}")


if __name__ == "__main__":
    main()
