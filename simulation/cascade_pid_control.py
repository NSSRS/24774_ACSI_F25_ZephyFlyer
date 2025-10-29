#!/usr/bin/env python3
"""
Cascade PID controller for CrazyFlie 2.1 drone with motor mixing.

This script implements a two-layer cascade PID control architecture:
1. Outer loop: Position control (x, y, z) -> desired attitude and thrust
2. Inner loop: Attitude control (roll, pitch, yaw) -> control moments

The control outputs are mapped to 4 individual motor thrusts through a
mixing matrix, including motor saturation handling.
"""

import os
import numpy as np
import mujoco
import mujoco.viewer


class PIDController:
    """
    Generic PID controller with anti-windup.

    Implements proportional-integral-derivative control with output limits
    and integral anti-windup protection.
    """

    def __init__(self, kp, ki, kd, output_limits=(-np.inf, np.inf)):
        """
        Initialize PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limits: Tuple of (min, max) output limits
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits

        # State variables
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, error, dt):
        """
        Compute PID control output.

        Args:
            error: Current error (setpoint - measurement)
            dt: Time step in seconds

        Returns:
            Control output (limited to output_limits)
        """
        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        i_term = self.ki * self.integral

        # Derivative term
        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative

        # Total output
        output = p_term + i_term + d_term

        # Apply output limits
        output_limited = np.clip(output, self.output_limits[0], self.output_limits[1])

        # Anti-windup: back-calculate integral if saturated
        if output != output_limited and self.ki != 0:
            # Reduce integral to prevent windup
            self.integral -= (output - output_limited) / self.ki * dt

        # Store for next iteration
        self.prev_error = error

        return output_limited


class MotorMixer:
    """
    Motor mixer for quadcopter with X configuration.

    Maps desired forces/moments to individual motor thrusts and handles
    motor saturation by re-computing actual forces/moments.
    """

    def __init__(self):
        """Initialize motor mixer with CrazyFlie 2.1 parameters."""
        # Physical parameters (from CrazyFlie 2.1 specifications)
        self.arm_length = 0.046  # meters (motor arm length)
        self.thrust_to_torque_ratio = 0.005824  # Nm per Newton of thrust
        self.max_motor_thrust = 0.16  # N per motor (max thrust)
        self.min_motor_thrust = 0.0   # N per motor (min thrust)

        # Create mixing matrix
        # Maps [total_thrust, roll_moment, pitch_moment, yaw_moment] -> [m1, m2, m3, m4]
        # Motor layout (X configuration):
        #     Front
        #   M3  M1     M1, M4: CW rotation
        #     \/       M2, M3: CCW rotation
        #     /\
        #   M2  M4
        #     Back

        L = self.arm_length
        k = self.thrust_to_torque_ratio
        sqrt2 = np.sqrt(2)

        # Mixing matrix derivation:
        # Total thrust: T = T1 + T2 + T3 + T4
        # Roll moment:  τ_φ = (T1 + T4 - T2 - T3) * L/√2
        # Pitch moment: τ_θ = (T1 + T2 - T3 - T4) * L/√2
        # Yaw moment:   τ_ψ = k * (T1 + T3 - T2 - T4)

        #                Thrust   Roll         Pitch        Yaw
        self.mix_matrix = np.array([
            [ 0.25,      sqrt2/(4*L),  sqrt2/(4*L), 1/(4*k)],  # Motor 1 (front-right)
            [ 0.25,     -sqrt2/(4*L),  sqrt2/(4*L),-1/(4*k)],  # Motor 2 (back-left)
            [ 0.25,     -sqrt2/(4*L), -sqrt2/(4*L), 1/(4*k)],  # Motor 3 (front-left)
            [ 0.25,      sqrt2/(4*L), -sqrt2/(4*L),-1/(4*k)],  # Motor 4 (back-right)
        ])

        # Inverse mixing matrix (pseudo-inverse for redundant system)
        self.inv_mix_matrix = np.linalg.pinv(self.mix_matrix)

    def allocate(self, desired_forces):
        """
        Allocate desired forces/moments to individual motors.

        Args:
            desired_forces: Array [thrust, roll_moment, pitch_moment, yaw_moment]

        Returns:
            motor_thrusts: Array [m1, m2, m3, m4] of individual motor thrusts
            saturated: Boolean indicating if any motor was saturated
        """
        # Compute motor thrusts
        motor_thrusts = self.mix_matrix @ desired_forces

        # Check for saturation
        saturated = np.any(motor_thrusts < self.min_motor_thrust) or \
                   np.any(motor_thrusts > self.max_motor_thrust)

        # Apply motor limits
        motor_thrusts = np.clip(motor_thrusts, self.min_motor_thrust, self.max_motor_thrust)

        return motor_thrusts, saturated

    def compute_actual_forces(self, motor_thrusts):
        """
        Compute actual forces/moments from saturated motor thrusts.

        Args:
            motor_thrusts: Array [m1, m2, m3, m4] of motor thrusts

        Returns:
            actual_forces: Array [thrust, roll_moment, pitch_moment, yaw_moment]
        """
        actual_forces = self.inv_mix_matrix @ motor_thrusts
        return actual_forces


class CascadePIDController:
    """
    Cascade PID controller for quadcopter position and attitude control.

    Outer loop: Position control
    Inner loop: Attitude control
    """

    def __init__(self, model, data):
        """
        Initialize cascade controller.

        Args:
            model: MuJoCo model
            data: MuJoCo data structure
        """
        self.model = model
        self.data = data
        self.mass = model.body_mass[1]  # Drone mass in kg
        self.gravity = 9.81  # m/s^2

        # Target position (can be modified)
        self.target_position = np.array([0.0, 0.0, 0.5])  # x, y, z in meters
        self.target_yaw = 0.0  # radians

        # Outer loop PIDs (position control)
        # These output desired accelerations
        self.z_pid = PIDController(kp=10.0, ki=3.0, kd=5.0, output_limits=(-5, 5))
        self.x_pid = PIDController(kp=2.0, ki=0.5, kd=2.0, output_limits=(-0.3, 0.3))
        self.y_pid = PIDController(kp=2.0, ki=0.5, kd=2.0, output_limits=(-0.3, 0.3))

        # Inner loop PIDs (attitude control)
        # These output desired angular accelerations (moments)
        self.roll_pid = PIDController(kp=0.015, ki=0.001, kd=0.003, output_limits=(-0.5, 0.5))
        self.pitch_pid = PIDController(kp=0.015, ki=0.001, kd=0.003, output_limits=(-0.5, 0.5))
        self.yaw_pid = PIDController(kp=0.01, ki=0.001, kd=0.002, output_limits=(-0.2, 0.2))

        # Motor mixer
        self.mixer = MotorMixer()

        # Logging
        self.motor_saturated = False

    def quaternion_to_euler(self, quat):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).

        Args:
            quat: Quaternion [w, x, y, z]

        Returns:
            euler: Euler angles [roll, pitch, yaw] in radians
        """
        w, x, y, z = quat

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])

    def compute_control(self, dt):
        """
        Compute control outputs using cascade PID.

        Args:
            dt: Time step in seconds

        Returns:
            ctrl: Control array [thrust, roll_moment, pitch_moment, yaw_moment]
            motor_thrusts: Individual motor thrusts [m1, m2, m3, m4]
        """
        # Get current state
        position = self.data.qpos[0:3]  # [x, y, z]
        velocity = self.data.qvel[0:3]  # [vx, vy, vz]
        quaternion = self.data.qpos[3:7]  # [qw, qx, qy, qz]
        angular_velocity = self.data.qvel[3:6]  # [wx, wy, wz]

        # Convert quaternion to Euler angles
        euler = self.quaternion_to_euler(quaternion)
        roll, pitch, yaw = euler

        # ==================== OUTER LOOP: Position Control ====================

        # Position errors
        pos_error = self.target_position - position

        # Z-axis (altitude) control -> desired thrust
        z_accel = self.z_pid.compute(pos_error[2], dt)
        # Total thrust = compensation for gravity + desired acceleration
        desired_thrust = self.mass * (self.gravity + z_accel)

        # X-axis control -> desired pitch (negative because forward is negative pitch)
        x_accel = self.x_pid.compute(pos_error[0], dt)
        desired_pitch = -np.arcsin(np.clip(x_accel / self.gravity, -0.9, 0.9))

        # Y-axis control -> desired roll
        y_accel = self.y_pid.compute(pos_error[1], dt)
        desired_roll = np.arcsin(np.clip(y_accel / self.gravity, -0.9, 0.9))

        # ==================== INNER LOOP: Attitude Control ====================

        # Attitude errors
        roll_error = desired_roll - roll
        pitch_error = desired_pitch - pitch
        yaw_error = self.target_yaw - yaw

        # Normalize yaw error to [-pi, pi]
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))

        # Attitude PIDs -> desired moments
        roll_moment = self.roll_pid.compute(roll_error, dt)
        pitch_moment = self.pitch_pid.compute(pitch_error, dt)
        yaw_moment = self.yaw_pid.compute(yaw_error, dt)

        # ==================== MOTOR MIXING ====================

        # Pack desired forces/moments
        desired_forces = np.array([desired_thrust, roll_moment, pitch_moment, yaw_moment])

        # Allocate to motors
        motor_thrusts, saturated = self.mixer.allocate(desired_forces)
        self.motor_saturated = saturated

        # Compute actual forces after saturation
        actual_forces = self.mixer.compute_actual_forces(motor_thrusts)

        return actual_forces, motor_thrusts


def main():
    """
    Main function to run cascade PID control simulation.
    """
    # Get the path to the simulation model
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "crazyflie_sim", "scene.xml")

    # Check if model file exists
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model file not found: {model_path}")

    print(f"Loading model from: {model_path}")

    # Load the MuJoCo model and create data structure
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # Print model information
    print(f"\n=== Model Information ===")
    print(f"Drone mass: {model.body_mass[1]:.4f} kg")
    print(f"Number of actuators: {model.nu}")
    print(f"Simulation timestep: {model.opt.timestep} seconds")

    # Initialize from hover keyframe
    hover_keyframe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "hover")
    if hover_keyframe_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, hover_keyframe_id)
        print(f"Initialized from hover keyframe")

    # Create cascade PID controller
    controller = CascadePIDController(model, data)

    print(f"\n=== Cascade PID Controller ===")
    print(f"Target position: {controller.target_position}")
    print(f"Target yaw: {controller.target_yaw:.2f} rad")
    print(f"\n=== Starting Simulation ===")
    print("The drone will fly to the target position using cascade PID control.")
    print("Close the viewer window to exit.\n")

    # Simulation parameters
    dt = model.opt.timestep
    last_print_time = 0.0
    print_interval = 1.0  # Print status every 1 second

    # Launch the interactive viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Run simulation loop
        while viewer.is_running():
            # Compute control
            ctrl, motor_thrusts = controller.compute_control(dt)

            # Apply control to MuJoCo
            data.ctrl[0] = ctrl[0]  # Total thrust
            data.ctrl[1] = ctrl[1]  # Roll moment
            data.ctrl[2] = ctrl[2]  # Pitch moment
            data.ctrl[3] = ctrl[3]  # Yaw moment

            # Step the physics simulation
            mujoco.mj_step(model, data)

            # Sync the viewer with the simulation state
            viewer.sync()

            # Print status periodically
            if data.time - last_print_time >= print_interval:
                position = data.qpos[0:3]
                velocity = data.qvel[0:3]
                pos_error = controller.target_position - position

                print(f"Time: {data.time:6.2f}s | "
                      f"Pos: [{position[0]:6.3f}, {position[1]:6.3f}, {position[2]:6.3f}] | "
                      f"Err: [{pos_error[0]:6.3f}, {pos_error[1]:6.3f}, {pos_error[2]:6.3f}] | "
                      f"Motors: [{motor_thrusts[0]:5.3f}, {motor_thrusts[1]:5.3f}, "
                      f"{motor_thrusts[2]:5.3f}, {motor_thrusts[3]:5.3f}]" +
                      (" [SAT]" if controller.motor_saturated else ""))
                last_print_time = data.time

    print("\nSimulation ended.")


if __name__ == "__main__":
    main()
