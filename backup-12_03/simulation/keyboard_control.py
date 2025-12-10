#!/usr/bin/env python3
"""
Keyboard-controlled MuJoCo simulation for CrazyFlie 2.1 drone.

This script allows manual control of the drone using keyboard inputs:
- Arrow keys for horizontal movement (pitch/roll)
- W/S keys for altitude control (thrust)
- A/D keys for yaw rotation
- Space to reset to hover position
- ESC to exit

Controls are applied incrementally on top of the base hover thrust.
"""

import os
import mujoco
import mujoco.viewer
import numpy as np


class KeyboardController:
    """
    Keyboard controller for CrazyFlie drone simulation.

    Handles keyboard input and translates it to motor commands.
    """

    def __init__(self, model, data):
        """
        Initialize the keyboard controller.

        Args:
            model: MuJoCo model
            data: MuJoCo data structure
        """
        self.model = model
        self.data = data

        # Get hover control values from keyframe
        hover_keyframe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "hover")
        if hover_keyframe_id >= 0:
            self.hover_ctrl = model.key_ctrl[hover_keyframe_id].copy()
        else:
            # Fallback: calculate hover thrust
            hover_thrust = model.body_mass[1] * 9.81
            self.hover_ctrl = np.array([hover_thrust, 0.0, 0.0, 0.0])

        # Control increments for keyboard input
        self.thrust_increment = 0.02      # Vertical thrust adjustment (increased for visibility)
        self.moment_increment = 0.001     # Roll/pitch/yaw moment adjustment (10x increase)

        # Current control deltas (added to hover control)
        self.thrust_delta = 0.0
        self.roll_delta = 0.0    # X-axis moment (roll, controlled by left/right arrows)
        self.pitch_delta = 0.0   # Y-axis moment (pitch, controlled by up/down arrows)
        self.yaw_delta = 0.0     # Z-axis moment (yaw, controlled by A/D keys)

        # Keyboard state tracking
        self.key_pressed = set()

        print("\n=== Keyboard Controls ===")
        print("Arrow Keys:")
        print("  ↑/↓  : Pitch forward/backward")
        print("  ←/→  : Roll left/right")
        print("\nAltitude Control:")
        print("  W    : Increase thrust (climb)")
        print("  S    : Decrease thrust (descend)")
        print("\nYaw Control:")
        print("  A    : Rotate left (yaw)")
        print("  D    : Rotate right (yaw)")
        print("\nOther:")
        print("  SPACE: Reset to hover")
        print("  ESC  : Exit simulation")
        print("========================\n")

    def reset_to_hover(self):
        """Reset all control deltas to zero (return to hover state)."""
        self.thrust_delta = 0.0
        self.roll_delta = 0.0
        self.pitch_delta = 0.0
        self.yaw_delta = 0.0
        print("Reset to hover position")

    def process_keyboard(self, key_code, is_pressed):
        """
        Process keyboard input events.

        Args:
            key_code: Integer key code from MuJoCo viewer
            is_pressed: Boolean indicating if key is pressed (True) or released (False)
        """
        # Track key state
        if is_pressed:
            self.key_pressed.add(key_code)
        else:
            self.key_pressed.discard(key_code)

    def update_control(self):
        """
        Update control values based on currently pressed keys.
        This is called every simulation step.
        """
        # Thrust control (W/S keys)
        if ord('W') in self.key_pressed or ord('w') in self.key_pressed:
            self.thrust_delta += self.thrust_increment * 0.1  # Climb
        if ord('S') in self.key_pressed or ord('s') in self.key_pressed:
            self.thrust_delta -= self.thrust_increment * 0.1  # Descend

        # Roll control (Left/Right arrow keys)
        # Left arrow = 263, Right arrow = 262 in GLFW
        if 263 in self.key_pressed:  # Left arrow
            self.roll_delta += self.moment_increment
        if 262 in self.key_pressed:  # Right arrow
            self.roll_delta -= self.moment_increment

        # Pitch control (Up/Down arrow keys)
        # Up arrow = 265, Down arrow = 264 in GLFW
        if 265 in self.key_pressed:  # Up arrow
            self.pitch_delta += self.moment_increment
        if 264 in self.key_pressed:  # Down arrow
            self.pitch_delta -= self.moment_increment

        # Yaw control (A/D keys)
        if ord('A') in self.key_pressed or ord('a') in self.key_pressed:
            self.yaw_delta += self.moment_increment  # Rotate left
        if ord('D') in self.key_pressed or ord('d') in self.key_pressed:
            self.yaw_delta -= self.moment_increment  # Rotate right

        # Reset on Space key
        if ord(' ') in self.key_pressed:
            self.reset_to_hover()
            self.key_pressed.discard(ord(' '))  # Only reset once per press

        # Clamp thrust delta to reasonable limits
        max_thrust_delta = 0.1
        self.thrust_delta = np.clip(self.thrust_delta, -max_thrust_delta, max_thrust_delta)

        # Clamp moment deltas to prevent excessive rotation
        max_moment_delta = 0.5  # Increased from 0.01 for more visible control
        self.roll_delta = np.clip(self.roll_delta, -max_moment_delta, max_moment_delta)
        self.pitch_delta = np.clip(self.pitch_delta, -max_moment_delta, max_moment_delta)
        self.yaw_delta = np.clip(self.yaw_delta, -max_moment_delta, max_moment_delta)

        # Apply damping to gradually reduce control deltas when keys are released
        damping_factor = 0.95  # Slightly faster damping for more responsive feel
        if 263 not in self.key_pressed and 262 not in self.key_pressed:
            self.roll_delta *= damping_factor
        if 265 not in self.key_pressed and 264 not in self.key_pressed:
            self.pitch_delta *= damping_factor
        if ord('a') not in self.key_pressed and ord('A') not in self.key_pressed and \
           ord('d') not in self.key_pressed and ord('D') not in self.key_pressed:
            self.yaw_delta *= damping_factor
        if ord('w') not in self.key_pressed and ord('W') not in self.key_pressed and \
           ord('s') not in self.key_pressed and ord('S') not in self.key_pressed:
            self.thrust_delta *= damping_factor

        # Compute final control values
        self.data.ctrl[0] = self.hover_ctrl[0] + self.thrust_delta  # Body thrust
        self.data.ctrl[1] = self.hover_ctrl[1] + self.roll_delta    # X moment (roll)
        self.data.ctrl[2] = self.hover_ctrl[2] + self.pitch_delta   # Y moment (pitch)
        self.data.ctrl[3] = self.hover_ctrl[3] + self.yaw_delta     # Z moment (yaw)


def main():
    """
    Main function to run the keyboard-controlled simulation.
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
    print(f"Actuator names: {[model.actuator(i).name for i in range(model.nu)]}")
    print(f"Simulation timestep: {model.opt.timestep} seconds")

    # Initialize from hover keyframe
    hover_keyframe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "hover")
    if hover_keyframe_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, hover_keyframe_id)
        print(f"Initialized from hover keyframe")

    # Create keyboard controller
    controller = KeyboardController(model, data)

    print("\n=== Starting Simulation ===")
    print("Use keyboard controls to fly the drone.")
    print("Close the viewer window or press ESC to exit.\n")

    # Launch the interactive viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Set up keyboard callback
        # Note: MuJoCo viewer doesn't provide direct keyboard callback API
        # We'll need to check for key presses in the main loop

        # Variables for periodic status printing
        last_print_time = 0.0
        print_interval = 1.0  # Print status every 1 second

        # Run simulation loop
        while viewer.is_running():
            # Update control based on keyboard state
            controller.update_control()

            # Step the physics simulation
            mujoco.mj_step(model, data)

            # Sync the viewer with the simulation state
            viewer.sync()

            # Print drone state periodically
            if data.time - last_print_time >= print_interval:
                position = data.qpos[0:3]  # x, y, z position
                velocity = data.qvel[0:3]  # linear velocity
                print(f"Time: {data.time:6.2f}s | "
                      f"Pos: [{position[0]:6.3f}, {position[1]:6.3f}, {position[2]:6.3f}] | "
                      f"Vel: [{velocity[0]:6.3f}, {velocity[1]:6.3f}, {velocity[2]:6.3f}] | "
                      f"Ctrl: T={controller.thrust_delta:6.3f} "
                      f"R={controller.roll_delta:6.3f} "
                      f"P={controller.pitch_delta:6.3f} "
                      f"Y={controller.yaw_delta:6.3f}")
                last_print_time = data.time

    print("\nSimulation ended.")


if __name__ == "__main__":
    main()
