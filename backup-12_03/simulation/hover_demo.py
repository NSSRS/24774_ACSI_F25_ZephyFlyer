#!/usr/bin/env python3
"""
Simple MuJoCo hover demonstration for CrazyFlie 2.1 drone.

This script demonstrates basic hovering control by applying a constant
thrust force to keep the drone stable in the air.
"""

import os
import mujoco
import mujoco.viewer
import numpy as np


def main():
    """
    Main function to run the hover demonstration.

    Loads the CrazyFlie simulation scene and applies hover control
    to keep the drone stable at approximately 0.1m altitude.
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

    # Get hover control values from the predefined keyframe
    # The keyframe named "hover" contains the control values for stable hovering
    hover_keyframe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "hover")
    if hover_keyframe_id >= 0:
        hover_ctrl = model.key_ctrl[hover_keyframe_id]
        print(f"\nUsing hover keyframe controls:")
        print(f"  Body thrust: {hover_ctrl[0]:.5f} N")
        print(f"  X moment: {hover_ctrl[1]:.5f}")
        print(f"  Y moment: {hover_ctrl[2]:.5f}")
        print(f"  Z moment: {hover_ctrl[3]:.5f}")
    else:
        # Fallback to manual hover thrust calculation if keyframe not found
        # Hover thrust = mass * gravity
        hover_thrust = model.body_mass[1] * 9.81
        hover_ctrl = np.array([hover_thrust, 0.0, 0.0, 0.0])
        print(f"\nKeyframe not found, using calculated hover thrust: {hover_thrust:.5f} N")

    # Set initial state from hover keyframe
    if hover_keyframe_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, hover_keyframe_id)

    # Control callback function that runs at each simulation step
    def controller(model, data):
        """
        Simple hover controller that applies constant thrust.

        Args:
            model: MuJoCo model
            data: MuJoCo data structure
        """
        # Apply hover control values to actuators
        # Index 0: body_thrust (vertical thrust)
        # Index 1: x_moment (roll control)
        # Index 2: y_moment (pitch control)
        # Index 3: z_moment (yaw control)
        data.ctrl[:] = hover_ctrl

    print("\n=== Starting Simulation ===")
    print("The drone should hover at approximately 0.1m altitude.")
    print("Close the viewer window to exit.\n")

    # Launch the interactive viewer with the controller callback
    # The viewer will call the controller function at each timestep
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Run simulation loop
        while viewer.is_running():
            # Advance simulation by one step
            step_start = data.time

            # Apply control
            controller(model, data)

            # Step the physics simulation
            mujoco.mj_step(model, data)

            # Sync the viewer with the simulation state
            viewer.sync()

            # Optional: Print drone state every 100 steps (1 second)
            if int(data.time * 100) % 100 == 0:
                position = data.qpos[0:3]  # x, y, z position
                print(f"Time: {data.time:6.2f}s | Position (x,y,z): [{position[0]:6.3f}, {position[1]:6.3f}, {position[2]:6.3f}]")


if __name__ == "__main__":
    main()
