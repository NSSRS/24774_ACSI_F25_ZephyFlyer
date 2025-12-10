"""
Trajectory Controller - Based on Official Bitcraze PID Controller
Extends official crazyflie_controller_py with trajectory tracking

Author: Based on Bitcraze official code
Date: 2025-11-05
"""

from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard
from math import cos, sin, pi
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

        elif traj_type == "square":
            self.target_position = current_pos.copy()
            print(f"\n✅ Trajectory: SQUARE (0.5m side)")

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

        elif self.trajectory_type == "square":
            # Square path: 0.5m side, 5s per side
            side_length = 0.5
            side_duration = 5.0
            cycle_time = t % (4 * side_duration)

            if cycle_time < side_duration:  # Side 1: +X
                progress = cycle_time / side_duration
                x = self.start_position[0] + progress * side_length
                y = self.start_position[1]
            elif cycle_time < 2 * side_duration:  # Side 2: +Y
                progress = (cycle_time - side_duration) / side_duration
                x = self.start_position[0] + side_length
                y = self.start_position[1] + progress * side_length
            elif cycle_time < 3 * side_duration:  # Side 3: -X
                progress = (cycle_time - 2 * side_duration) / side_duration
                x = self.start_position[0] + side_length - progress * side_length
                y = self.start_position[1] + side_length
            else:  # Side 4: -Y
                progress = (cycle_time - 3 * side_duration) / side_duration
                x = self.start_position[0]
                y = self.start_position[1] + side_length - progress * side_length

            return [x, y, self.start_position[2]]

        return self.target_position.copy()


if __name__ == '__main__':
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

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

    print("\n" + "="*70)
    print("🚁 CrazyFlie Trajectory Controller (Official PID)")
    print("="*70)
    print("\n📋 MANUAL MODE Controls:")
    print("  Arrow Keys: Move horizontally (body frame)")
    print("  W/S: Altitude up/down")
    print("  Q/E: Yaw rotation")
    print("\n🎯 TRAJECTORY MODE:")
    print("  1: Hover at current position")
    print("  2: Line trajectory +X (1.0m forward)")
    print("  3: Line trajectory +Y (1.0m left)")
    print("  4: Circle trajectory (0.3m radius)")
    print("  5: Square trajectory (0.5m side)")
    print("  M: Toggle Manual/Auto mode")
    print("="*70 + "\n")

    # Wait for sensors
    for _ in range(5):
        robot.step(timestep)

    # Main loop
    while robot.step(timestep) != -1:
        dt = robot.getTime() - past_time

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

        # Keyboard input
        key = keyboard.getKey()
        while key > 0:
            # Mode toggle
            if key == ord('M') or key == ord('m'):
                auto_mode = not auto_mode
                mode_str = "TRAJECTORY AUTO" if auto_mode else "MANUAL"
                print(f"\n🔄 Mode: {mode_str}")

            # Trajectory selection (only in auto mode)
            elif key == ord('1'):
                trajectory.set_trajectory("hover", robot.getTime(), current_pos)
                auto_mode = True
            elif key == ord('2'):
                trajectory.set_trajectory("line_x", robot.getTime(), current_pos)
                auto_mode = True
            elif key == ord('3'):
                trajectory.set_trajectory("line_y", robot.getTime(), current_pos)
                auto_mode = True
            elif key == ord('4'):
                trajectory.set_trajectory("circle", robot.getTime(), current_pos)
                auto_mode = True
            elif key == ord('5'):
                trajectory.set_trajectory("square", robot.getTime(), current_pos)
                auto_mode = True

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
            target_pos = trajectory.get_target_position(robot.getTime())

            # Simple position controller: P-gain on position error -> velocity
            pos_error_x = target_pos[0] - x_global
            pos_error_y = target_pos[1] - y_global

            # Global to body frame
            pos_error_x_body = pos_error_x * cosyaw + pos_error_y * sinyaw
            pos_error_y_body = -pos_error_x * sinyaw + pos_error_y * cosyaw

            # Position P-gain (conservative)
            Kp_pos = 0.5
            forward_desired = Kp_pos * pos_error_x_body
            sideways_desired = Kp_pos * pos_error_y_body

            # Limit velocity commands
            forward_desired = max(-0.3, min(0.3, forward_desired))
            sideways_desired = max(-0.3, min(0.3, sideways_desired))

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

        # Update state
        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global
