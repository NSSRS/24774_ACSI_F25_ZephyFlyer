"""
Fixed Position Controller with Disturbance Testing

This controller uses the new pid_fixed_position_controller to maintain
a fixed XY position and altitude, even under external disturbances.

Based on Official Bitcraze PID Controller + Supervisor for force application

Author: Based on Bitcraze official code
Date: 2025-11-06
"""

from controller import Supervisor
from controller import Motor, InertialUnit, GPS, Gyro, Keyboard
from math import cos, sin
import sys
sys.path.append('../../../../controllers_shared/python_based')
from pid_controller import pid_fixed_position_controller


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

    # Initialize variables
    past_x_global = 0
    past_y_global = 0
    past_time = robot.getTime()

    # Use new fixed position PID controller
    PID_CF = pid_fixed_position_controller()

    # Target position (fixed in world frame)
    target_x = 0.0
    target_y = 0.0
    target_z = 1.0  # Default hover height

    # Force disturbance settings
    force_magnitude = 0.5  # Default: 0.5N (impulse force)
    last_force_time = -10.0  # Track last force application time

    # Continuous wind mode
    wind_enabled = False
    wind_magnitude = 0.01  # Default: 0.01N (much smaller for continuous force!)
    wind_force = [0.0, 0.0, 0.0]  # Continuous wind force [Fx, Fy, Fz]

    # Keyboard debouncing
    last_key_time = {}  # Track last time each key was pressed
    KEY_DEBOUNCE = 0.3  # 300ms debounce for sensitive keys

    # Status printing
    last_print_time = 0.0
    print_interval = 2.0  # Print status every 2 seconds

    print("\n" + "="*70)
    print("🚁 CrazyFlie Fixed Position Controller (XYZ Position Hold)")
    print("="*70)
    print("\n✨ NEW: pid_fixed_position_controller")
    print("  Position Control: Kp=0.6, Kd=0.3, Ki=0.05")
    print("  Max velocity: 0.3 m/s")
    print("  Features: Anti-windup, integral control for constant disturbances")
    print("\n🎯 TARGET POSITION Controls:")
    print("  Arrow Keys: Adjust target X/Y (±0.5m)")
    print("  W/S: Adjust target altitude (±0.2m)")
    print("  R: Reset target to origin (0, 0, 1.0)")
    print("  SPACE: Print current status")
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

    print(f"🎯 Target Position: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
    print("   Drone will try to maintain this position under disturbances!\n")

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

        # Get body-fixed velocities
        cosyaw = cos(yaw)
        sinyaw = sin(yaw)
        v_x = v_x_global * cosyaw + v_y_global * sinyaw
        v_y = -v_x_global * sinyaw + v_y_global * cosyaw

        # Keyboard input with debouncing
        key = keyboard.getKey()
        while key > 0:
            # Check if this key should be debounced
            key_needs_debounce = key in [
                ord('V'), ord('v'), ord('R'), ord('r'),
                ord('['), ord(']'), ord(','), ord('<'), ord('.'), ord('>'),
                ord('7'), ord('8'), ord('9'), ord('0'),
                ord('-'), ord('_'), ord('='), ord('+'),
                ord('C'), ord('c'), Keyboard.SPACE
            ]

            # Debounce check
            if key_needs_debounce:
                if key in last_key_time and (current_time - last_key_time[key]) < KEY_DEBOUNCE:
                    key = keyboard.getKey()
                    continue  # Skip this key, too soon
                last_key_time[key] = current_time

            # Print status
            if key == Keyboard.SPACE:
                pos_error = ((target_x - x_global)**2 + (target_y - y_global)**2 + (target_z - altitude)**2)**0.5
                print(f"\n📊 Status (t={current_time:.2f}s):")
                print(f"  Target:   ({target_x:+.3f}, {target_y:+.3f}, {target_z:+.3f})")
                print(f"  Current:  ({x_global:+.3f}, {y_global:+.3f}, {altitude:+.3f})")
                print(f"  Error:    {pos_error:.4f} m")
                print(f"  Velocity: ({v_x_global:+.3f}, {v_y_global:+.3f}) m/s")
                if wind_enabled:
                    print(f"  Wind:     [{wind_force[0]:+.3f}, {wind_force[1]:+.3f}, {wind_force[2]:+.3f}] N")
                print()

            # Reset target to origin
            elif key == ord('R') or key == ord('r'):
                target_x = 0.0
                target_y = 0.0
                target_z = 1.0
                print(f"🎯 Target RESET: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")

            # Adjust target position
            elif key == Keyboard.UP:
                target_x += 0.5
                print(f"🎯 Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == Keyboard.DOWN:
                target_x -= 0.5
                print(f"🎯 Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == Keyboard.LEFT:
                target_y += 0.5
                print(f"🎯 Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == Keyboard.RIGHT:
                target_y -= 0.5
                print(f"🎯 Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == ord('W'):
                target_z += 0.2
                print(f"🎯 Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            elif key == ord('S'):
                target_z = max(0.3, target_z - 0.2)  # Prevent going below 0.3m
                print(f"🎯 Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")

            # Force magnitude adjustment (impulse)
            elif key == ord('['):
                force_magnitude = max(0.5, force_magnitude - 0.5)
                print(f"💥 Impulse force: {force_magnitude:.1f} N")
            elif key == ord(']'):
                force_magnitude = min(50.0, force_magnitude + 0.5)
                print(f"💥 Impulse force: {force_magnitude:.1f} N")

            # Wind magnitude adjustment (continuous)
            elif key == ord(',') or key == ord('<'):
                wind_magnitude = max(0.005, wind_magnitude - 0.005)
                print(f"🌬️  Wind force: {wind_magnitude:.3f} N")
            elif key == ord('.') or key == ord('>'):
                wind_magnitude = min(0.5, wind_magnitude + 0.005)
                print(f"🌬️  Wind force: {wind_magnitude:.3f} N")

            # Wind mode toggle
            elif key == ord('V') or key == ord('v'):
                wind_enabled = not wind_enabled
                status = "ENABLED" if wind_enabled else "DISABLED"
                print(f"\n🌬️  Wind mode: {status}")
                if wind_enabled and (wind_force[0] != 0 or wind_force[1] != 0 or wind_force[2] != 0):
                    print(f"   Wind force: [{wind_force[0]:+.3f}, {wind_force[1]:+.3f}, {wind_force[2]:+.3f}] N")

            # Set continuous wind direction
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

            # Apply external forces (with cooldown)
            elif current_time - last_force_time > 0.5:  # 0.5s cooldown
                force_applied = False

                if key == ord('F') or key == ord('f'):
                    crazyflie_node.addForce([force_magnitude, 0, 0], False)
                    print(f"💥 Applied force: +X {force_magnitude:.3f}N (forward)")
                    force_applied = True
                elif key == ord('G') or key == ord('g'):
                    crazyflie_node.addForce([0, force_magnitude, 0], False)
                    print(f"💥 Applied force: +Y {force_magnitude:.3f}N (left)")
                    force_applied = True
                elif key == ord('H') or key == ord('h'):
                    crazyflie_node.addForce([-force_magnitude, 0, 0], False)
                    print(f"💥 Applied force: -X {force_magnitude:.3f}N (backward)")
                    force_applied = True
                elif key == ord('J') or key == ord('j'):
                    crazyflie_node.addForce([0, -force_magnitude, 0], False)
                    print(f"💥 Applied force: -Y {force_magnitude:.3f}N (right)")
                    force_applied = True
                elif key == ord('U') or key == ord('u'):
                    crazyflie_node.addForce([0, 0, force_magnitude], False)
                    print(f"💥 Applied force: +Z {force_magnitude:.3f}N (upward)")
                    force_applied = True
                elif key == ord('D') or key == ord('d'):
                    crazyflie_node.addForce([0, 0, -force_magnitude], False)
                    print(f"💥 Applied force: -Z {force_magnitude:.3f}N (downward)")
                    force_applied = True

                if force_applied:
                    last_force_time = current_time

            key = keyboard.getKey()

        # Call the fixed position controller
        motor_power = PID_CF.pid(
            dt,
            target_x, target_y,           # Desired X, Y position
            0.0,                           # Desired yaw rate (keep current heading)
            target_z,                      # Desired altitude
            x_global, y_global,            # Actual X, Y position
            yaw,                           # Actual yaw (NEEDED for coordinate transform!)
            roll, pitch, yaw_rate,         # Actual attitude
            altitude,                      # Actual altitude
            v_x, v_y                       # Actual body velocity
        )

        # Apply motor commands (use official sign convention)
        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        # Apply continuous wind force (if enabled)
        if wind_enabled:
            crazyflie_node.addForce(wind_force, False)

        # Periodic status printing
        if current_time - last_print_time >= print_interval:
            pos_error = ((target_x - x_global)**2 + (target_y - y_global)**2 + (target_z - altitude)**2)**0.5
            print(f"[{current_time:6.1f}s] Pos:({x_global:+.3f},{y_global:+.3f},{altitude:+.3f}) | "
                  f"Target:({target_x:+.2f},{target_y:+.2f},{target_z:+.2f}) | Err:{pos_error:.4f}m")
            last_print_time = current_time

        # Update state
        past_time = current_time
        past_x_global = x_global
        past_y_global = y_global
