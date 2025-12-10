#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/

# MIT License
# Copyright (c) 2022 Bitcraze

"""crazyflie_controller_py controller (with debug output)."""

from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard, Camera, DistanceSensor
from math import cos, sin
import sys, numpy as np
sys.path.append('../../../../controllers_shared/python_based')
from pid_controller import pid_velocity_fixed_height_controller

FLYING_ATTITUDE = 1

if __name__ == '__main__':

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # === Initialize motors ===
    m1_motor = robot.getDevice("m1_motor"); m1_motor.setPosition(float('inf')); m1_motor.setVelocity(-1)
    m2_motor = robot.getDevice("m2_motor"); m2_motor.setPosition(float('inf')); m2_motor.setVelocity(1)
    m3_motor = robot.getDevice("m3_motor"); m3_motor.setPosition(float('inf')); m3_motor.setVelocity(-1)
    m4_motor = robot.getDevice("m4_motor"); m4_motor.setPosition(float('inf')); m4_motor.setVelocity(1)

    # === Initialize sensors ===
    imu = robot.getDevice("inertial_unit"); imu.enable(timestep)
    gps = robot.getDevice("gps"); gps.enable(timestep)
    gyro = robot.getDevice("gyro"); gyro.enable(timestep)
    camera = robot.getDevice("camera"); camera.enable(timestep)
    range_front = robot.getDevice("range_front"); range_front.enable(timestep)
    range_left  = robot.getDevice("range_left");  range_left.enable(timestep)
    range_back  = robot.getDevice("range_back");  range_back.enable(timestep)
    range_right = robot.getDevice("range_right"); range_right.enable(timestep)

    keyboard = Keyboard(); keyboard.enable(timestep)

    # === Variables ===
    past_x_global = 0.0
    past_y_global = 0.0
    past_time = robot.getTime()
    PID_CF = pid_velocity_fixed_height_controller()
    height_desired = FLYING_ATTITUDE

    print("\n====== Crazyflie Keyboard Controls ======\n")
    print("- Arrow keys : Move horizontally")
    print("- Q / E      : Rotate yaw")
    print("- W / S      : Change altitude\n")

    # === Main loop ===
    while robot.step(timestep) != -1:
        t = robot.getTime()
        dt = max(t - past_time, 1e-3)
        past_time = t

        # --- Sensor readings ---
        roll, pitch, yaw = imu.getRollPitchYaw()
        yaw_rate = gyro.getValues()[2]
        altitude = gps.getValues()[2]
        x_global = gps.getValues()[0]
        y_global = gps.getValues()[1]
        v_x_global = (x_global - past_x_global) / dt
        v_y_global = (y_global - past_y_global) / dt
        past_x_global, past_y_global = x_global, y_global

        # --- Convert to body frame velocities ---
        cosyaw, sinyaw = cos(yaw), sin(yaw)
        v_x =  v_x_global * cosyaw + v_y_global * sinyaw
        v_y = -v_x_global * sinyaw + v_y_global * cosyaw

        # --- Keyboard input ---
        forward_desired = sideways_desired = yaw_desired = 0.0
        height_diff_desired = 0.0
        key = keyboard.getKey()
        while key > 0:
            if key == Keyboard.UP: forward_desired += 0.5
            elif key == Keyboard.DOWN: forward_desired -= 0.5
            elif key == Keyboard.RIGHT: sideways_desired -= 0.5
            elif key == Keyboard.LEFT: sideways_desired += 0.5
            elif key == ord('Q'): yaw_desired = +1
            elif key == ord('E'): yaw_desired = -1
            elif key == ord('W'): height_diff_desired = 0.1
            elif key == ord('S'): height_diff_desired = -0.1
            key = keyboard.getKey()

        height_desired += height_diff_desired * dt

        # --- PID velocity controller ---
        motor_power = PID_CF.pid(
            dt,
            forward_desired, sideways_desired,
            yaw_desired, height_desired,
            roll, pitch, yaw_rate,
            altitude, v_x, v_y
        )

        # --- Apply motor speeds ---
        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity( motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity( motor_power[3])

        # --- Debug output every 0.25 s ---
        if int(t * 4) % 1 == 0:   # 4 Hz print rate
            print(
                f"[t={t:5.2f}s] "
                f"alt={altitude:+.2f}m  "
                f"vel_b=({v_x:+.2f},{v_y:+.2f})m/s  "
                f"cmd_f/s=({forward_desired:+.2f},{sideways_desired:+.2f})  "
                f"motors=({motor_power[0]:+.2f},{motor_power[1]:+.2f},"
                f"{motor_power[2]:+.2f},{motor_power[3]:+.2f})"
            )
