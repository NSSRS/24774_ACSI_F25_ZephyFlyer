# -*- coding: utf-8 -*-
#
#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  MIT Licence
#
#  Copyright (C) 2023 Bitcraze AB
#

"""
file: pid_controller.py

A simple PID controller for the Crazyflie
ported from pid_controller.c in the c-based controller of the Crazyflie
in Webots
"""

import numpy as np


class pid_velocity_fixed_height_controller():
    def __init__(self):
        self.past_vx_error = 0.0
        self.past_vy_error = 0.0
        self.past_alt_error = 0.0
        self.past_pitch_error = 0.0
        self.past_roll_error = 0.0
        self.altitude_integrator = 0.0
        self.last_time = 0.0

    def pid(self, dt, desired_vx, desired_vy, desired_yaw_rate, desired_altitude, actual_roll, actual_pitch, actual_yaw_rate,
            actual_altitude, actual_vx, actual_vy):
        # Velocity PID control (converted from Crazyflie c code)
        gains = {"kp_att_y": 1, "kd_att_y": 0.5, "kp_att_rp": 0.5, "kd_att_rp": 0.1,
                 "kp_vel_xy": 2, "kd_vel_xy": 0.5, "kp_z": 10, "ki_z": 5, "kd_z": 5}

        # Velocity PID control
        vx_error = desired_vx - actual_vx
        vx_deriv = (vx_error - self.past_vx_error) / dt
        vy_error = desired_vy - actual_vy
        vy_deriv = (vy_error - self.past_vy_error) / dt
        desired_pitch = gains["kp_vel_xy"] * np.clip(vx_error, -1, 1) + gains["kd_vel_xy"] * vx_deriv
        desired_roll = -gains["kp_vel_xy"] * np.clip(vy_error, -1, 1) - gains["kd_vel_xy"] * vy_deriv
        self.past_vx_error = vx_error
        self.past_vy_error = vy_error

        # Altitude PID control
        alt_error = desired_altitude - actual_altitude
        alt_deriv = (alt_error - self.past_alt_error) / dt
        self.altitude_integrator += alt_error * dt
        alt_command = gains["kp_z"] * alt_error + gains["kd_z"] * alt_deriv + \
            gains["ki_z"] * np.clip(self.altitude_integrator, -2, 2) + 48
        self.past_alt_error = alt_error

        # Attitude PID control
        pitch_error = desired_pitch - actual_pitch
        pitch_deriv = (pitch_error - self.past_pitch_error) / dt
        roll_error = desired_roll - actual_roll
        roll_deriv = (roll_error - self.past_roll_error) / dt
        yaw_rate_error = desired_yaw_rate - actual_yaw_rate
        roll_command = gains["kp_att_rp"] * np.clip(roll_error, -1, 1) + gains["kd_att_rp"] * roll_deriv
        pitch_command = -gains["kp_att_rp"] * np.clip(pitch_error, -1, 1) - gains["kd_att_rp"] * pitch_deriv
        yaw_command = gains["kp_att_y"] * np.clip(yaw_rate_error, -1, 1)
        self.past_pitch_error = pitch_error
        self.past_roll_error = roll_error

        # Motor mixing
        m1 = alt_command - roll_command + pitch_command + yaw_command
        m2 = alt_command - roll_command - pitch_command - yaw_command
        m3 = alt_command + roll_command - pitch_command + yaw_command
        m4 = alt_command + roll_command + pitch_command - yaw_command

        # Limit the motor command
        m1 = np.clip(m1, 0, 600)
        m2 = np.clip(m2, 0, 600)
        m3 = np.clip(m3, 0, 600)
        m4 = np.clip(m4, 0, 600)

        return [m1, m2, m3, m4]


class pid_fixed_position_controller():
    """
    Fixed Position Controller with Cascade PID Structure

    This controller extends pid_velocity_fixed_height_controller by adding
    an outer position loop that converts position errors to velocity commands.

    Control Structure:
    Position Error → [Position PID] → Desired Velocity
                                    ↓
    Velocity Error → [Velocity PID] → Desired Attitude (pitch/roll)
                                    ↓
    Attitude Error → [Attitude PID] → Motor Commands

    Height Error   → [Height PID with Integral] → Thrust Command
    """

    def __init__(self):
        # Inner velocity controller state
        self.past_vx_error = 0.0
        self.past_vy_error = 0.0
        self.past_alt_error = 0.0
        self.past_pitch_error = 0.0
        self.past_roll_error = 0.0
        self.altitude_integrator = 0.0

        # Outer position controller state
        self.past_x_error = 0.0
        self.past_y_error = 0.0
        self.position_integrator_x = 0.0
        self.position_integrator_y = 0.0

        self.last_time = 0.0

    def pid(self, dt, desired_x, desired_y, desired_yaw_rate, desired_altitude,
            actual_x, actual_y, actual_yaw, actual_roll, actual_pitch, actual_yaw_rate,
            actual_altitude, actual_vx, actual_vy):
        """
        Fixed position PID controller

        Args:
            dt: Time step
            desired_x: Target X position in world frame
            desired_y: Target Y position in world frame
            desired_yaw_rate: Target yaw rate
            desired_altitude: Target altitude (Z position)
            actual_x: Current X position in world frame
            actual_y: Current Y position in world frame
            actual_yaw: Current yaw angle (NEEDED for coordinate transformation!)
            actual_roll: Current roll angle
            actual_pitch: Current pitch angle
            actual_yaw_rate: Current yaw rate
            actual_altitude: Current altitude
            actual_vx: Current X velocity in body frame
            actual_vy: Current Y velocity in body frame

        Returns:
            List of 4 motor commands [m1, m2, m3, m4]
        """

        # PID gains (same structure as velocity controller)
        gains = {
            # Position control gains (new outer loop) - VERY AGGRESSIVE TUNING
            "kp_pos_xy": 3.0,    # Position proportional gain (VERY HIGH - was 1.5, originally 0.6)
            "kd_pos_xy": 0.15,   # Position derivative gain (LOW damping for fast response)
            "ki_pos_xy": 0.2,    # Position integral gain (STRONG integral action)

            # Velocity control gains (existing)
            "kp_vel_xy": 2.0,
            "kd_vel_xy": 0.5,

            # Attitude control gains (existing)
            "kp_att_rp": 0.5,
            "kd_att_rp": 0.1,
            "kp_att_y": 1.0,
            "kd_att_y": 0.5,

            # Altitude control gains (existing)
            "kp_z": 10.0,
            "ki_z": 5.0,
            "kd_z": 5.0
        }

        # Maximum velocity command (m/s) - VERY HIGH for fast response
        max_velocity = 1.0  # Increased to 1.0 m/s (CrazyFlie can handle this!)

        # ============================================================
        # OUTER LOOP: Position PID Control (X, Y)
        # ============================================================
        # IMPORTANT: Must transform from world frame to body frame!

        # Step 1: Position errors in world frame
        x_error_world = desired_x - actual_x
        y_error_world = desired_y - actual_y

        # Step 2: Transform world frame errors to body frame using yaw
        # Rotation matrix: [cos(yaw)  sin(yaw)]
        #                  [-sin(yaw) cos(yaw)]
        from math import cos, sin
        cosyaw = cos(actual_yaw)
        sinyaw = sin(actual_yaw)

        x_error_body = x_error_world * cosyaw + y_error_world * sinyaw
        y_error_body = -x_error_world * sinyaw + y_error_world * cosyaw

        # Step 3: Position derivative (in body frame)
        x_deriv = (x_error_body - self.past_x_error) / dt
        y_deriv = (y_error_body - self.past_y_error) / dt

        # Step 4: Position integral (in body frame with anti-windup)
        self.position_integrator_x += x_error_body * dt
        self.position_integrator_y += y_error_body * dt

        # Clamp integrators to prevent windup
        self.position_integrator_x = np.clip(self.position_integrator_x, -2.0, 2.0)
        self.position_integrator_y = np.clip(self.position_integrator_y, -2.0, 2.0)

        # Step 5: PD+I control: position error → desired velocity (body frame)
        # Note: Using velocity damping (Kd * actual_velocity) instead of derivative of error
        desired_vx = (gains["kp_pos_xy"] * x_error_body -
                     gains["kd_pos_xy"] * actual_vx +
                     gains["ki_pos_xy"] * self.position_integrator_x)

        desired_vy = (gains["kp_pos_xy"] * y_error_body -
                     gains["kd_pos_xy"] * actual_vy +
                     gains["ki_pos_xy"] * self.position_integrator_y)

        # Step 6: Saturate velocity commands
        desired_vx = np.clip(desired_vx, -max_velocity, max_velocity)
        desired_vy = np.clip(desired_vy, -max_velocity, max_velocity)

        # Step 7: Anti-windup - if saturated and pushing further, undo integral
        if abs(desired_vx) >= max_velocity and (desired_vx * x_error_body) > 0.0:
            self.position_integrator_x -= x_error_body * dt
        if abs(desired_vy) >= max_velocity and (desired_vy * y_error_body) > 0.0:
            self.position_integrator_y -= y_error_body * dt

        # Update past errors (body frame)
        self.past_x_error = x_error_body
        self.past_y_error = y_error_body

        # ============================================================
        # INNER LOOPS: Same as pid_velocity_fixed_height_controller
        # ============================================================

        # Velocity PID control (same as original)
        vx_error = desired_vx - actual_vx
        vx_deriv = (vx_error - self.past_vx_error) / dt
        vy_error = desired_vy - actual_vy
        vy_deriv = (vy_error - self.past_vy_error) / dt
        desired_pitch = gains["kp_vel_xy"] * np.clip(vx_error, -1, 1) + gains["kd_vel_xy"] * vx_deriv
        desired_roll = -gains["kp_vel_xy"] * np.clip(vy_error, -1, 1) - gains["kd_vel_xy"] * vy_deriv
        self.past_vx_error = vx_error
        self.past_vy_error = vy_error

        # Altitude PID control (same as original)
        alt_error = desired_altitude - actual_altitude
        alt_deriv = (alt_error - self.past_alt_error) / dt
        self.altitude_integrator += alt_error * dt
        alt_command = gains["kp_z"] * alt_error + gains["kd_z"] * alt_deriv + \
            gains["ki_z"] * np.clip(self.altitude_integrator, -2, 2) + 48
        self.past_alt_error = alt_error

        # Attitude PID control (same as original)
        pitch_error = desired_pitch - actual_pitch
        pitch_deriv = (pitch_error - self.past_pitch_error) / dt
        roll_error = desired_roll - actual_roll
        roll_deriv = (roll_error - self.past_roll_error) / dt
        yaw_rate_error = desired_yaw_rate - actual_yaw_rate
        roll_command = gains["kp_att_rp"] * np.clip(roll_error, -1, 1) + gains["kd_att_rp"] * roll_deriv
        pitch_command = -gains["kp_att_rp"] * np.clip(pitch_error, -1, 1) - gains["kd_att_rp"] * pitch_deriv
        yaw_command = gains["kp_att_y"] * np.clip(yaw_rate_error, -1, 1)
        self.past_pitch_error = pitch_error
        self.past_roll_error = roll_error

        # Motor mixing (same as original)
        m1 = alt_command - roll_command + pitch_command + yaw_command
        m2 = alt_command - roll_command - pitch_command - yaw_command
        m3 = alt_command + roll_command - pitch_command + yaw_command
        m4 = alt_command + roll_command + pitch_command - yaw_command

        # Limit the motor command
        m1 = np.clip(m1, 0, 600)
        m2 = np.clip(m2, 0, 600)
        m3 = np.clip(m3, 0, 600)
        m4 = np.clip(m4, 0, 600)

        return [m1, m2, m3, m4]