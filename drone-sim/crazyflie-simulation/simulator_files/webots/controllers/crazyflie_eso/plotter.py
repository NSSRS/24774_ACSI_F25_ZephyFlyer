"""
plotter.py
-----------
Simple logger for Webots Crazyflie ESO controller.

Usage:
    from plotter import DataLogger

    logger = DataLogger("crazyflie_log")
    logger.log(t, gps_pos, eso_p, eso_v, eso_att, imu_rpy, target, motor_power)
"""

import os
from datetime import datetime


class DataLogger:
    def __init__(self, prefix="crazyflie_log"):
        # Create logs directory
        self.log_dir = os.path.join(os.path.dirname(__file__), "logs")
        os.makedirs(self.log_dir, exist_ok=True)

        # Build filename: logs/crazyflie_log_20250126_153012.txt
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(self.log_dir, f"{prefix}_{timestamp}.txt")

        # Open file
        self.file = open(self.filename, "w")

        # Write header
        self.file.write(
            "# t    "
            "gps_x  gps_y  gps_z    "
            "eso_x  eso_y  eso_z    "
            "eso_vx eso_vy eso_vz   "
            "eso_roll eso_pitch eso_yaw   "
            "imu_roll imu_pitch imu_yaw   "
            "target_x target_y target_z   "
            "m1 m2 m3 m4\n"
        )

        print(f"[LOGGER] Logging to: {self.filename}")

    def log(self, t, gps_pos, eso_p, eso_v, eso_att, imu_rpy, target, motor_power):
        """Write one timestep of log data."""
        self.file.write(
            f"{t:.3f}\t"

            f"{gps_pos[0]:.4f}\t{gps_pos[1]:.4f}\t{gps_pos[2]:.4f}\t"

            f"{eso_p[0]:.4f}\t{eso_p[1]:.4f}\t{eso_p[2]:.4f}\t"
            f"{eso_v[0]:.4f}\t{eso_v[1]:.4f}\t{eso_v[2]:.4f}\t"
            f"{eso_att[0]:.4f}\t{eso_att[1]:.4f}\t{eso_att[2]:.4f}\t"

            f"{imu_rpy[0]:.4f}\t{imu_rpy[1]:.4f}\t{imu_rpy[2]:.4f}\t"

            f"{target[0]:.4f}\t{target[1]:.4f}\t{target[2]:.4f}\t"

            f"{motor_power[0]:.2f}\t{motor_power[1]:.2f}\t"
            f"{motor_power[2]:.2f}\t{motor_power[3]:.2f}\n"
        )

    def close(self):
        self.file.close()
        print(f"[LOGGER] File saved: {self.filename}")
