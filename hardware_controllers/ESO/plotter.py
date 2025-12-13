import os
from datetime import datetime

class DataLogger:
    def __init__(self, prefix="crazyflie_log"):
        self.log_dir = os.path.join(os.path.dirname(__file__), "logs")
        os.makedirs(self.log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(self.log_dir, f"{prefix}_{timestamp}.txt")

        self.file = open(self.filename, "w")

        # ======== 31 Columns ========
        self.file.write(
            "# t  "
            "gps_x gps_y gps_z  "
            "eso_x eso_y eso_z  "
            "eso_vx eso_vy eso_vz  "
            "eso_roll eso_pitch eso_yaw  "
            "dist_x dist_y dist_z  "
            "imu_roll imu_pitch imu_yaw  "
            "target_x target_y target_z  "
            "m1 m2 m3 m4  "
            "pid1 pid2 pid3 pid4  "
            "eso1 eso2 eso3 eso4\n"
        )

        print(f"[LOGGER] Logging to: {self.filename}")

    def log(self, t, gps_pos, eso_p, eso_v, eso_att, eso_d_f,
            imu_rpy, target, motor_power, pid_u, eso_u):

        self.file.write(
            f"{t:.3f}\t"

            f"{gps_pos[0]:.4f}\t{gps_pos[1]:.4f}\t{gps_pos[2]:.4f}\t"

            f"{eso_p[0]:.4f}\t{eso_p[1]:.4f}\t{eso_p[2]:.4f}\t"
            f"{eso_v[0]:.4f}\t{eso_v[1]:.4f}\t{eso_v[2]:.4f}\t"
            f"{eso_att[0]:.4f}\t{eso_att[1]:.4f}\t{eso_att[2]:.4f}\t"

            f"{eso_d_f[0]:.4f}\t{eso_d_f[1]:.4f}\t{eso_d_f[2]:.4f}\t"

            f"{imu_rpy[0]:.4f}\t{imu_rpy[1]:.4f}\t{imu_rpy[2]:.4f}\t"

            f"{target[0]:.4f}\t{target[1]:.4f}\t{target[2]:.4f}\t"

            f"{motor_power[0]:.1f}\t{motor_power[1]:.1f}\t{motor_power[2]:.1f}\t{motor_power[3]:.1f}\t"

            f"{pid_u[0]:.1f}\t{pid_u[1]:.1f}\t{pid_u[2]:.1f}\t{pid_u[3]:.1f}\t"

            f"{eso_u[0]:.2f}\t{eso_u[1]:.2f}\t{eso_u[2]:.2f}\t{eso_u[3]:.2f}\n"
        )

        self.file.flush()

    def close(self):
        self.file.close()
