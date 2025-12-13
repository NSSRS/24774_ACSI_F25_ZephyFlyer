import os
from datetime import datetime
import numpy as np # 需要导入 numpy 来处理 pos_error

class DataLogger:
    def __init__(self, prefix="crazyflie_log"):
        # 1. 路径和文件名: 确保 .csv 扩展名，并保存到当前脚本目录
        current_dir = os.path.dirname(os.path.abspath(__file__))
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(current_dir, f".csv")
        self.file = open(self.filename, "w")

        # 2. 标头 FIX: 严格对齐 plot_lqr_trajectory.py 的要求，并使用逗号分隔
        self.header = (
            "t,"
            "mode,"              # 绘图脚本必须有
            "px,py,pz,"          # 绘图脚本要求: 实际位置 (GPS)
            "ref_px,ref_py,ref_pz," # 绘图脚本要求: 参考位置 (Target)
            "pos_error,"         # 绘图脚本必须有 (计算值)
            "yaw,"               # 绘图脚本要求: 实际偏航角 (IMU Yaw)
            
            # ESO 特定数据 (保留用于详细分析)
            "eso_x,eso_y,eso_z,"
            "eso_vx,eso_vy,eso_vz,"
            "eso_roll,eso_pitch,eso_yaw,"
            "dist_x,dist_y,dist_z,"
            "imu_roll,imu_pitch," # IMU Yaw 已重命名为 'yaw'
            
            "m1,m2,m3,m4,"
            "pid1,pid2,pid3,pid4,"
            "eso1,eso2,eso3,eso4\n"
        )
        self.file.write(self.header)

        print(f"[LOGGER] Logging to: {self.filename}")


    # 3. log 函数 FIX: 签名和数据写入对齐
    def log(self, t, mode, pos_error, gps_pos, eso_p, eso_v, eso_att, eso_d_f,
             imu_rpy, target, motor_power, pid_u, eso_u):
        
        # IMU Yaw (rpy[2]) 对应绘图脚本的 'yaw'
        imu_yaw = imu_rpy[2] 
        imu_roll = imu_rpy[0]
        imu_pitch = imu_rpy[1]

        self.file.write(
            f"{t:.3f},"
            f"{mode},"
            
            # 绘图脚本所需的核心数据
            f"{gps_pos[0]:.4f},{gps_pos[1]:.4f},{gps_pos[2]:.4f}," # px, py, pz
            f"{target[0]:.4f},{target[1]:.4f},{target[2]:.4f}," # ref_px, ref_py, ref_pz
            f"{pos_error:.4f},"
            f"{imu_yaw:.4f}," # yaw
            
            # ESO 特定数据
            f"{eso_p[0]:.4f},{eso_p[1]:.4f},{eso_p[2]:.4f},"
            f"{eso_v[0]:.4f},{eso_v[1]:.4f},{eso_v[2]:.4f},"
            f"{eso_att[0]:.4f},{eso_att[1]:.4f},{eso_att[2]:.4f},"
            f"{eso_d_f[0]:.4f},{eso_d_f[1]:.4f},{eso_d_f[2]:.4f},"
            f"{imu_roll:.4f},{imu_pitch:.4f}," # imu_roll, imu_pitch
            f"{motor_power[0]:.1f},{motor_power[1]:.1f},{motor_power[2]:.1f},{motor_power[3]:.1f},"
            f"{pid_u[0]:.1f},{pid_u[1]:.1f},{pid_u[2]:.1f},{pid_u[3]:.1f},"
            f"{eso_u[0]:.2f},{eso_u[1]:.2f},{eso_u[2]:.2f},{eso_u[3]:.2f}\n"
        )

        self.file.flush()

    def close(self):
        self.file.close()