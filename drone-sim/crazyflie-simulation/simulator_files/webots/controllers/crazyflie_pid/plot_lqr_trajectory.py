#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LQR Trajectory Plotter
简单的LQR日志可视化工具

用法:
    python plot_lqr_trajectory.py <log_file.csv> [--wind_start <time>] [--wind_end <time>]
    
注意: 如果未指定 --wind_start 和 --wind_end，脚本将默认假设风力从 5.0s 开始，到 20.0s 结束。
    
示例:
    # 使用默认风力时间 (5.0s 到 20.0s)
    python plot_lqr_trajectory.py crazyflie_hover_log.csv 
    
    # 覆盖默认时间 (假设风力从 10.0 秒开始，到 30.0 秒结束)
    python plot_lqr_trajectory.py crazyflie_hover_log.csv --wind_start 10.0 --wind_end 30.0
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import argparse 

def plot_trajectory(csv_file, wind_start=None, wind_end=None):
    """
    绘制LQR轨迹对比图
    
    Args:
        csv_file: CSV日志文件路径
        wind_start: 风力开始的时间 (s)
        wind_end: 风力结束的时间 (s)
    """
    # 读取数据
    df = pd.read_csv(csv_file)
    
    # 打印摘要
    print("="*60)
    print("Quick Log Summary")
    print("="*60)
    print(f"File: {os.path.basename(csv_file)}")
    print(f"Duration: {df['t'].iloc[-1] - df['t'].iloc[0]:.1f}s")
    print(f"Samples: {len(df)}")
    
    for mode in df['mode'].unique():
        mode_data = df[df['mode'] == mode]
        duration = mode_data['t'].iloc[-1] - mode_data['t'].iloc[0] if len(mode_data) > 1 else 0
        print(f"{mode}: {len(mode_data)} samples ({duration:.1f}s)")
    
    print(f"\nPosition error: mean={df['pos_error'].mean():.3f}m, max={df['pos_error'].max():.3f}m")
    
    # 计算yaw漂移
    yaw_drift = (df['yaw'].iloc[-1] - df['yaw'].iloc[0]) * 180 / np.pi
    print(f"Yaw drift: {yaw_drift:.2f}°")
    print("="*60)
    
    # 创建图形
    fig = plt.figure(figsize=(14, 8))
    
    # ===============================
    # 1. 3D Trajectory (with equal aspect ratio)
    # ===============================
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.plot(df['px'], df['py'], df['pz'], 'b-', linewidth=2.5, label='Actual', alpha=0.8)
    ax1.plot(df['ref_px'], df['ref_py'], df['ref_pz'], 'r--', linewidth=2, label='Reference', alpha=0.7)
    ax1.scatter(df['px'].iloc[0], df['py'].iloc[0], df['pz'].iloc[0], 
               c='lime', s=100, marker='o', label='Start', zorder=5)
    ax1.scatter(df['px'].iloc[-1], df['py'].iloc[-1], df['pz'].iloc[-1], 
               c='red', s=100, marker='X', label='End', zorder=5)
    
    # Force equal aspect ratio to avoid visual distortion
    max_range = np.array([
        df['px'].max()-df['px'].min(),
        df['py'].max()-df['py'].min(),
        df['pz'].max()-df['pz'].min()
    ]).max() / 2.0
    
    mid_x = (df['px'].max() + df['px'].min()) * 0.5
    mid_y = (df['py'].max() + df['py'].min()) * 0.5
    mid_z = (df['pz'].max() + df['pz'].min()) * 0.5
    
    ax1.set_xlim(mid_x - max_range, mid_x + max_range)
    ax1.set_ylim(mid_y - max_range, mid_y + max_range)
    ax1.set_zlim(mid_z - max_range, mid_z + max_range)
    
    ax1.set_xlabel('X (m)', fontsize=10)
    ax1.set_ylabel('Y (m)', fontsize=10)
    ax1.set_zlabel('Z (m)', fontsize=10)
    ax1.set_title('3D Trajectory', fontsize=12, fontweight='bold')
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)
    
    # ===============================
    # 2. XY Top View
    # ===============================
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(df['px'], df['py'], 'b-', linewidth=2.5, label='Actual', alpha=0.8)
    ax2.plot(df['ref_px'], df['ref_py'], 'r--', linewidth=2, label='Reference', alpha=0.7)
    ax2.scatter(df['px'].iloc[0], df['py'].iloc[0], 
               c='lime', s=150, marker='o', edgecolors='black', linewidths=2, zorder=5, label='Start')
    ax2.scatter(df['px'].iloc[-1], df['py'].iloc[-1], 
               c='red', s=150, marker='X', edgecolors='black', linewidths=2, zorder=5, label='End')
    ax2.set_xlabel('X (m)', fontsize=10)
    ax2.set_ylabel('Y (m)', fontsize=10)
    ax2.set_title('XY Trajectory (Top View)', fontsize=12, fontweight='bold')
    ax2.axis('equal')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    # ===============================
    # 3. Position vs Time
    # ===============================
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(df['t'], df['px'], 'b-', linewidth=1.5, label='X actual')
    ax3.plot(df['t'], df['py'], 'g-', linewidth=1.5, label='Y actual')
    ax3.plot(df['t'], df['pz'], 'r-', linewidth=1.5, label='Z actual')
    ax3.plot(df['t'], df['ref_px'], 'b--', linewidth=1, alpha=0.5, label='X ref')
    ax3.plot(df['t'], df['ref_py'], 'g--', linewidth=1, alpha=0.5, label='Y ref')
    ax3.plot(df['t'], df['ref_pz'], 'r--', linewidth=1, alpha=0.5, label='Z ref')
    
    # --- 标记风力开始/结束 (Position vs Time) ---
    if wind_start is not None and wind_start > 0:
        ax3.axvline(x=wind_start, color='orange', linestyle='-', linewidth=2, label='Wind Start')
    if wind_end is not None and wind_end > 0:
        ax3.axvline(x=wind_end, color='purple', linestyle='-', linewidth=2, label='Wind End')
    
    ax3.set_xlabel('Time (s)', fontsize=10)
    ax3.set_ylabel('Position (m)', fontsize=10)
    ax3.set_title('Position vs Time', fontsize=12, fontweight='bold')
    ax3.legend(fontsize=8, ncol=2)
    ax3.grid(True, alpha=0.3)
    
    # ===============================
    # 4. Tracking Error
    # ===============================
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(df['t'], df['pos_error'], 'b-', linewidth=2)
    ax4.axhline(y=df['pos_error'].mean(), color='r', linestyle='--', 
               linewidth=1.5, label=f'Mean = {df["pos_error"].mean():.3f}m')
    ax4.fill_between(df['t'], 0, 0.3, alpha=0.2, color='green', label='Target < 0.3m')
    
    # --- 标记风力开始/结束 (Tracking Error) ---
    if wind_start is not None and wind_start > 0:
        ax4.axvline(x=wind_start, color='orange', linestyle='-', linewidth=2, label='Wind Start')
    if wind_end is not None and wind_end > 0:
        ax4.axvline(x=wind_end, color='purple', linestyle='-', linewidth=2, label='Wind End')
    
    ax4.set_xlabel('Time (s)', fontsize=10)
    ax4.set_ylabel('Position Error (m)', fontsize=10)
    ax4.set_title('Tracking Error', fontsize=12, fontweight='bold')
    ax4.legend(fontsize=9)
    ax4.grid(True, alpha=0.3)
    
    # 整体标题
    title_suffix = ""
    if wind_start is not None and wind_end is not None:
        # 只有当 wind_start 和 wind_end 都有值时，才显示在标题中
        title_suffix = f" | Wind: {wind_start}s to {wind_end}s"
        
    fig.suptitle(f'PID Trajectory Comparison | Mean Error: {df["pos_error"].mean():.3f}m | Yaw Drift: {yaw_drift:.2f}°' + title_suffix, 
                fontsize=13, fontweight='bold')
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # 保存图片
    output_file = csv_file.replace('.csv', '_trajectory.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n✅ Trajectory plot saved to: {output_file}")
    
    # 显示图片（如果在交互环境中）
    plt.show()
    
    return output_file


def main():
    """主函数，处理命令行参数"""
    
    parser = argparse.ArgumentParser(description="LQR Trajectory Plotter. Plots actual vs reference positions and tracking error from a CSV log file.")
    parser.add_argument('log_file', help='The path to the CSV log file.')
    # 默认假设风力时间为 5.0s 和 20.0s
    parser.add_argument('--wind_start', type=float, default=5.0, help='Time (in seconds) when the wind disturbance started (Default: 5.0s).')
    parser.add_argument('--wind_end', type=float, default=20.0, help='Time (in seconds) when the wind disturbance ended (Default: 20.0s).')
    
    # 检查是否提供了日志文件名
    if len(sys.argv) < 2:
        print("Usage: python plot_lqr_trajectory.py <log_file.csv> [--wind_start <time>] [--wind_end <time>]")
        print("\nExample (using defaults 5.0s and 20.0s):")
        print("  python plot_lqr_trajectory.py crazyflie_hover_log.csv")
        sys.exit(1)
        
    args = parser.parse_args()
    csv_file = args.log_file
    
    if not os.path.exists(csv_file):
        print(f"Error: File '{csv_file}' not found!")
        sys.exit(1)
    
    try:
        plot_trajectory(csv_file, args.wind_start, args.wind_end)
    except Exception as e:
        print(f"Error plotting trajectory: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()