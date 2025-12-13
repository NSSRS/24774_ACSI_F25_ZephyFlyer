#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Controller Trajectory Plotter
简单的控制器日志可视化工具 (支持 LQR 和 TinyMPC)

用法:
    python plot_tinympc_trajectory.py <log_file.csv>
    
示例:
    python plot_tinympc_trajectory.py tinympc_20251211_120000.csv
    python plot_tinympc_trajectory.py lqr_v4_20251210_141448.csv
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

def plot_trajectory(csv_file):
    """
    绘制LQR轨迹对比图
    
    Args:
        csv_file: CSV日志文件路径
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
    ax4.set_xlabel('Time (s)', fontsize=10)
    ax4.set_ylabel('Position Error (m)', fontsize=10)
    ax4.set_title('Tracking Error', fontsize=12, fontweight='bold')
    ax4.legend(fontsize=9)
    ax4.grid(True, alpha=0.3)
    
    # 整体标题
    controller_type = df['mode'].iloc[0] if 'mode' in df.columns else 'Controller'
    fig.suptitle(f'{controller_type} Trajectory | Mean Error: {df["pos_error"].mean():.3f}m | Yaw Drift: {yaw_drift:.2f}°', 
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
    """主函数"""
    if len(sys.argv) < 2:
        print("Usage: python plot_tinympc_trajectory.py <log_file.csv>")
        print("\nExample:")
        print("  python plot_tinympc_trajectory.py tinympc_20251211_120000.csv")
        print("  python plot_tinympc_trajectory.py lqr_v4_20251210_141448.csv")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    
    if not os.path.exists(csv_file):
        print(f"Error: File '{csv_file}' not found!")
        sys.exit(1)
    
    try:
        plot_trajectory(csv_file)
    except Exception as e:
        print(f"Error plotting trajectory: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()