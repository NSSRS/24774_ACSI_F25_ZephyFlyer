#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LQR Trajectory Plotter
可视化 Crazyflie PID + ESO 的飞行日志
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import argparse 


def plot_trajectory(csv_file, wind_start=None, wind_end=None):
    """
    绘制 Crazyflie 轨迹 + ESO 扰动估计
    """

    # =============================
    # Load CSV
    # =============================
    df = pd.read_csv(csv_file)

    # =============================
    # Summary
    # =============================
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

    yaw_drift = (df['yaw'].iloc[-1] - df['yaw'].iloc[0]) * 180 / np.pi
    print(f"Yaw drift: {yaw_drift:.2f}°")
    print("="*60)

    # =============================
    # Main Figure (Trajectory)
    # =============================
    fig = plt.figure(figsize=(14, 8))

    # -----------------------------------------------------
    # 1. 3D Trajectory
    # -----------------------------------------------------
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.plot(df['px'], df['py'], df['pz'], 'b-', linewidth=2.5, alpha=0.8, label='Actual')
    ax1.plot(df['ref_px'], df['ref_py'], df['ref_pz'], 'r--', linewidth=2, alpha=0.7, label='Reference')

    ax1.scatter(df['px'].iloc[0], df['py'].iloc[0], df['pz'].iloc[0],
                c='lime', s=100, marker='o', label='Start')
    ax1.scatter(df['px'].iloc[-1], df['py'].iloc[-1], df['pz'].iloc[-1],
                c='red', s=100, marker='X', label='End')

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

    ax1.set_title("3D Trajectory", fontsize=12, fontweight='bold')
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    # -----------------------------------------------------
    # 2. XY Top View
    # -----------------------------------------------------
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(df['px'], df['py'], 'b-', linewidth=2.5, alpha=0.8, label='Actual')
    ax2.plot(df['ref_px'], df['ref_py'], 'r--', linewidth=2, alpha=0.7, label='Reference')

    ax2.scatter(df['px'].iloc[0], df['py'].iloc[0],
                c='lime', s=150, marker='o', edgecolors='black', linewidths=2, label='Start')
    ax2.scatter(df['px'].iloc[-1], df['py'].iloc[-1],
                c='red', s=150, marker='X', edgecolors='black', linewidths=2, label='End')

    ax2.set_title("XY Trajectory (Top View)", fontsize=12, fontweight='bold')
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.axis("equal")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # -----------------------------------------------------
    # 3. Position vs Time
    # -----------------------------------------------------
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(df['t'], df['px'], 'b-', linewidth=1.5, label='X actual')
    ax3.plot(df['t'], df['py'], 'g-', linewidth=1.5, label='Y actual')
    ax3.plot(df['t'], df['pz'], 'r-', linewidth=1.5, label='Z actual')

    ax3.plot(df['t'], df['ref_px'], 'b--', alpha=0.5, label='X ref')
    ax3.plot(df['t'], df['ref_py'], 'g--', alpha=0.5, label='Y ref')
    ax3.plot(df['t'], df['ref_pz'], 'r--', alpha=0.5, label='Z ref')

    if wind_start:
        ax3.axvline(wind_start, color='orange', linestyle='-', linewidth=2)
    if wind_end:
        ax3.axvline(wind_end, color='purple', linestyle='-', linewidth=2)

    ax3.set_title("Position vs Time", fontsize=12, fontweight='bold')
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Position (m)")
    ax3.legend(fontsize=7, ncol=2)
    ax3.grid(True, alpha=0.3)

    # -----------------------------------------------------
    # 4. Tracking Error
    # -----------------------------------------------------
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(df['t'], df['pos_error'], 'b-', linewidth=2)

    ax4.axhline(df['pos_error'].mean(), color='r', linestyle='--',
                label=f"Mean = {df['pos_error'].mean():.3f}m")

    if wind_start:
        ax4.axvline(wind_start, color='orange', linestyle='-', linewidth=2)
    if wind_end:
        ax4.axvline(wind_end, color='purple', linestyle='-', linewidth=2)

    ax4.set_title("Tracking Error", fontsize=12, fontweight='bold')
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Position Error (m)")
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    fig.suptitle(
        f"PID+ESO Trajectory | Mean Error = {df['pos_error'].mean():.3f} m | Yaw Drift = {yaw_drift:.2f}°",
        fontsize=14,
        fontweight='bold'
    )

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    # Save trajectory
    output_file = csv_file.replace(".csv", "_trajectory.png")
    plt.savefig(output_file, dpi=150)
    print(f"✅ Trajectory plot saved to {output_file}")

    plt.show()

    # ======================================================
    # Additional Window: ESO Disturbance vs Time
    # ======================================================
    fig2 = plt.figure(figsize=(10, 6))

    axd = fig2.add_subplot(1, 1, 1)
    axd.plot(df['t'], df['dist_x'], label="ESO Fx", linewidth=1.6)
    axd.plot(df['t'], df['dist_y'], label="ESO Fy", linewidth=1.6)
    axd.plot(df['t'], df['dist_z'], label="ESO Fz", linewidth=1.6)

    axd.set_title("ESO Disturbance Forces vs Time", fontsize=14, fontweight='bold')
    axd.set_xlabel("Time (s)")
    axd.set_ylabel("Force Estimate (N)")
    axd.axhline(0, color='black', linewidth=1)

    axd.grid(True, alpha=0.3)
    axd.legend(fontsize=10)

    dist_file = csv_file.replace(".csv", "_eso_disturbance.png")
    plt.savefig(dist_file, dpi=150)
    print(f"✅ ESO disturbance plot saved to {dist_file}")

    plt.show()

    return output_file



def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("log_file")
    parser.add_argument("--wind_start", type=float, default=5.0)
    parser.add_argument("--wind_end", type=float, default=20.0)

    args = parser.parse_args()

    if not os.path.exists(args.log_file):
        print(f"❌ File not found: {args.log_file}")
        sys.exit(1)

    plot_trajectory(args.log_file, args.wind_start, args.wind_end)



if __name__ == "__main__":
    main()
