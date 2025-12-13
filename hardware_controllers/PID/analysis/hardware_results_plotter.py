#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESO Performance Comparison Tool
Compare PID (without ESO) vs PID+ESO in hover and circle flight modes

Usage:
    python compare_eso_performance.py
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from pathlib import Path

# Data file configuration
DATA_DIR = "data"
FILES = {
    'hover_no_eso': 'hardware_pid_noeso_hover_no_wind.csv',
    'hover_eso': 'hardware_pid_eso_hover_no_wind.csv',
    'circle_no_eso': 'hardware_pid_noeso_circle_wind.csv',
    'circle_eso': 'hardware_pid_eso_circle_wind.csv',
}

def load_data():
    """Load all data files"""
    data = {}
    for key, filename in FILES.items():
        filepath = os.path.join(DATA_DIR, filename)
        if os.path.exists(filepath):
            data[key] = pd.read_csv(filepath)
            print(f"✅ Loaded: {filename}")
        else:
            print(f"❌ Missing: {filename}")
    return data

def cleanse_initialization(df, init_time=2.0):
    """Remove initialization period data (first 1-2 seconds)"""
    df_clean = df.copy()
    
    # Find the time threshold
    t_start = df['t'].iloc[0]
    t_threshold = t_start + init_time
    
    # Keep only data after initialization
    df_clean = df_clean[df_clean['t'] >= t_threshold].reset_index(drop=True)
    
    # Reset time to start at 0
    if len(df_clean) > 0:
        df_clean['t'] = df_clean['t'] - df_clean['t'].iloc[0]
    
    return df_clean

def calculate_metrics(df):
    """Calculate key performance metrics"""
    metrics = {
        'mean_error': df['pos_error'].mean(),
        'max_error': df['pos_error'].max(),
        'std_error': df['pos_error'].std(),
        'rmse': np.sqrt((df['pos_error']**2).mean()),
        'duration': df['t'].iloc[-1] - df['t'].iloc[0],
        'yaw_drift': (df['yaw'].iloc[-1] - df['yaw'].iloc[0]) * 180 / np.pi,
    }
    return metrics

def normalize_trajectory(df):
    """Normalize trajectory to start at (0,0) by offsetting initial position"""
    df_norm = df.copy()
    
    # Get initial offsets
    x_offset = df['px'].iloc[0]
    y_offset = df['py'].iloc[0]
    
    # Apply offsets to actual trajectory
    df_norm['px'] = df['px'] - x_offset
    df_norm['py'] = df['py'] - y_offset
    
    # Apply same offsets to reference trajectory
    df_norm['ref_px'] = df['ref_px'] - x_offset
    df_norm['ref_py'] = df['ref_py'] - y_offset
    
    return df_norm

def plot_overlaid_comparison(data):
    """Plot all comparisons overlaid on the same graphs"""
    fig = plt.figure(figsize=(20, 12))
    fig.suptitle('ESO Hardware Performance Comparison: PID vs PID+ESO', fontsize=18, fontweight='bold')
    
    # Cleanse initialization data (first 2 seconds)
    hover_no_eso_clean = cleanse_initialization(data['hover_no_eso'], init_time=2.0)
    hover_eso_clean = cleanse_initialization(data['hover_eso'], init_time=2.0)
    circle_no_eso_clean = cleanse_initialization(data['circle_no_eso'], init_time=2.0)
    circle_eso_clean = cleanse_initialization(data['circle_eso'], init_time=2.0)
    
    # Normalize trajectories to start at (0,0)
    hover_no_eso = normalize_trajectory(hover_no_eso_clean)
    hover_eso = normalize_trajectory(hover_eso_clean)
    circle_no_eso = normalize_trajectory(circle_no_eso_clean)
    circle_eso = normalize_trajectory(circle_eso_clean)
    
    # Calculate metrics (use cleansed data for metrics)
    hover_no_eso_metrics = calculate_metrics(hover_no_eso)
    hover_eso_metrics = calculate_metrics(hover_eso)
    circle_no_eso_metrics = calculate_metrics(circle_no_eso)
    circle_eso_metrics = calculate_metrics(circle_eso)
    
    # ========================================
    # Row 1: HOVER MODE
    # ========================================
    
    # Hover - 3D Trajectory (overlaid)
    ax = fig.add_subplot(2, 4, 1, projection='3d')
    ax.plot(hover_no_eso['px'], hover_no_eso['py'], hover_no_eso['pz'], 
            'b-', linewidth=2.5, alpha=0.7, label='PID Only')
    ax.plot(hover_eso['px'], hover_eso['py'], hover_eso['pz'], 
            'g-', linewidth=2.5, alpha=0.7, label='PID+ESO')
    ax.plot(hover_no_eso['ref_px'], hover_no_eso['ref_py'], hover_no_eso['ref_pz'], 
            'r--', linewidth=1.5, alpha=0.5, label='Reference')
    ax.set_xlabel('X (m)', fontsize=10)
    ax.set_ylabel('Y (m)', fontsize=10)
    ax.set_zlabel('Z (m)', fontsize=10)
    ax.set_title('Hover - 3D Trajectory', fontsize=12, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    
    # Hover - XY Top View (overlaid)
    ax = fig.add_subplot(2, 4, 2)
    ax.plot(hover_no_eso['ref_px'], hover_no_eso['ref_py'], 'r--', linewidth=2.5, alpha=0.8, label='Reference', zorder=1)
    ax.plot(hover_no_eso['px'], hover_no_eso['py'], 'b-', linewidth=2.5, alpha=0.7, label='PID Only', zorder=2)
    ax.plot(hover_eso['px'], hover_eso['py'], 'g-', linewidth=2.5, alpha=0.7, label='PID+ESO', zorder=3)
    ax.scatter([0], [0], c='red', s=150, marker='X', edgecolors='black', linewidths=2, zorder=5, label='Start (0,0)')
    ax.set_xlabel('X (m)', fontsize=10)
    ax.set_ylabel('Y (m)', fontsize=10)
    ax.set_title('Hover - XY Top View', fontsize=12, fontweight='bold')
    ax.axis('equal')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # Hover - Position Error (overlaid)
    ax = fig.add_subplot(2, 4, 3)
    ax.plot(hover_no_eso['t'], hover_no_eso['pos_error'], 'b-', linewidth=2, alpha=0.7, label='PID Only')
    ax.plot(hover_eso['t'], hover_eso['pos_error'], 'g-', linewidth=2, alpha=0.7, label='PID+ESO')
    ax.axhline(y=0.3, color='k', linestyle=':', linewidth=1.5, alpha=0.5, label='Target (0.3m)')
    ax.set_xlabel('Time (s)', fontsize=10)
    ax.set_ylabel('Position Error (m)', fontsize=10)
    ax.set_title('Hover - Tracking Error', fontsize=12, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)
    
    # Hover - Metrics Table
    ax = fig.add_subplot(2, 4, 4)
    ax.axis('off')
    
    # Create metrics table
    metrics_text = "HOVER MODE METRICS\n" + "="*45 + "\n\n"
    metrics_text += f"{'Metric':<18} {'PID':<10} {'PID+ESO':<10} {'Δ%':<8}\n"
    metrics_text += "-"*45 + "\n"
    
    improvement = (1 - hover_eso_metrics['mean_error']/hover_no_eso_metrics['mean_error'])*100
    metrics_text += f"{'Mean Error (m)':<18} {hover_no_eso_metrics['mean_error']:>9.4f} {hover_eso_metrics['mean_error']:>9.4f} {improvement:>7.1f}%\n"
    
    improvement = (1 - hover_eso_metrics['max_error']/hover_no_eso_metrics['max_error'])*100
    metrics_text += f"{'Max Error (m)':<18} {hover_no_eso_metrics['max_error']:>9.4f} {hover_eso_metrics['max_error']:>9.4f} {improvement:>7.1f}%\n"
    
    improvement = (1 - hover_eso_metrics['std_error']/hover_no_eso_metrics['std_error'])*100
    metrics_text += f"{'Std Dev (m)':<18} {hover_no_eso_metrics['std_error']:>9.4f} {hover_eso_metrics['std_error']:>9.4f} {improvement:>7.1f}%\n"
    
    improvement = (1 - hover_eso_metrics['rmse']/hover_no_eso_metrics['rmse'])*100
    metrics_text += f"{'RMSE (m)':<18} {hover_no_eso_metrics['rmse']:>9.4f} {hover_eso_metrics['rmse']:>9.4f} {improvement:>7.1f}%\n"
    
    metrics_text += f"{'Yaw Drift (deg)':<18} {hover_no_eso_metrics['yaw_drift']:>9.2f} {hover_eso_metrics['yaw_drift']:>9.2f}\n"
    
    ax.text(0.1, 0.5, metrics_text, fontsize=10, family='monospace', 
            verticalalignment='center', transform=ax.transAxes)
    ax.set_title('Hover - Performance Metrics', fontsize=12, fontweight='bold')
    
    # ========================================
    # Row 2: CIRCLE MODE
    # ========================================
    
    # Circle - 3D Trajectory (overlaid)
    ax = fig.add_subplot(2, 4, 5, projection='3d')
    ax.plot(circle_no_eso['px'], circle_no_eso['py'], circle_no_eso['pz'], 
            'b-', linewidth=2.5, alpha=0.7, label='PID Only')
    ax.plot(circle_eso['px'], circle_eso['py'], circle_eso['pz'], 
            'g-', linewidth=2.5, alpha=0.7, label='PID+ESO')
    ax.plot(circle_no_eso['ref_px'], circle_no_eso['ref_py'], circle_no_eso['ref_pz'], 
            'r--', linewidth=1.5, alpha=0.5, label='Reference')
    ax.set_xlabel('X (m)', fontsize=10)
    ax.set_ylabel('Y (m)', fontsize=10)
    ax.set_zlabel('Z (m)', fontsize=10)
    ax.set_title('Circle - 3D Trajectory (with wind)', fontsize=12, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    
    # Circle - XY Top View (overlaid)
    ax = fig.add_subplot(2, 4, 6)
    ax.plot(circle_no_eso['ref_px'], circle_no_eso['ref_py'], 'r--', linewidth=2.5, alpha=0.8, label='Reference', zorder=1)
    ax.plot(circle_no_eso['px'], circle_no_eso['py'], 'b-', linewidth=2.5, alpha=0.7, label='PID Only', zorder=2)
    ax.plot(circle_eso['px'], circle_eso['py'], 'g-', linewidth=2.5, alpha=0.7, label='PID+ESO', zorder=3)
    ax.scatter([0], [0], c='red', s=150, marker='X', edgecolors='black', linewidths=2, zorder=5, label='Start (0,0)')
    ax.set_xlabel('X (m)', fontsize=10)
    ax.set_ylabel('Y (m)', fontsize=10)
    ax.set_title('Circle - XY Top View (with wind)', fontsize=12, fontweight='bold')
    ax.axis('equal')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # Circle - Position Error (overlaid)
    ax = fig.add_subplot(2, 4, 7)
    ax.plot(circle_no_eso['t'], circle_no_eso['pos_error'], 'b-', linewidth=2, alpha=0.7, label='PID Only')
    ax.plot(circle_eso['t'], circle_eso['pos_error'], 'g-', linewidth=2, alpha=0.7, label='PID+ESO')
    ax.axhline(y=0.3, color='k', linestyle=':', linewidth=1.5, alpha=0.5, label='Target (0.3m)')
    ax.set_xlabel('Time (s)', fontsize=10)
    ax.set_ylabel('Position Error (m)', fontsize=10)
    ax.set_title('Circle - Tracking Error (with wind)', fontsize=12, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)
    
    # Circle - Metrics Table
    ax = fig.add_subplot(2, 4, 8)
    ax.axis('off')
    
    # Create metrics table
    metrics_text = "CIRCLE MODE METRICS (with wind)\n" + "="*45 + "\n\n"
    metrics_text += f"{'Metric':<18} {'PID':<10} {'PID+ESO':<10} {'Δ%':<8}\n"
    metrics_text += "-"*45 + "\n"
    
    improvement = (1 - circle_eso_metrics['mean_error']/circle_no_eso_metrics['mean_error'])*100
    metrics_text += f"{'Mean Error (m)':<18} {circle_no_eso_metrics['mean_error']:>9.4f} {circle_eso_metrics['mean_error']:>9.4f} {improvement:>7.1f}%\n"
    
    improvement = (1 - circle_eso_metrics['max_error']/circle_no_eso_metrics['max_error'])*100
    metrics_text += f"{'Max Error (m)':<18} {circle_no_eso_metrics['max_error']:>9.4f} {circle_eso_metrics['max_error']:>9.4f} {improvement:>7.1f}%\n"
    
    improvement = (1 - circle_eso_metrics['std_error']/circle_no_eso_metrics['std_error'])*100
    metrics_text += f"{'Std Dev (m)':<18} {circle_no_eso_metrics['std_error']:>9.4f} {circle_eso_metrics['std_error']:>9.4f} {improvement:>7.1f}%\n"
    
    improvement = (1 - circle_eso_metrics['rmse']/circle_no_eso_metrics['rmse'])*100
    metrics_text += f"{'RMSE (m)':<18} {circle_no_eso_metrics['rmse']:>9.4f} {circle_eso_metrics['rmse']:>9.4f} {improvement:>7.1f}%\n"
    
    metrics_text += f"{'Yaw Drift (deg)':<18} {circle_no_eso_metrics['yaw_drift']:>9.2f} {circle_eso_metrics['yaw_drift']:>9.2f}\n"
    
    ax.text(0.1, 0.5, metrics_text, fontsize=10, family='monospace', 
            verticalalignment='center', transform=ax.transAxes)
    ax.set_title('Circle - Performance Metrics', fontsize=12, fontweight='bold')
    
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    
    # Save figure
    output_file = 'eso_overlaid_comparison.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n✅ Overlaid comparison saved to: {output_file}")
    
    # Print metrics comparison to console
    print("\n" + "="*70)
    print("HOVER MODE METRICS COMPARISON")
    print("="*70)
    print(f"{'Metric':<20} {'PID Only':<15} {'PID+ESO':<15} {'Improvement':<15}")
    print("-"*70)
    print(f"{'Mean Error (m)':<20} {hover_no_eso_metrics['mean_error']:>14.4f} {hover_eso_metrics['mean_error']:>14.4f} {(1 - hover_eso_metrics['mean_error']/hover_no_eso_metrics['mean_error'])*100:>13.1f}%")
    print(f"{'Max Error (m)':<20} {hover_no_eso_metrics['max_error']:>14.4f} {hover_eso_metrics['max_error']:>14.4f} {(1 - hover_eso_metrics['max_error']/hover_no_eso_metrics['max_error'])*100:>13.1f}%")
    print(f"{'Std Dev (m)':<20} {hover_no_eso_metrics['std_error']:>14.4f} {hover_eso_metrics['std_error']:>14.4f} {(1 - hover_eso_metrics['std_error']/hover_no_eso_metrics['std_error'])*100:>13.1f}%")
    print(f"{'RMSE (m)':<20} {hover_no_eso_metrics['rmse']:>14.4f} {hover_eso_metrics['rmse']:>14.4f} {(1 - hover_eso_metrics['rmse']/hover_no_eso_metrics['rmse'])*100:>13.1f}%")
    print(f"{'Yaw Drift (deg)':<20} {hover_no_eso_metrics['yaw_drift']:>14.2f} {hover_eso_metrics['yaw_drift']:>14.2f}")
    print("="*70)
    
    print("\n" + "="*70)
    print("CIRCLE MODE METRICS COMPARISON (with wind)")
    print("="*70)
    print(f"{'Metric':<20} {'PID Only':<15} {'PID+ESO':<15} {'Improvement':<15}")
    print("-"*70)
    print(f"{'Mean Error (m)':<20} {circle_no_eso_metrics['mean_error']:>14.4f} {circle_eso_metrics['mean_error']:>14.4f} {(1 - circle_eso_metrics['mean_error']/circle_no_eso_metrics['mean_error'])*100:>13.1f}%")
    print(f"{'Max Error (m)':<20} {circle_no_eso_metrics['max_error']:>14.4f} {circle_eso_metrics['max_error']:>14.4f} {(1 - circle_eso_metrics['max_error']/circle_no_eso_metrics['max_error'])*100:>13.1f}%")
    print(f"{'Std Dev (m)':<20} {circle_no_eso_metrics['std_error']:>14.4f} {circle_eso_metrics['std_error']:>14.4f} {(1 - circle_eso_metrics['std_error']/circle_no_eso_metrics['std_error'])*100:>13.1f}%")
    print(f"{'RMSE (m)':<20} {circle_no_eso_metrics['rmse']:>14.4f} {circle_eso_metrics['rmse']:>14.4f} {(1 - circle_eso_metrics['rmse']/circle_no_eso_metrics['rmse'])*100:>13.1f}%")
    print(f"{'Yaw Drift (deg)':<20} {circle_no_eso_metrics['yaw_drift']:>14.2f} {circle_eso_metrics['yaw_drift']:>14.2f}")
    print("="*70)
    
    return fig

def main():
    """Main function"""
    print("="*70)
    print("ESO Performance Comparison Tool")
    print("="*70)
    
    # Load all data
    data = load_data()
    
    if len(data) < 4:
        print("\n❌ Error: Not all required data files found!")
        print("Expected files in 'data/' directory:")
        for filename in FILES.values():
            print(f"  - {filename}")
        return
    
    # Generate comparison plot
    print("\n📊 Generating overlaid comparison plot...")
    print("   - Removing first 2 seconds of data (sensor initialization)")
    print("   - Normalizing all trajectories to start at (0,0)")
    
    plot_overlaid_comparison(data)
    
    print("\n" + "="*70)
    print("✅ Overlaid comparison plot generated successfully!")
    print("="*70)
    print("\nGenerated file:")
    print("  - eso_overlaid_comparison.png   (2x4 grid with all comparisons overlaid)")
    print("\nNote: First 2 seconds of data removed to exclude sensor initialization")
    print("="*70)

if __name__ == '__main__':
    main()