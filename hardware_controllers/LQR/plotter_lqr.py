#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESO+PID Hover Log Plotter
Visualizes hover performance with ESO disturbance estimates

Usage:
    python plotter.py                      # Opens file dialog
    python plotter.py <log_file.txt>       # Direct file path
    
Example:
    python plotter.py hover_eso_log_20251211_143022.txt
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
from tkinter import Tk
from tkinter.filedialog import askopenfilename

def select_file():
    """Open file dialog to select log file"""
    print("Opening file dialog...")
    Tk().withdraw()  # Hide the root window
    filename = askopenfilename(
        title="Select ESO Hover Log File",
        filetypes=[
            ("Text files", "*.txt"),
            ("All files", "*.*")
        ],
        initialdir=os.path.expanduser("~/Downloads")
    )
    return filename

def plot_hover_data(log_file):
    """
    Plot ESO+PID hover data
    
    Args:
        log_file: Log file path (.txt format)
    """
    # Read file and handle # header correctly
    try:
        # Read first line to check for # header
        with open(log_file, 'r') as f:
            first_line = f.readline().strip()
        
        # If first line starts with #, parse it as header
        if first_line.startswith('#'):
            # Remove # and split by tab to get column names
            col_names = first_line.lstrip('#').strip().split('\t')
            col_names = [c.strip() for c in col_names]
            
            # Read data starting from line 2 (skip header)
            df = pd.read_csv(log_file, sep='\t', skiprows=1, names=col_names)
        else:
            # No # header, read normally
            df = pd.read_csv(log_file, sep='\t')
    except Exception as e:
        print(f"Error reading file: {e}")
        # Fallback: try different delimiters
        try:
            df = pd.read_csv(log_file, delim_whitespace=True)
        except:
            df = pd.read_csv(log_file)
    
    # Debug: print what columns we found
    print(f"Found columns: {list(df.columns)}")
    
    # Check if we have the expected columns
    expected_cols = ['t', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'dax', 'day', 'daz', 'dthrust']
    missing_cols = [col for col in expected_cols if col not in df.columns]
    if missing_cols:
        print(f"Warning: Missing columns: {missing_cols}")
        print(f"Available columns: {list(df.columns)}")
    
    # Print summary
    print("="*60)
    print("Hover Test Summary")
    print("="*60)
    print(f"File: {os.path.basename(log_file)}")
    print(f"Duration: {df['t'].iloc[-1] - df['t'].iloc[0]:.1f}s")
    print(f"Samples: {len(df)}")
    
    # Position stats
    print(f"\nPosition stats:")
    print(f"  X: mean={df['x'].mean():.3f}m, std={df['x'].std():.3f}m")
    print(f"  Y: mean={df['y'].mean():.3f}m, std={df['y'].std():.3f}m")
    print(f"  Z: mean={df['z'].mean():.3f}m, std={df['z'].std():.3f}m")
    
    # ESO disturbance stats (only if ESO data exists)
    if 'dax' in df.columns and df['dax'].abs().max() < 1000:  # Check if ESO has reasonable values
        print(f"\nESO disturbance estimates:")
        print(f"  d_ax: mean={df['dax'].mean():.3f} m/s², std={df['dax'].std():.3f} m/s²")
        print(f"  d_ay: mean={df['day'].mean():.3f} m/s², std={df['day'].std():.3f} m/s²")
        print(f"  d_az: mean={df['daz'].mean():.3f} m/s², std={df['daz'].std():.3f} m/s²")
        print(f"  d_thrust: mean={df['dthrust'].mean():.3f} N")
    
    print("="*60)
    
    # Create figure with subplots
    fig = plt.figure(figsize=(14, 10))
    
    # ===============================
    # 1. 3D Trajectory with Force Vectors
    # ===============================
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(df['x'], df['y'], df['z'], 'b-', linewidth=2, alpha=0.7)
    ax1.scatter(df['x'].iloc[0], df['y'].iloc[0], df['z'].iloc[0], 
               c='lime', s=100, marker='o', label='Start', zorder=5)
    ax1.scatter(df['x'].iloc[-1], df['y'].iloc[-1], df['z'].iloc[-1], 
               c='red', s=100, marker='X', label='End', zorder=5)
    
    # Add force vectors from ESO disturbances
    if 'dax' in df.columns:
        # Subsample to avoid too many arrows (every N points)
        N_arrows = 10
        step = max(1, len(df) // N_arrows)
        
        # Scale factor for visualization (adjust to make arrows visible)
        scale = 0.2  # Scale disturbance (m/s²) to arrow length (m) - INCREASED for visibility
        
        for i in range(0, len(df), step):
            x = df['x'].iloc[i]
            y = df['y'].iloc[i]
            z = df['z'].iloc[i]
            
            dx = df['dax'].iloc[i] * scale
            dy = df['day'].iloc[i] * scale
            dz = df['daz'].iloc[i] * scale
            
            # Only draw if disturbance is significant
            if abs(dx) > 0.001 or abs(dy) > 0.001 or abs(dz) > 0.001:
                ax1.quiver(x, y, z, dx, dy, dz, 
                          color='red', alpha=0.6, arrow_length_ratio=0.3,
                          linewidth=1.5)
    
    ax1.set_xlabel('X (m)', fontsize=10)
    ax1.set_ylabel('Y (m)', fontsize=10)
    ax1.set_zlabel('Z (m)', fontsize=10)
    ax1.set_title('3D Hover Trajectory + ESO Forces', fontsize=11, fontweight='bold')
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)
    
    # ===============================
    # 2. XY Top View with Force Vectors
    # ===============================
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(df['x'], df['y'], 'b-', linewidth=2, alpha=0.7)
    ax2.scatter(df['x'].iloc[0], df['y'].iloc[0], 
               c='lime', s=150, marker='o', edgecolors='black', linewidths=2, zorder=5, label='Start')
    ax2.scatter(df['x'].iloc[-1], df['y'].iloc[-1], 
               c='red', s=150, marker='X', edgecolors='black', linewidths=2, zorder=5, label='End')
    
    # Add XY force vectors from ESO disturbances
    if 'dax' in df.columns and 'day' in df.columns:
        # More frequent arrows for top view (every N points)
        N_arrows_xy = 20  # Show more arrows in 2D
        step = max(1, len(df) // N_arrows_xy)
        
        # Scale factor for visualization - INCREASED for visibility
        scale = 0.2  # Scale disturbance (m/s²) to arrow length (m)
        
        for i in range(0, len(df), step):
            x = df['x'].iloc[i]
            y = df['y'].iloc[i]
            
            dx = df['dax'].iloc[i] * scale
            dy = df['day'].iloc[i] * scale
            
            # Only draw if disturbance is significant
            if abs(dx) > 0.001 or abs(dy) > 0.001:
                ax2.quiver(x, y, dx, dy, 
                          color='red', alpha=0.6, 
                          scale_units='xy', scale=1,
                          width=0.003, headwidth=4, headlength=5)
    
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.axvline(x=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_xlabel('X (m)', fontsize=10)
    ax2.set_ylabel('Y (m)', fontsize=10)
    ax2.set_title('XY Position + ESO Forces (Top View)', fontsize=11, fontweight='bold')
    ax2.axis('equal')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    # ===============================
    # 3. Position vs Time
    # ===============================
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(df['t'], df['x'], 'b-', linewidth=1.5, label='X position')
    ax3.plot(df['t'], df['y'], 'g-', linewidth=1.5, label='Y position')
    ax3.plot(df['t'], df['z'], 'r-', linewidth=1.5, label='Z position')
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax3.set_xlabel('Time (s)', fontsize=10)
    ax3.set_ylabel('Position (m)', fontsize=10)
    ax3.set_title('Position vs Time', fontsize=11, fontweight='bold')
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)
    
    # ===============================
    # 4. ESO Disturbances (X, Y, Z)
    # ===============================
    ax4 = fig.add_subplot(2, 3, 4)
    if 'dax' in df.columns:
        ax4.plot(df['t'], df['dax'], 'b-', linewidth=1.5, label='d_ax')
        ax4.plot(df['t'], df['day'], 'g-', linewidth=1.5, label='d_ay')
        ax4.plot(df['t'], df['daz'], 'r-', linewidth=1.5, label='d_az')
        ax4.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax4.set_ylim(-2, 2)  # Reasonable range
    ax4.set_xlabel('Time (s)', fontsize=10)
    ax4.set_ylabel('Disturbance (m/s²)', fontsize=10)
    ax4.set_title('ESO Acceleration Disturbances', fontsize=11, fontweight='bold')
    ax4.legend(fontsize=9)
    ax4.grid(True, alpha=0.3)
    
    # ===============================
    # 5. ESO Thrust Disturbance
    # ===============================
    ax5 = fig.add_subplot(2, 3, 5)
    if 'dthrust' in df.columns:
        ax5.plot(df['t'], df['dthrust'], 'purple', linewidth=2)
        ax5.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax5.axhline(y=df['dthrust'].mean(), color='r', linestyle='--', 
                   linewidth=1.5, alpha=0.7, label=f'Mean = {df["dthrust"].mean():.3f} N')
        ax5.set_ylim(-0.5, 0.5)  # Reasonable range
    ax5.set_xlabel('Time (s)', fontsize=10)
    ax5.set_ylabel('Thrust Disturbance (N)', fontsize=10)
    ax5.set_title('ESO Thrust Disturbance Estimate', fontsize=11, fontweight='bold')
    ax5.legend(fontsize=9)
    ax5.grid(True, alpha=0.3)
    
    # ===============================
    # 6. Attitude
    # ===============================
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.plot(df['t'], df['roll'], 'b-', linewidth=1.5, label='Roll')
    ax6.plot(df['t'], df['pitch'], 'g-', linewidth=1.5, label='Pitch')
    ax6.plot(df['t'], df['yaw'], 'r-', linewidth=1.5, label='Yaw')
    ax6.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax6.set_xlabel('Time (s)', fontsize=10)
    ax6.set_ylabel('Angle (deg)', fontsize=10)
    ax6.set_title('Attitude', fontsize=11, fontweight='bold')
    ax6.legend(fontsize=9)
    ax6.grid(True, alpha=0.3)
    
    # Overall title
    fig.suptitle(f'ESO+PID Hover Performance | Duration: {df["t"].iloc[-1]:.1f}s | Z std: {df["z"].std():.3f}m', 
                fontsize=13, fontweight='bold')
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # Save figure
    output_file = log_file.replace('.txt', '_plots.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n✅ Plots saved to: {output_file}")
    
    # Show the plot
    plt.show()
    
    return output_file


def main():
    """Main function"""
    # If no argument provided, open file dialog
    if len(sys.argv) < 2:
        print("No file specified. Opening file dialog...")
        log_file = select_file()
        
        if not log_file:
            print("No file selected. Exiting.")
            sys.exit(0)
    else:
        log_file = sys.argv[1]
    
    if not os.path.exists(log_file):
        print(f"Error: File '{log_file}' not found!")
        sys.exit(1)
    
    print(f"\nProcessing: {log_file}\n")
    
    try:
        plot_hover_data(log_file)
    except Exception as e:
        print(f"Error plotting data: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()