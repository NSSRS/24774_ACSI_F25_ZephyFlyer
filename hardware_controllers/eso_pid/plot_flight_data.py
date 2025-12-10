import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# --------------------------------------------------------------------------
# CONFIGURATION
# --------------------------------------------------------------------------
# Path is now the current folder where this script (and the log) resides
SEARCH_PATH = os.path.dirname(os.path.abspath(__file__))
FILE_PATTERN = "circle_eso_log_*.txt"

def get_latest_log_file():
    """Finds the most recent log file in the CURRENT folder."""
    full_search_path = os.path.join(SEARCH_PATH, FILE_PATTERN)
    list_of_files = glob.glob(full_search_path)
    
    if not list_of_files:
        return None
    
    # Sort by modification time (newest first)
    latest_file = max(list_of_files, key=os.path.getmtime)
    return latest_file

def main():
    print(f"Searching for log files in: {SEARCH_PATH}")
    filepath = get_latest_log_file()
    
    if filepath is None:
        print(f"No log files found matching '{FILE_PATTERN}'")
        return

    print(f"Loading data from: {os.path.basename(filepath)}")
    
    try:
        data = pd.read_csv(filepath, sep='\t')
    except Exception as e:
        print(f"Error reading file: {e}")
        return

    t = data['t']

    # ----------------------------------------------------------------------
    # PLOT 1: Position Comparison
    # ----------------------------------------------------------------------
    fig1, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig1.suptitle(f'Position Estimation: Internal vs ESO\nFile: {os.path.basename(filepath)}')

    axs[0].plot(t, data['x'], 'k--', label='CF Internal (Flow)')
    axs[0].plot(t, data['eso_x'], 'b', label='ESO Estimate')
    axs[0].set_ylabel('X (m)')
    axs[0].legend(loc='upper right')
    axs[0].grid(True)

    axs[1].plot(t, data['y'], 'k--', label='CF Internal')
    axs[1].plot(t, data['eso_y'], 'b', label='ESO Estimate')
    axs[1].set_ylabel('Y (m)')
    axs[1].grid(True)

    axs[2].plot(t, data['z'], 'k--', label='CF Internal')
    axs[2].plot(t, data['eso_z'], 'b', label='ESO Estimate')
    axs[2].set_ylabel('Z (m)')
    axs[2].set_xlabel('Time (s)')
    axs[2].grid(True)

    plt.tight_layout()

    # ----------------------------------------------------------------------
    # PLOT 2: Estimated Disturbances
    # ----------------------------------------------------------------------
    fig2, axs2 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig2.suptitle('Estimated External Disturbances (Force in Newtons)')

    axs2[0].plot(t, data['dist_x'], 'r', alpha=0.8)
    axs2[0].set_ylabel('Dist. X (N)')
    axs2[0].grid(True)
    axs2[0].set_title("Forward/Backward Drag/Wind")

    axs2[1].plot(t, data['dist_y'], 'r', alpha=0.8)
    axs2[1].set_ylabel('Dist. Y (N)')
    axs2[1].grid(True)
    axs2[1].set_title("Left/Right Drag/Wind")

    axs2[2].plot(t, data['dist_z'], 'r', alpha=0.8)
    axs2[2].set_ylabel('Dist. Z (N)')
    axs2[2].set_xlabel('Time (s)')
    axs2[2].grid(True)
    axs2[2].set_title("Vertical Disturbances (Mass Error + Ground Effect)")

    plt.tight_layout()

    # ----------------------------------------------------------------------
    # PLOT 3: 2D Trajectory
    # ----------------------------------------------------------------------
    plt.figure(figsize=(8, 8))
    plt.plot(data['x'], data['y'], 'k--', label='CF Internal')
    plt.plot(data['eso_x'], data['eso_y'], 'b', alpha=0.7, label='ESO Estimate')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Top-Down Flight Path')
    plt.axis('equal')
    plt.legend()
    plt.grid(True)

    plt.show()

if __name__ == "__main__":
    main()