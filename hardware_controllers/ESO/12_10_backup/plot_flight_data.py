import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# --------------------------------------------------------------------------
# CONFIGURATION
# --------------------------------------------------------------------------
SEARCH_PATH = os.path.dirname(os.path.abspath(__file__))
FILE_PATTERN = "circle_eso_log_*.txt"


def get_latest_log_file():
    """Finds the most recent log file in the current folder."""
    full_search_path = os.path.join(SEARCH_PATH, FILE_PATTERN)
    files = glob.glob(full_search_path)

    if not files:
        return None

    return max(files, key=os.path.getmtime)


# --------------------------------------------------------------------------
# MAIN
# --------------------------------------------------------------------------
def main():
    print(f"Searching for logs in {SEARCH_PATH}")
    filepath = get_latest_log_file()

    if filepath is None:
        print("No log files found.")
        return

    print(f"Loading: {os.path.basename(filepath)}")

    # Read tab-separated CF logs
    data = pd.read_csv(filepath, sep="\t")

    # Extract data
    x = data["eso_x"]
    y = data["eso_y"]
    z = data["eso_z"]
    Fx = data["dist_x"]
    Fy = data["dist_y"]
    Fz = data["dist_z"]

    # ==========================================================
    # 3D PLOT FIRST (AND ONLY)
    # ==========================================================
    fig = plt.figure(figsize=(10, 9))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title(f"3D ESO Trajectory + Force Vectors\n{os.path.basename(filepath)}")

    # Plot trajectory
    ax.plot3D(x, y, z, "b", label="ESO Trajectory")

    # Downsample force vectors
    step = max(5, len(data) // 200)
    ax.quiver(
        x[::step], y[::step], z[::step],
        Fx[::step], Fy[::step], Fz[::step],
        length=0.1, normalize=False, color="red"
    )

    # Labels
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()
    ax.grid(True)

    plt.show()


if __name__ == "__main__":
    main()
