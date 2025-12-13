import numpy as np
import matplotlib.pyplot as plt
import os


def load_log(filename):
    data = []
    with open(filename, "r") as f:
        for line in f:
            if line.startswith("#") or len(line.strip()) == 0:
                continue

            cols = line.strip().split()

            # must be 34 columns
            if len(cols) != 34:
                print(f"⚠️ Skipping malformed row with {len(cols)} columns")
                continue

            data.append([float(x) for x in cols])

    return np.array(data)


def plot_pid_vs_eso(filename):

    print(f"\n📂 Loading: {filename}")
    data = load_log(filename)

    (
        t,
        gps_x, gps_y, gps_z,
        eso_x, eso_y, eso_z,
        eso_vx, eso_vy, eso_vz,
        eso_roll, eso_pitch, eso_yaw,
        dist_x, dist_y, dist_z,
        imu_roll, imu_pitch, imu_yaw,
        target_x, target_y, target_z,
        m1, m2, m3, m4,
        pid1, pid2, pid3, pid4,
        eso1, eso2, eso3, eso4,
    ) = data.T

    # ----------------------------------------------------------
    # 1. Motor / PID / ESO
    # ----------------------------------------------------------
    motors = [m1, m2, m3, m4]
    pids = [pid1, pid2, pid3, pid4]
    esos = [eso1, eso2, eso3, eso4]
    labels = ["Motor 1", "Motor 2", "Motor 3", "Motor 4"]

    plt.figure(figsize=(14, 8))
    for i in range(4):
        plt.subplot(2, 2, i + 1)
        plt.plot(t, pids[i], label="PID")
        plt.plot(t, esos[i], label="ESO")
        plt.plot(t, motors[i], label="Final")
        plt.title(labels[i])
        plt.grid(True)
        plt.legend()
    plt.tight_layout()
    plt.show()

    # ----------------------------------------------------------
    # 2. Disturbances
    # ----------------------------------------------------------
    plt.figure(figsize=(10, 5))
    plt.plot(t, dist_x, label="Fx")
    plt.plot(t, dist_y, label="Fy")
    plt.plot(t, dist_z, label="Fz")
    plt.title("ESO Disturbances")
    plt.grid(True)
    plt.legend()
    plt.show()

    # ----------------------------------------------------------
    # 3. Position / XY
    # ----------------------------------------------------------
    plt.figure(figsize=(12, 6))

    plt.subplot(1, 2, 1)
    plt.plot(t, gps_z, label="Altitude")
    plt.plot(t, target_z, "--", label="Target")
    plt.title("Altitude Tracking")
    plt.grid(True)
    plt.legend()

    plt.subplot(1, 2, 2)
    plt.plot(gps_x, gps_y, label="Trajectory")
    plt.scatter(target_x[0], target_y[0], color="red", label="Target XY")
    plt.title("XY Tracking")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

    # ----------------------------------------------------------
    # 4. Attitude vs ESO
    # ----------------------------------------------------------
    plt.figure(figsize=(14, 5))

    plt.subplot(1, 3, 1)
    plt.plot(t, imu_roll, label="roll")
    plt.plot(t, eso_roll, '--', label="eso_roll")
    plt.grid(True)
    plt.legend()

    plt.subplot(1, 3, 2)
    plt.plot(t, imu_pitch, label="pitch")
    plt.plot(t, eso_pitch, '--', label="eso_pitch")
    plt.grid(True)
    plt.legend()

    plt.subplot(1, 3, 3)
    plt.plot(t, imu_yaw, label="yaw")
    plt.plot(t, eso_yaw, '--', label="eso_yaw")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()


# ----------------------------------------------------------
# Auto detect newest log
# ----------------------------------------------------------
if __name__ == "__main__":
    log_dir = os.path.join(os.path.dirname(__file__), "logs")
    logs = [f for f in os.listdir(log_dir) if f.endswith(".txt")]

    if len(logs) == 0:
        print("❌ No logs found")
        exit()

    latest = max(logs, key=lambda x: os.path.getmtime(os.path.join(log_dir, x)))
    fullpath = os.path.join(log_dir, latest)

    print(f"📌 Using latest log: {latest}")
    plot_pid_vs_eso(fullpath)
