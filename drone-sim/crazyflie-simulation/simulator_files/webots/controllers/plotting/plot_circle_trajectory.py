# -*- coding: utf-8 -*-
"""
Slide 3 — Circle Trajectory Tracking (No Wind)

Plots (0–25 s only):
1) XY Trajectory
2) Tracking Error vs Time (with RMS / Max error summary)

Notes:
- No wind
- No ESO
- Controllers: PID, LQR, TinyMPC
"""

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ======================================================
# ---------------- PATH CONFIG -------------------------
# ======================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = BASE_DIR   # CSVs are in the same folder

# ======================================================
# ---------------- CONTROLLER CONFIG ------------------
# ======================================================

CONTROLLERS = {
    "PID": {
        "file": "pid_circle.csv",
        "color": "tab:blue",
        "linestyle": "-",
    },
    "LQR": {
        "file": "lqr_circle.csv",
        "color": "tab:green",
        "linestyle": "-",
    },
    "TinyMPC": {
        "file": "tinympc_circle.csv",
        "color": "tab:purple",
        "linestyle": "-",
    },
}

PLOT_CONTROLLERS = {
    "PID": True,
    "LQR": True,
    "TinyMPC": True,
}

# ======================================================
# ---------------- TIME WINDOW -------------------------
# ======================================================

T_END = 25.0

# ======================================================
# ---------------- HELPER FUNCTIONS --------------------
# ======================================================

def safe_load(path):
    if not os.path.exists(path):
        print(f"[INFO] missing: {path}")
        return None
    try:
        return pd.read_csv(path)
    except Exception as e:
        print(f"[WARN] failed to load {path}: {e}")
        return None


def crop_time(df, t_end):
    return df[df["t"] <= t_end]


def compute_error_metrics(err):
    rms = np.sqrt(np.mean(err**2))
    max_err = np.max(err)
    return rms, max_err


# ======================================================
# ----------- FIGURE 1: XY TRAJECTORY ------------------
# ======================================================

def plot_xy_trajectory():
    fig, ax = plt.subplots(figsize=(6, 6))

    for ctrl, enabled in PLOT_CONTROLLERS.items():
        if not enabled:
            continue

        cfg = CONTROLLERS[ctrl]
        df = safe_load(os.path.join(LOG_DIR, cfg["file"]))
        if df is None:
            continue

        df = crop_time(df, T_END)

        ax.plot(
            df["px"],
            df["py"],
            label=ctrl,
            color=cfg["color"],
            linestyle=cfg["linestyle"],
            linewidth=2,
        )

    # ---- Reference circle (assumed available) ----
    if df is not None and "px_ref" in df.columns:
        ax.plot(
            df["px_ref"],
            df["py_ref"],
            "k--",
            linewidth=2,
            label="Reference",
        )

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Circle Trajectory Tracking (No Wind, 0–25 s)")
    ax.grid(True)

    # ✅ Legend OUTSIDE plot (guaranteed visible)
    ax.legend(
        loc="center left",
        bbox_to_anchor=(1.02, 0.5),
        frameon=True
    )

    plt.tight_layout()
    plt.show()


# ======================================================
# ----------- FIGURE 2: TRACKING ERROR -----------------
# ======================================================

def plot_tracking_error():
    fig, ax = plt.subplots(figsize=(10, 4))

    summary_lines = []

    for ctrl, enabled in PLOT_CONTROLLERS.items():
        if not enabled:
            continue

        cfg = CONTROLLERS[ctrl]
        df = safe_load(os.path.join(LOG_DIR, cfg["file"]))
        if df is None:
            continue

        df = crop_time(df, T_END)

        t = df["t"]

        if "pos_error" in df.columns:
            err = df["pos_error"].values
        else:
            err = np.sqrt(df["px"]**2 + df["py"]**2 + df["pz"]**2)

        # ---- Plot error ----
        ax.plot(
            t,
            err,
            label=ctrl,
            color=cfg["color"],
            linestyle=cfg["linestyle"],
            linewidth=2,
        )

        # ---- Metrics ----
        rms = np.sqrt(np.mean(err**2))
        max_err = np.max(err)

        summary_lines.append(
            f"{ctrl}\n  RMS: {rms:.3f} m\n  Max: {max_err:.3f} m"
        )

    # ---- Axis formatting ----
    ax.set_xlim(0, T_END)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Tracking Error [m]")
    ax.set_title("Circle Tracking Error (No Wind, 0–25 s)")
    ax.grid(True)

    # ---- Legend (outside, clean) ----
    ax.legend(
        loc="upper left",
        bbox_to_anchor=(1.02, 1.0),
        frameon=True
    )

    # ---- RMS / Max summary box (outside, below legend) ----
    ax.text(
        1.02,
        0.55,
        "\n\n".join(summary_lines),
        transform=ax.transAxes,
        fontsize=10,
        verticalalignment="top",
        bbox=dict(facecolor="white", edgecolor="gray", alpha=0.9),
    )

    # Make room on the right
    fig.subplots_adjust(right=0.72)

    plt.show()


# ======================================================
# ---------------------- MAIN --------------------------
# ======================================================

if __name__ == "__main__":
    plot_xy_trajectory()
    plot_tracking_error()
