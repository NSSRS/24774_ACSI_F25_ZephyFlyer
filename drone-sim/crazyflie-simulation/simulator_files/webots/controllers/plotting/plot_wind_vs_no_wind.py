# -*- coding: utf-8 -*-
"""
Slide 2 — Hover with Step Wind Disturbance (RMSE Version)

Plots (0–30 s):
1) Position vs Time (x, y, z)
2) Tracking Error vs Time

Metrics:
- RMSE of 3D position tracking error over 0–30 s

Wind:
- OFF: 0–5 s
- ON : 5–20 s
- OFF: 20–30 s
"""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ======================================================
# ---------------- PATH CONFIG -------------------------
# ======================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = BASE_DIR

# ======================================================
# ---------------- CONTROLLER CONFIG ------------------
# ======================================================

CONTROLLERS = {
    "PID": {
        "file": "pid_hover_wind.csv",
        "color": "tab:blue",
        "linestyle": "-",
    },
    "PID+ESO": {
        "file": "pid_eso_hover_wind.csv",
        "color": "tab:blue",
        "linestyle": ":",
    },
    "LQR": {
        "file": "lqr_hover_wind.csv",
        "color": "tab:green",
        "linestyle": "-",
    },
    "LQR+ESO": {
        "file": "lqr_eso_hover_wind.csv",
        "color": "tab:green",
        "linestyle": ":",
    },
    "TinyMPC": {
        "file": "tinympc_hover_wind.csv",
        "color": "tab:purple",
        "linestyle": "-",
    },
}

PLOT_CONTROLLERS = {k: True for k in CONTROLLERS}

# ======================================================
# ---------------- TIME CONFIG -------------------------
# ======================================================

T_END = 30.0
WIND_ON_TIME = 5.0
WIND_OFF_TIME = 20.0

# ======================================================
# ---------------- HELPERS -----------------------------
# ======================================================

def safe_load(path):
    if not os.path.exists(path):
        print(f"[INFO] missing: {path}")
        return None
    return pd.read_csv(path)

def crop_time(df):
    return df[df["t"] <= T_END]

def shade_wind(ax):
    ax.axvspan(
        WIND_ON_TIME,
        WIND_OFF_TIME,
        color="gray",
        alpha=0.2,
        label="Wind ON"
    )

def plot_reference(ax, value):
    ax.axhline(
        value,
        color="black",
        linestyle="--",
        linewidth=1.5,
        alpha=0.8,
        label="Reference"
    )

def compute_rmse(df):
    err = np.sqrt(
        (df["px"] - df["ref_px"])**2 +
        (df["py"] - df["ref_py"])**2 +
        (df["pz"] - df["ref_pz"])**2
    )
    return np.sqrt(np.mean(err**2))


# ======================================================
# -------- FIGURE 1: POSITION vs TIME ------------------
# ======================================================

def plot_position_vs_time():
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    labels = ["x [m]", "y [m]", "z [m]"]

    rmse = {}
    shaded = False

    for name, cfg in CONTROLLERS.items():
        if not PLOT_CONTROLLERS[name]:
            continue

        df = safe_load(os.path.join(LOG_DIR, cfg["file"]))
        if df is None:
            continue

        df = crop_time(df)

        t = df["t"]
        c = cfg["color"]
        ls = cfg["linestyle"]

        axs[0].plot(t, df["px"], color=c, linestyle=ls, label=name)
        axs[1].plot(t, df["py"], color=c, linestyle=ls, label=name)
        axs[2].plot(t, df["pz"], color=c, linestyle=ls, label=name)

        rmse[name] = compute_rmse(df)

        if not shaded:
            for ax in axs:
                shade_wind(ax)
            shaded = True

    # Reference
    plot_reference(axs[0], 0.0)
    plot_reference(axs[1], 0.0)
    plot_reference(axs[2], 0.5)

    # Formatting
    for i, ax in enumerate(axs):
        ax.set_ylabel(labels[i])
        ax.grid(True)
        ax.set_xlim(0, T_END)

    axs[1].set_ylim(-0.6, 0.6)
    axs[-1].set_xlabel("Time [s]")
    axs[0].set_title("Hover with Step Wind Disturbance — Position Response")

    # ---- RMSE text box (top-right of x plot) ----
    text = "RMSE (0–30 s)\n"
    for k, v in rmse.items():
        text += f"{k}: {v:.3f} m\n"

    # axs[0].text(
    #     0.98, 0.98, text,
    #     transform=axs[0].transAxes,
    #     ha="left", va="top",
    #     fontsize=10,
    #     bbox=dict(facecolor="white", alpha=0.9, edgecolor="gray")
    # )

    # ---- Shared legend (right side) ----
    handles_dict = {}
    for ax in axs:
        h, l = ax.get_legend_handles_labels()
        for handle, label in zip(h, l):
            handles_dict[label] = handle

    fig.legend(
        handles_dict.values(),
        handles_dict.keys(),
        loc="upper left",
        bbox_to_anchor=(1.02, 0.5),
        frameon=True,
        title="Legend"
    )

    plt.tight_layout(rect=[0, 0, 0.82, 1])
    plt.show()

# ======================================================
# -------- FIGURE 2: TRACKING ERROR --------------------
# ======================================================

def plot_tracking_error():
    plt.figure(figsize=(11, 4))
    shaded = False

    for name, cfg in CONTROLLERS.items():
        if not PLOT_CONTROLLERS[name]:
            continue

        df = safe_load(os.path.join(LOG_DIR, cfg["file"]))
        if df is None:
            continue

        df = crop_time(df)
        t = df["t"]

        if "pos_error" in df.columns:
            err = df["pos_error"]
        else:
            err = np.sqrt(df["px"]**2 + df["py"]**2 + df["pz"]**2)

        plt.plot(
            t, err,
            color=cfg["color"],
            linestyle=cfg["linestyle"],
            label=name
        )

        if not shaded:
            shade_wind(plt.gca())
            shaded = True

    plt.xlim(0, T_END)
    plt.xlabel("Time [s]")
    plt.ylabel("Tracking Error [m]")
    plt.title("Hover with Step Wind Disturbance — Tracking Error")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

# ======================================================
# ---------------------- MAIN --------------------------
# ======================================================

if __name__ == "__main__":
    plot_position_vs_time()
    plot_tracking_error()
