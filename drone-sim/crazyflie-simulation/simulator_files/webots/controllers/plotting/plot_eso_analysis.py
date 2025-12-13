# -*- coding: utf-8 -*-
"""
Slide 4 — ESO Analysis

Figures:
1) ESO disturbance estimates (dx, dy, dz) — force-level
2) Total disturbance magnitude

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

FILES = {
    "PID+ESO": {
        "file": "pid_eso_hover_wind.csv",
        "color": "tab:blue",
        "linestyle": "-"
    },
    "LQR+ESO": {
        "file": "lqr_eso_hover_wind.csv",
        "color": "tab:green",
        "linestyle": "-"
    },
}

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

# ======================================================
# --------- FIGURE 1: ESO DISTURBANCES -----------------
# ======================================================

def plot_eso_disturbances():
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    labels = ["dist_x [N]", "dist_y [N]", "dist_z [N]"]

    shaded = False

    for name, cfg in FILES.items():
        df = safe_load(os.path.join(LOG_DIR, cfg["file"]))
        if df is None:
            continue

        if not {"d_fx_filt", "d_fy_filt", "d_fz_filt"}.issubset(df.columns):
            print(f"[WARN] Missing ESO disturbance columns for {name}")
            continue

        df = crop_time(df)
        t = df["t"]

        axs[0].plot(t, df["d_fx_filt"], label=name, color=cfg["color"], linestyle=cfg["linestyle"])
        axs[1].plot(t, df["d_fy_filt"], label=name, color=cfg["color"], linestyle=cfg["linestyle"])
        axs[2].plot(t, df["d_fz_filt"], label=name, color=cfg["color"], linestyle=cfg["linestyle"])

        if not shaded:
            for ax in axs:
                shade_wind(ax)
            shaded = True

    for i, ax in enumerate(axs):
        ax.set_ylabel(labels[i])
        ax.grid(True)
        ax.set_xlim(0, T_END)

    axs[-1].set_xlabel("Time [s]")
    axs[0].set_title("ESO Disturbance Estimates (Force-Level)")
    axs[1].set_ylim(-0.5, 0.5)
    # One shared legend on the right
    handles, labels = axs[0].get_legend_handles_labels()
    fig.legend(
        handles, labels,
        loc="center left",
        bbox_to_anchor=(1.02, 0.5),
        frameon=True
    )

    plt.tight_layout(rect=[0, 0, 0.85, 1])
    plt.show()

# ======================================================
# ------ FIGURE 2: TOTAL DISTURBANCE MAGNITUDE ---------
# ======================================================

def plot_total_disturbance():
    plt.figure(figsize=(10, 4))
    shaded = False

    for name, cfg in FILES.items():
        df = safe_load(os.path.join(LOG_DIR, cfg["file"]))
        if df is None:
            continue

        if not {"d_fx_filt", "d_fy_filt", "d_fz_filt"}.issubset(df.columns):
            continue

        df = crop_time(df)
        t = df["t"]

        mag = np.sqrt(
            df["d_fx_filt"]**2 +
            df["d_fy_filt"]**2 +
            df["d_fz_filt"]**2
        )

        plt.plot(
            t, mag,
            label=name,
            color=cfg["color"],
            linestyle=cfg["linestyle"]
        )

        mean_mag = mag.mean()
        plt.axhline(
            mean_mag,
            color=cfg["color"],
            linestyle="--",
            alpha=0.6
        )

        if not shaded:
            shade_wind(plt.gca())
            shaded = True

    plt.xlim(0, T_END)
    plt.xlabel("Time [s]")
    plt.ylabel("‖d‖ [N]")
    plt.title("Total Disturbance Magnitude (ESO)")

    plt.grid(True)
    plt.legend(loc="upper right")
    plt.tight_layout()
    plt.show()

# ======================================================
# ---------------------- MAIN --------------------------
# ======================================================

if __name__ == "__main__":
    plot_eso_disturbances()
    plot_total_disturbance()
