"""
circle_flight_with_eso_v3.py
---------------------------------------------------------------
Hardware circle-flight script with:
  - Real-time ESO state estimation
  - Optional ESO-based state substitution (for LOGGING ONLY)
  - Optional force-feedback estimation (PRINT ONLY)
  - Automatic plotting after flight

IMPORTANT:
  - Trajectory and control are handled entirely by MotionCommander.
  - ESO NEVER sends commands to the drone here.
"""

import logging
import math
import time
import os
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

# ==============================================================#
# ESO IMPORTS                                                   #
# ==============================================================#
from eso import AttitudeESO
from design_L import compute_L

# ==============================================================#
# USER SETTINGS (TOGGLES)                                       #
# ==============================================================#
URI = 'radio://0/80/2M'

# NOTE: MotionCommander runs onboard PID; Do NOT override it here.
USE_ESO_FOR_CONTROL = False   # kept for future custom controllers
USE_ESO_STATES      = True   # if True: log/plot ESO pos instead of CF state
USE_FORCE_FEEDBACK  = False   # if True: print ESO disturbance forces in body frame

LOG_PERIOD_MS = 10            # 100 Hz logging
RADIUS = 0.5                  # circle radius (m)
HEIGHT = 0.5                  # flight height (m)
CIRCLE_TIME = 10.0            # one full circle duration (s)

MASS_KG = 0.033               # CF2+Flow
GRAVITY = 9.81


# ==============================================================#
# HELPERS                                                       #
# ==============================================================#
def pwm_to_force(pwm):
    """Maps Crazyflie uint16 thrust (0-65535) to Newtons (very rough model)."""
    pwm_norm = pwm / 65535.0
    thrust_g = 60.0 * (pwm_norm ** 2)          # ~60 g max
    return (thrust_g / 1000.0) * GRAVITY       # convert g → kg → N


# ==============================================================#
# ESO LOGGER                                                    #
# ==============================================================#
class ESOLogger:
    def __init__(self, cf, filename):
        self.cf = cf
        self.filename = filename
        self.file = open(filename, "w")
        self.is_running = False

        # History buffer (for plotting later)
        self.history = {
            't': [],
            'x': [], 'y': [], 'z': [],
            'roll': [], 'pitch': [], 'yaw': [],
            'eso_x': [], 'eso_y': [], 'eso_z': [],
            'eso_roll': [], 'eso_pitch': [], 'eso_yaw': [],
            'dist_x': [], 'dist_y': [], 'dist_z': []
        }

        header = (
            "t\tx\ty\tz\troll\tpitch\tyaw\t"
            "g_x\tg_y\tg_z\tthrust\t"
            "eso_x\teso_y\teso_z\tdist_x\tdist_y\tdist_z\n"
        )
        self.file.write(header)

        self.start_time = None
        self.latest_inputs = {'g_x': 0.0, 'g_y': 0.0, 'g_z': 0.0, 'thrust': 0.0}

        # ------------------------------------------------------#
        # ESO INITIALIZATION                                    #
        # ------------------------------------------------------#
        print("Initializing ESO...")
        Ts = LOG_PERIOD_MS / 1000.0
        self.eso = AttitudeESO(Ts, np.zeros((12, 6)), MASS_KG, GRAVITY)

        print("Computing ESO gains...")
        try:
            L = compute_L(self.eso, MASS_KG)
            self.eso.L = L
            print("ESO gains OK.")
        except Exception as e:
            print("ESO gain failure:", e)
            self.eso.L = np.zeros((12, 6))

        # ------------------------------------------------------#
        # LOG BLOCKS                                            #
        # ------------------------------------------------------#
        # Inputs
        self.log_conf_inputs = LogConfig("Inputs", LOG_PERIOD_MS)
        self.log_conf_inputs.add_variable('gyro.x', 'float')
        self.log_conf_inputs.add_variable('gyro.y', 'float')
        self.log_conf_inputs.add_variable('gyro.z', 'float')
        self.log_conf_inputs.add_variable('stabilizer.thrust', 'float')
        cf.log.add_config(self.log_conf_inputs)
        self.log_conf_inputs.data_received_cb.add_callback(self._cb_inputs)

        # States
        self.log_conf_states = LogConfig("States", LOG_PERIOD_MS)
        self.log_conf_states.add_variable('stateEstimate.x', 'float')
        self.log_conf_states.add_variable('stateEstimate.y', 'float')
        self.log_conf_states.add_variable('stateEstimate.z', 'float')
        self.log_conf_states.add_variable('stateEstimate.roll', 'float')
        self.log_conf_states.add_variable('stateEstimate.pitch', 'float')
        self.log_conf_states.add_variable('stateEstimate.yaw', 'float')
        cf.log.add_config(self.log_conf_states)
        self.log_conf_states.data_received_cb.add_callback(
            self._cb_states_and_process
        )

    # ----------------------------------------------------------#
    def start(self):
        self.is_running = True
        self.log_conf_inputs.start()
        self.log_conf_states.start()
        self.start_time = time.time()

    # ----------------------------------------------------------#
    def stop(self):
        self.is_running = False
        self.log_conf_inputs.stop()
        self.log_conf_states.stop()
        time.sleep(0.2)
        if not self.file.closed:
            self.file.flush()
            self.file.close()

    # ----------------------------------------------------------#
    def _cb_inputs(self, ts, data, logconf):
        if not self.is_running:
            return
        self.latest_inputs['g_x'] = data['gyro.x']
        self.latest_inputs['g_y'] = data['gyro.y']
        self.latest_inputs['g_z'] = data['gyro.z']
        self.latest_inputs['thrust'] = data['stabilizer.thrust']

    # ----------------------------------------------------------#
    def _cb_states_and_process(self, ts, data, logconf):
        if not self.is_running or self.start_time is None:
            return

        # Time
        t = time.time() - self.start_time

        # ------------------------------------------------------#
        # RAW CF STATES                                         #
        # ------------------------------------------------------#
        x_cf = data['stateEstimate.x']
        y_cf = data['stateEstimate.y']
        z_cf = data['stateEstimate.z']
        roll_cf = data['stateEstimate.roll']
        pitch_cf = data['stateEstimate.pitch']
        yaw_cf = data['stateEstimate.yaw']

        y_meas = np.array([
            x_cf, y_cf, z_cf,
            math.radians(roll_cf),
            math.radians(pitch_cf),
            math.radians(yaw_cf)
        ])

        # ------------------------------------------------------#
        # INPUTS                                                #
        # ------------------------------------------------------#
        omega_body = np.radians([
            self.latest_inputs['g_x'],
            self.latest_inputs['g_y'],
            self.latest_inputs['g_z']
        ])
        u_thrust = pwm_to_force(self.latest_inputs['thrust'])

        # ------------------------------------------------------#
        # ESO UPDATE                                            #
        # ------------------------------------------------------#
        if np.all(self.eso.z == 0):
            self.eso.initialize_from_measurement(y_meas)

        z_hat = self.eso.step(y_meas, u_thrust, omega_body)

        eso_x, eso_y, eso_z = z_hat[0], z_hat[1], z_hat[2]
        eso_roll = math.degrees(z_hat[6])
        eso_pitch = math.degrees(z_hat[7])
        eso_yaw = math.degrees(z_hat[8])
        dist_x, dist_y, dist_z = z_hat[9], z_hat[10], z_hat[11]

        # ------------------------------------------------------#
        # OPTIONAL FORCE FEEDBACK (PRINT ONLY, NO CONTROL)      #
        # ------------------------------------------------------#
        if USE_FORCE_FEEDBACK:
            rr = math.radians(roll_cf)
            pr = math.radians(pitch_cf)
            yr = math.radians(yaw_cf)

            cr, sr = math.cos(rr), math.sin(rr)
            cp, sp = math.cos(pr), math.sin(pr)
            cy, sy = math.cos(yr), math.sin(yr)

            R_wb = np.array([
                [cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr],
                [sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr],
                [-sp,      cp * sr,                cp * cr]
            ])

            Fx_b, Fy_b, Fz_b = R_wb.T @ np.array([dist_x, dist_y, dist_z])

            print(
                f"\n[FORCE FEEDBACK]\n"
                f"Body disturbance: Fx={Fx_b:+.3f} Fy={Fy_b:+.3f} Fz={Fz_b:+.3f}\n"
            )

        # ------------------------------------------------------#
        # LOGGING STATES (ESO vs CF) — LOGGING ONLY             #
        # ------------------------------------------------------#
        if USE_ESO_STATES:
            x_log, y_log, z_log = eso_x, eso_y, eso_z
            roll_log, pitch_log, yaw_log = eso_roll, eso_pitch, eso_yaw
        else:
            x_log, y_log, z_log = x_cf, y_cf, z_cf
            roll_log, pitch_log, yaw_log = roll_cf, pitch_cf, yaw_cf

        # Save in-memory history
        self.history['t'].append(t)

        self.history['x'].append(x_log)
        self.history['y'].append(y_log)
        self.history['z'].append(z_log)

        self.history['roll'].append(roll_log)
        self.history['pitch'].append(pitch_log)
        self.history['yaw'].append(yaw_log)

        self.history['eso_x'].append(eso_x)
        self.history['eso_y'].append(eso_y)
        self.history['eso_z'].append(eso_z)

        self.history['eso_roll'].append(eso_roll)
        self.history['eso_pitch'].append(eso_pitch)
        self.history['eso_yaw'].append(eso_yaw)

        self.history['dist_x'].append(dist_x)
        self.history['dist_y'].append(dist_y)
        self.history['dist_z'].append(dist_z)

        # Save to file (raw CF + ESO)
        self.file.write(
            f"{t:.3f}\t"
            f"{x_cf:.3f}\t{y_cf:.3f}\t{z_cf:.3f}\t"
            f"{roll_cf:.2f}\t{pitch_cf:.2f}\t{yaw_cf:.2f}\t"
            f"{self.latest_inputs['g_x']:.2f}\t"
            f"{self.latest_inputs['g_y']:.2f}\t"
            f"{self.latest_inputs['g_z']:.2f}\t"
            f"{self.latest_inputs['thrust']:.0f}\t"
            f"{eso_x:.3f}\t{eso_y:.3f}\t{eso_z:.3f}\t"
            f"{dist_x:.3f}\t{dist_y:.3f}\t{dist_z:.3f}\n"
        )

        # Simple terminal status (height tracking)
        print(
            f"t={t:5.2f} | z_cf={z_cf:+.2f} z_eso={eso_z:+.2f} | "
            f"USE_ESO_STATES={USE_ESO_STATES}",
            end="\r"
        )


# ==============================================================#
# PLOTTING                                                      #
# ==============================================================#
def plot_results(H):
    print("\nPlotting results...")

    t = H['t']

    # Position
    fig1, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    if USE_ESO_STATES:
        fig1.suptitle("Position (LOGGING using ESO) vs ESO Estimate")
    else:
        fig1.suptitle("Position (LOGGING using CF states) vs ESO Estimate")

    axs[0].plot(t, H['x'], 'k--', label="Logged position")
    axs[0].plot(t, H['eso_x'], 'b', label="ESO x")
    axs[0].set_ylabel("X (m)")
    axs[0].grid(True)
    axs[0].legend()

    axs[1].plot(t, H['y'], 'k--')
    axs[1].plot(t, H['eso_y'], 'b')
    axs[1].set_ylabel("Y (m)")
    axs[1].grid(True)

    axs[2].plot(t, H['z'], 'k--')
    axs[2].plot(t, H['eso_z'], 'b')
    axs[2].set_ylabel("Z (m)")
    axs[2].set_xlabel("Time (s)")
    axs[2].grid(True)

    plt.show()


# =============================================================
# FLIGHT ROUTINE (v3 FEATURES + v2 TRAJECTORY CONTROL)
# =============================================================
def fly_circle(scf, radius=RADIUS, height=HEIGHT, circle_time=CIRCLE_TIME, filename=None):
    cf = scf.cf
    logger = ESOLogger(cf, filename)

    print("\n===================================================")
    print(f" USE_ESO_STATES      = {USE_ESO_STATES}")
    print(f" USE_ESO_FOR_CONTROL = {USE_ESO_FOR_CONTROL}")
    print(f" USE_FORCE_FEEDBACK  = {USE_FORCE_FEEDBACK}")
    print("===================================================\n")

    logger.start()

    try:
        with MotionCommander(scf, default_height=height) as mc:

            print(f"Taking off to {height} m...")
            time.sleep(2.0)

            print("Flying circle... (v2 trajectory restored)")

            # === RESTORE ORIGINAL v2 CONTROL LOOP ===
            steps = 60
            angle_per_step = 2 * math.pi / steps
            step_time = circle_time / steps
            circle_rate = 2 * math.pi / circle_time

            for i in range(steps):
                angle = i * angle_per_step

                # === EXACT ORIGINAL v2 VELOCITY COMMANDS ===
                vx = -math.sin(angle) * radius * circle_rate
                vy =  math.cos(angle) * radius * circle_rate

                mc.start_linear_motion(vx, vy, 0)
                time.sleep(step_time)

            mc.stop()
            print("\nCircle finished. Hovering...")
            time.sleep(1.5)

            print("Landing...")

    finally:
        logger.stop()
        print("Log saved:", os.path.basename(filename))

        if len(logger.history['t']) > 10:
            plot_results(logger.history)



# ==============================================================#
# MAIN                                                          #
# ==============================================================#
if __name__ == "__main__":
    print("===================================================")
    print(" Crazyflie Circle Flight + ESO Logging + Plotting ")
    print("===================================================\n")

    cflib.crtp.init_drivers(enable_debug_driver=False)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(script_dir, f"circle_eso_log_{timestamp}.txt")

    print(f"Connecting to {URI}...")

    for attempt in range(3):
        try:
            with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
                print("Connected.")
                fly_circle(scf, filename=filename)
                print("Flight complete.")
                break
        except Exception as e:
            print(f"Connection attempt {attempt+1} failed: {e}")
            time.sleep(2.0)

    print("Disconnected.")
