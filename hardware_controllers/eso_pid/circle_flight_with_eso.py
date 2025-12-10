"""
circle_flight_with_eso_v2.py
------------------------
Combines circle flight, Real-Time ESO, and Automatic Plotting.

New Features:
 1. Boolean 'USE_ESO_FOR_CONTROL' toggle.
 2. Live data collection in memory.
 3. Automatic Matplotlib popup after landing.
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

# --- ESO IMPORTS ---
# Ensure eso.py and design_L.py are in the same folder!
from eso import AttitudeESO
from design_L import compute_L

# -------------------------------------------------------------
# USER SETTINGS
# -------------------------------------------------------------
URI = 'radio://0/80/2M'

# --- THE TOGGLE YOU REQUESTED ---
USE_ESO_FOR_CONTROL = False  # Set TRUE to enable ESO feedback logic (Placeholder)

LOG_PERIOD_MS = 10   # 10ms = 100 Hz 
RADIUS = 0.5         # Circle radius [m]
HEIGHT = 0.5         # Flight height [m]
CIRCLE_TIME = 10.0   # Circle duration [s]

MASS_KG = 0.033      # Approx mass of CF2 + Flowdeck
GRAVITY = 9.81

# -------------------------------------------------------------
# HELPER FUNCTIONS
# -------------------------------------------------------------
def pwm_to_force(pwm):
    """Maps Crazyflie uint16 thrust (0-65535) to Newtons."""
    pwm_norm = pwm / 65535.0
    # Simple quadratic model: Max thrust ~60g
    thrust_g = 60.0 * (pwm_norm ** 2)
    return (thrust_g / 1000.0) * GRAVITY

# -------------------------------------------------------------
# ESO LOGGER CLASS
# -------------------------------------------------------------
class ESOLogger:
    def __init__(self, cf, filename):
        self.cf = cf
        self.filename = filename
        self.file = open(filename, "w")
        self.is_running = False
        
        # We store data in RAM for plotting later
        # Format: {'t':[], 'x':[], 'eso_x':[], ...}
        self.history = {
            't': [], 
            'x': [], 'y': [], 'z': [],
            'roll': [], 'pitch': [], 'yaw': [],
            'eso_x': [], 'eso_y': [], 'eso_z': [],
            'eso_roll': [], 'eso_pitch': [], 'eso_yaw': [],
            'dist_x': [], 'dist_y': [], 'dist_z': []
        }
        
        # Header
        header = "t\tx\ty\tz\troll\tpitch\tyaw\tg_x\tg_y\tg_z\tthrust\teso_x\teso_y\teso_z\tdist_x\tdist_y\tdist_z\n"
        self.file.write(header)
        
        self.start_time = None
        
        # Buffer for inputs
        self.latest_inputs = {'g_x': 0.0, 'g_y': 0.0, 'g_z': 0.0, 'thrust': 0.0}

        # --- Initialize ESO ---
        print("Initializing ESO Model...")
        ts_seconds = LOG_PERIOD_MS / 1000.0
        self.eso = AttitudeESO(Ts=ts_seconds, L=np.zeros((12,6)), m=MASS_KG, g=GRAVITY)
        
        print("Computing ESO Gains...")
        try:
            L_discrete = compute_L(self.eso, MASS_KG)
            self.eso.L = L_discrete
            print("ESO Gains Ready.")
        except Exception as e:
            print(f"ESO Gain Error: {e}")
            self.eso.L = np.zeros((12,6))

        # --- Logging Config ---
        # 1. Inputs
        self.log_conf_inputs = LogConfig(name='Inputs', period_in_ms=LOG_PERIOD_MS)
        self.log_conf_inputs.add_variable('gyro.x', 'float')
        self.log_conf_inputs.add_variable('gyro.y', 'float')
        self.log_conf_inputs.add_variable('gyro.z', 'float')
        self.log_conf_inputs.add_variable('stabilizer.thrust', 'float') 
        self.cf.log.add_config(self.log_conf_inputs)
        self.log_conf_inputs.data_received_cb.add_callback(self._cb_inputs)

        # 2. States
        self.log_conf_states = LogConfig(name='States', period_in_ms=LOG_PERIOD_MS)
        self.log_conf_states.add_variable('stateEstimate.x', 'float')
        self.log_conf_states.add_variable('stateEstimate.y', 'float')
        self.log_conf_states.add_variable('stateEstimate.z', 'float')
        self.log_conf_states.add_variable('stateEstimate.roll', 'float')
        self.log_conf_states.add_variable('stateEstimate.pitch', 'float')
        self.log_conf_states.add_variable('stateEstimate.yaw', 'float')
        self.cf.log.add_config(self.log_conf_states)
        self.log_conf_states.data_received_cb.add_callback(self._cb_states_and_process)

    def start(self):
        self.is_running = True
        self.log_conf_inputs.start()
        self.log_conf_states.start()
        self.start_time = time.time()

    def stop(self):
        self.is_running = False
        self.log_conf_inputs.stop()
        self.log_conf_states.stop()
        time.sleep(0.2)
        if not self.file.closed:
            self.file.flush()
            self.file.close()

    def _cb_inputs(self, timestamp, data, logconf):
        if not self.is_running: return
        self.latest_inputs['g_x'] = data['gyro.x']
        self.latest_inputs['g_y'] = data['gyro.y']
        self.latest_inputs['g_z'] = data['gyro.z']
        self.latest_inputs['thrust'] = data['stabilizer.thrust']

    def _cb_states_and_process(self, timestamp, data, logconf):
        if not self.is_running or self.file.closed: return
        if self.start_time is None: return
        
        t = time.time() - self.start_time
        
        # --- A. Get Measurements ---
        x, y, z = data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']
        roll, pitch, yaw = data['stateEstimate.roll'], data['stateEstimate.pitch'], data['stateEstimate.yaw']
        
        y_meas = np.array([x, y, z, np.radians(roll), np.radians(pitch), np.radians(yaw)])
        
        # --- B. Get Inputs ---
        omega_body = np.array([
            np.radians(self.latest_inputs['g_x']),
            np.radians(self.latest_inputs['g_y']),
            np.radians(self.latest_inputs['g_z'])
        ])
        u_thrust = pwm_to_force(self.latest_inputs['thrust'])

        # --- C. Run ESO ---
        if np.all(self.eso.z == 0):
            self.eso.initialize_from_measurement(y_meas)
        
        z_est = self.eso.step(y_meas, u_thrust, omega_body)
        
        # Unpack ESO states
        eso_x, eso_y, eso_z = z_est[0], z_est[1], z_est[2]
        eso_roll = np.degrees(z_est[6])
        eso_pitch = np.degrees(z_est[7])
        eso_yaw = np.degrees(z_est[8])
        dist_x, dist_y, dist_z = z_est[9], z_est[10], z_est[11]

        # --- D. LOGGING vs CONTROL ---
        if USE_ESO_FOR_CONTROL:
            # Placeholder: This is where you would calculate PID error using ESO states
            # e.g., error_x = setpoint_x - eso_x
            pass 
        
        # Print
        print(f"t={t:5.2f} | Z_cf={z:+.2f} Z_eso={eso_z:+.2f} | Ctrl={USE_ESO_FOR_CONTROL}", end="\r")

        # Save to RAM (for plotting)
        self.history['t'].append(t)
        self.history['x'].append(x); self.history['y'].append(y); self.history['z'].append(z)
        self.history['roll'].append(roll); self.history['pitch'].append(pitch); self.history['yaw'].append(yaw)
        
        self.history['eso_x'].append(eso_x); self.history['eso_y'].append(eso_y); self.history['eso_z'].append(eso_z)
        self.history['eso_roll'].append(eso_roll); self.history['eso_pitch'].append(eso_pitch); self.history['eso_yaw'].append(eso_yaw)
        self.history['dist_x'].append(dist_x); self.history['dist_y'].append(dist_y); self.history['dist_z'].append(dist_z)

        # Save to File
        row = (
            f"{t:.3f}\t{x:.3f}\t{y:.3f}\t{z:.3f}\t{roll:.2f}\t{pitch:.2f}\t{yaw:.2f}\t"
            f"{self.latest_inputs['g_x']:.2f}\t{self.latest_inputs['g_y']:.2f}\t{self.latest_inputs['g_z']:.2f}\t"
            f"{self.latest_inputs['thrust']:.0f}\t"
            f"{eso_x:.3f}\t{eso_y:.3f}\t{eso_z:.3f}\t"
            f"{dist_x:.3f}\t{dist_y:.3f}\t{dist_z:.3f}\n"
        )
        try:
            if not self.file.closed:
                self.file.write(row)
        except ValueError:
            pass

# -------------------------------------------------------------
# PLOTTING FUNCTION
# -------------------------------------------------------------
def plot_results(history):
    """Generates plots from the recorded history dictionary."""
    print("\nGenerating Plots...")
    t = history['t']
    
    # FIG 1: XYZ Position
    fig1, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig1.suptitle('Position: Drone Internal vs ESO')
    
    axs[0].plot(t, history['x'], 'k--', label='CF Internal')
    axs[0].plot(t, history['eso_x'], 'b', label='ESO')
    axs[0].set_ylabel('X (m)'); axs[0].legend(); axs[0].grid(True)
    
    axs[1].plot(t, history['y'], 'k--', label='CF Internal')
    axs[1].plot(t, history['eso_y'], 'b', label='ESO')
    axs[1].set_ylabel('Y (m)'); axs[1].grid(True)
    
    axs[2].plot(t, history['z'], 'k--', label='CF Internal')
    axs[2].plot(t, history['eso_z'], 'b', label='ESO')
    axs[2].set_ylabel('Z (m)'); axs[2].grid(True)
    axs[2].set_xlabel('Time (s)')

    # FIG 2: Attitude (Roll/Pitch/Yaw)
    fig2, axs2 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig2.suptitle('Attitude: Drone Internal vs ESO')

    axs2[0].plot(t, history['roll'], 'k--', label='CF Internal')
    axs2[0].plot(t, history['eso_roll'], 'b', label='ESO')
    axs2[0].set_ylabel('Roll (deg)'); axs2[0].legend(); axs2[0].grid(True)
    
    axs2[1].plot(t, history['pitch'], 'k--', label='CF Internal')
    axs2[1].plot(t, history['eso_pitch'], 'b', label='ESO')
    axs2[1].set_ylabel('Pitch (deg)'); axs2[1].grid(True)
    
    axs2[2].plot(t, history['yaw'], 'k--', label='CF Internal')
    axs2[2].plot(t, history['eso_yaw'], 'b', label='ESO')
    axs2[2].set_ylabel('Yaw (deg)'); axs2[2].grid(True)
    axs2[2].set_xlabel('Time (s)')

    # Show
    plt.show()

# -------------------------------------------------------------
# FLIGHT ROUTINE
# -------------------------------------------------------------
def fly_circle(scf, radius=RADIUS, height=HEIGHT, circle_time=CIRCLE_TIME, filename=None):
    cf = scf.cf
    logger = ESOLogger(cf, filename)

    print("------------------------------------------------------")
    print(f" CONTROL MODE: {'ESO FEEDBACK' if USE_ESO_FOR_CONTROL else 'ONBOARD DEFAULT'} ")
    print("------------------------------------------------------")

    logger.start()
    
    try:
        with MotionCommander(scf, default_height=height) as mc:
            print(f"Taking off to {height:.2f} m...")
            time.sleep(2.0)

            print(f"Flying circle...")
            steps = 60
            angle_per_step = 2 * math.pi / steps
            step_time = circle_time / steps

            for i in range(steps):
                angle = i * angle_per_step
                vx = -math.sin(angle) * radius * (2 * math.pi / circle_time)
                vy =  math.cos(angle) * radius * (2 * math.pi / circle_time)
                
                # NOTE: If USE_ESO_FOR_CONTROL were True and we had a python controller,
                # we would calculate setpoints here instead of using MotionCommander
                mc.start_linear_motion(vx, vy, 0)
                time.sleep(step_time)

            mc.stop()
            print("\nCircle complete. Hovering briefly...")
            time.sleep(1.5)

            print("Landing...")
            
    finally:
        logger.stop()
        print(f"\nLog saved to: {os.path.basename(filename)}")
        
        # Trigger Plotting
        if len(logger.history['t']) > 10:
            plot_results(logger.history)
        else:
            print("Not enough data to plot.")

# -------------------------------------------------------------
# MAIN SCRIPT
# -------------------------------------------------------------
if __name__ == '__main__':
    print("======================================================")
    print(" Crazyflie Circle Flight + Real-Time ESO + Plotting ")
    print("======================================================")

    cflib.crtp.init_drivers(enable_debug_driver=False)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(script_dir, f"circle_eso_log_{timestamp}.txt")

    print(f"Connecting to {URI}...")
    
    retries = 2
    for attempt in range(retries + 1):
        try:
            with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
                print("Connection established.")
                fly_circle(scf, filename=filename)
                print("Flight complete.")
                break 
        except Exception as e:
            print(f"Connection attempt {attempt+1} failed: {e}")
            if attempt < retries:
                time.sleep(2)

    print("Disconnected.")