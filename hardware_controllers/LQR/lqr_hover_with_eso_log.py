"""
hover_with_lqr_eso_log.py
-------------------------
Hover Crazyflie at 0.5 m using LQR controller while streaming ESO disturbance states.
Includes automatic plotting after flight.

Logs:
 - x, y, z
 - roll/pitch/yaw
 - eso disturbances (force estimates)

Requires firmware with LOG_GROUP "eso" and LQR controller registered.
"""

import logging
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
import subprocess

# -------------------------------------------------------------
# USER SETTINGS
# -------------------------------------------------------------
URI = 'radio://0/80/2M'
LOG_PERIOD_MS = 100      # 10 Hz
HOVER_HEIGHT = 0.5        # meters
HOVER_TIME = 10.0         # seconds
LQR_CONTROLLER_ID = 6     # Controller ID for LQR (check your firmware)

logging.basicConfig(level=logging.ERROR)


def setup_logging(cf, filename):
    """Configure logging for stateEstimate + ESO output."""
    
    # Config 1: State variables
    logconf_state = LogConfig(name='StateLog', period_in_ms=LOG_PERIOD_MS)
    logconf_state.add_variable('stateEstimate.x', 'float')
    logconf_state.add_variable('stateEstimate.y', 'float')
    logconf_state.add_variable('stateEstimate.z', 'float')
    logconf_state.add_variable('stateEstimate.roll',  'float')
    logconf_state.add_variable('stateEstimate.pitch', 'float')
    logconf_state.add_variable('stateEstimate.yaw',   'float')
    
    # Config 2: ESO variables - DON'T START YET
    logconf_eso = LogConfig(name='ESOLog', period_in_ms=LOG_PERIOD_MS)
    logconf_eso.add_variable('eso.x', 'int32_t')
    logconf_eso.add_variable('eso.y', 'int32_t')
    logconf_eso.add_variable('eso.z', 'int32_t')
    logconf_eso.add_variable('eso.t', 'int32_t')

    cf.log.add_config(logconf_state)
    cf.log.add_config(logconf_eso)

    file = open(filename, "w")
    file.write("# t\tx\ty\tz\troll\tpitch\tyaw\tdax\tday\tdaz\tdthrust\n")

    start_time = time.time()
    
    # Storage for latest values from both configs
    latest_data = {
        'x': 0, 'y': 0, 'z': 0,
        'roll': 0, 'pitch': 0, 'yaw': 0,
        'dax': 0, 'day': 0, 'daz': 0, 'dth': 0
    }
    
    # History buffer for plotting
    history = {
        't': [], 'x': [], 'y': [], 'z': [],
        'roll': [], 'pitch': [], 'yaw': [],
        'dax': [], 'day': [], 'daz': [], 'dth': []
    }

    def state_callback(timestamp, data, logconf):
        t = time.time() - start_time
        latest_data['x'] = data['stateEstimate.x']
        latest_data['y'] = data['stateEstimate.y']
        latest_data['z'] = data['stateEstimate.z']
        latest_data['roll'] = data['stateEstimate.roll']
        latest_data['pitch'] = data['stateEstimate.pitch']
        latest_data['yaw'] = data['stateEstimate.yaw']
        
        # Print state without ESO for now
        print(f"t={t:5.2f}s | x={latest_data['x']:+.2f} y={latest_data['y']:+.2f} z={latest_data['z']:+.2f}", end="\r")

    def eso_callback(timestamp, data, logconf):
        t = time.time() - start_time
        
        latest_data['dax'] = data['eso.x'] / 1000.0
        latest_data['day'] = data['eso.y'] / 1000.0
        latest_data['daz'] = data['eso.z'] / 1000.0
        latest_data['dth'] = data['eso.t'] / 1000.0

        print(
            f"t={t:5.2f}s | z={latest_data['z']:+.2f}m | "
            f"ESO(ax={latest_data['dax']:+.2f}, ay={latest_data['day']:+.2f}, "
            f"az={latest_data['daz']:+.2f}, Fz={latest_data['dth']:+.3f})",
            end="\r"
        )

        # Store to history
        history['t'].append(t)
        history['x'].append(latest_data['x'])
        history['y'].append(latest_data['y'])
        history['z'].append(latest_data['z'])
        history['roll'].append(latest_data['roll'])
        history['pitch'].append(latest_data['pitch'])
        history['yaw'].append(latest_data['yaw'])
        history['dax'].append(latest_data['dax'])
        history['day'].append(latest_data['day'])
        history['daz'].append(latest_data['daz'])
        history['dth'].append(latest_data['dth'])

        # Write to file
        file.write(
            f"{t:.3f}\t{latest_data['x']:.3f}\t{latest_data['y']:.3f}\t{latest_data['z']:.3f}\t"
            f"{latest_data['roll']:.2f}\t{latest_data['pitch']:.2f}\t{latest_data['yaw']:.2f}\t"
            f"{latest_data['dax']:.4f}\t{latest_data['day']:.4f}\t{latest_data['daz']:.4f}\t"
            f"{latest_data['dth']:.4f}\n"
        )

    logconf_state.data_received_cb.add_callback(state_callback)
    logconf_eso.data_received_cb.add_callback(eso_callback)
    
    return logconf_state, logconf_eso, file, history


def hover(scf, filename):
    """Hover at a fixed height using LQR controller while logging ESO output."""
    cf = scf.cf
    
    # Wait for params to sync
    print("Waiting for parameters to sync...")
    time.sleep(1.0)
    
    # Switch to LQR controller
    print(f"\nSwitching to LQR controller (ID={LQR_CONTROLLER_ID})...")
    cf.param.set_value('stabilizer.controller', str(LQR_CONTROLLER_ID))
    time.sleep(0.5)  # Give it time to switch
    
    # Verify controller
    controller_type = cf.param.get_value('stabilizer.controller')
    print(f"Active controller: {controller_type} (6=LQR)")
    
    if int(controller_type) != LQR_CONTROLLER_ID:
        print(f"WARNING: Controller did not switch! Still at {controller_type}")
        print("Continuing anyway, but results may be unexpected...")
    
    logconf_state, logconf_eso, logfile, history = setup_logging(cf, filename)
    
    # Start state logging only
    logconf_state.start()

    print("\n------------------------------------------------------")
    print("  LQR HOVER TEST START — ESO LOGGING DELAYED")
    print("------------------------------------------------------")

    try:
        with MotionCommander(scf, default_height=HOVER_HEIGHT) as mc:
            print(f"Taking off to {HOVER_HEIGHT:.2f} m...")
            time.sleep(3.0)
            
            print("\n>>> STARTING ESO LOGGING NOW <<<")
            logconf_eso.start()
            time.sleep(0.5)

            print(f"Hovering for {HOVER_TIME:.1f} seconds with LQR controller...")
            time.sleep(HOVER_TIME)

            print("\nLanding...")
            time.sleep(1.0)

    finally:
        logconf_state.stop()
        logconf_eso.stop()
        time.sleep(0.2)
        logfile.close()

        print("\nLog saved to:", filename)
        print("LQR hover test complete.\n")
        
        # Generate plots automatically
        if len(history['t']) > 10:
            print("Generating plots...")
            try:
                # Find plotter.py
                plotter_script = os.path.join(os.path.dirname(__file__), 'plotter.py')
                if not os.path.exists(plotter_script):
                    plotter_script = 'plotter.py'  # Try current directory
                
                # Call the plotter
                result = subprocess.run(['python3', plotter_script, filename], 
                                       capture_output=True, text=True)
                if result.returncode == 0:
                    print("✅ Plots generated successfully!")
                else:
                    print(f"Warning: Plotter failed: {result.stderr}")
            except Exception as e:
                print(f"Warning: Could not generate plots: {e}")
                print(f"You can manually run: python3 plotter.py {filename}")
        else:
            print("Not enough data for plotting")


# -------------------------------------------------------------
# MAIN ENTRY
# -------------------------------------------------------------
if __name__ == '__main__':
    print("======================================================")
    print(" Crazyflie LQR Hover + ESO Live Logging + Plotting")
    print("======================================================")

    cflib.crtp.init_drivers(enable_debug_driver=False)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(os.path.expanduser("~/Downloads"),
                            f"hover_lqr_eso_log_{timestamp}.txt")

    print(f"Connecting to Crazyflie at URI: {URI}")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        try:
            print("Connection established.")
            hover(scf, filename)
        except Exception as e:
            print("ERROR:", e)

    print("Disconnected.")
    print("======================================================")