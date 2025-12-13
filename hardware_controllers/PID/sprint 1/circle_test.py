"""
circle_flight_log_txt.py
------------------------
Crazyflie 2.x circular flight with live logging and file output

Features:
 - Flies a circular path (radius 0.5 m)
 - Logs x, y, z, roll, pitch, yaw in real time
 - Prints telemetry in terminal
 - Automatically saves a .txt log in your Downloads folder
 - PID tuning block included but commented out

Requirements:
 - Crazyflie 2.x with Flow deck
 - Crazyradio PA dongle
"""

import logging
import math
import time
import os
from datetime import datetime
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

# -------------------------------------------------------------
# USER SETTINGS
# -------------------------------------------------------------
URI = 'radio://0/80/2M'
LOG_PERIOD_MS = 100  # 10 Hz logging rate
RADIUS = 0.5         # Circle radius [m]
HEIGHT = 0.5         # Flight height [m]
CIRCLE_TIME = 10.0    # Circle duration [s]

# -------------------------------------------------------------
# INITIALIZATION
# -------------------------------------------------------------
logging.basicConfig(level=logging.ERROR)


def set_pid_params(cf):
    """Set PID parameters (disabled by default)."""
    print("\n------------------------------------------------------")
    print(" PID SETUP (disabled) ")
    print("------------------------------------------------------")
    print("PID tuning skipped — running with firmware default parameters.")
    print("------------------------------------------------------\n")


def setup_logging(cf, filename):
    """Configure onboard logging for position and attitude."""
    logconf = LogConfig(name='StateEstimate', period_in_ms=LOG_PERIOD_MS)
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')
    logconf.add_variable('stateEstimate.roll', 'float')
    logconf.add_variable('stateEstimate.pitch', 'float')
    logconf.add_variable('stateEstimate.yaw', 'float')
    cf.log.add_config(logconf)

    file = open(filename, "w")
    file.write("# time(s)\tx(m)\ty(m)\tz(m)\troll(deg)\tpitch(deg)\tyaw(deg)\n")

    start_time = time.time()

    def log_callback(timestamp, data, logconf):
        t = time.time() - start_time
        x, y, z = data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']
        roll, pitch, yaw = data['stateEstimate.roll'], data['stateEstimate.pitch'], data['stateEstimate.yaw']

        # Print rolling telemetry to console
        print(f"t={t:5.2f}s | x={x:+.2f} y={y:+.2f} z={z:+.2f} | roll={roll:+.1f} pitch={pitch:+.1f} yaw={yaw:+.1f}", end="\r")

        # Write to file
        file.write(f"{t:.3f}\t{x:.3f}\t{y:.3f}\t{z:.3f}\t{roll:.2f}\t{pitch:.2f}\t{yaw:.2f}\n")

    logconf.data_received_cb.add_callback(log_callback)
    return logconf, file


def fly_circle(scf, radius=RADIUS, height=HEIGHT, circle_time=CIRCLE_TIME, filename=None):
    """Fly one full circle at constant speed while logging."""
    cf = scf.cf
    logconf, logfile = setup_logging(cf, filename)

    print("------------------------------------------------------")
    print(" FLIGHT SEQUENCE STARTED ")
    print("------------------------------------------------------")

    logconf.start()
    with MotionCommander(scf, default_height=height) as mc:
        print(f"Taking off to {height:.2f} m...")
        time.sleep(2.0)

        print(f"Flying a circle: radius = {radius:.2f} m, duration = {circle_time:.1f} s")
        steps = 60
        angle_per_step = 2 * math.pi / steps
        step_time = circle_time / steps

        for i in range(steps):
            angle = i * angle_per_step
            vx = -math.sin(angle) * radius * (2 * math.pi / circle_time)
            vy =  math.cos(angle) * radius * (2 * math.pi / circle_time)
            mc.start_linear_motion(vx, vy, 0)
            time.sleep(step_time)

        mc.stop()
        print("\nCircle complete. Hovering briefly...")
        time.sleep(1.5)

        print("Landing...")

    logconf.stop()
    logfile.close()
    print(f"\nLog saved to: {os.path.abspath(filename)}")

    print("------------------------------------------------------")
    print(" FLIGHT SEQUENCE COMPLETE ")
    print("------------------------------------------------------\n")


# -------------------------------------------------------------
# MAIN SCRIPT
# -------------------------------------------------------------
if __name__ == '__main__':
    print("======================================================")
    print(" Crazyflie Circle Flight with State Logging to TXT ")
    print("======================================================")

    cflib.crtp.init_drivers(enable_debug_driver=False)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Determine Downloads folder path
    downloads_path = os.path.join(os.path.expanduser("~"), "Downloads")
    if not os.path.exists(downloads_path):
        os.makedirs(downloads_path)
    filename = os.path.join(downloads_path, f"circle_flight_log_{timestamp}.txt")

    print(f"Connecting to Crazyflie at URI: {URI}")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        try:
            cf = scf.cf
            # set_pid_params(cf)   # PID tuning disabled
            print("Connection established.")
            fly_circle(scf, filename=filename)
            print("Flight complete.")
        except Exception as e:
            print("Error:", e)

    print("Disconnected.")
    print("======================================================\n")
