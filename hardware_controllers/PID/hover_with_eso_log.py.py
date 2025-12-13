# hover_with_eso_log.py
# FULL, WORKING.
# - Logs motor.m1–m4 (unitless / PWM) to CSV
# - Prints ESO-related state to console
# - When ESO first becomes active, starts a 5s timer and PROMPTS in console

import time
import os
import numpy as np
from datetime import datetime
import logging

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

# =============================
# USER SETTINGS
# =============================
URI = 'radio://0/80/2M'
LOG_PERIOD_MS = 100
HOVER_HEIGHT = 0.5
HOVER_TIME = 10.0

logging.basicConfig(level=logging.ERROR)

# =============================
# LOGGING SETUP
# =============================
def setup_logging(cf, filename):

    # --- Log configs ---
    logconf_state = LogConfig('StateLog', LOG_PERIOD_MS)
    logconf_state.add_variable('stateEstimate.x', 'float')
    logconf_state.add_variable('stateEstimate.y', 'float')
    logconf_state.add_variable('stateEstimate.z', 'float')
    logconf_state.add_variable('stateEstimate.roll', 'float')
    logconf_state.add_variable('stateEstimate.pitch', 'float')
    logconf_state.add_variable('stateEstimate.yaw', 'float')

    logconf_motor = LogConfig('MotorLog', LOG_PERIOD_MS)
    logconf_motor.add_variable('motor.m1', 'uint16_t')
    logconf_motor.add_variable('motor.m2', 'uint16_t')
    logconf_motor.add_variable('motor.m3', 'uint16_t')
    logconf_motor.add_variable('motor.m4', 'uint16_t')

    # ESO used for timing + console prints
    logconf_eso = LogConfig('ESOLog', LOG_PERIOD_MS)
    logconf_eso.add_variable('eso.x', 'int32_t')
    logconf_eso.add_variable('eso.y', 'int32_t')
    logconf_eso.add_variable('eso.z', 'int32_t')
    logconf_eso.add_variable('eso.t', 'int32_t')

    cf.log.add_config(logconf_state)
    cf.log.add_config(logconf_motor)
    cf.log.add_config(logconf_eso)

    # --- CSV ---
    f = open(filename, "w")
    f.write(
        "t,mode,"
        "px,py,pz,"
        "vx,vy,vz,"
        "roll,pitch,yaw,"
        "ref_px,ref_py,ref_pz,"
        "u_thrust,u_roll,u_pitch,u_yaw,"
        "m1,m2,m3,m4,"
        "pos_error,solve_time_ms\n"
    )

    start_time = time.time()

    latest = {
        'x': 0.0, 'y': 0.0, 'z': 0.0,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        'm1': 0, 'm2': 0, 'm3': 0, 'm4': 0,
        'motor_valid': False,
        'eso_active': False,
        'eso_start_time': None
    }

    # --- Callbacks ---
    def state_cb(ts, data, _):
        latest['x'] = data['stateEstimate.x']
        latest['y'] = data['stateEstimate.y']
        latest['z'] = data['stateEstimate.z']
        latest['roll']  = data['stateEstimate.roll']
        latest['pitch'] = data['stateEstimate.pitch']
        latest['yaw']   = data['stateEstimate.yaw']

    def motor_cb(ts, data, _):
        latest['m1'] = int(data['motor.m1'])
        latest['m2'] = int(data['motor.m2'])
        latest['m3'] = int(data['motor.m3'])
        latest['m4'] = int(data['motor.m4'])
        latest['motor_valid'] = True

    def eso_cb(ts, data, _):
        if not latest['motor_valid']:
            return

        t = time.time() - start_time

        # ESO disturbance (scaled like before)
        dax = data['eso.x'] / 1000.0
        day = data['eso.y'] / 1000.0
        daz = data['eso.z'] / 1000.0
        dth = data['eso.t'] / 1000.0

        # Detect ESO start
        if not latest['eso_active']:
            latest['eso_active'] = True
            latest['eso_start_time'] = time.time()
            print("\n>>> ESO ACTIVE <<<")

        # 5-second prompt after ESO starts
        if latest['eso_active'] and latest['eso_start_time'] is not None:
            if time.time() - latest['eso_start_time'] >= 5.0:
                print("\n>>> 5 seconds since ESO start <<<")
                latest['eso_start_time'] = None  # fire once

        ref_px, ref_py, ref_pz = 0.0, 0.0, HOVER_HEIGHT
        pos_error = np.sqrt(
            (latest['x'] - ref_px)**2 +
            (latest['y'] - ref_py)**2 +
            (latest['z'] - ref_pz)**2
        )

        # CSV write
        f.write(
            f"{t:.3f},HOVER,"
            f"{latest['x']:.6f},{latest['y']:.6f},{latest['z']:.6f},"
            f"0.0,0.0,0.0,"
            f"{latest['roll']:.6f},{latest['pitch']:.6f},{latest['yaw']:.6f},"
            f"{ref_px:.3f},{ref_py:.3f},{ref_pz:.3f},"
            f"0.0,0.0,0.0,0.0,"
            f"{latest['m1']},{latest['m2']},{latest['m3']},{latest['m4']},"
            f"{pos_error:.6f},0.0\n"
        )

        # Console print (STATE + ESO + MOTORS)
        print(
            f"t={t:5.2f}s | "
            f"z={latest['z']:+.3f} m | "
            f"ESO(ax={dax:+.2f}, ay={day:+.2f}, az={daz:+.2f}, Fz={dth:+.3f}) | "
            f"m=[{latest['m1']} {latest['m2']} {latest['m3']} {latest['m4']}]",
            end="\r"
        )

    logconf_state.data_received_cb.add_callback(state_cb)
    logconf_motor.data_received_cb.add_callback(motor_cb)
    logconf_eso.data_received_cb.add_callback(eso_cb)

    return logconf_state, logconf_motor, logconf_eso, f

# =============================
# HOVER
# =============================
def hover(scf, filename):
    cf = scf.cf
    time.sleep(1.0)

    print("Controller:", cf.param.get_value('stabilizer.controller'))

    logconf_state, logconf_motor, logconf_eso, logfile = setup_logging(cf, filename)

    logconf_state.start()
    logconf_motor.start()
    logconf_eso.start()

    try:
        with MotionCommander(scf, default_height=HOVER_HEIGHT):
            time.sleep(HOVER_TIME)
    finally:
        logconf_state.stop()
        logconf_motor.stop()
        logconf_eso.stop()
        logfile.close()
        print("\nSaved:", filename)

# =============================
# MAIN
# =============================
if __name__ == "__main__":
    cflib.crtp.init_drivers(enable_debug_driver=False)

    filename = os.path.join(
        os.path.expanduser("~/Downloads"),
        f"hover_log_{datetime.now():%Y%m%d_%H%M%S}.csv"
    )

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        hover(scf, filename)
