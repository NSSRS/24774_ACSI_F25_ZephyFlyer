"""
circle_flight_no_eso_csv.py
---------------------------
Crazyflie circular flight using STOCK firmware (NO ESO).

Behavior:
- Start at (0,0)
- Take off to 0.5 m
- Hover
- Fly 1 m diameter CCW circle
- Return to (0,0)
- Save CSV in EXACT required format
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
# SETTINGS
# -------------------------------------------------------------
URI = 'radio://0/80/2M'
LOG_PERIOD_MS = 100

HEIGHT = 0.5
HOVER_TIME = 2.0

RADIUS = 0.5          # 1 m diameter
CIRCLE_TIME = 10.0
STEPS = 60

logging.basicConfig(level=logging.ERROR)

# -------------------------------------------------------------
# LOGGING
# -------------------------------------------------------------
def setup_logging(cf, filename):

    logconf = LogConfig('StateLog', LOG_PERIOD_MS)
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')
    logconf.add_variable('stateEstimate.roll', 'float')
    logconf.add_variable('stateEstimate.pitch', 'float')
    logconf.add_variable('stateEstimate.yaw', 'float')

    cf.log.add_config(logconf)

    f = open(filename, "w")
    f.write(
        "t,mode,px,py,pz,vx,vy,vz,roll,pitch,yaw,"
        "ref_px,ref_py,ref_pz,"
        "u_thrust,u_roll,u_pitch,u_yaw,"
        "m1,m2,m3,m4,"
        "pos_error,solve_time_ms\n"
    )

    start_time = time.time()

    state = {
        'x':0,'y':0,'z':0,
        'roll':0,'pitch':0,'yaw':0,
        'vx':0,'vy':0,'vz':0,
        'last_t':None,
        'last_x':0,'last_y':0,'last_z':0,
        'mode':'HOVER'
    }

    def cb(ts, data, _):
        t = time.time() - start_time

        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        z = data['stateEstimate.z']

        if state['last_t'] is not None:
            dt = t - state['last_t']
            if dt > 0:
                state['vx'] = (x - state['last_x']) / dt
                state['vy'] = (y - state['last_y']) / dt
                state['vz'] = (z - state['last_z']) / dt

        state.update({
            'x':x,'y':y,'z':z,
            'roll':data['stateEstimate.roll'],
            'pitch':data['stateEstimate.pitch'],
            'yaw':data['stateEstimate.yaw'],
            'last_t':t,
            'last_x':x,'last_y':y,'last_z':z
        })

        ref_px, ref_py, ref_pz = 0.0, 0.0, HEIGHT
        pos_error = math.sqrt(
            (x-ref_px)**2 +
            (y-ref_py)**2 +
            (z-ref_pz)**2
        )

        f.write(
            f"{t:.3f},{state['mode']},"
            f"{x:.6f},{y:.6f},{z:.6f},"
            f"{state['vx']:.6f},{state['vy']:.6f},{state['vz']:.6f},"
            f"{state['roll']:.6f},{state['pitch']:.6f},{state['yaw']:.6f},"
            f"{ref_px:.3f},{ref_py:.3f},{ref_pz:.3f},"
            f"0,0,0,0,"
            f"0,0,0,0,"
            f"{pos_error:.6f},0\n"
        )

        print(
            f"t={t:5.2f}s | {state['mode']} | "
            f"x={x:+.2f} y={y:+.2f} z={z:+.2f}",
            end="\r"
        )

    logconf.data_received_cb.add_callback(cb)
    return logconf, f, state

# -------------------------------------------------------------
# FLIGHT
# -------------------------------------------------------------
def fly(scf, filename):
    cf = scf.cf
    logconf, logfile, state = setup_logging(cf, filename)

    logconf.start()

    with MotionCommander(scf, default_height=HEIGHT) as mc:

        # Hover
        state['mode'] = 'HOVER'
        time.sleep(HOVER_TIME)

        # Circle CCW
        state['mode'] = 'CIRCLE'
        omega = 2 * math.pi / CIRCLE_TIME
        dt = CIRCLE_TIME / STEPS

        for i in range(STEPS):
            theta = i * 2 * math.pi / STEPS
            vx = -RADIUS * omega * math.sin(theta)
            vy =  RADIUS * omega * math.cos(theta)
            mc.start_linear_motion(vx, vy, 0)
            time.sleep(dt)

        mc.stop()

        # Return to (0,0)
        state['mode'] = 'RETURN'
        for _ in range(30):
            mc.start_linear_motion(-state['x'], -state['y'], 0)
            time.sleep(0.05)

        mc.stop()
        time.sleep(1.0)

    logconf.stop()
    logfile.close()
    print("\nSaved:", filename)

# -------------------------------------------------------------
# MAIN
# -------------------------------------------------------------
if __name__ == "__main__":
    cflib.crtp.init_drivers(enable_debug_driver=False)

    filename = os.path.join(
        os.path.expanduser("~/Downloads"),
        f"circle_no_eso_{datetime.now():%Y%m%d_%H%M%S}.csv"
    )

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        fly(scf, filename)

    print("Disconnected.")
