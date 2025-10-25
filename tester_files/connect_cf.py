import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Change this to match your radio channel (from cfclient)
URI = 'radio://0/80/2M'

def main():
    print("Initializing drivers...")
    cflib.crtp.init_drivers()
    print("Scanning interfaces...")
    available = cflib.crtp.scan_interfaces()
    print("Available interfaces:")
    for i in available:
        print(i[0])
    if not available:
        print("No Crazyflie found. Check dongle/driver.")
        return

    uri = available[0][0]
    print(f"Connecting to {uri} ...")
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Connected to Crazyflie!")
        scf.cf.commander.send_stop_setpoint()

if __name__ == '__main__':
    main()