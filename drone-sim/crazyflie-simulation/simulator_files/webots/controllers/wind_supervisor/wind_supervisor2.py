"""
Wind Supervisor Controller

This supervisor controller manages wind forces in the simulation environment.
It applies external forces to the CrazyFlie robot independently of the
flight controller, allowing any controller to be tested with wind disturbances.

Features:
- Impulse forces (one-time push) - F/G/H/J/U/D keys
- Continuous wind (constant force) - V to toggle, 7/8/9/0 for direction
- Works with any CrazyFlie controller
- Centralized disturbance management

Architecture Benefits:
- Separates environment (wind) from control logic
- No need to modify flight controllers for wind testing
- Reusable across different control experiments

Author: ZephyFlyer Team
Date: 2025-12-03
"""

from controller import Supervisor, Keyboard
import sys

if __name__ == '__main__':
    # Initialize supervisor
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    # Get the CrazyFlie node
    crazyflie = supervisor.getFromDef("Crazyflie")
    if crazyflie is None:
        print("❌ Error: Could not find 'Crazyflie' DEF in world file")
        print("   Make sure your Crazyflie node has: DEF Crazyflie Crazyflie { ... }")
        sys.exit(1)

    # Initialize keyboard
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # Wind parameters
    wind_enabled = False
    wind_force = [0.0, 0.0, 0.0]  # Continuous wind force [Fx, Fy, Fz]
    wind_magnitude = 0.03  # 30mN default

    # Impulse force parameters
    impulse_magnitude = 0.15  # 150mN impulse (larger for visibility)
    last_impulse_time = 0.0
    IMPULSE_COOLDOWN = 0.5  # 0.5s cooldown between impulses

    # Debouncing for toggle keys
    KEY_DEBOUNCE = 0.3  # seconds
    last_key_time = {}

    print("\n" + "="*60)
    print("🌬️  WIND SUPERVISOR CONTROLLER")
    print("="*60)
    print("\nThis supervisor applies disturbances to the CrazyFlie.")
    print("The flight controller is independent and unaware of wind.\n")
    print("KEYBOARD CONTROLS:")
    print("\n📍 CONTINUOUS WIND (constant force):")
    print("  V             : Toggle wind ON/OFF")
    print("  7 / 8 / 9 / 0 : Set wind direction")
    print("                  7 = +X (forward)")
    print("                  8 = +Y (left)")
    print("                  9 = -X (backward)")
    print("                  0 = -Y (right)")
    print("  - / +         : Set vertical wind (up/down)")
    print("  , / .         : Decrease/increase wind magnitude")
    print("  C             : Clear wind (set to zero)")
    print("\n💥 IMPULSE FORCES (one-time push):")
    print("  F / G / H / J : Horizontal impulse (+X/+Y/-X/-Y)")
    print("  U / D         : Vertical impulse (up/down)")
    print("  [ / ]         : Adjust impulse magnitude")
    print("\n📊 UTILITIES:")
    print("  SPACE         : Print current wind status")
    print("="*60 + "\n")

    # Main control loop
    step_count = 0
    while supervisor.step(timestep) != -1:
        current_time = supervisor.getTime()
        step_count += 1

        # Keyboard input handling
        key = keyboard.getKey()

        while key > 0:
            # Check if this key needs debouncing
            key_needs_debounce = key in [
                ord('V'), ord('v'), ord('C'), ord('c'), ord(' ')
            ]

            # Debounce check
            if key_needs_debounce:
                if key in last_key_time and (current_time - last_key_time[key]) < KEY_DEBOUNCE:
                    key = keyboard.getKey()
                    continue
                last_key_time[key] = current_time

            # ========== CONTINUOUS WIND CONTROLS ==========

            # Toggle wind on/off
            if key == ord('V') or key == ord('v'):
                wind_enabled = not wind_enabled
                status = "ENABLED ✅" if wind_enabled else "DISABLED ❌"
                print(f"\n🌬️  Continuous wind: {status}")
                if wind_enabled and (wind_force[0] != 0 or wind_force[1] != 0 or wind_force[2] != 0):
                    print(f"   Force: [{wind_force[0]:+.3f}, {wind_force[1]:+.3f}, {wind_force[2]:+.3f}] N")
                print()

            # Set wind direction - Horizontal
            elif key == ord('7'):
                wind_force = [wind_magnitude, 0, 0]
                print(f"🌬️  Wind direction: +X {wind_magnitude:.3f}N (forward)")
                if not wind_enabled:
                    print("   ⚠️  Wind is OFF. Press 'V' to enable.")

            elif key == ord('8'):
                wind_force = [0, wind_magnitude, 0]
                print(f"🌬️  Wind direction: +Y {wind_magnitude:.3f}N (left)")
                if not wind_enabled:
                    print("   ⚠️  Wind is OFF. Press 'V' to enable.")

            elif key == ord('9'):
                wind_force = [-wind_magnitude, 0, 0]
                print(f"🌬️  Wind direction: -X {wind_magnitude:.3f}N (backward)")
                if not wind_enabled:
                    print("   ⚠️  Wind is OFF. Press 'V' to enable.")

            elif key == ord('0'):
                wind_force = [0, -wind_magnitude, 0]
                print(f"🌬️  Wind direction: -Y {wind_magnitude:.3f}N (right)")
                if not wind_enabled:
                    print("   ⚠️  Wind is OFF. Press 'V' to enable.")

            # Set wind direction - Vertical
            elif key == ord('-') or key == ord('_'):
                wind_force = [0, 0, wind_magnitude]
                print(f"🌬️  Wind direction: +Z {wind_magnitude:.3f}N (upward)")
                if not wind_enabled:
                    print("   ⚠️  Wind is OFF. Press 'V' to enable.")

            elif key == ord('=') or key == ord('+'):
                wind_force = [0, 0, -wind_magnitude]
                print(f"🌬️  Wind direction: -Z {wind_magnitude:.3f}N (downward)")
                if not wind_enabled:
                    print("   ⚠️  Wind is OFF. Press 'V' to enable.")

            # Adjust wind magnitude
            elif key == ord(',') or key == ord('<'):
                wind_magnitude = max(0.005, wind_magnitude - 0.005)
                print(f"🌬️  Wind magnitude: {wind_magnitude:.3f} N")
                # Update current wind force with new magnitude
                if wind_force[0] != 0:
                    wind_force[0] = wind_magnitude if wind_force[0] > 0 else -wind_magnitude
                if wind_force[1] != 0:
                    wind_force[1] = wind_magnitude if wind_force[1] > 0 else -wind_magnitude
                if wind_force[2] != 0:
                    wind_force[2] = wind_magnitude if wind_force[2] > 0 else -wind_magnitude

            elif key == ord('.') or key == ord('>'):
                wind_magnitude = min(0.5, wind_magnitude + 0.005)
                print(f"🌬️  Wind magnitude: {wind_magnitude:.3f} N")
                # Update current wind force with new magnitude
                if wind_force[0] != 0:
                    wind_force[0] = wind_magnitude if wind_force[0] > 0 else -wind_magnitude
                if wind_force[1] != 0:
                    wind_force[1] = wind_magnitude if wind_force[1] > 0 else -wind_magnitude
                if wind_force[2] != 0:
                    wind_force[2] = wind_magnitude if wind_force[2] > 0 else -wind_magnitude

            # Clear wind
            elif key == ord('C') or key == ord('c'):
                wind_force = [0, 0, 0]
                print("🌬️  Wind cleared (set to zero)")

            # ========== UTILITIES (must be before impulse cooldown check) ==========

            # Print status
            elif key == ord(' '):
                print("\n" + "="*60)
                print(f"📊 WIND STATUS (t={current_time:.2f}s)")
                print("="*60)
                print(f"  Continuous wind:     {'ENABLED ✅' if wind_enabled else 'DISABLED ❌'}")
                print(f"  Wind magnitude:      {wind_magnitude:.3f} N")
                print(f"  Wind force:          [{wind_force[0]:+.3f}, {wind_force[1]:+.3f}, {wind_force[2]:+.3f}] N")
                print(f"  Impulse magnitude:   {impulse_magnitude:.3f} N")
                if wind_enabled:
                    print("  Status:              🌬️  Applying continuous wind")
                else:
                    print("  Status:              ⏸️  No wind applied")
                print("="*60 + "\n")

            # ========== IMPULSE FORCE CONTROLS ==========

            # Adjust impulse magnitude
            elif key == ord('['):
                impulse_magnitude = max(0.05, impulse_magnitude - 0.05)
                print(f"💥 Impulse magnitude: {impulse_magnitude:.3f} N")

            elif key == ord(']'):
                impulse_magnitude = min(1.0, impulse_magnitude + 0.05)
                print(f"💥 Impulse magnitude: {impulse_magnitude:.3f} N")

            # Apply impulse forces (with cooldown)
            elif current_time - last_impulse_time > IMPULSE_COOLDOWN:
                impulse_applied = False

                if key == ord('F') or key == ord('f'):
                    crazyflie.addForce([impulse_magnitude, 0, 0], False)
                    print(f"💥 Impulse: +X {impulse_magnitude:.3f}N (forward)")
                    impulse_applied = True

                elif key == ord('G') or key == ord('g'):
                    crazyflie.addForce([0, impulse_magnitude, 0], False)
                    print(f"💥 Impulse: +Y {impulse_magnitude:.3f}N (left)")
                    impulse_applied = True

                elif key == ord('H') or key == ord('h'):
                    crazyflie.addForce([-impulse_magnitude, 0, 0], False)
                    print(f"💥 Impulse: -X {impulse_magnitude:.3f}N (backward)")
                    impulse_applied = True

                elif key == ord('J') or key == ord('j'):
                    crazyflie.addForce([0, -impulse_magnitude, 0], False)
                    print(f"💥 Impulse: -Y {impulse_magnitude:.3f}N (right)")
                    impulse_applied = True

                elif key == ord('U') or key == ord('u'):
                    crazyflie.addForce([0, 0, impulse_magnitude], False)
                    print(f"💥 Impulse: +Z {impulse_magnitude:.3f}N (upward)")
                    impulse_applied = True

                elif key == ord('D') or key == ord('d'):
                    crazyflie.addForce([0, 0, -impulse_magnitude], False)
                    print(f"💥 Impulse: -Z {impulse_magnitude:.3f}N (downward)")
                    impulse_applied = True

                if impulse_applied:
                    last_impulse_time = current_time

            key = keyboard.getKey()

        # Apply continuous wind force if enabled
        if wind_enabled:
            # addForce(force_vector, relative=False)
            # relative=False means force is in world frame
            crazyflie.addForce(wind_force, False)
