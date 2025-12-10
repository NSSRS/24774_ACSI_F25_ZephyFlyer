"""
Wind Supervisor Controller with Debug Prints + Visual Arrow.
"""

from controller import Supervisor, Keyboard
import sys
import math

if __name__ == '__main__':
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    # Get drone node
    crazyflie = supervisor.getFromDef("Crazyflie")
    if crazyflie is None:
        print("❌ ERROR: DEF Crazyflie not found")
        sys.exit(1)

    keyboard = Keyboard()
    keyboard.enable(timestep)

    # ============================
    # Wind State
    # ============================
    wind_enabled = False
    wind_force = [0.0, 0.0, 0.0]
    wind_magnitude = 0.03   # default 30mN

    print("\n=== WIND SUPERVISOR READY ===")
    print("Keys:")
    print("  V = toggle wind")
    print("  7/8/9/0 = set horizontal wind dir")
    print("  B = activate wind from 6→12 (world −X)")
    print("  C = clear wind")
    print("  ,/. = change magnitude\n")

    # ============================
    # Create Wind Arrow (VRML)
    # ============================
    root = supervisor.getRoot()
    children = root.getField("children")

    arrow_proto = """
    Transform {
      translation 0 0 1
      rotation 0 0 1 0
      children [
        Shape {
          appearance Appearance {
            material Material {
              diffuseColor 1 0 0
            }
          }
          geometry Cylinder {
            radius 0.01
            height 0.2
          }
        }
      ]
    }
    """

    children.importMFNodeFromString(-1, arrow_proto)
    wind_arrow = children.getMFNode(children.getCount() - 1)

    # Hide arrow initially
    wind_arrow.getField("translation").setSFVec3f([0, -10, 0])

    # ============================
    # UPDATE VISUAL ARROW
    # ============================
    def update_arrow(force_vector):
        Fx, Fy, Fz = force_vector
        mag = math.sqrt(Fx*Fx + Fy*Fy + Fz*Fz)

        if mag < 1e-6:
            wind_arrow.getField("translation").setSFVec3f([0, -10, 0])
            return

        # Arrow near drone
        pos = crazyflie.getPosition()
        arrow_pos = [pos[0], pos[1], pos[2] + 0.2]

        # Orientation in XY plane
        angle = math.atan2(Fy, Fx)

        wind_arrow.getField("translation").setSFVec3f(arrow_pos)
        wind_arrow.getField("rotation").setSFRotation([0, 0, 1, angle])

        # Scale height with magnitude
        height = 0.2 + mag * 5

        # Access Shape node
        shape = wind_arrow.getField("children").getMFNode(0)

        # Access Cylinder geometry node
        cyl = shape.getField("geometry").getSFNode()

        # Set height
        cyl.getField("height").setSFFloat(height)

    # ============================
    # MAIN LOOP
    # ============================
    while supervisor.step(timestep) != -1:

        key = keyboard.getKey()

        while key > 0:

            # --------------------------------------------------
            # Toggle wind
            # --------------------------------------------------
            if key == ord('V') or key == ord('v'):
                wind_enabled = not wind_enabled
                print("Wind:", "ENABLED" if wind_enabled else "DISABLED")

            # --------------------------------------------------
            # ORIGINAL B MODE — 6→12 wind (world −X)
            # --------------------------------------------------
            elif key == ord('B') or key == ord('b'):
                wind_enabled = True
                wind_force = [-wind_magnitude, 0.0, 0.0]
                print("\n🌬️  B-MODE WIND ACTIVATED")
                print("     Direction: 6 → 12 (world −X)")
                print(f"     Force: Fx={wind_force[0]:+.3f}\n")

            # --------------------------------------------------
            # Direction controls
            # --------------------------------------------------
            elif key == ord('7'): wind_force = [ wind_magnitude, 0, 0 ]
            elif key == ord('8'): wind_force = [ 0, wind_magnitude, 0 ]
            elif key == ord('9'): wind_force = [-wind_magnitude, 0, 0 ]
            elif key == ord('0'): wind_force = [ 0, -wind_magnitude, 0 ]

            elif key == ord('-'): wind_force = [ 0, 0,  wind_magnitude ]
            elif key == ord('+') or key == ord('='): wind_force = [ 0, 0, -wind_magnitude ]

            # --------------------------------------------------
            # Magnitude adjust
            # --------------------------------------------------
            elif key == ord(','):
                wind_magnitude = max(0.005, wind_magnitude - 0.005)
                print("Wind magnitude:", wind_magnitude)

            elif key == ord('.'):
                wind_magnitude = min(0.5, wind_magnitude + 0.005)
                print("Wind magnitude:", wind_magnitude)

            # --------------------------------------------------
            # Clear wind
            # --------------------------------------------------
            elif key == ord('C') or key == ord('c'):
                wind_force = [0,0,0]
                print("Wind cleared")

            key = keyboard.getKey()

        # --------------------------------------------------
        # Apply wind + update arrow
        # --------------------------------------------------
        if wind_enabled:
            crazyflie.addForce(wind_force, False)
            update_arrow(wind_force)
        else:
            wind_arrow.getField("translation").setSFVec3f([0, -10, 0])
