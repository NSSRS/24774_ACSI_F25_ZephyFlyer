"""
Wind Supervisor Controller - Fully Automatic Timer Control (5s to 20s)
Displays debug prints and a visual wind arrow.
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

    # --- Wind Configuration ---
    # Wind starts at 5.0 seconds
    WIND_START_TIME = 5.0
    # Wind stops at 20.0 seconds
    WIND_END_TIME = 20.0
    
    # Wind State
    wind_enabled = False
    wind_force = [0.0, 0.0, 0.0]
    wind_magnitude = 0.03   # Force magnitude (30mN)
    
    # Wind Direction: Constant -X direction (same as B-mode)
    WIND_DIRECTION = [-wind_magnitude, 0.0, 0.0]

    # 由于不再使用键盘，可以移除键盘设备或注释掉
    # keyboard = Keyboard()
    # keyboard.enable(timestep)

    print("\n=== WIND SUPERVISOR READY (Automatic Mode) ===")
    print(f"[CONFIG] Wind will start at t={WIND_START_TIME}s and stop at t={WIND_END_TIME}s.")
    print(f"[CONFIG] Wind Force: {wind_magnitude}N in world -X direction (Fx={WIND_DIRECTION[0]:+.3f})\n")

    # ============================
    # Create Wind Arrow (VRML) - (保持不变)
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
    # UPDATE VISUAL ARROW - (保持不变)
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
    start_time = supervisor.getTime()

    while supervisor.step(timestep) != -1:

        current_time = supervisor.getTime() - start_time

        # --------------------------------------------------
        # AUTO WIND ACTIVATION (Start at 5s, Stop at 20s)
        # --------------------------------------------------
        
        # 1. 启动风力
        if current_time >= WIND_START_TIME and current_time < WIND_END_TIME and not wind_enabled:
            wind_enabled = True
            wind_force = WIND_DIRECTION
            print(f"\n🌬️ AUTO-WIND ACTIVATED at t = {current_time:.2f}s (Target Start: {WIND_START_TIME}s)")
            
        # 2. 停止风力
        elif current_time >= WIND_END_TIME and wind_enabled:
            wind_enabled = False
            wind_force = [0.0, 0.0, 0.0]
            print(f"\n🛑 AUTO-WIND DEACTIVATED at t = {current_time:.2f}s (Target End: {WIND_END_TIME}s)\n")

        # --------------------------------------------------
        # Apply wind + update arrow
        # --------------------------------------------------
        if wind_enabled:
            crazyflie.addForce(wind_force, False)
            update_arrow(wind_force)
        else:
            # 停止风力，隐藏箭头
            wind_arrow.getField("translation").setSFVec3f([0, -10, 0])