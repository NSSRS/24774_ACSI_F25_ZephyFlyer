import numpy as np
from controller import Robot, Keyboard
from math import cos, sin, pi, sqrt, atan2
from scipy.linalg import solve_continuous_are
import csv
import os
from datetime import datetime

THRUST_SCALE = 1.0
MOMENT_SCALE = 2.0e-4


class LQRDataLogger:
    """Data logger for LQR flight logs."""
    
    def __init__(self, log_dir="logs", prefix="lqr", log_decimation=10):
        self.log_dir = log_dir
        self.prefix = prefix
        self.data = []
        self.log_decimation = log_decimation
        self.log_counter = 0
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
    
    def log(self, sim_time, state, x_ref, u, motors, mode="HOVER"):
        self.log_counter += 1
        if self.log_counter % self.log_decimation != 0:
            return
        
        pos_error = sqrt((state[0]-x_ref[0])**2 + 
                        (state[1]-x_ref[1])**2 + 
                        (state[2]-x_ref[2])**2)
        
        self.data.append({
            't': sim_time, 'mode': mode,
            'px': state[0], 'py': state[1], 'pz': state[2],
            'vx': state[3], 'vy': state[4], 'vz': state[5],
            'roll': state[6], 'pitch': state[7], 'yaw': state[8],
            'ref_px': x_ref[0], 'ref_py': x_ref[1], 'ref_pz': x_ref[2],
            'u0': u[0], 'u1': u[1], 'u2': u[2],
            'm1': motors[0], 'm2': motors[1], 'm3': motors[2], 'm4': motors[3],
            'pos_error': pos_error
        })
    
    def save(self):
        if not self.data:
            return None
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(self.log_dir, f"{self.prefix}_{timestamp}.csv")
        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.data[0].keys())
            writer.writeheader()
            writer.writerows(self.data)
        print(f"📊 Log saved: {filepath} ({len(self.data)} samples)")
        return filepath


def get_linearized_model(mass=0.027, g=9.81, Ixx=1.6e-5, Iyy=1.6e-5, Izz=2.9e-5):
    """Linearized quadrotor model near hover."""
    nx, nu = 12, 4
    A = np.zeros((nx, nx))
    
    # Position dynamics
    A[0, 3] = 1.0
    A[1, 4] = 1.0
    A[2, 5] = 1.0
    
    # Velocity dynamics (linearized)
    A[3, 7] = g
    A[4, 6] = -g
    
    # Attitude kinematics
    A[6, 9] = 1.0
    A[7, 10] = 1.0
    A[8, 11] = 1.0
    
    # Damping
    A[3, 3] = -2.0
    A[4, 4] = -2.0
    A[5, 5] = -3.0
    A[9, 9] = -10.0
    A[10, 10] = -10.0
    A[11, 11] = -10.0
    
    # Control input matrix
    B = np.zeros((nx, nu))
    B[5, 0] = THRUST_SCALE / mass
    B[9, 1] = MOMENT_SCALE / Ixx
    B[10, 2] = MOMENT_SCALE / Iyy
    B[11, 3] = MOMENT_SCALE / Izz
    
    return A, B


class LQRController:
    """LQR controller with integral action."""
    
    def __init__(self, dt=0.01, hover_height=0.5):
        self.dt = dt
        self.hover_height = hover_height
        
        self.mass = 2.7e-2
        self.g = 9.81
        self.Ixx = 1.6e-5
        self.Iyy = 1.6e-5
        self.Izz = 2.9e-5
        
        # Velocity filter
        self.vx_f = self.vy_f = self.vz_f = 0.0
        self.alpha_v = 0.4
        
        # Integral terms with anti-windup
        self.int_ez = 0.0
        self.int_ex = 0.0
        self.int_ey = 0.0
        self.Ki_z = 0.05
        self.Ki_xy = 0.03
        self.int_limit_z = 0.05
        self.int_limit_xy = 0.05
        
        self.A, self.B = get_linearized_model(
            mass=self.mass, g=self.g,
            Ixx=self.Ixx, Iyy=self.Iyy, Izz=self.Izz
        )
        self.K = self._compute_lqr_gain()
        
        self.u_min = np.array([-5.000E-02, -2.000E-02, -2.000E-02, -5.000E-02])
        self.u_max = np.array([+5.000E-02, +2.000E-02, +2.000E-02, +5.000E-02])

        self.x_ref = np.zeros(12, dtype=np.float64)
        self.x_ref[2] = hover_height
        self.x_ref_current = self.x_ref.copy()
        
        self.ARM_LENGTH = 0.046
        self.THRUST2TORQUE = 0.005022
        self.MOTOR_TO_THRUST = 1.63e-3
        
        print(f"\n🚁 LQR Controller initialized")
        print(f"  Ki_z={self.Ki_z}, Ki_xy={self.Ki_xy}")
        print(f"  Input limits: thrust=[{self.u_min[0]:.2f}, {self.u_max[0]:.2f}]")
        print(f"                moments=[{self.u_min[1]:.3f}, {self.u_max[1]:.3f}]")
    
    def _compute_lqr_gain(self):
        """Compute LQR gain matrix."""
        Q = np.diag([
            500.0, 500.0, 20.0,
            40.0, 40.0, 15.0,
            30.0, 30.0, 2.0,
            6.0, 6.0, 3.0
        ])
        
        R = np.diag([
            400.0,
            600.0,
            600.0,
            20000.0
        ])
        
        try:
            P = solve_continuous_are(self.A, self.B, Q, R)
            K = np.linalg.inv(R) @ (self.B.T @ P)
            
            print(f"\n📊 LQR Gains:")
            print(f"  K[0,2] (thrust←z):  {K[0,2]:.6f}")
            print(f"  K[1,6] (roll←φ):    {K[1,6]:.6f}")
            print(f"  K[2,7] (pitch←θ):   {K[2,7]:.6f}")
            print(f"  K[0,5] (thrust←vz): {K[0,5]:.6f}")
            
            return K
        except Exception as e:
            print(f"⚠️ ARE failed: {e}, using fallback gains")
            K = np.zeros((4, 12))
            K[0, 2] = 0.01; K[0, 5] = 0.005
            K[1, 1] = 0.001; K[1, 4] = 0.0005; K[1, 6] = 0.002
            K[2, 0] = 0.001; K[2, 3] = 0.0005; K[2, 7] = 0.002
            return K
    
    def reset_integrators(self):
        self.int_ez = self.int_ex = self.int_ey = 0.0
    
    def set_target_position(self, x, y, z):
        self.x_ref[0] = float(x)
        self.x_ref[1] = float(y)
        self.x_ref[2] = float(z)
        self.x_ref_current = self.x_ref.copy()
        self.reset_integrators()
    
    def get_state(self, gps, imu, gyro, past_pos, past_time, current_time):
        dt = max(current_time - past_time, 1e-6)
        px, py, pz = gps
        
        # Velocity estimation with low-pass filter
        vx_raw = (px - past_pos[0]) / dt
        vy_raw = (py - past_pos[1]) / dt
        vz_raw = (pz - past_pos[2]) / dt
        
        self.vx_f = (1 - self.alpha_v) * self.vx_f + self.alpha_v * vx_raw
        self.vy_f = (1 - self.alpha_v) * self.vy_f + self.alpha_v * vy_raw
        self.vz_f = (1 - self.alpha_v) * self.vz_f + self.alpha_v * vz_raw
        
        return np.array([px, py, pz, self.vx_f, self.vy_f, self.vz_f,
                        imu[0], imu[1], imu[2], gyro[0], gyro[1], gyro[2]])
    
    def compute_control(self, state):
        """LQR control with integral compensation."""
        e = state - self.x_ref_current
        
        # Wrap yaw
        while e[8] > pi: e[8] -= 2*pi
        while e[8] < -pi: e[8] += 2*pi
        
        # Integral update with anti-windup
        self.int_ez += e[2] * self.dt
        self.int_ex += e[0] * self.dt
        self.int_ey += e[1] * self.dt
        
        self.int_ez = np.clip(self.int_ez, -self.int_limit_z, self.int_limit_z)
        self.int_ex = np.clip(self.int_ex, -self.int_limit_xy, self.int_limit_xy)
        self.int_ey = np.clip(self.int_ey, -self.int_limit_xy, self.int_limit_xy)
        
        # LQR feedback
        u = -self.K @ e
        
        # Integral compensation
        u[0] += -self.Ki_z * self.int_ez
        u[1] += -self.Ki_xy * self.int_ey
        u[2] += -self.Ki_xy * self.int_ex
        u[3] = 0.0
        
        u = np.clip(u, self.u_min, self.u_max)
        
        return u
    
    def control_to_motors(self, u):
        """Force-torque mixer (CrazyFlie firmware)."""
        u_thrust, u_roll, u_pitch, u_yaw = u
        
        hover_thrust = self.mass * self.g
        total_thrust = max(0.0, hover_thrust + u_thrust * THRUST_SCALE)
        
        tau_x = u_roll * MOMENT_SCALE
        tau_y = u_pitch * MOMENT_SCALE
        tau_z = 0
        
        arm = 0.707106781 * self.ARM_LENGTH
        
        thrust_part = 0.25 * total_thrust
        roll_part = 0.25 / arm * tau_x
        pitch_part = 0.25 / arm * tau_y
        yaw_part = 0.25 * tau_z / self.THRUST2TORQUE
        
        F1 = thrust_part - roll_part - pitch_part - yaw_part
        F2 = thrust_part - roll_part + pitch_part + yaw_part
        F3 = thrust_part + roll_part + pitch_part - yaw_part
        F4 = thrust_part + roll_part - pitch_part + yaw_part
        
        m1 = np.clip(F1 / self.MOTOR_TO_THRUST, 0, 600)
        m2 = np.clip(F2 / self.MOTOR_TO_THRUST, 0, 600)
        m3 = np.clip(F3 / self.MOTOR_TO_THRUST, 0, 600)
        m4 = np.clip(F4 / self.MOTOR_TO_THRUST, 0, 600)
        
        return np.array([m1, m2, m3, m4])


class CircleTrajectory:
    """Circular trajectory generator."""
    
    def __init__(self, z=0.5, radius=0.5, period=10.0):
        self.z = z
        self.radius = radius
        self.period = period
        self.start_time = 0
        self.center_x = 0.0
        self.center_y = 0.5
        self.initial_angle = 0.0
    
    def start(self, t, current_x, current_y):
        """Start circle from current position."""
        self.start_time = t
        
        dx = current_x - self.center_x
        dy = current_y - self.center_y
        self.initial_angle = atan2(dy, dx)
    
    def get_position(self, t):
        """Get position at time t."""
        rel_t = t - self.start_time
        omega = 2.0 * pi / self.period
        angle = self.initial_angle + omega * rel_t
        
        x = self.center_x + self.radius * cos(angle)
        y = self.center_y + self.radius * sin(angle)
        
        return x, y, self.z


class LineTrajectory:
    """Linear trajectory generator."""
    
    def __init__(self, z=0.5, distance=0.5, duration=5.0, axis='x'):
        self.z = z
        self.distance = distance
        self.duration = duration
        self.axis = axis
        self.start_time = 0
        self.start_x = 0
        self.start_y = 0
    
    def start(self, t, current_x, current_y):
        """Start line from current position."""
        self.start_time = t
        self.start_x = current_x
        self.start_y = current_y
    
    def get_position(self, t):
        """Get position at time t."""
        rel_t = t - self.start_time
        progress = min(1.0, rel_t / self.duration)
        displacement = progress * self.distance
        
        if self.axis == 'x':
            return self.start_x + displacement, self.start_y, self.z
        else:
            return self.start_x, self.start_y + displacement, self.z


class KeyDebouncer:
    """Keyboard debouncer to prevent double-press."""
    
    def __init__(self, debounce_time=0.3):
        self.debounce_time = debounce_time
        self.last_press = {}
    
    def is_key_ready(self, key, current_time):
        if key not in self.last_press:
            self.last_press[key] = current_time
            return True
        
        if current_time - self.last_press[key] >= self.debounce_time:
            self.last_press[key] = current_time
            return True
        
        return False


def main():
    """Main control loop."""
    robot = Robot()
    timestep = 2
    control_dt = timestep / 1000.0
    
    # Sensors
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    keyboard = Keyboard()
    keyboard.enable(timestep)
    
    # Motors
    motors = [
        robot.getDevice("m1_motor"),
        robot.getDevice("m2_motor"),
        robot.getDevice("m3_motor"),
        robot.getDevice("m4_motor")
    ]
    for m in motors:
        m.setPosition(float('inf'))
    
    hover_height = 0.5
    lqr = LQRController(dt=control_dt, hover_height=hover_height)
    logger = LQRDataLogger(log_dir="logs", prefix="lqr_v4", log_decimation=10)
    debouncer = KeyDebouncer(debounce_time=0.3)
    
    target_x, target_y, target_z = 0.0, 0.0, hover_height
    
    circle_traj = CircleTrajectory(z=hover_height, radius=0.5, period=10.0)
    line_traj_x = LineTrajectory(z=hover_height, distance=0.5, duration=5.0, axis='x')
    line_traj_y = LineTrajectory(z=hover_height, distance=0.5, duration=5.0, axis='y')
    
    mode = "HOVER"
    past_pos = [0, 0, 0]
    past_time = 0.0
    last_print_time = 0.0
    
    print("\n" + "="*70)
    print("Pure LQR Controller for CrazyFlie")
    print("="*70)
    print("Controls:")
    print("  Arrow keys : Move target X/Y (±0.1m)")
    print("  W/S        : Adjust altitude (±0.1m)")
    print("  R          : Reset to origin")
    print("  T          : Toggle CIRCLE trajectory")
    print("  X          : Toggle LINE-X trajectory")
    print("  Y          : Toggle LINE-Y trajectory")
    print("  H          : Return to HOVER mode")
    print("  +/-        : Adjust circle radius")
    print("  SPACE      : Print status")
    print("  K          : Save log")
    print("="*70 + "\n")
    
    for _ in range(10):
        robot.step(timestep)
    
    while robot.step(timestep) != -1:
        t = robot.getTime()
        
        gps_val = gps.getValues()
        imu_val = imu.getRollPitchYaw()
        gyro_val = gyro.getValues()
        
        # Keyboard handling with debouncing
        key = keyboard.getKey()
        while key > 0:
            if not debouncer.is_key_ready(key, t):
                key = keyboard.getKey()
                continue
            
            if key == Keyboard.UP:
                target_x += 0.1; mode = "HOVER"
            elif key == Keyboard.DOWN:
                target_x -= 0.1; mode = "HOVER"
            elif key == Keyboard.LEFT:
                target_y += 0.1; mode = "HOVER"
            elif key == Keyboard.RIGHT:
                target_y -= 0.1; mode = "HOVER"
            elif key == ord('W'):
                target_z = min(2.0, target_z + 0.1); mode = "HOVER"
            elif key == ord('S'):
                target_z = max(0.2, target_z - 0.1); mode = "HOVER"
            elif key == ord('R') or key == ord('r'):
                target_x, target_y, target_z = 0.0, 0.0, hover_height
                mode = "HOVER"
                lqr.reset_integrators()
                print("🔄 Reset to origin")
            elif key == ord('T') or key == ord('t'):
                if mode != "CIRCLE":
                    mode = "CIRCLE"
                    circle_traj.z = target_z
                    circle_traj.start(t, gps_val[0], gps_val[1])
                    lqr.reset_integrators()
                    print(f"🔵 CIRCLE: r={circle_traj.radius}m")
                else:
                    mode = "HOVER"
                    target_x, target_y = gps_val[0], gps_val[1]
                    print("⬛ CIRCLE stopped")
            elif key == ord('X') or key == ord('x'):
                if mode != "LINE_X":
                    mode = "LINE_X"
                    line_traj_x.z = target_z
                    line_traj_x.start(t, gps_val[0], gps_val[1])
                    lqr.reset_integrators()
                    print(f"📏 LINE-X: d={line_traj_x.distance}m")
                else:
                    mode = "HOVER"
                    target_x, target_y = gps_val[0], gps_val[1]
                    print("⬛ LINE-X stopped")
            elif key == ord('Y') or key == ord('y'):
                if mode != "LINE_Y":
                    mode = "LINE_Y"
                    line_traj_y.z = target_z
                    line_traj_y.start(t, gps_val[0], gps_val[1])
                    lqr.reset_integrators()
                    print(f"📏 LINE-Y: d={line_traj_y.distance}m")
                else:
                    mode = "HOVER"
                    target_x, target_y = gps_val[0], gps_val[1]
                    print("⬛ LINE-Y stopped")
            elif key == ord('H') or key == ord('h'):
                mode = "HOVER"
                target_x, target_y = gps_val[0], gps_val[1]
                print("🏠 HOVER mode")
            elif key == ord('+') or key == ord('='):
                circle_traj.radius = min(1.0, circle_traj.radius + 0.1)
                print(f"Circle radius: {circle_traj.radius:.1f}m")
            elif key == ord('-') or key == ord('_'):
                circle_traj.radius = max(0.1, circle_traj.radius - 0.1)
                print(f"Circle radius: {circle_traj.radius:.1f}m")
            elif key == ord(' '):
                pos_err = sqrt((gps_val[0]-lqr.x_ref_current[0])**2 +
                              (gps_val[1]-lqr.x_ref_current[1])**2 +
                              (gps_val[2]-lqr.x_ref_current[2])**2)
                print(f"\n[{t:.1f}s] Mode: {mode}")
                print(f"  Pos: ({gps_val[0]:.3f}, {gps_val[1]:.3f}, {gps_val[2]:.3f})")
                print(f"  Ref: ({lqr.x_ref_current[0]:.3f}, {lqr.x_ref_current[1]:.3f}, {lqr.x_ref_current[2]:.3f})")
                print(f"  Error: {pos_err:.4f}m")
                print(f"  Integrals: z={lqr.int_ez:.3f}, x={lqr.int_ex:.3f}, y={lqr.int_ey:.3f}\n")
            elif key == ord('K') or key == ord('k'):
                logger.save()
            
            if key in [Keyboard.UP, Keyboard.DOWN, Keyboard.LEFT, Keyboard.RIGHT, 
                       ord('W'), ord('S'), ord('R'), ord('r')]:
                lqr.set_target_position(target_x, target_y, target_z)
                print(f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            
            key = keyboard.getKey()
        
        # Update reference based on mode
        if mode == "CIRCLE":
            x, y, z = circle_traj.get_position(t)
            lqr.x_ref_current[0] = x
            lqr.x_ref_current[1] = y
            lqr.x_ref_current[2] = z

            # Velocity feedforward
            rel_t = t - circle_traj.start_time
            omega = 2.0 * np.pi / circle_traj.period
            R = circle_traj.radius
            
            angle = circle_traj.initial_angle + omega * rel_t
            
            vx_ref = -R * omega * np.sin(angle)
            vy_ref = R * omega * np.cos(angle)

            lqr.x_ref_current[3] = vx_ref
            lqr.x_ref_current[4] = vy_ref
            lqr.x_ref_current[5] = 0.0

            # Attitude feedforward for centripetal acceleration
            ax = -R * omega**2 * np.cos(angle)
            ay = -R * omega**2 * np.sin(angle)
            
            pitch_ff = ax / lqr.g
            roll_ff = -ay / lqr.g
            
            lqr.x_ref_current[6] = roll_ff
            lqr.x_ref_current[7] = pitch_ff
            lqr.x_ref_current[8:12] = 0.0

        elif mode == "LINE_X":
            x, y, z = line_traj_x.get_position(t)
            lqr.x_ref_current[0] = x
            lqr.x_ref_current[1] = y
            lqr.x_ref_current[2] = z
        elif mode == "LINE_Y":
            x, y, z = line_traj_y.get_position(t)
            lqr.x_ref_current[0] = x
            lqr.x_ref_current[1] = y
            lqr.x_ref_current[2] = z
        else:
            lqr.x_ref_current[0] = target_x
            lqr.x_ref_current[1] = target_y
            lqr.x_ref_current[2] = target_z
        
        # Control
        state = lqr.get_state(gps_val, imu_val, gyro_val, past_pos, past_time, t)
        u = lqr.compute_control(state)
        motor_cmds = lqr.control_to_motors(u)
        
        logger.log(t, state, lqr.x_ref_current, u, motor_cmds, mode)
        
        # Apply motors
        motors[0].setVelocity(-motor_cmds[0])
        motors[1].setVelocity(motor_cmds[1])
        motors[2].setVelocity(-motor_cmds[2])
        motors[3].setVelocity(motor_cmds[3])
        
        # Print status
        if t - last_print_time >= 1.0:
            pos_err = sqrt((gps_val[0]-lqr.x_ref_current[0])**2 +
                          (gps_val[1]-lqr.x_ref_current[1])**2 +
                          (gps_val[2]-lqr.x_ref_current[2])**2)
            print(f"[{t:5.1f}s] {mode:7s} | "
                  f"pos:({gps_val[0]:+.2f},{gps_val[1]:+.2f},{gps_val[2]:.2f}) | "
                  f"err:{pos_err:.3f}m | "
                  f"u:[{u[0]:+.3f},{u[1]:+.4f},{u[2]:+.4f}]")
            last_print_time = t
        
        # Safety check
        if abs(imu_val[0]) > 1.0 or abs(imu_val[1]) > 1.0:
            print(f"🛑 FLIP at t={t:.2f}s!")
            logger.save()
            break
        
        past_pos = list(gps_val)
        past_time = t
        
        if t > 60.0:
            logger.save()
            break


if __name__ == '__main__':
    main()