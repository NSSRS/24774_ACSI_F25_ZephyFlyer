import numpy as np
from controller import Robot, Keyboard
from math import cos, sin, pi, sqrt
from scipy.linalg import solve_continuous_are
import csv, os, sys
from datetime import datetime

THRUST_SCALE = 1.0
MOMENT_SCALE = 2.0e-4
MASS = 0.027
GRAVITY = 9.81
Ixx = 1.6e-5
Iyy = 1.6e-5
Izz = 2.9e-5

CONFIG = {
    'use_disturbance_compensation': True,
    'use_eso_state': False,
}

DIST_FF_RATIO = 0.7      
K_DIST_CTRL = 0.5          
TAU_FROM_FORCE = 0.08      
MAX_DIST_U = np.array([0.0, 0.2, 0.2, 0.0])  
ESO_FILTER_CUTOFF = 1.0

class ESOLQRDataLogger:
    def __init__(self, log_dir="logs", prefix="lqr_STEP1", log_decimation=10):
        self.log_dir = log_dir
        self.prefix = prefix
        self.data = []
        self.log_decimation = log_decimation
        self.log_counter = 0
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
    
    def log(self, sim_time, state, x_ref, u_lqr, delta_u_dist, u_total, 
            motors, eso_d_f_raw, eso_d_f_filt, mode="HOVER", dist_enabled=True):
        self.log_counter += 1
        if self.log_counter % self.log_decimation != 0:
            return
        
        pos_error = sqrt((state[0]-x_ref[0])**2 + (state[1]-x_ref[1])**2 + (state[2]-x_ref[2])**2)
        
        self.data.append({
            't': sim_time, 'mode': mode, 'dist_enabled': dist_enabled,
            'px': state[0], 'py': state[1], 'pz': state[2],
            'vx': state[3], 'vy': state[4], 'vz': state[5],
            'roll': state[6], 'pitch': state[7], 'yaw': state[8],
            'ref_px': x_ref[0], 'ref_py': x_ref[1], 'ref_pz': x_ref[2],
            'u_lqr_0': u_lqr[0], 'u_lqr_1': u_lqr[1], 'u_lqr_2': u_lqr[2],
            'du_dist_0': delta_u_dist[0], 'du_dist_1': delta_u_dist[1], 'du_dist_2': delta_u_dist[2],
            'u_total_0': u_total[0], 'u_total_1': u_total[1], 'u_total_2': u_total[2],
            'm1': motors[0], 'm2': motors[1], 'm3': motors[2], 'm4': motors[3],
            'd_fx_raw': eso_d_f_raw[0], 'd_fy_raw': eso_d_f_raw[1], 'd_fz_raw': eso_d_f_raw[2],
            'd_fx_filt': eso_d_f_filt[0], 'd_fy_filt': eso_d_f_filt[1], 'd_fz_filt': eso_d_f_filt[2],
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
        print(f"📊 Log saved: {filepath}")
        return filepath

class LowPassFilter:
    def __init__(self, Ts, cutoff_freq):
        tau = 1.0 / (2 * np.pi * cutoff_freq)
        self.alpha = Ts / (tau + Ts)
        self.y = 0.0
    
    def filter(self, x):
        self.y = self.alpha * x + (1 - self.alpha) * self.y
        return self.y
    
    def reset(self, initial_value=0.0):
        self.y = initial_value

class ModelBasedESO:
    def __init__(self, Ts, mass, g, filter_cutoff=ESO_FILTER_CUTOFF):
        self.Ts = Ts
        self.mass = mass
        self.g = g
        self.z_hat = np.zeros(12)
        self._build_matrices()
        self._design_observer_gain()
        
        self.lpf_dx = LowPassFilter(Ts, filter_cutoff)
        self.lpf_dy = LowPassFilter(Ts, filter_cutoff)
        self.lpf_dz = LowPassFilter(Ts, filter_cutoff)
        

    
    def _build_matrices(self):
        A = np.zeros((12, 12))
        A[0, 3] = 1.0; A[1, 4] = 1.0; A[2, 5] = 1.0
        A[3, 7] = self.g; A[4, 6] = -self.g
        
        damping_v = -0.2
        A[3, 3] = damping_v; A[4, 4] = damping_v; A[5, 5] = damping_v
        
        A[3, 9] = 1.0   # d_fx → v̇x
        A[4, 10] = 1.0  # d_fy → v̇y
        A[5, 11] = 1.0  # d_fz → v̇z
        
        # disturbance (A[9:12, 9:12] = 0 already)
        
        self.A_cont = A
        
        B = np.zeros((12, 4))
        B[5, 0] = 1.0 / self.mass  # Only thrust
        self.B_cont = B
        
        C = np.zeros((6, 12))
        C[0, 0] = 1.0; C[1, 1] = 1.0; C[2, 2] = 1.0 
        C[3, 6] = 1.0; C[4, 7] = 1.0; C[5, 8] = 1.0  
        self.C = C
        
        self.A_disc = np.eye(12) + self.A_cont * self.Ts
        self.B_disc = self.B_cont * self.Ts
    
    def _design_observer_gain(self):
        # LQR-based observer design (from teammate's design_L.py)
        
        observer_speedup = 1.5    
        position_scale = 0.4     
        velocity_scale = 0.6     
        attitude_scale = 0.5      
        force_dist_scale = 0.3   

  
        Q_controller = np.diag([2000, 2000, 20, 40, 40, 15,
                                30, 30, 2, 6, 6, 3])
        Q_ctrl_diag = np.diag(Q_controller)
        
     
        Q_observer = np.zeros(12)
        Q_observer[0:3] = Q_ctrl_diag[0:3] * observer_speedup * position_scale    # pos
        Q_observer[3:6] = Q_ctrl_diag[3:6] * observer_speedup * velocity_scale    # vel
        Q_observer[6:9] = Q_ctrl_diag[6:9] * observer_speedup * attitude_scale    # att
        Q_observer[9:12] = Q_ctrl_diag[3:6] * force_dist_scale                    # dist
        
        Q_obs = np.diag(Q_observer)
        
        # Measurement noise (v2 stable level)
        R_obs = np.diag([0.02, 0.02, 0.02,  
                         0.08, 0.08, 0.08])  
        
        try:
            P = solve_continuous_are(self.A_cont.T, self.C.T, Q_obs, R_obs)
            L_cont = P @ self.C.T @ np.linalg.inv(R_obs)
            self.L = L_cont * self.Ts
            
            max_gain = np.max(np.abs(self.L))
            
        
            TARGET_MAX_GAIN = 30.0 
            if max_gain > TARGET_MAX_GAIN:
                scale_factor = (TARGET_MAX_GAIN / max_gain) ** 2
                Q_obs_scaled = Q_obs * scale_factor
                P = solve_continuous_are(self.A_cont.T, self.C.T, Q_obs_scaled, R_obs)
                L_cont = P @ self.C.T @ np.linalg.inv(R_obs)
                self.L = L_cont * self.Ts
                max_gain = np.max(np.abs(self.L))
            
            print(f"✓ ESO: LQR-based gains, max={max_gain:.1f}, speedup={observer_speedup}")
        except:
            self._fallback_gains()
    
    def _fallback_gains(self):
        print("⚠️ Using fallback gains")
        self.L = np.zeros((12, 6))
        # Position measurements
        self.L[0, 0] = 0.5 * self.Ts; self.L[1, 1] = 0.5 * self.Ts; self.L[2, 2] = 0.5 * self.Ts
        # Velocity from position
        self.L[3, 0] = 2.0 * self.Ts; self.L[4, 1] = 2.0 * self.Ts; self.L[5, 2] = 2.0 * self.Ts
        # Attitude measurements
        self.L[6, 3] = 0.8 * self.Ts; self.L[7, 4] = 0.8 * self.Ts; self.L[8, 5] = 0.8 * self.Ts
        # Disturbance from position innovation
        self.L[9, 0] = 0.5 * self.Ts; self.L[10, 1] = 0.5 * self.Ts; self.L[11, 2] = 0.2 * self.Ts
    
    def initialize_from_measurement(self, y):
        self.z_hat[0:3] = y[0:3]
        self.z_hat[6:9] = y[3:6]
        self.lpf_dx.reset(0.0)
        self.lpf_dy.reset(0.0)
        self.lpf_dz.reset(0.0)
    
    def step(self, y_meas, u_physical, dt):
        z_pred = self.A_disc @ self.z_hat + self.B_disc @ u_physical
        y_pred = self.C @ z_pred
        innovation = y_meas - y_pred
        

        if not np.isfinite(innovation).all():
            print("⚠️ NaN in innovation, resetting")
            innovation = np.zeros_like(innovation)
        
        self.z_hat = z_pred + self.L @ innovation
        
        # Clip disturbance estimates (increased limits)
        MAX_XY = 5; MAX_Z = 3.0 
        self.z_hat[9] = np.clip(self.z_hat[9], -MAX_XY, MAX_XY)
        self.z_hat[10] = np.clip(self.z_hat[10], -MAX_XY, MAX_XY)
        self.z_hat[11] = np.clip(self.z_hat[11], -MAX_Z, MAX_Z)
        
    
        if not np.isfinite(self.z_hat).all():
            print("⚠️ NaN in z_hat, emergency reset")
            self.z_hat = np.zeros(12)
            return self.z_hat.copy(), np.zeros(3), np.zeros(3)
        
        d_f_raw = self.z_hat[9:12].copy()
        
        d_fx_filt = self.lpf_dx.filter(d_f_raw[0])
        d_fy_filt = self.lpf_dy.filter(d_f_raw[1])
        d_fz_filt = self.lpf_dz.filter(d_f_raw[2])
        d_f_filtered = np.array([d_fx_filt, d_fy_filt, d_fz_filt])
        
        return self.z_hat.copy(), d_f_raw, d_f_filtered

def get_linearized_model(mass=MASS):
    A = np.zeros((12, 12))
    A[0, 3] = 1.0; A[1, 4] = 1.0; A[2, 5] = 1.0
    A[3, 7] = GRAVITY; A[4, 6] = -GRAVITY
    A[6, 9] = 1.0; A[7, 10] = 1.0; A[8, 11] = 1.0
    A[3, 3] = -2.0; A[4, 4] = -2.0; A[5, 5] = -3.0
    A[9, 9] = -10.0; A[10, 10] = -10.0; A[11, 11] = -10.0
    
    B = np.zeros((12, 4))
    B[5, 0] = THRUST_SCALE / mass
    B[9, 1] = MOMENT_SCALE / Ixx
    B[10, 2] = MOMENT_SCALE / Iyy
    B[11, 3] = MOMENT_SCALE / Izz
    
    return A, B

class LQRController:
    def __init__(self, dt=0.01, hover_height=0.5, mass=MASS):
        self.dt = dt
        self.hover_height = hover_height
        self.mass = mass
        self.g = GRAVITY
        
        self.vx_f = self.vy_f = self.vz_f = 0.0
        self.alpha_v = 0.4
        
        self.int_ez = self.int_ex = self.int_ey = 0.0
        self.Ki_z = 0.1
        self.Ki_xy = 0.2       
        self.int_limit_z = 0.05
        self.int_limit_xy = 1  
        
        self.A, self.B = get_linearized_model(mass=mass)
        self.K = self._compute_lqr_gain()
        
        self.u_min = np.array([-0.7, -1, -1, -0.5])
        self.u_max = np.array([+0.7, +1, +1, +0.5])
        
        self.x_ref = np.zeros(12)
        self.x_ref[2] = hover_height
        self.x_ref_current = self.x_ref.copy()
        
        self.ARM_LENGTH = 0.046
        self.THRUST2TORQUE = 0.005022
        self.MOTOR_TO_THRUST = 1.63e-3
        
        print(f"🚁 LQR: Q_pos=2000 (STEP 1: +67% for circle tracking), Ki_xy={self.Ki_xy}")
    
    def _compute_lqr_gain(self):
    
        Q = np.diag([
           
            500.0, 500.0, 20.0,
         
            40.0, 40.0, 15.0,
       
            30.0, 30.0, 2.0,
      
            6.0, 6.0, 3.0
        ])
     
        R = np.diag([
            800.0,    
            1500.0,   
            1500.0,  
            20000.0  
        ])
        
        try:
            P = solve_continuous_are(self.A, self.B, Q, R)
            K = np.linalg.inv(R) @ (self.B.T @ P)
            return K
        except:
            K = np.zeros((4, 12))
            K[0, 2] = 0.01; K[0, 5] = 0.005
            K[1, 1] = 0.001; K[1, 6] = 0.002
            K[2, 0] = 0.001; K[2, 7] = 0.002
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
        
        vx_raw = (px - past_pos[0]) / dt
        vy_raw = (py - past_pos[1]) / dt
        vz_raw = (pz - past_pos[2]) / dt
        
        self.vx_f = (1 - self.alpha_v) * self.vx_f + self.alpha_v * vx_raw
        self.vy_f = (1 - self.alpha_v) * self.vy_f + self.alpha_v * vy_raw
        self.vz_f = (1 - self.alpha_v) * self.vz_f + self.alpha_v * vz_raw
        
        return np.array([px, py, pz, self.vx_f, self.vy_f, self.vz_f,
                        imu[0], imu[1], imu[2], gyro[0], gyro[1], gyro[2]])
    
    def compute_control(self, state):
        e = state - self.x_ref_current
        
        while e[8] > pi: e[8] -= 2*pi
        while e[8] < -pi: e[8] += 2*pi
        
        self.int_ez += e[2] * self.dt
        self.int_ex += e[0] * self.dt
        self.int_ey += e[1] * self.dt
        
        self.int_ez = np.clip(self.int_ez, -self.int_limit_z, self.int_limit_z)
        self.int_ex = np.clip(self.int_ex, -self.int_limit_xy, self.int_limit_xy)
        self.int_ey = np.clip(self.int_ey, -self.int_limit_xy, self.int_limit_xy)
        
        u = -self.K @ e
        u[0] += -self.Ki_z * self.int_ez
        u[1] += -self.Ki_xy * self.int_ey
        u[2] += -self.Ki_xy * self.int_ex
        u[3] = 0.0
        
        u = np.clip(u, self.u_min, self.u_max)
        return u
    
    def control_to_motors(self, u):
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

def compute_disturbance_compensation(eso_d_f_filtered, eso_yaw, mass):
    delta_u_dist = np.zeros(4)
    
    d_xy = np.array([eso_d_f_filtered[0], eso_d_f_filtered[1], 0.0])
    d_world = mass * d_xy
    
    cy, sy = cos(eso_yaw), sin(eso_yaw)
    R_wb = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Fx_b, Fy_b, Fz_b = R_wb.T @ d_world
    
   
    d_tau_x = -TAU_FROM_FORCE * Fy_b  # Negative to compensate
    d_tau_y = -TAU_FROM_FORCE * Fx_b  # Negative to compensate
    
    delta_u_dist[0] = 0.0
    delta_u_dist[1] = DIST_FF_RATIO * K_DIST_CTRL * d_tau_x / MOMENT_SCALE
    delta_u_dist[2] = DIST_FF_RATIO * K_DIST_CTRL * d_tau_y / MOMENT_SCALE
    delta_u_dist[3] = 0.0
    
    delta_u_dist = np.clip(delta_u_dist, -MAX_DIST_U, +MAX_DIST_U)
    
    return delta_u_dist, True

class CircleTrajectory:
    def __init__(self, z=0.5, radius=0.3, period=10.0):
        self.z = z
        self.radius = radius
        self.period = period
        self.omega = 2*pi / period
        self.t0 = 0
        self.cx = self.cy = 0
    
    def start(self, t, cx, cy):
        self.t0 = t
        self.cx = cx
        self.cy = cy
    
    def get_position(self, t):
        dt = t - self.t0
        x = self.cx + self.radius * cos(self.omega * dt)
        y = self.cy + self.radius * sin(self.omega * dt)
        return x, y, self.z
    
    def get_velocity(self, t):
        dt = t - self.t0
        vx = -self.radius * self.omega * sin(self.omega * dt)
        vy = self.radius * self.omega * cos(self.omega * dt)
        return vx, vy, 0.0

class LineTrajectory:
    def __init__(self, z=0.5, distance=0.5, duration=5.0, axis='x'):
        self.z = z
        self.distance = distance
        self.duration = duration
        self.axis = axis
        self.t0 = 0
        self.x0 = self.y0 = 0
    
    def start(self, t, x0, y0):
        self.t0 = t
        self.x0 = x0
        self.y0 = y0
    
    def get_position(self, t):
        dt = t - self.t0
        if dt >= self.duration:
            progress = 1.0
        else:
            progress = dt / self.duration
        
        displacement = progress * self.distance
        
        if self.axis == 'x':
            return (self.x0 + displacement, self.y0, self.z)
        else:
            return (self.x0, self.y0 + displacement, self.z)

class KeyboardDebouncer:
    def __init__(self, debounce_time=0.3):
        self.debounce_time = debounce_time
        self.last_key_time = {}
    
    def is_key_ready(self, key, current_time):
        if key not in self.last_key_time:
            self.last_key_time[key] = current_time
            return True
        if (current_time - self.last_key_time[key]) >= self.debounce_time:
            self.last_key_time[key] = current_time
            return True
        return False

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    Ts = timestep / 1000.0
    
    print("\n" + "="*70)
    print("LQR Tuning - STEP 1 (Conservative Q_pos Increase)")
    print("="*70)
    print(f"🎯 GOAL: Improve circle tracking from 102mm → 60-70mm")
    print(f"")
    print(f"📊 Current Performance (baseline):")
    print(f"   Circle error: 102mm mean")
    print(f"   Phase lag: -10.4°")
    print(f"   Control usage: 31% (too conservative!)")
    print(f"")
    print(f"✅ STEP 1 Changes:")
    print(f"   Q_pos: 1200 → 2000 (+67%)")
    print(f"   Q_vel: 40 (unchanged)")
    print(f"   Ki_xy: 0.3 (unchanged)")
    print(f"")
    print(f"🎯 Expected:")
    print(f"   Error: 60-70mm (~40% improvement)")
    print(f"   Control: ~50% (more aggressive)")
    print(f"   Risk: Low (conservative increase)")
    print(f"")
    print(f"📝 Next Steps:")
    print(f"   If error still >50mm → Try STEP 2 (Q_pos=2500, Q_vel=60)")
    print(f"   If error <50mm → Success! Can test with ESO")
    print("="*70 + "\n")
    
    gps = robot.getDevice("gps"); gps.enable(timestep)
    imu = robot.getDevice("inertial_unit"); imu.enable(timestep)
    gyro = robot.getDevice("gyro"); gyro.enable(timestep)
    keyboard = Keyboard(); keyboard.enable(timestep)
    
    motors = []
    for i in range(1, 5):
        m = robot.getDevice(f"m{i}_motor")
        m.setPosition(float('inf')); m.setVelocity(0)
        motors.append(m)
    
    for _ in range(10):
        robot.step(timestep)
    
    pos0 = np.array(gps.getValues())
    rpy0 = imu.getRollPitchYaw()
    hover_height = 0.5
    
    eso = ModelBasedESO(Ts, MASS, GRAVITY, filter_cutoff=ESO_FILTER_CUTOFF)
    y_init = np.array([pos0[0], pos0[1], pos0[2], rpy0[0], rpy0[1], rpy0[2]])
    eso.initialize_from_measurement(y_init)
    
    lqr = LQRController(dt=Ts, hover_height=hover_height, mass=MASS)
    logger = ESOLQRDataLogger()
    debouncer = KeyboardDebouncer()
    
    circle_traj = CircleTrajectory(z=hover_height, radius=0.3, period=10.0)
    line_traj_x = LineTrajectory(z=hover_height, distance=0.5, duration=5.0, axis='x')
    line_traj_y = LineTrajectory(z=hover_height, distance=0.5, duration=5.0, axis='y')
    
    mode = "HOVER"
    target_x, target_y, target_z = pos0[0], pos0[1], hover_height
    lqr.set_target_position(target_x, target_y, target_z)
    
    past_pos = list(pos0)
    past_time = robot.getTime()
    last_print_time = 0
    prev_u_total = np.zeros(4)
    
    print(f"📋 Controls: Arrows/W/S: Move | T: Circle | X/Y: Line | D: ESO | K: Log\n")
    
    while robot.step(timestep) != -1:
        t = robot.getTime()
        
        gps_val = gps.getValues()
        imu_val = imu.getRollPitchYaw()
        gyro_val = gyro.getValues()
        
        T_phys = MASS * GRAVITY + prev_u_total[0] * THRUST_SCALE
        tau_x_phys = prev_u_total[1] * MOMENT_SCALE
        tau_y_phys = prev_u_total[2] * MOMENT_SCALE
        u_physical = np.array([T_phys, tau_x_phys, tau_y_phys, 0])
        
        y_meas = np.array([gps_val[0], gps_val[1], gps_val[2], imu_val[0], imu_val[1], imu_val[2]])
        z_hat, eso_d_f_raw, eso_d_f_filtered = eso.step(y_meas, u_physical, Ts)
        
        eso_yaw = z_hat[8]
        
        # [Keyboard handling - same as before but abbreviated for space]
        key = keyboard.getKey()
        while key > 0:
            if debouncer.is_key_ready(key, t):
                target_changed = False
                
                if key == Keyboard.UP:
                    target_x += 0.1; mode = "HOVER"; target_changed = True
                elif key == Keyboard.DOWN:
                    target_x -= 0.1; mode = "HOVER"; target_changed = True
                elif key == Keyboard.LEFT:
                    target_y += 0.1; mode = "HOVER"; target_changed = True
                elif key == Keyboard.RIGHT:
                    target_y -= 0.1; mode = "HOVER"; target_changed = True
                elif key == ord('W'):
                    target_z = min(2.0, target_z + 0.1); mode = "HOVER"; target_changed = True
                elif key == ord('S'):
                    target_z = max(0.2, target_z - 0.1); mode = "HOVER"; target_changed = True
                elif key == ord('R') or key == ord('r'):
                    target_x, target_y, target_z = pos0[0], pos0[1], hover_height
                    mode = "HOVER"; target_changed = True
                    lqr.reset_integrators()
                    print("🔄 Reset")
                elif key == ord('T') or key == ord('t'):
                    if mode != "CIRCLE":
                        mode = "CIRCLE"
                        circle_traj.z = target_z
                        circle_traj.start(t, gps_val[0], gps_val[1])
                        lqr.reset_integrators()
                        print(f"🔵 CIRCLE")
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
                        print(f"📏 LINE-X")
                    else:
                        mode = "HOVER"
                        target_x, target_y = gps_val[0], gps_val[1]
                elif key == ord('Y') or key == ord('y'):
                    if mode != "LINE_Y":
                        mode = "LINE_Y"
                        line_traj_y.z = target_z
                        line_traj_y.start(t, gps_val[0], gps_val[1])
                        lqr.reset_integrators()
                        print(f"📏 LINE-Y")
                    else:
                        mode = "HOVER"
                        target_x, target_y = gps_val[0], gps_val[1]
                elif key == ord('D') or key == ord('d'):
                    CONFIG['use_disturbance_compensation'] = not CONFIG['use_disturbance_compensation']
                    status = 'ON' if CONFIG['use_disturbance_compensation'] else 'OFF'
                    print(f"🔧 ESO: {status}")
                elif key == ord('K') or key == ord('k'):
                    logger.save()
                
                if target_changed:
                    lqr.set_target_position(target_x, target_y, target_z)
                    print(f"→ ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            
            key = keyboard.getKey()
        
        # Update reference based on mode
        if mode == "CIRCLE":
            x, y, z = circle_traj.get_position(t)
            vx, vy, vz = circle_traj.get_velocity(t)
            lqr.x_ref_current[0:3] = [x, y, z]
            lqr.x_ref_current[3:6] = [vx, vy, vz]
        elif mode == "LINE_X":
            x, y, z = line_traj_x.get_position(t)
            lqr.x_ref_current[0:3] = [x, y, z]
            lqr.x_ref_current[3:6] = [0, 0, 0]
        elif mode == "LINE_Y":
            x, y, z = line_traj_y.get_position(t)
            lqr.x_ref_current[0:3] = [x, y, z]
            lqr.x_ref_current[3:6] = [0, 0, 0]
        else:
            lqr.x_ref_current[0:3] = [target_x, target_y, target_z]
            lqr.x_ref_current[3:6] = [0, 0, 0]
        
        state = lqr.get_state(gps_val, imu_val, gyro_val, past_pos, past_time, t)
        u_lqr = lqr.compute_control(state)
        
        if CONFIG['use_disturbance_compensation']:
            delta_u_dist, dist_enabled = compute_disturbance_compensation(
                eso_d_f_filtered, eso_yaw, MASS)
        else:
            delta_u_dist = np.zeros(4)
            dist_enabled = False
        
        u_total = u_lqr + delta_u_dist
        u_total = np.clip(u_total, lqr.u_min, lqr.u_max)
        prev_u_total = u_total.copy()
        
        motor_cmds = lqr.control_to_motors(u_total)
        
        logger.log(t, state, lqr.x_ref_current, u_lqr, delta_u_dist, u_total,
                   motor_cmds, eso_d_f_raw, eso_d_f_filtered, mode, dist_enabled)
        
        motors[0].setVelocity(-motor_cmds[0])
        motors[1].setVelocity(motor_cmds[1])
        motors[2].setVelocity(-motor_cmds[2])
        motors[3].setVelocity(motor_cmds[3])
        
        if t - last_print_time >= 1.0:
            pos_err = sqrt((gps_val[0]-lqr.x_ref_current[0])**2 +
                          (gps_val[1]-lqr.x_ref_current[1])**2 +
                          (gps_val[2]-lqr.x_ref_current[2])**2)
            
            print(f"[{t:5.1f}s] {mode:7s} | "
                  f"err:{pos_err:.4f}m | "
                  f"d:({eso_d_f_filtered[0]:+.2f},{eso_d_f_filtered[1]:+.2f})")
            last_print_time = t
        
        if abs(imu_val[0]) > 1.0 or abs(imu_val[1]) > 1.0:
            print(f"🛑 FLIP!")
            for m in motors:
                m.setVelocity(0)
            logger.save()
            break
        
        past_pos = list(gps_val)
        past_time = t
        
        if t > 60.0:
            logger.save()
            break

if __name__ == '__main__':
    main()