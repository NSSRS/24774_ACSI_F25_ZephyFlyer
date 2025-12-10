import numpy as np

class AttitudeESO:
    """
    12-state Extended State Observer for position, velocity, attitude, and force disturbances.
    
    Uses GPS position AND IMU attitude measurements.
    Uses gyro angular rates as inputs (not estimated).
    
    State z (12×1):
        z = [ x, y, z,           # position (world frame)
              vx, vy, vz,         # velocity (world frame)
              φ, θ, ψ,            # attitude (roll, pitch, yaw) - ESTIMATED
              dx, dy, dz ]        # force disturbance (world frame, N)
    
    Measurements y (6×1):
        y = [ x, y, z,            # GPS position
              φ, θ, ψ ]           # IMU attitude
    
    Inputs:
        u = T (scalar thrust, N)
        omega_body = [p, q, r] (angular rates from gyro)
    """

    def __init__(self, Ts, L, m=0.027, g=9.81):
        self.Ts = Ts
        self.m = float(m)
        self.g = float(g)

        # ESO gain (12×6)
        self.L = np.array(L, dtype=float).reshape(12, 6)
        self.z = np.zeros(12)

    @staticmethod
    def _R(roll, pitch, yaw):
        """Rotation matrix body->world (ZYX Euler convention)"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        return np.array([
            [cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy],
            [cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy],
            [-sp,   sr*cp,            cr*cp]
        ])

    @staticmethod
    def _E(roll, pitch, yaw):
        """Euler kinematics matrix (body rates -> Euler rates)"""
        cr, sr = np.cos(roll), np.sin(roll)
        
        # Clamp pitch to avoid singularity at ±90°
        PITCH_LIMIT = np.pi/2 - 0.2  # Stay 11.5° away from vertical
        pitch_safe = np.clip(pitch, -PITCH_LIMIT, PITCH_LIMIT)
        cp, sp = np.cos(pitch_safe), np.sin(pitch_safe)
        tp = np.tan(pitch_safe)

        return np.array([
            [1.0,  sr*tp,  cr*tp],
            [0.0,  cr,    -sr],
            [0.0,  sr/cp,  cr/cp],
        ])

    def f_continuous(self, z, u, omega_body):
        """
        Continuous dynamics with gyro rates as input.
        
        Args:
            z: state [x,y,z, vx,vy,vz, φ,θ,ψ, dx,dy,dz]
            u: scalar thrust T (in Newtons)
            omega_body: angular rates [p,q,r] from gyro (rad/s)
        """
        z = np.asarray(z).reshape(12)
        T = float(u)
        omega_body = np.asarray(omega_body).reshape(3)

        pos = z[0:3]
        vel = z[3:6]
        attitude = z[6:9]
        d_f = z[9:12]

        roll, pitch, yaw = attitude
        
        # Position dynamics
        dp = vel

        # Velocity dynamics
        R = self._R(roll, pitch, yaw)
        thrust_body = np.array([0, 0, T])
        thrust_world = R @ thrust_body
        dv = -np.array([0, 0, self.g]) + thrust_world / self.m + d_f

        # Attitude dynamics (using gyro measurements)
        E = self._E(roll, pitch, yaw)
        dattitude = E @ omega_body

        # Disturbance dynamics (constant/slowly-varying)
        dd_f = np.zeros(3)

        return np.concatenate([dp, dv, dattitude, dd_f])

    def jacobian_x(self, z, u, omega_body):
        """Jacobian w.r.t. state using central differences."""
        z = np.asarray(z)
        n = len(z)
        A = np.zeros((n, n))
        
        for i in range(n):
            eps = max(1e-6, 1e-5 * abs(z[i]))
            
            zp = z.copy()
            zm = z.copy()
            zp[i] += eps
            zm[i] -= eps
            
            fp = self.f_continuous(zp, u, omega_body)
            fm = self.f_continuous(zm, u, omega_body)
            A[:, i] = (fp - fm) / (2 * eps)
        
        return A

    def jacobian_u(self, z, u, omega_body):
        """Jacobian w.r.t. thrust input."""
        z = np.asarray(z)
        B = np.zeros((len(z), 1))
        
        eps = max(1e-6, 1e-5 * abs(u)) if abs(u) > 1e-8 else 1e-6
        
        fp = self.f_continuous(z, u + eps, omega_body)
        fm = self.f_continuous(z, u - eps, omega_body)
        B[:, 0] = (fp - fm) / (2 * eps)
        
        return B

    def linearize_continuous(self, z, u, omega_body):
        """Linearize dynamics around current state."""
        A = self.jacobian_x(z, u, omega_body)
        B = self.jacobian_u(z, u, omega_body)
        return A, B

    def step(self, y_meas, u_thrust, omega_body):
        """
        Discrete-time predictor-corrector ESO update.
        
        Args:
            y_meas: measurements [x,y,z, φ,θ,ψ] (6 values)
            u_thrust: scalar thrust T (Newtons)
            omega_body: angular rates [p,q,r] from gyro (rad/s)
        """
        Ts = self.Ts
        
        # PREDICT
        z_pred = self.z + Ts * self.f_continuous(self.z, u_thrust, omega_body)
        
        # INNOVATION (from predicted measurements)
        y_pred = np.array([
            z_pred[0], z_pred[1], z_pred[2],  # position
            z_pred[6], z_pred[7], z_pred[8]   # attitude
        ])
        r = y_meas - y_pred
        
        # CORRECT
        self.z = z_pred + self.L @ r
        
        return self.z

    def initialize_from_measurement(self, y_meas):
        """
        Initialize ESO from first measurement.
        
        Args:
            y_meas: [x, y, z, φ, θ, ψ]
        """
        y = np.asarray(y_meas)
        self.z[:] = 0.0
        self.z[0:3] = y[0:3]  # position
        self.z[6:9] = y[3:6]  # attitude
        # velocity and disturbances initialize to zero
        return self.z