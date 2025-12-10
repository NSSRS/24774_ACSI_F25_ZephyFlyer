import numpy as np

class FullStateESO:
    """
    Full nonlinear 18-state Extended State Observer for Crazyflie-style quadrotor.
    """

    def __init__(self, Ts, L, m=0.027, J=None, g=9.81):
        self.Ts = Ts
        self.m = float(m)
        self.g = float(g)

        if J is None:
            J = np.diag([1.395e-5, 1.436e-5, 2.173e-5])
        self.J = np.array(J, dtype=float)
        self.J_inv = np.linalg.inv(self.J)

        # ESO gain (18×6) - MUST be discrete gain
        self.L = np.array(L, dtype=float).reshape(18, 6)
        self.z = np.zeros(18)

    @staticmethod
    def _R(eta):
        phi, theta, psi = eta
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)

        return np.array([
            [cth*cpsi,  cth*spsi, -sth],
            [sphi*sth*cpsi - cphi*spsi,
             sphi*sth*spsi + cphi*cpsi,
             sphi*cth],
            [cphi*sth*cpsi + sphi*spsi,
             cphi*sth*spsi - sphi*cpsi,
             cphi*cth]
        ])

    @staticmethod
    def _E(eta):
        phi, theta, _ = eta
        cphi, sphi = np.cos(phi), np.sin(phi)
        
        # Clamp theta to avoid singularity
        theta_safe = np.clip(theta, -np.pi/2 + 0.1, np.pi/2 - 0.1)
        cth, sth = np.cos(theta_safe), np.sin(theta_safe)
        tan_th = np.tan(theta_safe)

        return np.array([
            [1.0,  sphi*tan_th,  cphi*tan_th],
            [0.0,  cphi,        -sphi],
            [0.0,  sphi/cth,     cphi/cth],
        ])

    def f_continuous(self, z, u):
        z = np.asarray(z).reshape(18)
        u = np.asarray(u).reshape(4)

        p = z[0:3]
        v = z[3:6]
        eta = z[6:9]
        omega = z[9:12]
        d_f = z[12:15]
        d_tau = z[15:18]

        T = u[0]
        tau = u[1:4]

        R = self._R(eta)
        E = self._E(eta)

        dp = v
        
        thrust_world = R @ np.array([0, 0, T])
        dv = -np.array([0, 0, self.g]) + thrust_world / self.m + d_f

        deta = E @ omega

        domega = self.J_inv @ (tau - np.cross(omega, self.J @ omega) + d_tau)

        dd_f = np.zeros(3)
        dd_tau = np.zeros(3)

        return np.concatenate([dp, dv, deta, domega, dd_f, dd_tau])

    def jacobian_x(self, z, u):
        """Improved Jacobian with central differences."""
        z = np.asarray(z)
        u = np.asarray(u)
        n = len(z)
        A = np.zeros((n, n))
        
        for i in range(n):
            # Adaptive epsilon
            eps = max(1e-6, 1e-5 * abs(z[i]))
            
            zp = z.copy()
            zm = z.copy()
            zp[i] += eps
            zm[i] -= eps
            
            fp = self.f_continuous(zp, u)
            fm = self.f_continuous(zm, u)
            A[:, i] = (fp - fm) / (2 * eps)
        
        return A

    def jacobian_u(self, z, u):
        """Improved Jacobian with central differences."""
        z = np.asarray(z)
        u = np.asarray(u)
        m = len(u)
        B = np.zeros((len(z), m))
        
        for j in range(m):
            eps = max(1e-6, 1e-5 * abs(u[j]))
            
            up = u.copy()
            um = u.copy()
            up[j] += eps
            um[j] -= eps
            
            fp = self.f_continuous(z, up)
            fm = self.f_continuous(z, um)
            B[:, j] = (fp - fm) / (2 * eps)
        
        return B

    def linearize_continuous(self, z, u):
        A = self.jacobian_x(z, u)
        B = self.jacobian_u(z, u)
        return A, B

    def linearize_discrete(self, z, u):
        A, B = self.linearize_continuous(z, u)
        Ts = self.Ts
        Ad = np.eye(18) + Ts * A
        Bd = Ts * B
        return Ad, Bd

    def step(self, y_meas, u):
        """
        Discrete-time predictor-corrector ESO update.
        CRITICAL: L must be the discrete gain (not continuous).
        """
        Ts = self.Ts
        
        # PREDICT
        z_pred = self.z + Ts * self.f_continuous(self.z, u)
        
        # INNOVATION from predicted state
        y_pred = np.array([
            z_pred[0], z_pred[1], z_pred[2],
            z_pred[6], z_pred[7], z_pred[8]
        ])
        r = y_meas - y_pred
        
        # CORRECT
        self.z = z_pred + self.L @ r
        
        return self.z

    def initialize_from_measurement(self, y_meas):
        y = np.asarray(y_meas)
        self.z[:] = 0.0
        self.z[0:3] = y[0:3]
        self.z[6:9] = y[3:6]
        return self.z