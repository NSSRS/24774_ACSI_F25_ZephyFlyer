import numpy as np
from scipy.linalg import solve_continuous_are

def compute_L(eso, mass):
    """
    Observer gain design using Q/R from your tuned LQR controller.
    """
    
    print("\n" + "="*60)
    print("OBSERVER GAIN DESIGN (From Controller LQR)")
    print("="*60)
    
    # ------------------------------------------------------------
    # 1) Get system matrices
    # ------------------------------------------------------------
    z_eq = np.zeros(18)
    z_eq[6] = 0.01
    z_eq[7] = 0.01
    u_eq = np.array([mass * 9.81, 0.0, 0.0, 0.0])
    
    A, _ = eso.linearize_continuous(z_eq, u_eq)
    A_mod = A.copy()
    
    # Add disturbance structure
    A_mod[12:15, 12:15] = 0
    A_mod[15:18, 15:18] = 0
    A_mod[3:6, 12:15] = np.eye(3)
    A_mod[9:12, 15:18] = eso.J_inv
    
    C = np.zeros((6, 18))
    C[0:3, 0:3] = np.eye(3)
    C[3:6, 6:9] = np.eye(3)
    
    # Check observability
    n = A_mod.shape[0]
    O = C
    for i in range(1, n):
        O = np.vstack([O, C @ np.linalg.matrix_power(A_mod, i)])
    rank = np.linalg.matrix_rank(O)
    print(f"Observability: rank={rank}/{n}")
    
    # ------------------------------------------------------------
    # 2) YOUR CONTROLLER Q/R (from LQR controller)
    # ------------------------------------------------------------
    
    # Baseline Q from your controller
    Q_controller = np.diag([
        50, 50, 20,     # roll, pitch, yaw
        10, 10, 5,      # p, q, r
        30, 30, 60,     # x, y, z
        10, 10, 20      # vx, vy, vz
    ])
    
    R_controller = np.diag([1e-3, 1e-3, 1e-3, 1e-3])
    
    # Wind-optimized (alternative)
    Q_wind = np.diag([
        80, 80, 20,    # roll, pitch, yaw
        12, 12, 4,     # p, q, r
        40, 40, 80,    # x, y, z
        12, 12, 25     # vx, vy, vz
    ])
    
    R_wind = np.diag([6e-4, 8e-4, 8e-4, 1.2e-3])
    
    # Choose which Q/R to use
    USE_WIND_TUNING = False  # Set to True if flying in wind
    
    if USE_WIND_TUNING:
        Q_ctrl = Q_wind
        R_ctrl = R_wind
        print("Using WIND-OPTIMIZED Q/R")
    else:
        Q_ctrl = Q_controller
        R_ctrl = R_controller
        print("Using BASELINE Q/R")
    
    # ============================================
    # MANUAL TUNING KNOBS - ADJUST THESE!
    # ============================================
    
    # Reduce these if gains are too high (slower observer)
    observer_speedup = 0.8
    position_scale = 0.15
    velocity_scale = 0.15
    attitude_scale = 0.15
    omega_scale = 0.15
    force_dist_scale = 0.05
    torque_dist_scale = 0.02
    gps_noise = 0.1
    imu_noise = 0.05
    
    # ============================================
    
    print(f"\nTuning parameters:")
    print(f"  observer_speedup = {observer_speedup}")
    print(f"  velocity_scale = {velocity_scale}")
    print(f"  omega_scale = {omega_scale}")

    
    # ------------------------------------------------------------
    # 3) Map controller Q to observer Q (18-state ESO)
    # ------------------------------------------------------------

    # Your controller state order: [roll,pitch,yaw, p,q,r, x,y,z, vx,vy,vz]
    # ESO state order: [x,y,z, vx,vy,vz, roll,pitch,yaw, p,q,r, dx,dy,dz, τx,τy,τz]

    # Extract diagonal from controller Q matrix
    Q_ctrl_diag = np.diag(Q_ctrl)

    # Map controller Q (12 states) to observer Q (18 states)
    Q_observer = np.zeros(18)

    # NOTE: Your controller has order [roll,pitch,yaw, p,q,r, x,y,z, vx,vy,vz]
    #       indices:                   [0,1,2,         3,4,5, 6,7,8, 9,10,11]

    # Position states (x, y, z) - indices 0:3 in ESO
    # In controller Q, x,y,z are at indices 6,7,8
    Q_observer[0:3] = Q_ctrl_diag[6:9] * observer_speedup * position_scale

    # Velocity states (vx, vy, vz) - indices 3:6 in ESO
    # In controller Q, vx,vy,vz are at indices 9,10,11
    Q_observer[3:6] = Q_ctrl_diag[9:12] * observer_speedup * velocity_scale

    # Attitude states (roll, pitch, yaw) - indices 6:9 in ESO
    # In controller Q, roll,pitch,yaw are at indices 0,1,2
    Q_observer[6:9] = Q_ctrl_diag[0:3] * observer_speedup * attitude_scale

    # Angular velocity (p, q, r) - indices 9:12 in ESO
    # In controller Q, p,q,r are at indices 3,4,5
    Q_observer[9:12] = Q_ctrl_diag[3:6] * observer_speedup * omega_scale

    # Force disturbances (dx, dy, dz) - indices 12:15
    # Scale from velocity weights
    Q_observer[12:15] = Q_ctrl_diag[9:12] * force_dist_scale

    # Torque disturbances (τx, τy, τz) - indices 15:18
    # Scale from angular velocity weights
    Q_observer[15:18] = Q_ctrl_diag[3:6] * torque_dist_scale

    Q_observer = np.diag(Q_observer)
    
    # Observer R: measurement noise covariance
    R_observer = np.diag([
        gps_noise, gps_noise, gps_noise,      # GPS
        imu_noise, imu_noise, imu_noise       # IMU
    ])
    
    print(f"\nQ_observer range: [{np.min(np.diag(Q_observer)):.2f}, {np.max(np.diag(Q_observer)):.2f}]")
    print(f"R_observer range: [{np.min(np.diag(R_observer)):.4f}, {np.max(np.diag(R_observer)):.4f}]")
    
    # ------------------------------------------------------------
    # 4) Solve observer Riccati equation
    # ------------------------------------------------------------
    try:
        P = solve_continuous_are(A_mod.T, C.T, Q_observer, R_observer)
        L_continuous = P @ C.T @ np.linalg.inv(R_observer)
        L_discrete = eso.Ts * L_continuous
        
        max_gain = np.max(np.abs(L_discrete))
        print(f"\n✓ Observer gain computed")
        print(f"Max continuous gain: {np.max(np.abs(L_continuous)):.2f}")
        print(f"Max discrete gain: {max_gain:.2f}")
        
        # ------------------------------------------------------------
        # 5) Auto-scaling if still too high
        # ------------------------------------------------------------
        TARGET_MAX_GAIN = 50.0
        
        if max_gain > TARGET_MAX_GAIN:
            print(f"\n⚠ Gain still too high ({max_gain:.1f} > {TARGET_MAX_GAIN})")
            print(f"Auto-scaling Q_observer...")
            
            scale_factor = (TARGET_MAX_GAIN / max_gain) ** 2
            Q_observer_scaled = Q_observer * scale_factor
            
            P = solve_continuous_are(A_mod.T, C.T, Q_observer_scaled, R_observer)
            L_continuous = P @ C.T @ np.linalg.inv(R_observer)
            L_discrete = eso.Ts * L_continuous
            max_gain = np.max(np.abs(L_discrete))
            
            print(f"✓ Auto-scaled max discrete gain: {max_gain:.2f}")
        
        # Verify stability
        A_cl = A_mod - L_continuous @ C
        eigenvalues = np.linalg.eigvals(A_cl)
        max_real_part = np.max(eigenvalues.real)
        
        print(f"\nClosed-loop stability:")
        print(f"Max eigenvalue real part: {max_real_part:.2f} (must be < 0)")
        
        if max_real_part >= 0:
            raise ValueError("Unstable observer!")
        
        # Show bandwidth
        freqs_hz = -eigenvalues.real / (2 * np.pi)
        print(f"Observer bandwidth: {freqs_hz.min():.2f} - {freqs_hz.max():.2f} Hz")
        
        print("="*60 + "\n")
        return L_discrete
        
    except Exception as e:
        print(f"\n✗ LQR observer design failed: {e}")
        print("Using conservative fallback gains...\n")
        
        # Fallback
        L = np.zeros((18, 6))
        L[0:3, 0:3] = 1.5 * eso.Ts * np.eye(3)
        L[3:6, 0:3] = 2.5 * eso.Ts * np.eye(3)
        L[6:9, 3:6] = 1.5 * eso.Ts * np.eye(3)
        L[9:12, 3:6] = 2.5 * eso.Ts * np.eye(3)
        L[12:15, 0:3] = 0.5 * eso.Ts * np.eye(3)
        L[15:18, 3:6] = 0.5 * eso.Ts * np.eye(3)
        
        print(f"Fallback max gain: {np.max(np.abs(L)):.2f}")
        print("="*60 + "\n")
        return L