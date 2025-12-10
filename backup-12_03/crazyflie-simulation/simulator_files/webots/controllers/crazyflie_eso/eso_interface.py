"""
ESO Interface for External Controllers (LQR, TinyMPC, etc.)

Provides clean API to read ESO estimates and convert disturbances 
to control input format (thrust + torques).
"""

import numpy as np
import json
import os
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class ESOState:
    """Container for ESO state estimates."""
    # Position (world frame, meters)
    position: np.ndarray  # [x, y, z]
    
    # Velocity (world frame, m/s)
    velocity: np.ndarray  # [vx, vy, vz]
    
    # Attitude (Euler angles, radians)
    attitude: np.ndarray  # [roll, pitch, yaw]
    
    # Force disturbances (world frame, N)
    force_disturbance: np.ndarray  # [dx, dy, dz]
    
    # Timestamp
    timestamp: float


@dataclass
class ESODisturbanceControl:
    """Disturbances expressed as equivalent control inputs."""
    # Thrust disturbance (N) - in body Z direction
    thrust_disturbance: float
    
    # Torque disturbances (N·m) - in body frame
    torque_disturbance: np.ndarray  # [τx, τy, τz]
    
    # Original force disturbance (world frame, for reference)
    force_disturbance_world: np.ndarray  # [dx, dy, dz]


class ESOInterface:
    """
    Interface between ESO and external controllers.
    
    Typical usage inside a controller loop:
    
        interface = ESOInterface(eso, mass=m)
        
        # each step:
        roll, pitch, yaw = current_attitude
        u_nominal = -K @ e       # from LQR/MPC/etc.
        u_cmd = interface.apply_feedforward(u_nominal, roll, pitch, yaw)
    """
    
    def __init__(self, eso, mass: float, inertia: Optional[np.ndarray] = None):
        """
        Args:
            eso: ESO object (9-state, 12-state, 15-state, or 18-state)
            mass: drone mass (kg)
            inertia: 3x3 inertia matrix (kg·m²), optional
        """
        self.eso = eso
        self.mass = mass
        
        if inertia is None:
            # Default Crazyflie inertia
            inertia = np.diag([1.4e-5, 1.4e-5, 2.17e-5])
        self.inertia = np.array(inertia)
        
        # Determine ESO type from state size
        self.eso_type = self._detect_eso_type()
        
        # Logging
        self.log_file = None
        self.enable_logging = False
    
    def _detect_eso_type(self) -> str:
        """Detect whether ESO is 9-state, 12-state, 15-state, or 18-state."""
        state_size = len(self.eso.z)
        if state_size == 9:
            return "9-state"   # pos, vel, force_dist
        elif state_size == 12:
            return "12-state"  # pos, vel, attitude, force_dist
        elif state_size == 15:
            return "15-state"  # pos, vel, attitude, omega, force_dist
        elif state_size == 18:
            return "18-state"  # full state (incl. torque disturbances)
        else:
            raise ValueError(f"Unknown ESO type with {state_size} states")
    
    def get_state(self) -> ESOState:
        """
        Get current ESO state estimates.
        
        Returns:
            ESOState object with position, velocity, attitude, disturbances
        """
        z = self.eso.z
        
        if self.eso_type == "9-state":
            # 9-state: [x,y,z, vx,vy,vz, dx,dy,dz]
            position = z[0:3]
            velocity = z[3:6]
            attitude = np.array([0.0, 0.0, 0.0])  # Not estimated
            force_dist = z[6:9]
            
        elif self.eso_type == "12-state":
            # 12-state: [x,y,z, vx,vy,vz, φ,θ,ψ, dx,dy,dz]
            position = z[0:3]
            velocity = z[3:6]
            attitude = z[6:9]
            force_dist = z[9:12]
            
        elif self.eso_type == "15-state":
            # 15-state: [x,y,z, vx,vy,vz, φ,θ,ψ, p,q,r, dx,dy,dz]
            position = z[0:3]
            velocity = z[3:6]
            attitude = z[6:9]
            force_dist = z[12:15]
            
        elif self.eso_type == "18-state":
            # 18-state: [x,y,z, vx,vy,vz, φ,θ,ψ, p,q,r, dx,dy,dz, τx,τy,τz]
            position = z[0:3]
            velocity = z[3:6]
            attitude = z[6:9]
            force_dist = z[12:15]
        
        return ESOState(
            position=position.copy(),
            velocity=velocity.copy(),
            attitude=attitude.copy(),
            force_disturbance=force_dist.copy(),
            timestamp=0.0  # Set externally if needed
        )
    
    def get_disturbances_as_control(self, roll: float, pitch: float, yaw: float) -> ESODisturbanceControl:
        """
        Convert ESO force disturbances to equivalent control inputs.
        
        For feedforward compensation:
            u_compensated = u_nominal - disturbance_control
        
        Args:
            roll, pitch, yaw: Current attitude (for rotation matrix)
        
        Returns:
            ESODisturbanceControl with thrust and torque disturbances.
        """
        state = self.get_state()
        d_f_world = state.force_disturbance  # [dx, dy, dz] in world frame
        
        # Rotation matrix: world -> body
        R_world_to_body = self._rotation_matrix(roll, pitch, yaw).T
        
        # Transform force disturbance to body frame
        d_f_body = R_world_to_body @ d_f_world
        
        # Disturbance along body z axis (thrust direction)
        thrust_disturbance = d_f_body[2]
        
        # Torque disturbances: either directly from ESO (18-state)
        # or approximated from lateral forces + arm length.
        if self.eso_type == "18-state":
            torque_disturbance = self.eso.z[15:18].copy()
        else:
            # Changed: correct Crazyflie torque mapping from force disturbances
            # ---------------------------------------------------------------
            # Crazyflie arm length (meters)
            arm_length = 0.0465

            # Roll torque:
            #   Positive body-y force → positive roll moment
            tau_roll = d_f_body[1] * arm_length

            # Pitch torque:
            #   Positive body-x force → *negative* pitch moment
            tau_pitch = -d_f_body[0] * arm_length

            # Yaw torque:
            #   No direct mapping from force disturbances for CF
            tau_yaw = 0.0

            torque_disturbance = np.array([tau_roll, tau_pitch, tau_yaw])
            # ---------------------------------------------------------------
                    
        return ESODisturbanceControl(
            thrust_disturbance=thrust_disturbance,
            torque_disturbance=torque_disturbance,
            force_disturbance_world=d_f_world.copy()
        )
    
    @staticmethod
    def _rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Rotation matrix body->world (ZYX Euler)."""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        return np.array([
            [cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy],
            [cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy],
            [-sp,   sr*cp,            cr*cp]
        ])
    
    def get_state_vector(self) -> np.ndarray:
        """
        Get full state as vector for LQR/MPC controllers.
        
        Returns:
            State vector [x, y, z, vx, vy, vz, φ, θ, ψ, p, q, r]
            
        Note: If ESO doesn't estimate some states (e.g., 9-state doesn't 
              estimate attitude), those will be zero. Pass measured values 
              separately.
        """
        state = self.get_state()
        
        # Basic state vector (works for all ESO types)
        state_vec = np.concatenate([
            state.position,   # [x, y, z]
            state.velocity,   # [vx, vy, vz]
            state.attitude,   # [φ, θ, ψ]
        ])
        
        # Add angular velocity if available
        if self.eso_type in ["15-state", "18-state"]:
            omega = self.eso.z[9:12]
            state_vec = np.concatenate([state_vec, omega])
        else:
            # Angular velocity not estimated - append zeros
            state_vec = np.concatenate([state_vec, np.zeros(3)])
        
        return state_vec
    
    def enable_file_logging(self, log_dir: str = "./eso_logs"):
        """Enable logging ESO states to file for offline analysis."""
        os.makedirs(log_dir, exist_ok=True)
        
        timestamp = np.datetime64('now').astype(str).replace(':', '-')
        self.log_file = os.path.join(log_dir, f"eso_log_{timestamp}.json")
        self.enable_logging = True
        
        print(f"ESO logging enabled: {self.log_file}")
    
    def log_state(self, timestamp: float, measured_attitude: Optional[np.ndarray] = None):
        """Log current ESO state to file."""
        if not self.enable_logging:
            return
        
        state = self.get_state()
        
        log_entry = {
            "timestamp": timestamp,
            "position": state.position.tolist(),
            "velocity": state.velocity.tolist(),
            "attitude": state.attitude.tolist(),
            "force_disturbance": state.force_disturbance.tolist(),
        }
        
        if measured_attitude is not None:
            log_entry["measured_attitude"] = measured_attitude.tolist()
        
        # Append to file
        with open(self.log_file, 'a') as f:
            f.write(json.dumps(log_entry) + '\n')

    # Changed: central entry point so controller never touches torque logic
    def apply_feedforward(
        self,
        u_nominal: np.ndarray,
        roll: float,
        pitch: float,
        yaw: float,
        use_thrust_feedforward: bool = False,
    ) -> np.ndarray:
        """
        Apply ESO-based disturbance feedforward to a nominal control input.
        
        Args:
            u_nominal:
                Nominal control vector [T, τx, τy, τz] from LQR/MPC/etc.
            roll, pitch, yaw:
                Current attitude (for transforming disturbances).
            use_thrust_feedforward:
                If True, also compensate thrust with ESO disturbance.
                For controllers that already have a safe thrust PID
                (altitude loop), leave this as False.
        
        Returns:
            u_corrected:
                Control vector after disturbance compensation, same shape
                as u_nominal.
        """
        u_nominal = np.asarray(u_nominal, dtype=float).reshape(-1)
        if u_nominal.shape[0] != 4:
            raise ValueError(
                f"u_nominal must be length 4 [T, τx, τy, τz], got {u_nominal.shape[0]}"
            )

        # Get disturbance in control coordinates
        dist = self.get_disturbances_as_control(roll, pitch, yaw)
        force_gain, torque_gain = self.get_compensation_gains()

        u_corrected = u_nominal.copy()

        # Optional thrust feedforward
        if use_thrust_feedforward:
            u_corrected[0] -= force_gain * dist.thrust_disturbance

        # Torque feedforward
        u_corrected[1:4] -= torque_gain * dist.torque_disturbance

        return u_corrected
    
    def get_compensation_gains(self) -> Tuple[float, float]:
        """
        Get recommended disturbance compensation gains.
        
        Returns:
            (force_gain, torque_gain) - multiply disturbances by these.
            
        Typical values:
            force_gain = 0.8-1.0 (aggressive compensation)
            torque_gain = 0.5-0.8 (conservative, avoid instability)
        """
        return (0.9, 0.7)


def load_eso_interface(eso, mass: float = 0.031) -> ESOInterface:
    """
    Convenience function to create ESO interface.
    
    Usage:
        from eso_interface import load_eso_interface
        
        interface = load_eso_interface(eso, mass=0.031)
    """
    return ESOInterface(eso, mass)
