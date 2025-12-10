# Project Status Report: CrazyFlie 2.1 Simulation Control System

**Date**: 2025-10-29
**Branch**: `feature/crazyflie-sim`
**Status**: Development - Debugging PID Controller Stability

---

## Executive Summary

This project implements a MuJoCo-based physics simulation for the CrazyFlie 2.1 drone with cascade PID control. The simulation environment is fully functional, but the current PID controller implementation exhibits instability issues preventing stable hovering at the target position.

---

## Project Structure

```
24774_ACSI_F25_ZephyFlyer/
├── simulation/
│   ├── crazyflie_sim/          # MuJoCo simulation environment
│   │   ├── scene.xml           # Main simulation scene
│   │   ├── cf2.xml             # CrazyFlie 2.1 drone model
│   │   └── assets/             # 3D mesh files
│   ├── hover_demo.py           # Basic hover demonstration
│   ├── keyboard_control.py     # Manual keyboard control
│   ├── cascade_pid_control.py  # Initial PID controller
│   └── cascade_pid_improved.py # Current improved PID controller
└── tester_files/               # Hardware test scripts (CrazyFlie Python library)
```

---

## Completed Work

### 1. MuJoCo Simulation Environment
- **Model**: CrazyFlie 2.1 with simplified dynamics
- **Control Interface**:
  - 4 actuators: `body_thrust`, `x_moment` (roll), `y_moment` (pitch), `z_moment` (yaw)
  - 3 sensors: gyroscope, accelerometer, orientation (quaternion)
- **Hover Keyframe**: Predefined stable hover state at 0.1m altitude with thrust=0.26487N

**Note**: The model uses direct force/moment control rather than individual motor RPM for simplified dynamics.

### 2. Control System Development

#### Version History:
1. **hover_demo.py**: Basic constant-thrust hover using keyframe
2. **keyboard_control.py**: Interactive manual control with arrow keys
3. **cascade_pid_control.py**: Initial cascade PID implementation
4. **cascade_pid_improved.py**: Current version with stability enhancements

#### Current Controller Features (`cascade_pid_improved.py`):
- ✓ Conditional integration (disabled when error > 0.5m)
- ✓ Integral term clamping (prevents windup)
- ✓ Reduced Ki gains (conservative tuning)
- ✓ Tilt angle limiting (max 15°)
- ✓ Emergency reset mechanism (triggers at error > 2m)
- ✓ Derivative filtering (noise reduction)
- ✓ Anti-windup back-calculation
- ✓ Comprehensive diagnostic logging

### 3. Git History
```
f1336fa Add improved cascade PID with enhanced stability
4ddc0c2 Add cascade PID controller with motor mixing
980c2ee Update CLAUDE.md with MuJoCo simulation documentation
3c3c9e5 Increase keyboard control sensitivity and add debug output
58d1df7 Add keyboard control simulation for CrazyFlie 2.1
ce246bd Add MuJoCo hover simulation for CrazyFlie 2.1
```

---

## Current Problem: PID Controller Instability

### Problem Statement
**The cascade PID controller fails to stabilize the drone at the target position (0, 0, 0.5m).**

### Symptoms
- Oscillatory behavior around setpoint
- Position error fails to converge to zero
- Motor saturation may occur
- Emergency resets may trigger

### Controller Architecture

```
Target Position (x, y, z)
         ↓
    ┌─────────────────────┐
    │  OUTER LOOP (PID)   │  Position Control
    │  - X, Y, Z PIDs     │
    └─────────────────────┘
         ↓
    Desired Attitude (roll, pitch) + Thrust
         ↓
    ┌─────────────────────┐
    │  INNER LOOP (PID)   │  Attitude Control
    │  - Roll, Pitch, Yaw │
    └─────────────────────┘
         ↓
    Desired Forces/Moments
         ↓
    ┌─────────────────────┐
    │   MOTOR MIXER       │  4×4 Matrix
    └─────────────────────┘
         ↓
    Individual Motor Thrusts (4 motors)
```

### Current PID Parameters

**File**: `cascade_pid_improved.py:180-215`

#### Outer Loop (Position Control)
```python
# Z-axis (altitude)
Kp = 8.0, Ki = 1.5, Kd = 6.0
Output limits: (-3, 3)
Integral limits: (-1.0, 1.0)

# X-axis (→ pitch command)
Kp = 1.5, Ki = 0.2, Kd = 2.5
Output limits: (-0.2, 0.2)
Integral limits: (-0.1, 0.1)

# Y-axis (→ roll command)
Kp = 1.5, Ki = 0.2, Kd = 2.5
Output limits: (-0.2, 0.2)
Integral limits: (-0.1, 0.1)
```

#### Inner Loop (Attitude Control)
```python
# Roll
Kp = 0.02, Ki = 0.0005, Kd = 0.004
Output limits: (-0.3, 0.3)
Integral limits: (-0.1, 0.1)

# Pitch
Kp = 0.02, Ki = 0.0005, Kd = 0.004
Output limits: (-0.3, 0.3)
Integral limits: (-0.1, 0.1)

# Yaw
Kp = 0.008, Ki = 0.0003, Kd = 0.0015
Output limits: (-0.15, 0.15)
Integral limits: (-0.05, 0.05)
```

### Motor Mixer Configuration

**File**: `cascade_pid_improved.py:131-162`

```python
# Physical parameters
arm_length = 0.046m
thrust_to_torque_ratio = 0.005824
max_motor_thrust = 0.16N
min_motor_thrust = 0.0N

# 4×4 Mixing matrix (quadrotor X configuration)
# Maps [thrust, roll_moment, pitch_moment, yaw_moment] → [M1, M2, M3, M4]
```

---

## Technical Questions for Discussion

### 1. Control Architecture
- **Q1**: Is cascade PID the appropriate control strategy for this simplified force/moment model?
- **Q2**: Should we consider alternative approaches?
  - Linear Quadratic Regulator (LQR)
  - Model Predictive Control (MPC)
  - Geometric control on SE(3)
  - Sliding mode control

### 2. Parameter Tuning Strategy
- **Q3**: What systematic tuning approach should we use?
  - Manual tuning (current approach)
  - Ziegler-Nichols method
  - Relay feedback test
  - Optimization-based tuning (genetic algorithm, gradient descent)

- **Q4**: What are the target performance specifications?
  - Rise time
  - Settling time
  - Overshoot percentage
  - Steady-state error
  - Disturbance rejection

### 3. Model Validation
- **Q5**: Are the MuJoCo model parameters accurate?
  - Mass: 0.027kg (from `model.body_mass[1]`)
  - Inertia tensor values
  - Arm length: 0.046m
  - Motor thrust limits

- **Q6**: How does the simplified force/moment model compare to real CrazyFlie dynamics?
  - Real CrazyFlie uses individual motor RPM control
  - Our model uses direct force/moment commands
  - Potential for model-reality gap

### 4. Coupling and Interactions
- **Q7**: Are there coupling issues between control loops?
  - Outer loop bandwidth vs inner loop bandwidth
  - Time-scale separation principle
  - Cross-coupling between axes

- **Q8**: Is the motor mixer causing saturation issues?
  - Thrust limit: 4 × 0.16N = 0.64N total
  - Drone weight: 0.027kg × 9.81 = 0.265N
  - Theoretical thrust-to-weight ratio: 2.4
  - Are limits too conservative?

### 5. Diagnostic Data Needed
- **Q9**: What data should we collect to diagnose the issue?
  - Step response plots
  - Error convergence over time
  - PID term contributions (P, I, D)
  - Motor saturation frequency
  - Phase margins and gain margins (frequency analysis)

---

## Potential Root Causes

### Hypothesis 1: Outer Loop Tuning
- **Issue**: Z-axis Ki=1.5 may still cause integral windup
- **Evidence**: Large overshoot or oscillation in altitude
- **Test**: Reduce Ki to 0.5 or disable integral term

### Hypothesis 2: Inner-Outer Loop Mismatch
- **Issue**: Inner loop may be too slow compared to outer loop
- **Evidence**: Attitude cannot track commanded angles fast enough
- **Test**: Increase inner loop gains or decrease outer loop gains

### Hypothesis 3: Derivative Term Noise Amplification
- **Issue**: Despite filtering (α=0.1), derivative term may amplify sensor noise
- **Evidence**: High-frequency oscillations
- **Test**: Increase filter coefficient or remove derivative term

### Hypothesis 4: Model Inaccuracies
- **Issue**: Simplified actuator model doesn't capture real dynamics
- **Evidence**: Controller works in theory but fails in simulation
- **Test**: Add motor dynamics (first-order lag), propeller drag

### Hypothesis 5: Feedforward Missing
- **Issue**: Pure feedback control without gravity compensation
- **Evidence**: Large steady-state error or poor tracking
- **Note**: Currently includes `mass * gravity` feedforward for Z-axis

---

## Suggested Next Steps

### Immediate (Debugging):
1. **Data Collection**: Run simulation and log full state trajectory
2. **Step Response Test**: Command step changes in position, analyze response
3. **Frequency Analysis**: Identify oscillation frequencies
4. **Term Analysis**: Plot P, I, D contributions separately

### Short-term (Tuning):
1. **Systematic Parameter Sweep**: Grid search over Kp, Ki, Kd ranges
2. **Single-Axis Tests**: Test Z-only, then X-Y separately
3. **Gain Scaling**: Try reducing all gains by 50%, then incrementally increase
4. **Remove Integral Terms**: Test pure PD control first

### Medium-term (Architecture):
1. **Implement LQR**: Compare performance against PID
2. **Add Motor Dynamics**: Include realistic actuator lag
3. **Sensor Noise**: Add realistic sensor noise to test robustness
4. **Wind Disturbances**: Test disturbance rejection

---

## Code References

| Component | File | Lines |
|-----------|------|-------|
| Main Controller | `cascade_pid_improved.py` | 164-345 |
| PID Class | `cascade_pid_improved.py` | 20-129 |
| Motor Mixer | `cascade_pid_improved.py` | 131-162 |
| Control Computation | `cascade_pid_improved.py` | 271-332 |
| PID Parameters | `cascade_pid_improved.py` | 180-215 |
| MuJoCo Scene | `simulation/crazyflie_sim/scene.xml` | - |
| Drone Model | `simulation/crazyflie_sim/cf2.xml` | - |

---

## Running the Simulation

```bash
# Activate virtual environment
source .venv/bin/activate

# Run improved cascade PID controller
python simulation/cascade_pid_improved.py

# Expected output:
# - Position updates every 2 seconds
# - Error magnitude
# - Motor thrust values
# - Saturation warnings
# - Emergency reset count
```

---

## Additional Resources

- **CrazyFlie Firmware**: https://github.com/bitcraze/crazyflie-firmware
  - Reference PID parameters in `src/modules/src/controller_pid.c`

- **MuJoCo Documentation**: https://mujoco.readthedocs.io/

- **Control Theory References**:
  - Åström & Murray, "Feedback Systems" (Chapter 10: PID Control)
  - Beard & McLain, "Small Unmanned Aircraft" (Chapter 6: Autopilot Design)

---

## Contact & Collaboration

For questions about this project, please discuss:
- Control architecture decisions
- Parameter tuning strategies
- Model validation concerns
- Alternative control approaches

---

**Last Updated**: 2025-10-29
**Author**: Hanzel (with Claude Code assistance)
**Repository**: 24774_ACSI_F25_ZephyFlyer
