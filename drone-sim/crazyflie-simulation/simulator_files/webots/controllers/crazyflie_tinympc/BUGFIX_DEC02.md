# TinyMPC Bug Fixes - Dec 2, 2025

## Critical Bug: Thrust Scaling Error

**Location**: `crazyflie_tinympc.py:493`

**Problem**:
- MPC outputs Δthrust in range [-0.10, +0.10] N (physical units)
- Code incorrectly scaled it again by 0.10, making control 10x weaker
- Result: MPC output -0.1N → became -0.01N → insufficient control authority

**Fix**:
```python
# BEFORE (line 493):
d_thrust = 0.10 * d_thrust_n  # WRONG - double scaling

# AFTER:
d_thrust = d_thrust_n  # Correct - already in Newtons
```

## Performance Issues Observed

**Log file**: `logs/tinympc_20251202_143756.csv`

**Symptoms**:
- Position error diverged: 9.1m → 21.4m over 1.8s
- Velocity increased: 5.74 m/s → 7.92 m/s (upward)
- Motor commands stayed constant at 45.3 RPM (no control applied)

**Root causes**:
1. ✅ Thrust scaling bug (fixed above)
2. Poor initial conditions (pz=9.6m, vz=5.74m/s far from hover)
3. Linearization breakdown for large errors
4. Weak velocity damping in Q matrix

## Tuning Improvement v2 (After First Test)

**Log file**: `logs/tinympc_20251202_145744.csv`

**Observed issues**:
- Bang-bang control (u_thrust jumps between -0.1 and +0.1)
- Underdamped oscillation with increasing amplitude
- Velocity reached 4.43 m/s (too fast for 0.5m hover)

**Root cause - Poor Q/R balance**:
```python
# BEFORE:
Q_velocity = [5, 5, 10]     # Too low
Q_position = [50, 50, 80]   # Position/velocity ratio = 8:1
R_thrust = 50                # Too high penalty → bang-bang

# AFTER:
Q_velocity = [30, 30, 50]   # 6x increase for damping
Q_position = [50, 50, 80]   # Position/velocity ratio = 1.6:1
R_thrust = 5                 # 10x reduction → smooth control
```

**Expected improvements**:
- MPC will prioritize velocity control (anticipatory braking)
- Smooth control output instead of bang-bang
- Faster convergence with less overshoot

## Final Test Results v3

**Log file**: `logs/tinympc_20251202_150202.csv` (337 samples, 17 seconds)

**Performance achieved**:
- ✅ Steady-state error: **1.5cm** (0.015m)
- ✅ Steady-state velocity: **~1e-07 m/s** (effectively zero)
- ✅ Convergence time: **5 seconds**
- ✅ Oscillation amplitude reduced: **65%** (1.4m → 0.5m)
- ✅ Max velocity reduced: **73%** (4.4 m/s → 1.2 m/s)
- ✅ Smooth control output (no more bang-bang)
- ✅ Stable for 12+ seconds after convergence

**Comparison**:

| Metric | Before | After | Improvement |
|--------|--------|-------|-------------|
| Oscillation | 1.4m | 0.5m | 65% ↓ |
| Max velocity | 4.4 m/s | 1.2 m/s | 73% ↓ |
| Steady-state error | Unstable | 1.5cm | ✅ |
| Control smoothness | Bang-bang | Smooth | ✅ |
| Convergence | Diverging | 5s | ✅ |

**Status**: Production-ready. Initial transient oscillation is acceptable and typical for MPC controllers.

## Future Improvements (Optional)

- Increase prediction horizon (N=10→20) for better anticipation
- Initialize drone closer to target altitude
- Consider gain scheduling for large errors
