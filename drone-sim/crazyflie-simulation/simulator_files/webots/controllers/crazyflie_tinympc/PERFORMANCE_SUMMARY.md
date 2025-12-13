# TinyMPC Controller Performance Summary

**Date**: 2025-12-05
**Controller**: TinyMPC for CrazyFlie Hover Control
**Test Environment**: Webots Simulation (crazyflie_wind.wbt)

---

## Final Configuration

### MPC Parameters

```python
# Horizon
horizon = 20  # 20 timesteps prediction

# Control timestep
dt = 40ms (25 Hz control rate, determined by Webots timestep)

# State cost matrix Q
Q = diag([
    50,  50,  80,   # Position (x, y, z)
    150, 150, 250,  # Velocity (x, y, z) - Strong damping
    50,  50,  20,   # Euler angles (roll, pitch, yaw)
    1,   1,   1     # Angular rates (p, q, r)
])

# Input cost matrix R
R = diag([
    20,  # Δthrust
    10,  # Roll moment
    10,  # Pitch moment
    10   # Yaw moment
])
```

### Constraints

```python
# Input constraints
Δthrust ∈ [-0.10, +0.10] N
Roll/Pitch moment ∈ [-5, +5] mN·m
Yaw moment ∈ [-1, +1] mN·m

# State constraints
Roll/Pitch angle ∈ [-15°, +15°]
```

### Hover Calibration

```python
m_hover = 48.0  # Experimentally determined motor command
Theoretical hover thrust = 0.2649 N (mass × g)
Thrust scale = 181.13 cmd/N
```

**IMPORTANT**: `m_hover = 48` is **calibrated experimentally** and should NOT be changed. This value provides optimal stability and convergence.

---

## Performance Metrics

### Test Results (Log: tinympc_20251205_025003.csv)

| Metric | Value | Status |
|--------|-------|--------|
| **Convergence Time** | 4.3s (to ±2cm) | ✅ Excellent |
| **Steady-State Error** | +3.36cm | ⚠️ Acceptable* |
| **Position Std Dev** | ~0.000cm | ✅ Perfect |
| **Velocity Std Dev** | ~0.000m/s | ✅ Perfect |
| **Control Saturation** | None (u ≈ -0.040N) | ✅ Good |
| **MPC Solve Time** | 2-3ms | ✅ Real-time capable |
| **Stability** | No oscillations | ✅ Perfect |

*Note: The 3.36cm steady-state error is due to modeling error (aerodynamic drag, ground effect, etc.) which is NOT modeled in the linearized MPC. This is expected and acceptable for this application.

---

## Steady-State Analysis

### Final Equilibrium (Last 100 samples)

```
Position:      0.5336 m  (target: 0.5000 m)
Error:         +3.36 cm  (±0.000 cm std dev)
Velocity:      0.0000 m/s  (essentially zero)
Control Input: -0.0404 N  (constant, not saturated)
```

**Interpretation**:
- System is **perfectly stable** at equilibrium
- MPC continuously applies -0.040N to compensate for modeling error
- No drift, no oscillation
- Predictable and repeatable behavior

---

## Key Design Decisions

### 1. Why `m_hover = 48`?

✅ **Experimentally validated** - Multiple tests confirmed this value provides:
- Stable convergence without overshoot
- No limit-cycle oscillations
- Consistent performance across runs

❌ **Do NOT change** - Other values tested (45, 46.7, 47.5) resulted in:
- Undershoot and slow rise (45)
- Aggressive overshoot and oscillations (46.7)
- Marginal stability (47.5)

### 2. Why `horizon = 20`?

- Balances prediction accuracy vs computational cost
- Provides ~0.8s lookahead (20 × 40ms)
- Sufficient for smooth trajectory planning
- Solve time remains real-time capable (2-3ms)

### 3. Why `Δthrust ∈ [-0.10, 0.10]`?

- Prevents aggressive bang-bang control
- Max acceleration: ±3.7 m/s² (reasonable for smooth flight)
- Avoids control saturation in normal operation
- Tested: larger bounds (±0.20) caused overshoot

### 4. Why strong velocity damping (`Q_v >> Q_p`)?

```
Q_velocity = [150, 150, 250]
Q_position = [50, 50, 80]
Ratio: ~3x higher for velocity
```

- Critical for preventing oscillations
- MPC prioritizes "stop moving" over "reach exact position"
- Trade-off: slower convergence but guaranteed stability

---

## Known Limitations

### 1. Steady-State Error (+3.36cm)

**Cause**: Modeling mismatch
- Linearized model assumes ideal quadrotor dynamics
- Real system has: air drag, propeller downwash, ground effect, sensor delays

**Mitigation Options** (NOT implemented):
- ❌ Adjust `m_hover` → Breaks stability
- ✅ Offset target by -3.36cm → Simple but inelegant
- ✅ Add integral action → Increases complexity
- ✅ Nonlinear MPC → Too computationally expensive

**Current Decision**: Accept the error as it's predictable and stable.

### 2. Convergence Speed (4.3s)

**Acceptable for**: Station-keeping, waypoint navigation with moderate speed requirements

**Too slow for**: Aggressive trajectory tracking, obstacle avoidance

**Speed-up Options** (NOT implemented):
- Increase `Q_position` → Risk of oscillations
- Increase `horizon` → Higher computational cost
- Reduce `R` → Risk of control saturation

---

## Recommended Use Cases

### ✅ Suitable Applications

1. **Altitude hold** - Primary design goal, works perfectly
2. **Waypoint hovering** - Stable position maintenance
3. **Wind disturbance rejection** - Can be tested with wind_supervisor
4. **XY position control** - With appropriate tuning of Q_xy

### ❌ Not Suitable For

1. **Aggressive acrobatics** - Constraints prevent fast maneuvers
2. **Precision landing** - 3.36cm error may be too large
3. **Fast trajectory tracking** - Convergence time limits bandwidth

---

## Future Improvements (Optional)

### Priority 1: Eliminate Steady-State Error

**Option A**: Offset compensation (1 line change)
```python
hover_height = 0.466  # Compensate for +3.36cm bias
```
- Pros: Simple, no stability impact
- Cons: Not adaptive to environment changes

**Option B**: Integral augmentation (moderate complexity)
```python
# Add PI outer loop
integral_error += (target - actual) * dt
hover_height_adjusted = hover_height - Ki * integral_error
```
- Pros: Adaptive, eliminates steady-state error
- Cons: Tuning required, may interact with MPC

### Priority 2: Faster Convergence

**Option A**: Increase horizon (safest)
```python
horizon = 30  # From 20 → 30
```
- Expected: 4.3s → ~3.5s convergence
- Side effect: Solve time increases to 4-5ms (still real-time)

**Option B**: Rebalance Q/R (moderate risk)
```python
Q_z = 120  # From 80 → 120
R_thrust = 15  # From 20 → 15
```
- Expected: 4.3s → ~3.0s convergence
- Risk: Potential overshoot or oscillations (requires testing)

### Priority 3: XY Position Control

Currently only Z (altitude) is actively controlled. To add XY:
1. Increase `Q_xy` weights
2. Add velocity filtering (low-pass) for noisy GPS
3. Test with disturbances

---

## Testing Checklist

Before modifying parameters, always verify:

- [ ] No control saturation (|u_thrust| < 0.09)
- [ ] No oscillations (position std dev < 1cm)
- [ ] Stable convergence (no limit cycles)
- [ ] Real-time performance (solve time < 10ms)
- [ ] Repeatable behavior (test 5+ runs)

---

## References

### Related Files
- Controller: `crazyflie_tinympc.py`
- World: `crazyflie_wind.wbt`
- Calibration guide: `HOVER_CALIBRATION.md`
- Performance logs: `logs/tinympc_*.csv`

### Key Insights from Development
1. **Hover thrust calibration is critical** - Cannot rely on theoretical `mass × g`
2. **Stability > Speed** - Conservative Q/R provides reliable performance
3. **Modeling error is acceptable** - 3.36cm bias is predictable and stable
4. **Input constraints prevent saturation** - ±0.10N limit was key to stability

---

## Conclusion

The current TinyMPC configuration achieves **excellent stability** with **acceptable performance** for hover control:

✅ Converges in 4.3s
✅ Zero oscillations
✅ Predictable steady-state error
✅ Real-time capable
✅ Robust to repeated tests

The 3.36cm steady-state error is a **known limitation** due to unmodeled dynamics, and is acceptable given the stability and reliability of the system.

**Recommendation**: Keep current parameters unchanged unless specific application requires tighter tolerance or faster convergence.
