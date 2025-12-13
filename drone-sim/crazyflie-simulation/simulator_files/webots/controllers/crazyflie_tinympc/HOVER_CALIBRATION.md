# TinyMPC Hover Thrust Calibration Guide

## Why Calibration is Needed

The theoretical hover thrust (`mass × g = 0.027 kg × 9.81 m/s² = 0.265 N`) does not directly map to motor commands in Webots. There are several reasons:

1. **Motor model differences**: Webots motor model may have different thrust coefficients
2. **Drag and losses**: Air resistance, motor efficiency, etc.
3. **Sensor noise and delays**: GPS/IMU delays affect the effective hover point

Therefore, we need to **experimentally determine** the motor command `m_hover` that makes the drone hover stably at the target altitude.

---

## Calibration Procedure

### Step 1: Prepare Test Environment

1. Open Webots world: `crazyflie_wind.wbt` (or any simple world)
2. Set controller to: `crazyflie_tinympc`
3. Make sure there is **NO wind or disturbances**

### Step 2: Manual Hover Test

Create a simple test script or modify the controller temporarily:

```python
# In main loop, replace MPC control with manual command
motors = [m_test, m_test, m_test, m_test]  # All motors same speed

# Test different values of m_test
m1_motor.setVelocity(-motors[0])
m2_motor.setVelocity(motors[1])
m3_motor.setVelocity(-motors[2])
m4_motor.setVelocity(motors[3])
```

### Step 3: Binary Search for m_hover

Try different motor commands and observe altitude:

| m_test | Altitude Behavior | Action |
|--------|------------------|--------|
| 40 | Descends | Too low, increase |
| 50 | Ascends | Too high, decrease |
| 45 | Stable at 0.5m ± 0.01m | **Found m_hover!** |

**Key criteria for m_hover**:
- Drone maintains altitude within ±1cm for >5 seconds
- Roll and pitch angles remain near 0° (within ±2°)
- No continuous drift in XY position

### Step 4: Update Controller

Once you find `m_hover`, update in `crazyflie_tinympc.py`:

```python
# Line ~397
m_hover = 4.5E+01  # Replace with YOUR experimentally found value
```

---

## Expected Values

Based on similar CrazyFlie models in Webots:

- **Likely range**: `m_hover ∈ [40, 50]`
- **Starting guess**: Try `45` first
- **Fine-tuning**: Adjust by ±1 after initial test

---

## Verification After Calibration

After setting `m_hover`, run TinyMPC and check:

1. **Steady-state error**: Should be < 2cm
2. **No oscillations**: Position should converge smoothly
3. **Control saturation**: `u_thrust` should stay well within [-0.1, 0.1] N

If you see:
- **Steady-state error > 5cm**: Re-calibrate `m_hover`
- **Oscillations**: Check Q/R matrices, not calibration issue
- **Control saturates at ±0.1**: Increase constraint limits or reduce Q_position

---

## Advanced: Verify Thrust Scale

After calibration, the thrust scale should be reasonable:

```
thrust_scale = m_hover / (mass × g)
             ≈ 45 / 0.265
             ≈ 170 cmd/N
```

This means 1 Newton of thrust requires ~170 motor command units.

If your value is very different (e.g., < 100 or > 300), double-check:
- Mass parameter (should be 0.027 kg)
- Motor velocity limits in Webots
- Physics timestep and damping settings

---

## Troubleshooting

### Problem: Cannot find stable hover point

**Possible causes**:
1. Webots physics timestep too large → Reduce to 8ms or 16ms
2. Motors have saturation/limits → Check motor max velocity
3. IMU/GPS noise too high → Filter sensor data

### Problem: m_hover changes with altitude

This suggests **non-linear aerodynamics**. Solutions:
1. Use altitude-dependent hover thrust (lookup table)
2. Add drag compensation in MPC model
3. Operate at fixed altitude only

### Problem: Hover point drifts over time

**Likely causes**:
1. Battery drain (not modeled in Webots)
2. Temperature effects (not modeled)
3. Wind/disturbances → Check world settings

For Webots simulation, drift should be minimal if physics is deterministic.

---

## References

- Webots CrazyFlie model: `$WEBOTS/projects/robots/bitcraze/crazyflie/`
- Motor specifications: Check `crazyflie.proto` file
- Similar calibration: See `crazyflie_controller_py` for reference values
