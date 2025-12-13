# TinyMPC Hover Controller Oscillation Diagnosis (2025-12-04)

## Problem Report

TinyMPC hover controller showing severe bang-bang oscillations with no convergence after 10 seconds of flight.

**Log file**: `tinympc_20251204_174602.csv` (189 lines, ~10 seconds)

---

## Observed Symptoms

### Position Oscillation
- **Target altitude**: 0.5m
- **Actual altitude**: Oscillating between 0.025m and 0.55m
- **Amplitude**: ±0.25m (50% of target height!)
- **No convergence**: Error not decreasing over time

### Velocity Oscillation
- **Vertical velocity**: Swinging from -1.9 m/s to +1.2 m/s
- **Magnitude**: ±1.5 m/s average
- **Expected**: Near zero for stable hover
- **Reality**: Drone bouncing violently up and down

### Control Saturation
- **Thrust command**: Bang-banging between +0.1 and -0.1 N
- **Constraint**: u_thrust ∈ [-0.10, +0.10] N
- **Status**: **SATURATED** - controller hitting limits constantly
- **Motor commands**: Oscillating between 77.4 and 18.9

### Sample Data Points
```
t=2.608s: z=0.394m, vz=-1.04 m/s, err_z=-0.106m, thrust=+0.1  (max thrust)
t=4.528s: z=0.046m, vz=+0.95 m/s, err_z=-0.453m, thrust=+0.1  (max thrust)
t=6.528s: z=0.522m, vz=-1.25 m/s, err_z=+0.022m, thrust=+0.1  (max thrust)
t=9.568s: z=0.026m, vz=+0.91 m/s, err_z=-0.473m, thrust=+0.1  (max thrust)
t=9.608s: z=0.063m, vz=+0.95 m/s, err_z=-0.436m, thrust=-0.1  (FLIP to min)
```

**Key observation**: Thrust flips between +0.1 and -0.1 instantly, typical bang-bang control failure.

---

## Root Cause Analysis

### Issue 1: **INCORRECT MASS PARAMETER** 🚨 (CRITICAL)

**Code** (crazyflie_tinympc.py:339):
```python
self.mass = 1.68E-02  # kg  ← WRONG!
```

**Actual CrazyFlie 2.1 Mass**:
- Base platform: 27g (0.027 kg)
- With flow deck: 31g (0.031 kg)
- With battery: 27-31g typical

**Impact**:
```python
# With WRONG mass (0.0168 kg):
hover_thrust = 0.0168 * 9.81 = 0.165 N

# With CORRECT mass (0.027 kg):
hover_thrust = 0.027 * 9.81 = 0.265 N

# Error: 60% underestimate!
```

**Why this causes oscillation**:
1. MPC thinks hover thrust is 0.165 N
2. Real hover thrust is 0.265 N
3. With Δthrust ∈ [-0.10, +0.10], total thrust range is:
   - MPC thinks: [0.065, 0.265] N
   - Reality needs: 0.265 N just to hover
4. **MPC can barely provide enough thrust to hover, let alone control altitude!**
5. Result: Bang-bang between max thrust (still falling) and min thrust (rising too fast)

---

### Issue 2: **THRUST CONSTRAINTS TOO TIGHT**

**Code** (crazyflie_tinympc.py:362):
```python
u_min = np.array([-0.10, -5e-3, -5e-3, -1e-3])  # Δthrust ∈ [-0.10, +0.10] N
u_max = np.array([+0.10, +5e-3, +5e-3, +1e-3])
```

**Analysis** (assuming correct mass of 0.027 kg):
- Hover thrust: 0.265 N
- Max thrust: 0.265 + 0.10 = 0.365 N (1.38x hover) ← Too weak!
- Min thrust: 0.265 - 0.10 = 0.165 N (0.62x hover)

**For comparison**, real CrazyFlie can:
- Max thrust: ~0.6 N (2.3x hover weight)
- Min thrust: 0 N

**Current constraint allows only 38% increase above hover** - not enough authority for aggressive control!

---

### Issue 3: **INSUFFICIENT VELOCITY DAMPING IN COST FUNCTION**

**Code** (crazyflie_tinympc.py:300-305):
```python
Q = np.diag([
    5.0E+01, 5.0E+01, 8.0E+01,    # position (x, y, z)
    3.0E+01, 3.0E+01, 5.0E+01,    # velocity (vx, vy, vz) ← TOO LOW!
    5.0E+01, 5.0E+01, 2.0E+01,    # angles
    1.0E+00, 1.0E+00, 1.0E+00     # angular rates
])
```

**Problem**: Velocity cost ratio
- Q_z (position): 80
- Q_vz (velocity): 50
- Ratio: 1.6x

**For good damping**, velocity cost should be **3x-5x higher** than position cost!

**Why this matters**:
- MPC optimizes: minimize `∑(x^T Q x + u^T R u)`
- Low Q_velocity → MPC doesn't care if velocity is high
- High Q_position → MPC aggressively chases position setpoint
- Result: Fast position response but no damping → oscillation

---

### Issue 4: **THRUST INPUT COST TOO LOW**

**Code** (crazyflie_tinympc.py:308-313):
```python
R = np.diag([
    5.0E+00,    # Δthrust (reduced 10x: allow aggressive control) ← TOO LOW!
    1.0E+01,    # roll moment
    1.0E+01,    # pitch moment
    1.0E+01     # yaw moment
])
```

**Comment says**: "reduced 10x: allow aggressive control"
**Reality**: Too aggressive, controller doesn't penalize thrust saturation!

**Why this causes bang-bang**:
- Low R → cheap to use maximum thrust
- MPC saturates thrust to minimize position error (high Q_position)
- No penalty for saturating → bang-bang behavior

---

### Issue 5: **MODEL MISMATCH - MISSING AERODYNAMIC DAMPING**

**Linearized model** (crazyflie_tinympc.py:236-238):
```python
v̇x = g * theta      # Horizontal: gravity couples to tilt
v̇y = -g * phi
v̇z = thrust / mass  # Vertical: NO DAMPING TERM!
```

**Reality**: Aerodynamic drag provides natural damping
```python
v̇z = thrust/mass - b*vz  # b ≈ 0.5-2.0 (damping coefficient)
```

**Impact**:
- Real system has damping → velocity decays naturally
- MPC model has NO damping → predicts velocity will persist
- MPC over-corrects → oscillation

---

## Summary of Issues (Priority Order)

| Issue | Severity | Impact | Fix Difficulty |
|-------|----------|--------|----------------|
| **1. Wrong mass** | 🚨 CRITICAL | 60% model error | Easy (1 line) |
| **2. Tight thrust constraint** | 🔴 High | Control authority limited | Easy (1 line) |
| **3. Low velocity cost** | 🟠 Medium | Insufficient damping | Easy (3 lines) |
| **4. Low thrust cost** | 🟡 Low | Bang-bang tendency | Easy (1 line) |
| **5. Missing drag** | 🟢 Research | Model mismatch | Hard (model redesign) |

---

## Recommended Fixes

### Fix 1: Correct the Mass (CRITICAL - DO THIS FIRST!)

**File**: `crazyflie_tinympc.py:339`

**Change**:
```python
# BEFORE (WRONG):
self.mass = 1.68E-02  # kg

# AFTER (CORRECT):
self.mass = 2.7E-02  # kg (0.027 kg = 27g, CrazyFlie 2.1 base weight)
```

**Expected impact**:
- Hover thrust: 0.165 N → 0.265 N (correct!)
- Thrust range will match physical requirements
- Should reduce oscillation significantly

---

### Fix 2: Relax Thrust Constraints

**File**: `crazyflie_tinympc.py:362`

**Change**:
```python
# BEFORE:
u_min = np.array([-0.10, -5e-3, -5e-3, -1e-3])
u_max = np.array([+0.10, +5e-3, +5e-3, +1e-3])

# AFTER:
u_min = np.array([-0.20, -5e-3, -5e-3, -1e-3])  # Allow ±0.20 N (2x range)
u_max = np.array([+0.20, +5e-3, +5e-3, +1e-3])
```

**Rationale**:
- With correct mass (0.027 kg), hover thrust = 0.265 N
- Max thrust: 0.265 + 0.20 = 0.465 N (1.76x hover) ✅ Good control authority
- Min thrust: 0.265 - 0.20 = 0.065 N (0.24x hover) ✅ Can descend quickly

---

### Fix 3: Increase Velocity Cost for Better Damping

**File**: `crazyflie_tinympc.py:300-305`

**Change**:
```python
# BEFORE:
Q = np.diag([
    5.0E+01, 5.0E+01, 8.0E+01,    # position
    3.0E+01, 3.0E+01, 5.0E+01,    # velocity ← TOO LOW
    5.0E+01, 5.0E+01, 2.0E+01,
    1.0E+00, 1.0E+00, 1.0E+00
])

# AFTER:
Q = np.diag([
    5.0E+01, 5.0E+01, 8.0E+01,    # position (unchanged)
    1.5E+02, 1.5E+02, 2.5E+02,    # velocity (5x increase for damping!)
    5.0E+01, 5.0E+01, 2.0E+01,    # angles (unchanged)
    1.0E+00, 1.0E+00, 1.0E+00     # angular rates (unchanged)
])
```

**Rationale**:
- Q_vz / Q_z = 250 / 80 = 3.1x (good damping ratio)
- MPC will now prioritize reducing velocity oscillations
- Trade-off: Slightly slower position convergence, but stable!

---

### Fix 4: Increase Thrust Input Cost

**File**: `crazyflie_tinympc.py:308-313`

**Change**:
```python
# BEFORE:
R = np.diag([
    5.0E+00,    # Δthrust (too low)
    1.0E+01,
    1.0E+01,
    1.0E+01
])

# AFTER:
R = np.diag([
    2.0E+01,    # Δthrust (4x increase - penalize saturation)
    1.0E+01,    # moments (unchanged)
    1.0E+01,
    1.0E+01
])
```

**Rationale**:
- Higher R → controller prefers smooth control
- Penalizes bang-bang behavior
- Should eliminate thrust saturation

---

### Fix 5 (Advanced): Add Aerodynamic Damping to Model

**This is optional and more complex.**

**File**: `crazyflie_tinympc.py:224-285`

**Add damping to vertical dynamics**:
```python
def get_linearized_model(dt=0.01, mass=0.027, g=9.81,
                         Ixx=1.6e-5, Iyy=1.6e-5, Izz=2.9e-5,
                         drag_coeff=1.0):  # NEW: Add damping parameter
    # ... existing code ...

    # Velocity dynamics (linearized): v̇ = g * angles - drag * v
    A_cont[3, 7] = g      # v̇x = g * theta
    A_cont[4, 6] = -g     # v̇y = -g * phi
    A_cont[3, 3] = -drag_coeff  # NEW: Add drag damping to vx
    A_cont[4, 4] = -drag_coeff  # NEW: Add drag damping to vy
    A_cont[5, 5] = -drag_coeff  # NEW: Add drag damping to vz

    # ... rest of code ...
```

**Rationale**:
- Better matches real physics
- Natural damping without high Q_velocity
- Start with `drag_coeff = 1.0`, tune from there

---

## Implementation Priority

**Step 1: Critical Fixes (MUST DO)**
1. ✅ Fix mass to 0.027 kg
2. ✅ Increase thrust constraint to ±0.20 N

**Step 2: Tuning Fixes (RECOMMENDED)**
3. ✅ Increase velocity cost Q by 5x
4. ✅ Increase thrust cost R by 4x

**Step 3: Advanced Improvements (OPTIONAL)**
5. ⚠️ Add aerodynamic damping to model

---

## Expected Results After Fixes

### Before (Current):
- Position error: 0.07m to 0.47m (oscillating)
- Velocity: ±1.5 m/s (violent oscillation)
- Thrust: Bang-bang between ±0.1 N (saturated)
- Convergence: None after 10 seconds

### After (Expected):
- Position error: Should converge to <0.05m in 3-5 seconds
- Velocity: Should damp to <0.2 m/s within 2-3 seconds
- Thrust: Smooth commands, no saturation
- Convergence: Stable hover within 5 seconds

---

## Testing Procedure

1. **Apply Fix 1 (mass) and Fix 2 (thrust constraint)**
2. **Test for 30 seconds**
   - Check if oscillation reduces
   - Look for convergence
3. **If still oscillating**: Apply Fix 3 (velocity cost)
4. **If still bang-bang**: Apply Fix 4 (thrust cost)
5. **If stable but sluggish**: Reduce Q_velocity slightly
6. **If stable but overshoot**: Increase Q_velocity slightly

---

## Quick Fix Summary

**Minimum viable changes** (copy-paste ready):

```python
# Line 339: Fix mass
self.mass = 2.7E-02  # kg (CORRECTED from 1.68E-02)

# Line 362: Relax thrust constraint
u_min = np.array([-0.20, -5e-3, -5e-3, -1e-3])  # Increased from -0.10
u_max = np.array([+0.20, +5e-3, +5e-3, +1e-3])  # Increased from +0.10

# Line 300-305: Increase velocity damping
Q = np.diag([
    5.0E+01, 5.0E+01, 8.0E+01,    # position
    1.5E+02, 1.5E+02, 2.5E+02,    # velocity (5x increase)
    5.0E+01, 5.0E+01, 2.0E+01,
    1.0E+00, 1.0E+00, 1.0E+00
])

# Line 308-313: Increase thrust cost
R = np.diag([
    2.0E+01,    # Δthrust (4x increase)
    1.0E+01,
    1.0E+01,
    1.0E+01
])
```

**Test immediately after applying these changes!**

---

## Comparison with ESO Controller

| Metric | TinyMPC (Current) | ESO (Working) |
|--------|------------------|---------------|
| Convergence time | None (oscillating) | 5-8 seconds ✅ |
| Position stability | ±0.25m oscillation | ±0.05m stable ✅ |
| Velocity magnitude | ±1.5 m/s | <0.3 m/s ✅ |
| Control effort | Bang-bang (saturated) | Smooth ✅ |
| Root cause | Wrong mass + tight constraints | N/A |

**Key insight**: TinyMPC has potential but needs correct parameters. ESO works because it adapts to model mismatch automatically.

---

## Why ESO Works and TinyMPC Doesn't (Currently)

**ESO Advantages**:
1. **Disturbance estimation**: Compensates for model errors
2. **Adaptive**: Learns true dynamics online
3. **No need for exact mass**: ESO estimates it implicitly

**TinyMPC Disadvantages** (current implementation):
1. **Model-based**: Requires accurate mass, inertia, etc.
2. **Open-loop prediction**: No adaptation if model is wrong
3. **60% mass error** → catastrophic performance

**Once mass is fixed, TinyMPC should match or exceed ESO performance** due to:
- Optimal control (minimizes cost function)
- Constraint handling (MPC can respect limits explicitly)
- Predictive capability (plans ahead, not reactive)

---

## Next Steps

1. **Apply the 4 fixes above**
2. **Run simulation for 30 seconds**
3. **Compare new log with this diagnosis**
4. **Report back results**

If fixes work → TinyMPC should achieve stable hover!
If still oscillating → deeper investigation needed (check motor mixing, hover_thrust_cmd calibration, etc.)

---

**Last Updated**: 2025-12-04
**Status**: Diagnosis complete, fixes ready to implement
**Author**: Claude Code (Sonnet 4.5)
