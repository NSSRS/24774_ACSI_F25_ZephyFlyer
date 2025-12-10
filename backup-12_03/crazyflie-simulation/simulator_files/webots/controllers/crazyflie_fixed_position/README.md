# Fixed Position Controller

## Overview

This controller uses the new `pid_fixed_position_controller` to maintain a **fixed XYZ position** (position hold), even under external disturbances like impulse forces and continuous wind.

## Features

- 🎯 **Position Hold**: Maintains target position in world frame
- 💥 **Impulse Force Testing**: Apply single pushes to test transient response
- 🌬️ **Continuous Wind Testing**: Apply constant forces to test steady-state rejection
- 📍 **Dynamic Target Adjustment**: Change target position on-the-fly

## Control Architecture

```
Target Position (X,Y,Z) → [Position PID] → Desired Velocity (body frame)
                                          ↓
                          [Velocity PID] → Desired Attitude (roll/pitch)
                                          ↓
                          [Attitude PID] → Motor Commands
```

**Key Feature**: Coordinate frame transformation from world frame to body frame using yaw angle.

## Usage

### Step 1: Load World File

In Webots, load any world with CrazyFlie drone, then:
1. Select the CrazyFlie robot node
2. In the `controller` field, enter: `crazyflie_fixed_position`
3. Save and run the simulation

### Step 2: Wait for Stabilization

The drone will automatically take off and hover at **(0, 0, 1.0)** meters. Wait ~10 seconds for initial stabilization.

### Step 3: Test Disturbance Rejection

#### Impulse Forces
Press these keys to apply single impulse forces:
- **F** - Forward push (+X)
- **G** - Left push (+Y)
- **H** - Backward push (-X)
- **J** - Right push (-Y)
- **U** - Upward push (+Z)
- **D** - Downward push (-Z)

**Expected**: Drone returns to target position within ~5 seconds.

#### Continuous Wind
1. Press **7** to set wind direction to +X (forward)
2. Press **V** to enable wind
3. Observe drone maintaining position

**Expected**: Integral term accumulates to cancel constant wind force.

### Step 4: Adjust Target Position

- **Arrow Keys**: Move target ±0.5m in XY plane
- **W/S**: Move target altitude ±0.2m
- **R**: Reset target to origin (0, 0, 1.0)

### Step 5: Monitor Status

- **SPACE**: Print detailed status (position, error, velocity, wind)
- Console output: Auto-prints status every 2 seconds

## Keyboard Controls Reference

| Key | Action |
|-----|--------|
| **Arrow Keys** | Adjust target XY position (±0.5m) |
| **W/S** | Adjust target altitude (±0.2m) |
| **R** | Reset target to origin |
| **F/G/H/J** | Apply impulse force (horizontal) |
| **U/D** | Apply impulse force (vertical) |
| **[/]** | Decrease/Increase impulse magnitude |
| **7/8/9/0** | Set wind direction (+X/+Y/-X/-Y) |
| **-/=** | Set wind direction (+Z/-Z) |
| **V** | Toggle wind ON/OFF |
| **,/.** | Decrease/Increase wind magnitude |
| **C** | Clear wind |
| **SPACE** | Print status |

## PID Gains

### Position Loop (Outer)
```python
Kp = 0.6   # Position proportional gain
Kd = 0.3   # Velocity damping gain
Ki = 0.05  # Position integral gain (wind rejection)
```

### Velocity/Attitude/Altitude Loops
Uses official Bitcraze PID parameters (same as `pid_velocity_fixed_height_controller`).

## Expected Performance

- **Position hold error**: < 0.01m (steady-state)
- **Impulse recovery time**: ~5 seconds
- **Constant wind rejection**: Yes (via integral term)
- **Max velocity**: 0.3 m/s (saturated)

## Troubleshooting

### Drone doesn't return to position
- Check that `pid_controller.py` has been updated with coordinate frame transformation
- Verify yaw angle is being passed correctly to controller

### Drone oscillates
- Reduce `Kp` from 0.6 to 0.4
- Increase `Kd` from 0.3 to 0.5

### Drone drifts under wind
- Increase `Ki` from 0.05 to 0.1
- Verify wind is enabled (press V)
- Check wind magnitude is reasonable (< 0.05N)

## Technical Details

See `BUGFIX_COORDINATE_FRAME.md` for detailed explanation of the coordinate frame transformation fix.

## Files

- `crazyflie_fixed_position.py` - Main controller script
- `README.md` - This file
- `../controllers_shared/python_based/pid_controller.py` - PID controller class

## Author & Date

**Created**: 2025-11-06
**Based on**: Bitcraze official PID controller + position outer loop
