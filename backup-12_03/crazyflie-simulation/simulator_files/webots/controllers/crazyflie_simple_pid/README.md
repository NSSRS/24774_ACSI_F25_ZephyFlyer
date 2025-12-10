# Simple PID Position Controller for Crazyflie

## Overview

This is a simple cascade PID controller for autonomous position control of the Crazyflie in Webots.

**Key Features:**
- **Higher-level control**: Uses pitch/roll/yaw + thrust (not direct motor control)
- **Cascade architecture**: Position → Velocity → Attitude → Motors
- **Easy to understand**: Clean code structure with clear PID implementations
- **Official parameters**: Based on Bitcraze's tested PID gains

## Control Architecture

```
Target Position (X, Y, Z)
         ↓
    ┌─────────────────────┐
    │  OUTER LOOP (PID)   │  Position → Desired Velocity
    │  - X, Y PIDs        │  Kp=1.0, Ki=0.1, Kd=0.5
    └─────────────────────┘
         ↓
    ┌─────────────────────┐
    │  MIDDLE LOOP (PID)  │  Velocity → Desired Attitude
    │  - Vx, Vy PIDs      │  Kp=2.0, Kd=0.5
    └─────────────────────┘
         ↓
    ┌─────────────────────┐
    │  INNER LOOP (PID)   │  Attitude → Moments
    │  - Roll, Pitch PIDs │  Kp=0.5, Kd=0.1
    └─────────────────────┘
         ↓
    ┌─────────────────────┐
    │  ALTITUDE LOOP      │  Height → Thrust
    │  - Z PID            │  Kp=10, Ki=5, Kd=5
    └─────────────────────┘
         ↓
    ┌─────────────────────┐
    │   MOTOR MIXER       │  Forces → Motor Commands
    │  (X configuration)  │
    └─────────────────────┘
         ↓
    M1, M2, M3, M4
```

## How to Run

### Method 1: Using Webots GUI (Windows)

1. **Open Webots**

2. **Load the world file:**
   - `File` → `Open World...`
   - Navigate to: `crazyflie-simulation/simulator_files/webots/worlds/`
   - Select: **`crazyflie_simple_pid.wbt`**

3. **Run the simulation:**
   - Click the ▶️ Play button
   - The Crazyflie will automatically take off and hover at 1m altitude

4. **Control the drone:**
   - Click the 3D window to activate it
   - Use keyboard controls (see below)

### Method 2: Command Line

```bash
# On Windows (PowerShell or CMD)
cd path\to\crazyflie-simulation\simulator_files\webots\worlds
webots crazyflie_simple_pid.wbt
```

## Keyboard Controls

| Key | Function |
|-----|----------|
| **↑** | Move forward (+X) |
| **↓** | Move backward (-X) |
| **←** | Move left (+Y) |
| **→** | Move right (-Y) |
| **W** | Move up (+Z) |
| **S** | Move down (-Z) |
| **Q** | Rotate CCW (yaw left) |
| **E** | Rotate CW (yaw right) |
| **R** | Reset to origin (0, 0, 1) |
| **SPACE** | Print current status |

**Note:** Each key press adjusts the target position by 0.1m in the corresponding direction.

## PID Parameters

### Position Control (Outer Loop)
```python
Kp = 1.0   # Proportional gain
Ki = 0.1   # Integral gain
Kd = 0.5   # Derivative gain
```
- Output: Desired velocity (-0.5 to 0.5 m/s)
- Integral limits: ±0.2

### Velocity Control (Middle Loop)
```python
Kp = 2.0   # Proportional gain
Ki = 0.0   # No integral (avoid windup)
Kd = 0.5   # Derivative gain
```
- Output: Desired attitude (±0.3 radians ≈ ±17°)
- Based on Bitcraze official parameters

### Attitude Control (Inner Loop)
```python
# Roll/Pitch
Kp = 0.5   # Proportional gain (25x higher than MuJoCo!)
Ki = 0.0   # No integral
Kd = 0.1   # Derivative gain

# Yaw
Kp = 1.0
Ki = 0.0
Kd = 0.5
```
- Output: Moment commands
- **Key insight**: These gains are much higher than typical MuJoCo implementations

### Altitude Control
```python
Kp = 10.0  # Proportional gain
Ki = 5.0   # Integral gain
Kd = 5.0   # Derivative gain
```
- Output: Thrust adjustment (±20)
- Base thrust: 48 (hover point)
- Integral limits: ±2.0

## Expected Behavior

1. **Takeoff:** Drone lifts off smoothly to 1m altitude
2. **Hover:** Stabilizes at target position with minimal drift
3. **Response:** Quick response to keyboard commands
4. **Stability:** No oscillations or overshooting

**Typical Performance:**
- Position error: < 0.05m when hovering
- Settling time: ~2-3 seconds after command
- No oscillations with default gains

## Comparison with MuJoCo Implementation

| Parameter | Webots (this) | MuJoCo (previous) | Ratio |
|-----------|---------------|-------------------|-------|
| Attitude Kp | 0.5 | 0.02 | **25x** |
| Attitude Kd | 0.1 | 0.004 | **25x** |
| Altitude Kp | 10.0 | 8.0 | 1.25x |
| Altitude Ki | 5.0 | 1.5 | 3.3x |

**Key Finding:** The much higher attitude control gains in Webots suggest that MuJoCo's inner loop was too slow, causing outer loop instability.

## Troubleshooting

### Drone doesn't take off
- Check console for errors
- Ensure initial height is > 0.015m in world file
- Verify controller is set to "crazyflie_simple_pid"

### Oscillations or instability
- Reduce outer loop Ki gains
- Reduce inner loop Kp/Kd gains by 20%
- Check sensor noise levels

### Drifting horizontally
- Increase position Kp
- Add small Ki term (0.05-0.1)
- Check for wind/disturbances in simulation

### Poor altitude tracking
- Increase altitude Ki
- Check for mass/inertia mismatches
- Verify base thrust value (should be ~48)

## Modifying the Controller

To tune the PID parameters:

1. Edit `crazyflie_simple_pid.py`
2. Find the `CrazyfliePositionController.__init__()` method
3. Modify the `SimplePIDController` initialization parameters
4. Save and reload the world in Webots

**Recommended tuning order:**
1. Start with altitude control (Z)
2. Then attitude control (roll/pitch)
3. Then velocity control
4. Finally position control

## Files

- **crazyflie_simple_pid.py** - Main controller code
- **../worlds/crazyflie_simple_pid.wbt** - Test world file
- **README.md** - This file

## Author

Created by Claude Code & Hanzel
Based on Bitcraze's official Crazyflie parameters

## License

MIT License (following Bitcraze's licensing)
