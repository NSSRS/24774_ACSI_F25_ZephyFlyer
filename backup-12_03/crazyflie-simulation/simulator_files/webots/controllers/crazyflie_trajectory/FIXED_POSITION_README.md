# Fixed Position Controller (pid_fixed_position_controller)

## Overview

This is a new cascade PID controller that maintains **fixed XYZ position** by adding an outer position loop on top of the existing velocity controller.

## Control Architecture

```
Target Position (X, Y, Z)
         ↓
    ┌─────────────────────┐
    │  POSITION PID       │  Outer Loop (NEW!)
    │  X, Y position      │  → Velocity commands
    └─────────────────────┘
         ↓
    Desired Velocity (vx, vy)
         ↓
    ┌─────────────────────┐
    │  VELOCITY PID       │  Middle Loop (existing)
    │  vx, vy velocity    │  → Attitude commands
    └─────────────────────┘
         ↓
    Desired Attitude (roll, pitch)
         ↓
    ┌─────────────────────┐
    │  ATTITUDE PID       │  Inner Loop (existing)
    │  roll, pitch, yaw   │  → Motor commands
    └─────────────────────┘
         ↓
    ┌─────────────────────┐
    │  ALTITUDE PID       │  Parallel Loop (existing)
    │  Z altitude         │  → Thrust command
    └─────────────────────┘
         ↓
    Motor Commands [m1, m2, m3, m4]
```

## Key Features

### 1. Position Loop (NEW!)
- **PD+I Control**: Position error → Velocity command
- **Gains**: Kp=0.6, Kd=0.3, Ki=0.05
- **Max velocity**: 0.3 m/s (saturated)
- **Anti-windup**: Prevents integral saturation
- **Purpose**: Reject constant disturbances (e.g., wind)

### 2. Comparison with Velocity Controller

| Feature | `pid_velocity_fixed_height_controller` | `pid_fixed_position_controller` (NEW) |
|---------|---------------------------------------|--------------------------------------|
| **Input** | Desired velocity (vx, vy) | Desired position (x, y) |
| **XY Control** | Velocity tracking only | Position hold + velocity tracking |
| **Z Control** | Fixed altitude | Fixed altitude |
| **Disturbance Rejection** | No position feedback | Integral term rejects constant wind |
| **Use Case** | Manual flight, velocity commands | Hover, waypoint navigation, station-keeping |

## Files

### 1. Controller Class
**Location**: `controllers_shared/python_based/pid_controller.py`

**Class**: `pid_fixed_position_controller()`

**API**:
```python
motor_power = PID_CF.pid(
    dt,                      # Time step
    desired_x, desired_y,    # Target X, Y position (world frame)
    desired_yaw_rate,        # Desired yaw rate
    desired_altitude,        # Target altitude (Z)
    actual_x, actual_y,      # Current X, Y position (world frame)
    actual_roll, actual_pitch, actual_yaw_rate,  # Current attitude
    actual_altitude,         # Current altitude
    actual_vx, actual_vy     # Current body velocity
)
```

### 2. Test Script
**Location**: `controllers/crazyflie_trajectory/crazyflie_fixed_position.py`

**Purpose**: Demonstrates fixed position hold under impulse forces and continuous wind.

## Usage

### Step 1: Launch Webots Simulation

In Webots, you'll need to modify an existing world file to use the new controller. For example, modify `crazyflie_trajectory.wbt`:

1. Open `crazyflie-simulation/simulator_files/webots/worlds/crazyflie_trajectory.wbt`
2. Find the Crazyflie robot node
3. Change the `controller` field from `"crazyflie_trajectory"` to `"crazyflie_trajectory/crazyflie_fixed_position"`

Alternatively, create a new world file with:
```
controller "crazyflie_trajectory/crazyflie_fixed_position"
```

### Step 2: Run the Controller

The drone will automatically:
1. Take off and hover at `(0, 0, 1.0)` meters
2. Maintain this position using the fixed position controller
3. Print status every 2 seconds showing position error

### Step 3: Test Disturbance Rejection

#### Impulse Forces (Single Push)
- Press **F** - Forward push (+X, 0.5N default)
- Press **G** - Left push (+Y)
- Press **H** - Backward push (-X)
- Press **J** - Right push (-Y)
- Press **U** - Upward push (+Z)
- Press **D** - Downward push (-Z)

The drone should recover and return to the target position!

#### Continuous Wind
1. Press **7** to set wind direction to +X
2. Press **V** to enable wind
3. Observe the drone fighting against the wind and maintaining position
4. The **integral term** will accumulate to counteract the constant force

#### Adjust Target Position
- **Arrow Keys**: Move target ±0.5m in X/Y
- **W/S**: Move target ±0.2m in Z
- **R**: Reset target to origin (0, 0, 1.0)

### Step 4: Monitor Performance

Press **SPACE** to print detailed status:
```
📊 Status (t=25.50s):
  Target:   (+0.000, +0.000, +1.000)
  Current:  (+0.003, -0.002, +0.998)
  Error:    0.0041 m
  Velocity: (+0.001, -0.001) m/s
  Wind:     [+0.010, +0.000, +0.000] N
```

## PID Gains

### Position Loop (Outer)
```python
kp_pos_xy = 0.6    # [1/s] Position proportional gain
kd_pos_xy = 0.3    # [-] Velocity damping gain
ki_pos_xy = 0.05   # [1/s^2] Position integral gain
```

### Velocity Loop (Middle)
```python
kp_vel_xy = 2.0    # Velocity proportional gain
kd_vel_xy = 0.5    # Velocity derivative gain
```

### Attitude Loop (Inner)
```python
kp_att_rp = 0.5    # Roll/pitch proportional gain
kd_att_rp = 0.1    # Roll/pitch derivative gain
kp_att_y = 1.0     # Yaw proportional gain
kd_att_y = 0.5     # Yaw derivative gain
```

### Altitude Loop
```python
kp_z = 10.0        # Altitude proportional gain
ki_z = 5.0         # Altitude integral gain
kd_z = 5.0         # Altitude derivative gain
```

## Tuning Guidelines

### If position error is too large:
- Increase `kp_pos_xy` (faster response, may overshoot)
- Increase `ki_pos_xy` (better steady-state, may oscillate)

### If drone oscillates around target:
- Decrease `kp_pos_xy`
- Increase `kd_pos_xy` (more damping)

### If drone drifts under constant wind:
- Increase `ki_pos_xy` (stronger integral action)
- Check that anti-windup is working (integral limits at ±2.0)

### If velocity saturation occurs:
- Increase `max_velocity` from 0.3 m/s (line 169 in pid_controller.py)
- Or decrease `kp_pos_xy` to reduce commanded velocities

## Troubleshooting

### Problem: Drone doesn't hold position
- **Check**: Are you using `pid_fixed_position_controller` (not `pid_velocity_fixed_height_controller`)?
- **Check**: Is the target position within reasonable range (<5m from origin)?

### Problem: Drone oscillates
- **Tune**: Reduce `kp_pos_xy` or increase `kd_pos_xy`
- **Check**: Is there lag in the sensor readings? (GPS update rate)

### Problem: Drone drifts under wind
- **Increase**: `ki_pos_xy` from 0.05 to 0.1
- **Check**: Is wind force reasonable? (<0.05N for 27g drone)

### Problem: Drone reacts too slowly
- **Increase**: `kp_pos_xy` from 0.6 to 1.0
- **Increase**: `max_velocity` from 0.3 to 0.5 m/s

## Comparison with Previous Implementation

The `crazyflie_trajectory.py` script had a **hand-coded outer loop** in the main script:

```python
# Old approach (in crazyflie_trajectory.py)
ep_x_b = target_pos[0] - x_global
fwd_cmd = Kp_xy * ep_x_b - Kd_xy * vx_f + Ki_xy * int_ep_x
forward_desired = fwd_cmd  # Pass to velocity controller
```

The **new approach** encapsulates this in the controller class:

```python
# New approach (in pid_fixed_position_controller)
motor_power = PID_CF.pid(dt, target_x, target_y, ...)
```

### Benefits:
1. ✅ **Cleaner code**: Position control logic is in the controller, not scattered in main loop
2. ✅ **Reusable**: Can be used in any Webots script or even exported to hardware
3. ✅ **Consistent API**: Matches the style of `pid_velocity_fixed_height_controller`
4. ✅ **Easier tuning**: All gains are in one place

## Example Use Cases

### 1. Station-Keeping (Hover in Wind)
```python
# Hover at (1.0, 2.0, 1.5) under wind
target_x = 1.0
target_y = 2.0
target_z = 1.5

# Enable continuous wind
wind_enabled = True
wind_force = [0.02, 0.01, 0]  # Constant 20mN forward, 10mN left

# Controller will use integral term to maintain position!
```

### 2. Waypoint Navigation
```python
# Fly to waypoint sequence
waypoints = [(0,0,1), (1,0,1), (1,1,1), (0,1,1), (0,0,1)]

for wp in waypoints:
    target_x, target_y, target_z = wp
    while position_error > 0.1:  # Wait until reached
        # Controller holds position at waypoint
        motor_power = PID_CF.pid(...)
```

### 3. Disturbance Rejection Testing
```python
# Test response to impulse
at t=10s: apply force +X 1.0N
at t=15s: measure settling time
at t=20s: measure steady-state error
```

## Future Improvements

1. **Velocity filtering**: Add low-pass filter to reduce noise (like in `crazyflie_trajectory.py`)
2. **Yaw position control**: Extend to fix yaw angle (not just yaw rate)
3. **Trajectory tracking**: Add reference feedforward for smoother tracking
4. **Adaptive gains**: Adjust gains based on error magnitude or flight mode

## Author & Date

**Created**: 2025-11-06
**Based on**: Bitcraze official `pid_velocity_fixed_height_controller`
**Enhancement**: Added outer position loop with PD+I control
