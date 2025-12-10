# Trajectory Tracking Controller

This controller extends the simple PID position controller with automatic trajectory tracking capabilities.

## Features

### Trajectory Types

1. **Hover Mode** - Maintain position at a fixed point
2. **Circle Trajectory** - Fly in circular path (default: radius=0.5m, period=10s)
3. **Figure-8 Trajectory** - Fly in figure-8 pattern (lemniscate)
4. **Square Trajectory** - Fly in square pattern (default: 1m × 1m)

### Controls

#### Manual Position Control
- **Arrow Keys** - Move forward/backward/left/right [±0.2m]
- **W/S** - Move up/down [±0.1m]
- **A/D** - Alternative left/right [±0.2m]
- **Q/E** - Rotate left/right (Yaw)
- **R** - Reset to origin (0,0,1)

*Note: Any manual control automatically switches to hover mode*

#### Trajectory Commands
- **1** - Hover at current position
- **2** - Start circle trajectory
- **3** - Start figure-8 trajectory
- **4** - Start square trajectory
- **+/-** - Increase/decrease trajectory speed (period: 5-30s)

#### Info Commands
- **SPACE** - Print current status (mode, position, target, error, attitude)

## Usage

### In Webots

1. Open Webots
2. Load world file:
   ```
   crazyflie-simulation/simulator_files/webots/worlds/crazyflie_trajectory.wbt
   ```
3. Run simulation
4. **Click on 3D window** to ensure keyboard focus
5. Wait for drone to stabilize at 1m altitude (~20 seconds)
6. Press **2** to start circle trajectory

### Testing Different Trajectories

```
# Test Circle
Press: 2
Expected: Drone flies in 0.5m radius circle, 10s per lap

# Test Figure-8
Press: 3
Expected: Drone flies in figure-8 pattern

# Test Square
Press: 4
Expected: Drone flies in 1m × 1m square

# Speed up trajectory
Press: + (multiple times)
Expected: Faster motion (shorter period)

# Return to manual control
Press: Arrow keys or W/S
Expected: Switches back to hover mode at new position
```

## Trajectory Parameters

You can modify parameters in `crazyflie_trajectory.py`:

```python
# Circle parameters
self.radius = 0.5      # Circle radius in meters
self.period = 10.0     # Time to complete one circle (seconds)

# Figure-8 parameters
self.fig8_scale = 0.5  # Figure-8 size scale

# Square parameters
self.square_size = 1.0 # Square side length in meters
```

## PID Tuning

The controller uses the same optimized PID parameters as `crazyflie_simple_pid`:
- **Z-axis**: Kp=2.0, Ki=3.0, Kd=8.0 (reduces overshoot to ~220%)
- **X/Y position**: Kp=0.5, Ki=0.05, Kd=0.3
- **X/Y velocity**: Kp=2.0, Ki=0.0, Kd=0.5
- **Attitude**: Kp=0.5, Ki=0.0, Kd=0.1

## Output Format

During trajectory tracking, the controller prints status every 0.5s:
```
[  5.00s] Mode:circle   | Pos:[+0.50,+0.00,1.00] | Target:[+0.48,+0.16,1.00] | Err:0.015m
```

## Troubleshooting

**Keyboard not responding:**
- Click on Webots 3D window to ensure keyboard focus
- Check that you're not in pause mode

**Drone not following trajectory well:**
- Wait for initial hover stabilization (~20s)
- Try slower trajectory (press `-` multiple times)
- Check that trajectory center is at reasonable altitude (>0.5m)

**Trajectory too fast/slow:**
- Press `+` to speed up (reduce period)
- Press `-` to slow down (increase period)
- Period range: 5-30 seconds

## Architecture

The controller uses a cascade PID structure:
1. **Outer Loop**: Position error → Desired velocity
2. **Middle Loop**: Velocity error → Desired attitude (pitch/roll)
3. **Inner Loop**: Attitude error → Motor commands
4. **Altitude Loop**: Height error → Thrust command

The `TrajectoryGenerator` class computes target positions in real-time based on:
- Current time
- Trajectory start time
- Trajectory type
- Trajectory parameters (radius, period, etc.)
