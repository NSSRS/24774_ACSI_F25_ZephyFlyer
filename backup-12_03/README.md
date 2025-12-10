# 24774_ACSI_F25_ZephyFlyer
Repo for robust control project based on CrazyFlie 2.1+ drone and disturbance.

## 🧠 Simulation

This repository includes **two simulation environments** for the Crazyflie 2.1 drone:

1. **MuJoCo** - High-precision physics simulation for control algorithm development
2. **Webots** - Official Bitcraze simulation for rapid prototyping and testing

### 📊 Simulation Comparison

| Feature | MuJoCo | Webots |
|---------|--------|--------|
| **Physics Accuracy** | High | Medium |
| **Official Support** | General physics engine | CrazyFlie official |
| **Maintenance** | Active (DeepMind) | Limited (archived) |
| **Best For** | Control algorithm R&D | Quick demos, testing |
| **Documentation** | See below | [WEBOTS_SETUP.md](WEBOTS_SETUP.md) |

### 📖 Documentation

- **[PROJECT_STATUS.md](PROJECT_STATUS.md)** - Current development status, MuJoCo PID issues, technical analysis
- **[WEBOTS_QUICKSTART.md](WEBOTS_QUICKSTART.md)** - 5-minute Webots setup and testing guide
- **[WEBOTS_SETUP.md](WEBOTS_SETUP.md)** - Complete Webots installation and configuration

---

## MuJoCo Simulation

High-precision physics simulation for control algorithm development.

### Setup
```bash
# Activate virtual environment and install dependencies
source setup_env.sh
```

### Running Simulations

#### 1. Basic MuJoCo Viewer (No Control)
Launch the simulation viewer to inspect the model:
```bash
python -m mujoco.viewer --mjcf simulation/crazyflie_sim/scene.xml
```

#### 2. Hover Demonstration (Simple Hover Control)
Run the hover control demo to see the drone stabilize at 0.1m altitude:
```bash
python simulation/hover_demo.py
```
This script applies constant thrust based on the hover keyframe to keep the drone stable in the air

#### 3. Keyboard Control (Interactive Manual Control)
Fly the drone manually using keyboard controls:
```bash
python simulation/keyboard_control.py
```

**Keyboard Controls:**
- **Arrow Keys**: Pitch (↑/↓) and Roll (←/→)
- **W/S**: Increase/Decrease altitude (thrust)
- **A/D**: Yaw left/right (rotation)
- **Space**: Reset to hover position
- **ESC**: Exit simulation

The drone starts in hover mode, and you can add control inputs incrementally. Controls are damped for smooth flight.

#### 4. Cascade PID Control (Autonomous Position Control)
Run the cascade PID controller to autonomously fly to a target position:
```bash
python simulation/cascade_pid_control.py
```

**Features:**
- **Two-layer control**: Outer loop (position) + Inner loop (attitude)
- **Motor mixing**: Simulates 4 individual motor thrusts with saturation handling
- **Target position**: Default (0, 0, 0.5) meters - flies to 0.5m altitude
- **Real-time feedback**: Position error, motor thrusts, saturation warnings
- **Anti-windup**: PID integral protection to prevent overshoot

The controller uses a cascade architecture where the outer loop computes desired attitude from position error, and the inner loop tracks that attitude. Motor allocation distributes forces/moments to 4 individual motors with proper saturation handling.

#### 5. Improved Cascade PID (Enhanced Stability)
Run the improved version with better stability and anti-windup:
```bash
python simulation/cascade_pid_improved.py
```

**Improvements over basic cascade PID:**
- **Conditional integration**: Disables integral term when error > 0.5m to prevent windup
- **Integral clamping**: Explicit limits on integral accumulation
- **Reduced Ki gains**: Less aggressive integration (50-60% reduction)
- **Tilt angle limiting**: Maximum 15° tilt to prevent extreme maneuvers
- **Emergency reset**: Automatically resets PID on large errors (> 2m)
- **Derivative filtering**: Low-pass filter on D-term to reduce noise amplification
- **Enhanced diagnostics**: Detailed PID term logging and reset counter

Use this version if the basic PID becomes unstable after extended runtime.

#### 6. Tuned Cascade PID (Optimized for Faster Convergence)
Run the tuned version optimized for faster response and convergence:
```bash
python simulation/cascade_pid_tuned.py
```

**Key improvements over improved cascade PID:**
- **Relaxed conditional integration**: Increased error threshold from 0.5m to 3.0m to avoid deadlock
- **Increased integral gains**: Faster steady-state error elimination
- **Optimized proportional gains**: Reduced overshoot while maintaining responsiveness
- **Enhanced derivative gains**: Better damping and stability
- **More aggressive tuning**: Faster convergence to target position

This version resolves issues where the improved PID gets stuck due to overly conservative integration conditions.

---

## Webots Simulation

Official Bitcraze simulation environment for rapid prototyping and visualization.

### Quick Start (5 minutes)

**See [WEBOTS_QUICKSTART.md](WEBOTS_QUICKSTART.md) for detailed instructions.**

1. **Download Webots**: Visit https://cyberbotics.com/ and download Windows version
2. **Install**: Run installer with default options
3. **Open World**: `File` → `Open World...` → Navigate to:
   ```
   \\wsl.localhost\Ubuntu\home\hanzel\24774_project\24774_ACSI_F25_ZephyFlyer\crazyflie-simulation\simulator_files\webots\worlds\crazyflie_world.wbt
   ```
4. **Run**: Click ▶️ Play button
5. **Control**: Click 3D window, use arrow keys to fly

### Available Worlds

The `crazyflie-simulation/simulator_files/webots/worlds/` directory contains:

**Basic Demos:**
- **`crazyflie_world.wbt`** - Basic keyboard control in open environment
- **`crazyflie_apartement.wbt`** - Indoor scene with wall-following capability

**Advanced Control Demos:**
- **`crazyflie_simple_pid.wbt`** - Simple PID controller demonstration
- **`crazyflie_simple_pid_fixed.wbt`** - Fixed/improved version of simple PID
- **`crazyflie_trajectory.wbt`** - **Enhanced trajectory controller with XY position outer loop (PD+I)**
  - Features: Low-pass velocity filtering + Integral term for constant wind rejection
  - Supervisor-based external force application for disturbance testing
  - Real-time tunable parameters: Kp_xy=0.6, Kd_xy=0.3, Ki_xy=0.05
- **`crazyflie_disturbance.wbt`** - Disturbance rejection testing environment

### 🆕 NEW: Fixed Position Controller (Added 2025-11-06)

**Controller**: `crazyflie_fixed_position`

A high-performance position hold controller that maintains fixed XYZ position under strong disturbances.

**Key Features:**
- ⚡ **Very fast response**: ~1-2 seconds return time (for 1m displacement)
- 💪 **Strong disturbance rejection**: Impulse forces and continuous wind
- 🎯 **Precise position hold**: Steady-state error < 0.01m
- 🔄 **Coordinate frame transformation**: World frame → Body frame (critical fix!)

**Controller Architecture:**
```
Position Error → [PD+I Outer Loop] → Velocity → [Velocity PID] → Attitude → [Attitude PID] → Motors
```

**Aggressive Tuning:**
```python
Kp_pos_xy: 3.0      # Very high for fast response (was 0.6 originally)
Kd_pos_xy: 0.15     # Low damping for speed
Ki_pos_xy: 0.2      # Strong integral for wind rejection
max_velocity: 1.0 m/s  # High speed limit
```

**Keyboard Controls:**
- **Arrow Keys**: Adjust target XY (±0.5m)
- **W/S**: Adjust target altitude (±0.2m)
- **R**: Reset to origin
- **F/G/H/J**: Apply impulse forces (±X/Y)
- **7/8/9/0**: Set wind direction
- **V**: Toggle wind ON/OFF
- **SPACE**: Print status

**Testing:**
1. Open any Webots world with CrazyFlie
2. Set controller to: `crazyflie_fixed_position`
3. Run simulation → drone hovers at (0, 0, 1.0)
4. Press **F** to apply forward impulse → observe fast return!

**Technical Achievement:**
- Fixed coordinate frame bug (world vs body frame)
- Achieved 7-8× speed improvement over conservative tuning
- Return time reduced from ~15s → ~1-2s

### Enhanced Trajectory Controller (Wind/Disturbance Rejection)

The **`crazyflie_trajectory.wbt`** world now includes an **enhanced XY position outer loop** designed for robust position control under external disturbances (wind, impulse forces).

**Controller Architecture:**
```
Position Error → [PD+I Outer Loop] → Velocity Command → [Official Bitcraze Inner Loop] → Motor Commands
```

**Key Features:**
- **PD+I Control**: Proportional + Derivative (on filtered velocity) + Integral (for constant wind)
- **Low-pass Velocity Filter**: Reduces noise amplification (tau_v = 0.1s)
- **Anti-windup**: Simple back-calculation to prevent integral saturation
- **Tunable Parameters**:
  - `Kp_xy = 0.6` - Position gain (1/s)
  - `Kd_xy = 0.3` - Velocity damping
  - `Ki_xy = 0.05` - Integral gain for steady-state error elimination under constant wind
  - `v_xy_max = 0.3 m/s` - Maximum velocity command

**Testing Disturbances:**
- **Impulse forces**: Single push (F/G/H/J keys for X/Y directions)
- **Continuous wind**: Constant force applied every timestep (7/8/9/0 keys + V to enable)
- Press `P` to view outer loop status (integrals, filtered velocity)

**Use Case**: Test position hold performance under wind gusts and constant wind conditions.

### Key Finding: PID Parameters

Webots uses **official Bitcraze PID parameters** that differ significantly from our MuJoCo implementation:

```python
# Webots (official)         # MuJoCo (ours)
kp_att_rp: 0.5               kp_att_rp: 0.02  # 25x difference!
kd_att_rp: 0.1               kd_att_rp: 0.004 # 25x difference!
```

**This parameter analysis may help resolve MuJoCo PID instability issues.**

### Documentation

- **[WEBOTS_QUICKSTART.md](WEBOTS_QUICKSTART.md)** - Quick start guide
- **[WEBOTS_SETUP.md](WEBOTS_SETUP.md)** - Complete setup, comparison, troubleshooting
- **Controller source**: `crazyflie-simulation/controllers_shared/python_based/pid_controller.py`

---

## Project Status

See **[PROJECT_STATUS.md](PROJECT_STATUS.md)** for:
- Current MuJoCo PID controller development status
- Known stability issues and debugging strategies
- Technical architecture and parameter documentation
- Comparison with Webots parameters
