# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a robust control project based on the CrazyFlie 2.1+ drone platform. The codebase contains test scripts for drone control and hardware deck validation using the CrazyFlie Python library (cflib).

## Environment Setup

The project uses a Python virtual environment with dependencies managed through requirements.txt.

**Setup environment:**
```bash
source setup_env.sh
```

This script creates a `.venv` virtual environment, activates it, and installs all dependencies from requirements.txt.

**Manual activation (after initial setup):**
```bash
source .venv/bin/activate
```

**Install/update dependencies:**
```bash
pip install -r requirements.txt
```

## Running Scripts

All test scripts are located in `tester_files/` and should be run with the virtual environment activated.

**Basic connection test:**
```bash
python tester_files/connect_cf.py
```

**Flight demonstration (scripted path):**
```bash
python tester_files/test_flowdeck.py
```

**Multi-ranger deck test (obstacle avoidance):**
```bash
python tester_files/flight_demo.py
```

**Custom URI (optional):**
```bash
python tester_files/flight_demo.py radio://0/80/2M
```

## Architecture

### CrazyFlie Connection Pattern

All scripts follow a consistent pattern for connecting to the drone:

1. **Initialize CRTP drivers** with `cflib.crtp.init_drivers()`
2. **Scan for available interfaces** (or use predefined URI)
3. **Create SyncCrazyflie context** with the target URI
4. **Arm the drone** with `scf.cf.platform.send_arming_request(True)`
5. **Execute flight commands** within the context manager
6. **Automatic cleanup** on context exit

### Key Components

- **SyncCrazyflie**: Synchronous context manager for drone connection
- **MotionCommander**: High-level flight command interface for scripted movements
- **Multiranger**: Interface to multi-ranger deck for distance measurements in all directions
- **Cache**: Drone configuration cache stored in `./cache` directory

### URI Configuration

Default URI is `radio://0/80/2M`. This should be changed to match your specific radio channel configuration. URI can be:
- Hardcoded in the script's `URI` variable
- Passed as command-line argument (supported in flight_demo.py and test_rangerdeck.py)
- Auto-detected via `cflib.crtp.scan_interfaces()`

### Hardware Requirements

The test scripts assume the following hardware setup:
- CrazyFlie 2.1+ drone
- Crazyradio PA USB dongle
- Flow deck (for test_flowdeck.py)
- Multi-ranger deck (for flight_demo.py and test_rangerdeck.py)

## File Organization

### Hardware Test Scripts (tester_files/)
- `tester_files/connect_cf.py`: Basic connection and interface scanning
- `tester_files/test_flowdeck.py`: Scripted flight path demonstration using MotionCommander
- `tester_files/flight_demo.py`: Interactive obstacle avoidance using multi-ranger deck
- `tester_files/test_rangerdeck.py`: Duplicate of flight_demo.py for ranger deck testing

Note: `flight_demo.py` and `test_rangerdeck.py` are identical implementations.

### Simulation Scripts (simulation/)

#### MuJoCo Environment
The project includes a MuJoCo-based physics simulation for CrazyFlie 2.1 located in `simulation/crazyflie_sim/`:
- `scene.xml`: Main simulation scene with ground plane and lighting
- `cf2.xml`: CrazyFlie 2.1 drone model with actuators and sensors
- `assets/`: 3D mesh files for visual and collision geometry

The MuJoCo model uses a simplified control abstraction:
- **4 actuators**: body_thrust, x_moment (roll), y_moment (pitch), z_moment (yaw)
- **3 sensors**: gyroscope, accelerometer, orientation (quaternion)
- **Hover keyframe**: Predefined hover state at 0.1m with thrust=0.26487N

**Important**: This model uses direct force/moment control rather than individual motor RPM. Propellers in the visualization do not rotate - this is intentional for the simplified dynamics model.

#### Simulation Scripts

**Basic hover demonstration:**
```bash
python simulation/hover_demo.py
```
Demonstrates constant-thrust hover control using the predefined hover keyframe. The drone stabilizes at approximately 0.1m altitude.

**Keyboard-controlled flight:**
```bash
python simulation/keyboard_control.py
```
Interactive manual control with keyboard inputs:
- Arrow keys: Pitch (↑/↓) and Roll (←/→)
- W/S: Altitude control (increase/decrease thrust)
- A/D: Yaw rotation (left/right)
- Space: Reset to hover state
- ESC: Exit simulation

Controls are incremental with automatic damping for smooth flight behavior.

**Cascade PID controllers:**
```bash
python simulation/cascade_pid_control.py      # Basic cascade PID
python simulation/cascade_pid_improved.py     # Enhanced stability version
python simulation/cascade_pid_tuned.py        # Optimized for faster convergence
```
Three versions of cascade PID controller with progressive improvements:
- **Basic**: Two-layer control with motor mixing and anti-windup
- **Improved**: Additional stability features (conditional integration, tilt limiting, derivative filtering)
- **Tuned**: Relaxed integration conditions (3.0m threshold) and optimized gains for faster response

**Viewer-only mode (no control):**
```bash
python -m mujoco.viewer --mjcf simulation/crazyflie_sim/scene.xml
```
Launch MuJoCo's built-in viewer to inspect the model without running control scripts.

#### Simulation Development Notes

- All simulation scripts require `mujoco>=3.0.0` (see requirements.txt)
- The keyboard controller uses increased gains for better visibility (moment_increment=0.001)
- Debug output includes position, velocity, and all control deltas
- The model is cross-platform: works on Windows, Linux, macOS
- WSL2 users need X Server (VcXsrv) for graphics; native Windows recommended for best experience

### Webots Environment (Alternative Simulation)

The project now includes access to the official Bitcraze Webots simulation through the `crazyflie-simulation` repository (cloned locally, not tracked in git).

**Location**: `/crazyflie-simulation/` (gitignored external repository)

#### Key Files

**World files**: `crazyflie-simulation/simulator_files/webots/worlds/`

Basic Demos:
- `crazyflie_world.wbt` - Basic keyboard control in open environment
- `crazyflie_apartement.wbt` - Indoor environment with wall following

Advanced Control Demos:
- `crazyflie_simple_pid.wbt` - Simple PID controller demonstration
- `crazyflie_simple_pid_fixed.wbt` - Fixed/improved version of simple PID
- `crazyflie_trajectory.wbt` - **Enhanced trajectory controller with XY position outer loop (PD+I)**
  - Features: Low-pass velocity filtering, integral for wind rejection, anti-windup
  - Parameters: Kp_xy=0.6, Kd_xy=0.3, Ki_xy=0.05, v_xy_max=0.3 m/s, tau_v=0.1s
  - Disturbance testing: Impulse forces and continuous wind simulation
- `crazyflie_disturbance.wbt` - Disturbance rejection testing environment

**Controllers**: `crazyflie-simulation/simulator_files/webots/controllers/`
- `crazyflie_controller_py/` - Python keyboard controller
- `crazyflie_controller_py_firmware_pid/` - Firmware-based PID
- `crazyflie_controller_py_wallfollowing/` - Wall following demo
- `crazyflie_simple_pid/` - Simple PID implementation
- `crazyflie_trajectory/` - **Enhanced trajectory controller with XY outer loop (PD+I)**
  - Outer loop: Position error → Velocity command (with integral for constant wind)
  - Inner loop: Official Bitcraze velocity controller (unchanged)
  - Press 'P' to view outer loop status (integrals, filtered velocity)
- `crazyflie_fixed_position/` - **NEW: Fixed Position Controller (XYZ Position Hold)**
  - Full cascade controller with position outer loop
  - Maintains fixed XYZ position under disturbances
  - Features: Impulse force testing, continuous wind rejection
  - Aggressive tuning: Kp=3.0, max velocity=1.0 m/s
  - Coordinate frame transformation (world → body frame)
  - Return time: ~1-2 seconds for 1m displacement

**Shared PID controller**: `crazyflie-simulation/controllers_shared/python_based/pid_controller.py`
- `pid_velocity_fixed_height_controller()` - Velocity control with fixed altitude
- `pid_fixed_position_controller()` - **NEW: Position control with fixed XYZ** (added 2025-11-06)
  - Outer loop: Position error (world frame) → Velocity command (body frame)
  - Includes coordinate transformation using yaw angle
  - PD+I control with anti-windup
  - Aggressive tuning for fast response (Kp=3.0, Kd=0.15, Ki=0.2)
  - Max velocity: 1.0 m/s

#### Installation and Setup

**See [WEBOTS_QUICKSTART.md](WEBOTS_QUICKSTART.md) for 5-minute setup guide.**

**Installation requirements**:
1. Webots R2023b or newer (Windows recommended for WSL2 users)
2. Download from https://cyberbotics.com/
3. No additional Python dependencies needed for basic keyboard control
4. cflib already installed in project venv for advanced features

**Running Webots**:
```bash
# On Windows, open Webots GUI and load:
# File → Open World... → Navigate to:
\\wsl.localhost\Ubuntu\home\hanzel\24774_project\24774_ACSI_F25_ZephyFlyer\crazyflie-simulation\simulator_files\webots\worlds\crazyflie_world.wbt
```

**Keyboard controls in Webots**:
- Arrow keys: Forward/backward/left/right
- W/S: Up/down (altitude)
- Q/E: Yaw left/right
- A: Toggle wall following (in apartment world)

#### Webots vs MuJoCo

**Use Webots for**:
- Quick prototyping and demos
- Visual debugging in complex environments
- Testing with official Bitcraze parameters
- Reference implementation comparison

**Use MuJoCo for**:
- High-precision control algorithm development
- Parameter tuning and optimization
- Research-grade simulation accuracy
- Custom dynamics modeling

#### Important: Official PID Parameters

The Webots controller uses official Bitcraze PID gains that differ significantly from our MuJoCo implementation:

```python
# Webots (official Bitcraze parameters)
gains = {
    "kp_att_rp": 0.5,    # Roll/pitch attitude
    "kd_att_rp": 0.1,
    "kp_att_y": 1.0,     # Yaw attitude
    "kd_att_y": 0.5,
    "kp_vel_xy": 2.0,    # XY velocity
    "kd_vel_xy": 0.5,
    "kp_z": 10.0,        # Altitude
    "ki_z": 5.0,
    "kd_z": 5.0
}
```

**Key differences from MuJoCo implementation**:
- Attitude control gains are **25x higher** (0.5 vs 0.02)
- This may explain MuJoCo PID instability issues
- Consider adjusting MuJoCo inner loop gains based on these values

#### Documentation

- **[PROJECT_STATUS.md](PROJECT_STATUS.md)** - Current development status and technical analysis
- **[WEBOTS_SETUP.md](WEBOTS_SETUP.md)** - Complete Webots installation guide
- **[WEBOTS_QUICKSTART.md](WEBOTS_QUICKSTART.md)** - Quick start guide

---

## New Feature: Fixed Position Controller (2025-11-06)

### Overview

A new cascade PID controller `pid_fixed_position_controller` has been implemented to maintain **fixed XYZ position** under external disturbances (impulse forces and continuous wind).

### Key Features

1. **Position Outer Loop**
   - Converts position errors (world frame) to velocity commands (body frame)
   - PD+I control with anti-windup
   - Coordinate transformation: world frame → body frame using yaw angle

2. **Aggressive Tuning for Fast Response**
   ```python
   Kp_pos_xy: 3.0     # Very high proportional gain
   Kd_pos_xy: 0.15    # Low damping for fast response
   Ki_pos_xy: 0.2     # Strong integral for wind rejection
   max_velocity: 1.0 m/s  # High speed limit
   ```

3. **Performance**
   - Return time: ~1-2 seconds (for 1m displacement)
   - Wind rejection: Strong integral action
   - Overshoot: 10-20% (acceptable for speed)

### Files

**Controller Class**: `crazyflie-simulation/controllers_shared/python_based/pid_controller.py:86-260`
- Class: `pid_fixed_position_controller()`
- API: Takes desired position (x, y, z) and yaw, outputs motor commands
- Coordinate transformation included

**Test Controller**: `crazyflie-simulation/simulator_files/webots/controllers/crazyflie_fixed_position/`
- `crazyflie_fixed_position.py` - Main controller script
- `README.md` - Usage instructions

**Documentation**:
- `BUGFIX_COORDINATE_FRAME.md` - Explanation of coordinate transformation fix
- `TUNING_AGGRESSIVE.md` - First tuning iteration
- `TUNING_VERY_AGGRESSIVE.md` - Final aggressive tuning

### Usage

**In Webots**:
1. Open any world with CrazyFlie
2. Set controller field to: `crazyflie_fixed_position`
3. Run simulation
4. Drone hovers at (0, 0, 1.0) by default

**Keyboard Controls**:
- **Arrow Keys**: Adjust target XY position (±0.5m)
- **W/S**: Adjust target altitude (±0.2m)
- **R**: Reset target to origin
- **F/G/H/J**: Apply impulse forces (horizontal)
- **U/D**: Apply impulse forces (vertical)
- **7/8/9/0**: Set wind direction
- **V**: Toggle wind ON/OFF
- **SPACE**: Print status

### Technical Highlights

#### Coordinate Frame Transformation

**Problem**: Original implementation calculated position errors in world frame but passed them directly to velocity controller expecting body frame commands.

**Solution**: Transform world frame errors to body frame using yaw angle:
```python
# World frame error
x_error_world = target_x - actual_x
y_error_world = target_y - actual_y

# Transform to body frame
x_error_body = x_error_world * cos(yaw) + y_error_world * sin(yaw)
y_error_body = -x_error_world * sin(yaw) + y_error_world * cos(yaw)

# PID control in body frame
v_cmd = Kp * error_body - Kd * actual_velocity + Ki * integral
```

#### Tuning Evolution

| Version | Kp | max_velocity | Return Time | Status |
|---------|-----|-------------|-------------|--------|
| v1 (Conservative) | 0.6 | 0.3 m/s | ~15s | Too slow |
| v2 (Aggressive) | 1.5 | 0.6 m/s | ~6s | Still slow |
| v3 (Very Aggressive) | 3.0 | 1.0 m/s | ~1-2s | ✅ Current |

### Comparison with Trajectory Controller

| Feature | `crazyflie_trajectory` | `crazyflie_fixed_position` |
|---------|----------------------|---------------------------|
| **Position Control** | In main script | In controller class ✅ |
| **Reusability** | Script-specific | Fully reusable ✅ |
| **Tuning** | Mixed in code | Centralized ✅ |
| **Coordinate Transform** | In main loop | In controller ✅ |
| **API** | Velocity commands | Position commands ✅ |

### Future Work

- Add velocity filtering (low-pass filter) for smoother response
- Implement yaw position control (not just yaw rate)
- Add trajectory tracking with feedforward
- Test on real hardware
