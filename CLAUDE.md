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
