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

- `tester_files/connect_cf.py`: Basic connection and interface scanning
- `tester_files/test_flowdeck.py`: Scripted flight path demonstration using MotionCommander
- `tester_files/flight_demo.py`: Interactive obstacle avoidance using multi-ranger deck
- `tester_files/test_rangerdeck.py`: Duplicate of flight_demo.py for ranger deck testing

Note: `flight_demo.py` and `test_rangerdeck.py` are identical implementations.
