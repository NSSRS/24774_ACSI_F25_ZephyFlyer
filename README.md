# 24774_ACSI_F25_ZephyFlyer
Repo for robust control project based on CrazyFlie 2.1+ drone and disturbance.

## 🧠 Simulation

This repository now includes a MuJoCo simulation environment for the Crazyflie 2.1 drone.

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
