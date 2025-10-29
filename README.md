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
