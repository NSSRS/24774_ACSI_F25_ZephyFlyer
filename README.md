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
