# 24774_ACSI_F25_ZephyFlyer
Repo for robust control project based on CrazyFlie 2.1+ drone and disturbance.
 
## 🧠 Simulation

This repository now includes a MuJoCo simulation environment for the Crazyflie 2.1 drone.

### Setup
```bash
pip install mujoco 

Run the Simulation

You can launch the simulation viewer directly with:
python -m mujoco.viewer --mjcf simulation/crazyflie_sim/scene.xml
