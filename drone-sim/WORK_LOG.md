# Work Log - CrazyFlie Simulation Project

## 📅 Session: 2025-11-06

### ✅ Completed Work

#### 1. **Documentation Update and Review**
- **Reviewed entire directory structure** including:
  - All simulation files (MuJoCo and Webots)
  - Hardware test scripts
  - Documentation files

- **Updated README.md** with:
  - Added `cascade_pid_tuned.py` documentation (section 6)
  - Expanded Webots worlds list (basic + advanced control demos)
  - Added "Enhanced Trajectory Controller" section with detailed architecture
  - Parameters: Kp_xy=0.6, Kd_xy=0.3, Ki_xy=0.05, v_xy_max=0.3 m/s, tau_v=0.1s

- **Updated CLAUDE.md** with:
  - All three cascade PID variants (basic, improved, tuned)
  - Complete Webots world files list with categorization
  - Enhanced trajectory controller documentation
  - Controller architecture and usage instructions

#### 2. **Enhanced Webots Trajectory Controller**

**Location**: `crazyflie-simulation/simulator_files/webots/controllers/crazyflie_trajectory/crazyflie_trajectory.py`

**⚠️ IMPORTANT**: This file is in a gitignored directory - changes are LOCAL ONLY

**Implementation Details**:
Based on `suggetion.txt` requirements, added XY position outer loop with:

**Controller Architecture**:
```
Position Error (world) → Rotation to Body Frame → [PD+I Outer Loop] → Velocity Command → [Official Bitcraze Inner Loop] → Motors
```

**Added Parameters** (lines 136-147):
```python
Kp_xy = 6.000E-01      # [1/s] Position proportional gain
Kd_xy = 3.000E-01      # [-] Velocity damping gain
Ki_xy = 5.000E-02      # [1/s^2] Position integral gain (for constant wind)
v_xy_max = 3.000E-01   # [m/s] Maximum velocity command
tau_v = 1.000E-01      # [s] Low-pass filter time constant for velocity

# State variables
int_ep_x = 0.0  # Integral of body-frame X position error
int_ep_y = 0.0  # Integral of body-frame Y position error
vx_f = 0.0      # Filtered body-frame X velocity
vy_f = 0.0      # Filtered body-frame Y velocity
```

**Control Algorithm** (lines 395-442):
1. Compute position error in world frame
2. Rotate error to body frame using yaw angle
3. Rotate velocity measurements to body frame
4. Apply low-pass filter to velocity: `alpha = exp(-dt/tau_v)`
5. Integrate position error for wind rejection
6. Compute PD+I control: `cmd = Kp*error - Kd*vel_filtered + Ki*integral`
7. Saturate velocity commands to ±v_xy_max
8. Anti-windup: undo integral step if saturated and error pushing further

**Key Features**:
- ✅ Keeps inner loop (official Bitcraze velocity controller) **unchanged**
- ✅ Keeps Z-axis and yaw control logic **unchanged**
- ✅ Minimal code additions only
- ✅ Low-pass velocity filter reduces noise amplification
- ✅ Integral term eliminates steady-state error under constant wind
- ✅ Simple anti-windup prevents integral saturation

**New Keyboard Commands**:
- `P` - Print XY outer loop status (integrals, filtered velocity, position error)

#### 3. **Git Commit Created**

**Commit ID**: `7b2cd6c`
**Branch**: `feature/crazyflie-sim`
**Status**: ✅ Committed locally, ⚠️ NOT pushed to remote yet

**Files in Commit**:
- `simulation/cascade_pid_tuned.py` - NEW
- `README.md` - MODIFIED
- `CLAUDE.md` - MODIFIED
- `suggetion.txt` - NEW
- `test_data.txt` - NEW
- `.claude/settings.local.json` - NEW

**Commit Message**:
```
Add enhanced Webots controller and update documentation

Added:
- simulation/cascade_pid_tuned.py: Optimized cascade PID with relaxed integration threshold (3.0m) and faster convergence
- suggetion.txt: XY position outer loop design guide for Webots disturbance rejection
- .claude/settings.local.json: Local Claude Code settings
- test_data.txt: Test data logs

Updated:
- README.md: Added all Webots worlds, cascade_pid_tuned.py documentation, and enhanced trajectory controller section
- CLAUDE.md: Complete documentation of MuJoCo cascade PID variants and Webots controller architecture

Enhanced Webots trajectory controller (local changes in crazyflie-simulation/):
- Added XY position outer loop (PD+I) for wind/disturbance rejection
- Parameters: Kp_xy=0.6, Kd_xy=0.3, Ki_xy=0.05, v_xy_max=0.3 m/s
- Features: Low-pass velocity filter, integral for constant wind, anti-windup
- Press 'P' to view outer loop status during simulation
```

---

### ⚠️ Pending Tasks

#### 1. **Push to Remote Repository**
**Action Required**:
```bash
git push origin feature/crazyflie-sim
```

**Authentication Issue**: WSL + HTTPS requires credentials

**Solutions**:
- **Option 1 (Quick)**: Run push command, enter GitHub username + Personal Access Token
- **Option 2**: Configure Git Credential Manager:
  ```bash
  git config --global credential.helper "/mnt/c/Program\ Files/Git/mingw64/bin/git-credential-manager-core.exe"
  ```
- **Option 3**: Switch to SSH authentication

#### 2. **Test Enhanced Webots Controller**
**Not yet tested in simulation!**

**Test Plan**:
1. Open `crazyflie_trajectory.wbt` in Webots
2. Press Play ▶️
3. Test hover: Press `1` → `M` (auto mode) → `P` (view status)
4. Test impulse disturbance: `F`/`G`/`H`/`J` keys
5. Test constant wind:
   - Press `7` (set wind +X)
   - Press `V` (enable wind)
   - Press `P` (observe integral growth)
   - Verify position hold with minimal steady-state error
6. Test trajectory following under wind:
   - Press `4` (circle trajectory)
   - Apply wind during flight
   - Measure tracking error

**Expected Results**:
- Integral terms should grow to counteract constant wind
- Position error should converge to near-zero even under constant disturbance
- Velocity commands should saturate at ±0.3 m/s max

**Tuning if Needed**:
- Overshoot/jitter → Increase `Kd_xy` to 0.35 or `tau_v` to 0.2s
- Steady-state offset → Increase `Ki_xy` to 0.07

---

### 📂 Key Files and Locations

#### MuJoCo Simulation
- **Basic hover**: `simulation/hover_demo.py`
- **Keyboard control**: `simulation/keyboard_control.py`
- **Cascade PID (basic)**: `simulation/cascade_pid_control.py`
- **Cascade PID (improved)**: `simulation/cascade_pid_improved.py`
- **Cascade PID (tuned)**: `simulation/cascade_pid_tuned.py` ⭐ NEW

#### Webots Simulation (gitignored, local only)
- **Enhanced trajectory controller**: `crazyflie-simulation/simulator_files/webots/controllers/crazyflie_trajectory/crazyflie_trajectory.py` ⭐ MODIFIED
- **World files**: `crazyflie-simulation/simulator_files/webots/worlds/`
  - `crazyflie_world.wbt` - Basic keyboard control
  - `crazyflie_apartement.wbt` - Indoor with wall following
  - `crazyflie_trajectory.wbt` - Trajectory + disturbance testing ⭐ USE THIS
  - `crazyflie_simple_pid.wbt` - Simple PID demo
  - `crazyflie_disturbance.wbt` - Disturbance testing

#### Documentation
- **README.md** - User-facing documentation ⭐ UPDATED
- **CLAUDE.md** - Claude Code project instructions ⭐ UPDATED
- **WEBOTS_QUICKSTART.md** - 5-minute Webots setup guide
- **WEBOTS_SETUP.md** - Complete Webots installation
- **PROJECT_STATUS.md** - Development status and technical analysis
- **suggetion.txt** - XY outer loop design specification ⭐ NEW
- **WORK_LOG.md** - This file ⭐ NEW

---

### 🔧 Important Technical Details

#### Webots Enhanced Controller Parameters

**Position Outer Loop Gains**:
```python
Kp_xy = 0.6     # Position error → velocity (higher = faster response)
Kd_xy = 0.3     # Velocity damping (higher = more damping)
Ki_xy = 0.05    # Steady-state error elimination (higher = faster wind rejection)
```

**Velocity Limits and Filtering**:
```python
v_xy_max = 0.3  # Maximum velocity command [m/s]
tau_v = 0.1     # Low-pass filter time constant [s] (lower = more filtering)
```

**Official Bitcraze Inner Loop Gains** (unchanged):
```python
# Velocity → Attitude
kp_vel_xy = 2.0
kd_vel_xy = 0.5

# Attitude → Moments
kp_att_rp = 0.5  # Roll/pitch (25x higher than our MuJoCo!)
kd_att_rp = 0.1

# Altitude
kp_z = 10.0
ki_z = 5.0
kd_z = 5.0
```

#### Coordinate Frame Conventions

**World to Body Rotation**:
```python
# Given yaw angle psi:
cosyaw = cos(psi)
sinyaw = sin(psi)

# Position error
ep_x_body =  ep_x_world * cosyaw + ep_y_world * sinyaw
ep_y_body = -ep_x_world * sinyaw + ep_y_world * cosyaw

# Velocity
vx_body =  vx_world * cosyaw + vy_world * sinyaw
vy_body = -vx_world * sinyaw + vy_world * cosyaw
```

#### Anti-Windup Logic
```python
# After saturating velocity command:
if abs(fwd_cmd) >= v_xy_max and (fwd_cmd * ep_x_b) > 0.0:
    int_ep_x -= ep_x_b * dt  # Undo this step's integral
```
This prevents integral windup when the controller is saturated and error is pushing further in the same direction.

---

### 🎯 Next Session - Recommended Actions

#### Immediate (Start of Next Session)
1. ✅ **Push commit to remote** (requires authentication)
2. 🧪 **Test enhanced Webots controller** following test plan above
3. 📊 **Collect test data**:
   - Position error vs time under constant wind
   - Integral term growth
   - Velocity command saturation events

#### Short-term Improvements
1. **MuJoCo Parameter Tuning**:
   - Try using Bitcraze-inspired gains (0.5 instead of 0.02 for attitude)
   - Compare performance with Webots

2. **Add XY Outer Loop to MuJoCo**:
   - Similar implementation as Webots
   - Create `simulation/cascade_pid_with_outer_loop.py`

3. **Data Logging**:
   - Add CSV output for XY outer loop states
   - Plot integral terms vs wind force
   - Analyze steady-state errors

#### Long-term Goals
1. **Real Hardware Testing**:
   - Port enhanced controller to CrazyFlie firmware
   - Test on physical drone with fan-generated wind

2. **Advanced Control**:
   - Add feedforward term for known wind
   - Adaptive gains based on disturbance magnitude
   - Trajectory optimization under constraints

---

### 📚 Reference Information

#### Key Documentation Sections
- **README.md lines 101-114**: cascade_pid_tuned.py documentation
- **README.md lines 143-176**: Enhanced trajectory controller section
- **CLAUDE.md lines 135-144**: MuJoCo cascade PID variants
- **CLAUDE.md lines 174-191**: Webots controller documentation

#### Git Status
- **Current Branch**: `feature/crazyflie-sim`
- **Last Commit**: `7b2cd6c` (not pushed)
- **Remote**: `origin` → `https://hanze1iu@github.com/hanze1iu/24774_ACSI_F25_ZephyFlyer.git`

#### External Resources
- Official Bitcraze PID: `crazyflie-simulation/controllers_shared/python_based/pid_controller.py`
- CrazyFlie 2.1 model: `simulation/crazyflie_sim/cf2.xml`
- Webots documentation: https://cyberbotics.com/

---

## 📝 Notes for Future Development

### Design Decisions Made
1. **Why PD+I instead of full PID?**
   - D term on filtered velocity, not position error (reduces noise)
   - I term only for steady-state error under constant disturbance
   - P term provides fast response to position errors

2. **Why low-pass filter velocity?**
   - GPS velocity from finite differences is noisy
   - Filter prevents D-term from amplifying noise
   - tau_v = 0.1s balances responsiveness vs noise rejection

3. **Why body-frame control?**
   - Inner loop (Bitcraze) expects body-frame velocity commands
   - Easier to saturate (forward/sideways independent)
   - Natural for pilot/operator

### Known Issues
- ⚠️ Webots controller changes not in git (by design - external repo)
- ⚠️ No automated tests yet
- ⚠️ Parameters not experimentally validated

### Questions to Answer
- [ ] How do these gains compare to literature (Lee et al., Mellinger et al.)?
- [ ] What is the maximum wind speed this controller can reject?
- [ ] How does performance degrade with measurement noise?

---

**Last Updated**: 2025-11-06
**Session Duration**: ~2 hours
**Files Modified**: 6 (committed), 1 (local Webots controller)
**Lines Changed**: ~1000+

---

## Quick Start for Next Session

```bash
# 1. Navigate to project
cd /home/hanzel/24774_project/24774_ACSI_F25_ZephyFlyer

# 2. Check git status
git status

# 3. Push if not done
git push origin feature/crazyflie-sim

# 4. Test Webots controller
# Open Webots → Load crazyflie_trajectory.wbt → Press Play
# Test sequence: 1 (hover) → M (auto) → 7 (wind +X) → V (enable) → P (status)

# 5. Activate Python environment for MuJoCo
source setup_env.sh
```

**Ready to continue!** 🚀
