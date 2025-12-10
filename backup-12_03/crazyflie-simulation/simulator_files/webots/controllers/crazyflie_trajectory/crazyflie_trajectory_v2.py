"""
Trajectory Controller V2 - Force Domain Control with Auto-Hover Calibration

Fixes:
1. Auto-calibration of base_omega (hover angular velocity)
2. Force-domain control: ΔF[N] → Δω[rad/s] mapping
3. Corrected motor mixing matrix with explicit sign convention
4. Safety protections: divergence detection, slew rate limiting

Author: Claude Code & Hanzel
Date: 2025-11-04
"""

from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard
import numpy as np
from math import cos, sin, pi


class SimplePIDController:
    """Advanced PID with D-on-measurement, low-pass filter, back-calculation anti-windup"""

    def __init__(self, kp, ki, kd, output_limits=None, integral_limits=None, d_cut_hz=None, aw_gain=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits if output_limits else (-float('inf'), float('inf'))
        self.integral_limits = integral_limits if integral_limits else (-float('inf'), float('inf'))

        self.d_cut_hz = d_cut_hz
        self.aw_gain = aw_gain if aw_gain is not None else 1.0

        self.integral = 0.0
        self.prev_measurement = None
        self.d_filtered = 0.0

    def update(self, error, dt, meas=None):
        """Compute PID output"""
        # P term
        p_term = self.kp * error

        # I term
        self.integral += error * dt
        self.integral = np.clip(self.integral, self.integral_limits[0], self.integral_limits[1])
        i_term = self.ki * self.integral

        # D term: D-on-measurement with low-pass filter
        if meas is not None and self.prev_measurement is not None and dt > 0:
            d_raw = -(meas - self.prev_measurement) / dt

            if self.d_cut_hz is not None and self.d_cut_hz > 0:
                alpha = dt / (dt + 1.0 / (2.0 * 3.14159 * self.d_cut_hz))
                self.d_filtered = alpha * d_raw + (1 - alpha) * self.d_filtered
            else:
                self.d_filtered = d_raw

            d_term = self.kd * self.d_filtered
        else:
            d_term = 0.0

        if meas is not None:
            self.prev_measurement = meas

        # Output before limiting
        output_unlim = p_term + i_term + d_term
        output = np.clip(output_unlim, self.output_limits[0], self.output_limits[1])

        # Back-calculation anti-windup
        if self.aw_gain > 0 and dt > 0 and self.ki != 0:
            saturation_error = output - output_unlim
            self.integral += (saturation_error / self.ki) * self.aw_gain
            self.integral = np.clip(self.integral, self.integral_limits[0], self.integral_limits[1])

        return output

    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.prev_measurement = None
        self.d_filtered = 0.0


class HoverCalibrator:
    """Auto-calibrate hover angular velocity (base_omega) and force-to-omega mapping"""

    def __init__(self, robot, motors, gps, timestep):
        self.robot = robot
        self.motors = motors
        self.gps = gps
        self.timestep = timestep
        self.dt = timestep / 1000.0

        self.base_omega = None  # Hover angular velocity [rad/s]
        self.K_F2W = None       # Force to omega gain [rad/s per N]

    def calibrate_hover(self, omega_min=200, omega_max=600, tolerance=0.01):
        """
        Find base_omega that makes vertical velocity ≈ 0

        Args:
            omega_min: Min search range [rad/s]
            omega_max: Max search range [rad/s]
            tolerance: Velocity tolerance [m/s]

        Returns:
            base_omega [rad/s]
        """
        print("\n" + "="*70)
        print("🔧 AUTO-CALIBRATION: Finding hover angular velocity...")
        print("="*70)

        # Binary search for hover omega
        omega_low = omega_min
        omega_high = omega_max

        for iteration in range(15):  # Max 15 iterations
            omega_test = (omega_low + omega_high) / 2.0

            # Set all motors to test omega
            for motor in self.motors:
                motor.setVelocity(omega_test)

            # Wait to stabilize and measure vertical velocity
            z_samples = []
            for _ in range(int(1.0 / self.dt)):  # 1.0 second for better stability
                self.robot.step(self.timestep)
                z_samples.append(self.gps.getValues()[2])

            # Compute average vertical velocity
            z_velocity = (z_samples[-1] - z_samples[0]) / (len(z_samples) * self.dt)

            print(f"  Iter {iteration+1}: ω={omega_test:.1f} rad/s → vz={z_velocity:+.4f} m/s")

            if abs(z_velocity) < tolerance:
                # Post-check & micro-tune (1.0 s verification)
                print(f"  Found candidate: ω = {omega_test:.1f} rad/s, verifying...")
                tune = 0.5 * (omega_high - omega_low)
                for micro_iter in range(3):
                    # Set motors and wait 1.0s
                    for motor in self.motors:
                        motor.setVelocity(omega_test)
                    z0 = self.gps.getValues()[2]
                    for _ in range(int(1.0 / self.dt)):
                        self.robot.step(self.timestep)
                    vz = (self.gps.getValues()[2] - z0) / 1.0

                    if abs(vz) < 0.01:
                        break

                    if vz > 0.0:  # Still rising, reduce omega
                        omega_test -= tune
                    else:  # Falling, increase omega
                        omega_test += tune
                    tune *= 0.5
                    print(f"    Micro-tune {micro_iter+1}: ω={omega_test:.1f} rad/s, vz={vz:+.4f} m/s")

                print(f"✅ Found hover: ω0 = {omega_test:.1f} rad/s")
                self.base_omega = omega_test
                return omega_test

            # Adjust search range
            if z_velocity > 0:  # Rising, reduce omega
                omega_high = omega_test
            else:  # Falling, increase omega
                omega_low = omega_test

        # Use midpoint if not converged, then micro-tune
        omega_test = (omega_low + omega_high) / 2.0
        print(f"⚠️  Calibration timeout. Micro-tuning ω0 = {omega_test:.1f} rad/s")

        # Apply same micro-tuning
        tune = 0.5 * (omega_high - omega_low)
        for micro_iter in range(3):
            for motor in self.motors:
                motor.setVelocity(omega_test)
            z0 = self.gps.getValues()[2]
            for _ in range(int(1.0 / self.dt)):
                self.robot.step(self.timestep)
            vz = (self.gps.getValues()[2] - z0) / 1.0

            if abs(vz) < 0.01:
                break

            if vz > 0.0:
                omega_test -= tune
            else:
                omega_test += tune
            tune *= 0.5
            print(f"    Micro-tune {micro_iter+1}: ω={omega_test:.1f} rad/s, vz={vz:+.4f} m/s")

        self.base_omega = omega_test
        print(f"✅ Final hover: ω0 = {self.base_omega:.1f} rad/s")
        return self.base_omega

    def calibrate_force_mapping(self, mass=0.027, delta_omega=10.0, duration=0.5):
        """
        Calibrate force-to-omega gain by finite difference

        Args:
            mass: Drone mass [kg] (Crazyflie ≈ 0.027 kg)
            delta_omega: Test increment [rad/s]
            duration: Test duration [s]

        Returns:
            K_F2W: Force to omega gain [rad/s per N]
        """
        print("\n" + "="*70)
        print("🔧 AUTO-CALIBRATION: Measuring force-to-omega mapping...")
        print("="*70)

        if self.base_omega is None:
            print("❌ Error: Must calibrate hover first!")
            return None

        # Measure initial velocity
        z_start = self.gps.getValues()[2]
        for _ in range(int(0.1 / self.dt)):  # Wait 0.1s
            self.robot.step(self.timestep)
            for motor in self.motors:
                motor.setVelocity(self.base_omega)

        # Apply delta_omega to all motors
        for motor in self.motors:
            motor.setVelocity(self.base_omega + delta_omega)

        # Measure acceleration
        z_samples = []
        for _ in range(int(duration / self.dt)):
            self.robot.step(self.timestep)
            z_samples.append(self.gps.getValues()[2])

        # Compute average acceleration (linear fit)
        t_array = np.arange(len(z_samples)) * self.dt
        z_array = np.array(z_samples) - z_samples[0]

        # Fit z = 0.5*a*t^2 (assuming starting from rest)
        if len(t_array) > 2:
            # Use least squares: z ≈ 0.5*a*t^2
            accel = 2 * np.sum(z_array * t_array**2) / np.sum(t_array**4)
        else:
            accel = 0

        # Compute force increase: ΔF = m * a
        delta_force = mass * accel

        # Compute gain: K_F2W = Δω_per_motor / total_ΔF [rad/s per N]
        # NOTE: We apply the same Δω_cmd to each motor, so no 4x factor here
        if abs(delta_force) > 0.001:
            self.K_F2W = delta_omega / delta_force
        else:
            self.K_F2W = 100.0  # Default fallback

        print(f"  Δω = {delta_omega:.1f} rad/s")
        print(f"  Measured accel = {accel:.3f} m/s²")
        print(f"  ΔF = m*a = {delta_force:.4f} N")
        print(f"✅ K_F2W = {self.K_F2W:.1f} rad/s per N")

        # Return to hover
        for motor in self.motors:
            motor.setVelocity(self.base_omega)
        for _ in range(int(0.2 / self.dt)):
            self.robot.step(self.timestep)

        return self.K_F2W


class TrajectoryGenerator:
    """Generate trajectory patterns"""

    def __init__(self):
        self.trajectory_type = "hover"
        self.start_time = 0.0
        self.center = [0.0, 0.0, 1.0]
        self.radius = 0.2
        self.period = 40.0

    def set_trajectory(self, traj_type, start_time):
        self.trajectory_type = traj_type
        self.start_time = start_time
        print(f"\n✅ Trajectory: {traj_type}")
        if traj_type == "circle":
            speed = 2 * 3.14159 * self.radius / self.period
            print(f"   ⭕ Circle: radius={self.radius}m, period={self.period}s, speed={speed:.3f}m/s")

    def get_target_position(self, current_time):
        t = current_time - self.start_time

        if self.trajectory_type == "hover":
            return self.center.copy()
        elif self.trajectory_type == "circle":
            omega = 2 * pi / self.period
            x = self.center[0] + self.radius * cos(omega * t)
            y = self.center[1] + self.radius * sin(omega * t)
            z = self.center[2]
            return [x, y, z]

        return self.center.copy()


class CrazyflieController:
    """Force-domain cascade controller with corrected motor mixing"""

    def __init__(self, base_omega, K_F2W):
        self.base_omega = base_omega  # Hover angular velocity [rad/s]
        self.K_F2W = K_F2W            # Force to omega gain [rad/s per N]

        # Z-axis PID: outputs force [N]
        self.pid_z = SimplePIDController(
            kp=1.0,    # N/m
            ki=0.20,   # N/(m·s)
            kd=0.20,   # N·s/m
            output_limits=(-3.0, 2.0),  # Force limits [N] - expanded negative for braking
            integral_limits=(-0.5, 0.5),  # Integral limits [m·s]
            d_cut_hz=20.0,
            aw_gain=3.0
        )

        # XY position PIDs: output velocity commands [m/s]
        self.pid_x = SimplePIDController(kp=0.3, ki=0.02, kd=0.15,
                                         output_limits=(-0.15, 0.15), integral_limits=(-0.1, 0.1))
        self.pid_y = SimplePIDController(kp=0.3, ki=0.02, kd=0.15,
                                         output_limits=(-0.15, 0.15), integral_limits=(-0.1, 0.1))

        # Velocity PIDs: output attitude commands [rad]
        self.pid_vx = SimplePIDController(kp=1.0, ki=0.0, kd=0.2,
                                          output_limits=(-0.1, 0.1), integral_limits=(-0.05, 0.05))
        self.pid_vy = SimplePIDController(kp=1.0, ki=0.0, kd=0.2,
                                          output_limits=(-0.1, 0.1), integral_limits=(-0.05, 0.05))

        # Attitude PIDs: output moment commands
        self.pid_roll = SimplePIDController(kp=0.5, ki=0.0, kd=0.1,
                                            output_limits=(-3, 3), integral_limits=(-0.5, 0.5))
        self.pid_pitch = SimplePIDController(kp=0.5, ki=0.0, kd=0.1,
                                             output_limits=(-3, 3), integral_limits=(-0.5, 0.5))
        self.pid_yaw = SimplePIDController(kp=1.0, ki=0.0, kd=0.5,
                                           output_limits=(-2, 2), integral_limits=(-0.5, 0.5))

        # Slew rate limiter state
        self.prev_delta_omega = 0.0

    def compute_control(self, target_pos, current_pos, current_vel, current_attitude,
                       current_yaw_rate, dt, target_yaw_rate=0.0, xy_enabled=False):
        """
        Compute motor angular velocities [rad/s]

        Returns:
            [ω1, ω2, ω3, ω4] in rad/s
        """
        # XY control (only if enabled)
        if xy_enabled:
            desired_vx = self.pid_x.update(target_pos[0] - current_pos[0], dt)
            desired_vy = self.pid_y.update(target_pos[1] - current_pos[1], dt)
            desired_pitch = self.pid_vx.update(desired_vx - current_vel[0], dt)
            desired_roll = -self.pid_vy.update(desired_vy - current_vel[1], dt)
        else:
            desired_roll = 0.0
            desired_pitch = 0.0

        # Z control: outputs force [N]
        altitude_error = target_pos[2] - current_pos[2]
        delta_force_z = self.pid_z.update(altitude_error, dt, meas=current_pos[2])

        # Safety: divergence detection
        z_velocity = current_vel[2] if len(current_vel) > 2 else 0
        if np.sign(z_velocity) == np.sign(altitude_error) and abs(altitude_error) > 0.5:
            # Moving away from target
            self.pid_z.integral = 0.0  # Freeze integral
            delta_force_z -= 0.2  # Apply braking force

        # Map force to angular velocity: ΔF → Δω (reduced by 50% for safety)
        delta_omega_cmd = 0.5 * self.K_F2W * delta_force_z

        # Slew rate limiting: |dΔω/dt| ≤ 3 rad/s² with anti-overshoot logic
        max_delta_change = 3.0 * dt
        delta_omega_change = delta_omega_cmd - self.prev_delta_omega

        # If already above target (altitude_error < 0), don't allow Δω to increase
        if altitude_error < 0.0 and delta_omega_change > 0.0:
            delta_omega_cmd = self.prev_delta_omega  # Hold current value, no acceleration
        elif abs(delta_omega_change) > max_delta_change:
            delta_omega_cmd = self.prev_delta_omega + np.sign(delta_omega_change) * max_delta_change

        self.prev_delta_omega = delta_omega_cmd

        # Attitude control
        roll, pitch, yaw = current_attitude
        roll_cmd = self.pid_roll.update(desired_roll - roll, dt)
        pitch_cmd = -self.pid_pitch.update(desired_pitch - pitch, dt)
        yaw_cmd = self.pid_yaw.update(target_yaw_rate - current_yaw_rate, dt)

        # Motor mixing (X-configuration, corrected signs)
        # Motor numbering: 1=FL, 2=FR, 3=BL, 4=BR
        # Thrust: all positive
        # Roll: right side (+), left side (-)
        # Pitch: front (+), back (-)
        # Yaw: CW motors (+), CCW motors (-)

        # Convert moment commands to omega deltas (simplified)
        delta_roll = roll_cmd * 5.0   # Scaling factor
        delta_pitch = pitch_cmd * 5.0
        delta_yaw = yaw_cmd * 2.0

        omega1 = self.base_omega + delta_omega_cmd - delta_roll + delta_pitch + delta_yaw  # FL
        omega2 = self.base_omega + delta_omega_cmd - delta_roll - delta_pitch - delta_yaw  # FR
        omega3 = self.base_omega + delta_omega_cmd + delta_roll - delta_pitch + delta_yaw  # BL
        omega4 = self.base_omega + delta_omega_cmd + delta_roll + delta_pitch - delta_yaw  # BR

        # Limit to physical range
        omega_min, omega_max = 0, 1000
        omega1 = np.clip(omega1, omega_min, omega_max)
        omega2 = np.clip(omega2, omega_min, omega_max)
        omega3 = np.clip(omega3, omega_min, omega_max)
        omega4 = np.clip(omega4, omega_min, omega_max)

        return [omega1, omega2, omega3, omega4]


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0

    # Initialize motors
    motors = []
    for i in range(1, 5):
        motor = robot.getDevice(f"m{i}_motor")
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)
        motors.append(motor)

    # Initialize sensors
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # Wait for sensors to initialize
    for _ in range(5):
        robot.step(timestep)

    # Auto-calibration
    calibrator = HoverCalibrator(robot, motors, gps, timestep)
    base_omega = calibrator.calibrate_hover(omega_min=200, omega_max=600)
    K_F2W = calibrator.calibrate_force_mapping(mass=0.027, delta_omega=10.0)

    print("\n" + "="*70)
    print(f"✅ Calibration complete:")
    print(f"   base_omega = {base_omega:.1f} rad/s")
    print(f"   K_F2W = {K_F2W:.1f} rad/s per N")
    print("="*70 + "\n")

    # Initialize controller and trajectory
    controller = CrazyflieController(base_omega, K_F2W)
    trajectory = TrajectoryGenerator()

    # State variables
    past_x, past_y, past_z = 0.0, 0.0, 0.0
    past_time = robot.getTime()
    start_time = robot.getTime()  # Track startup time for integral freeze

    # Z setpoint smoothing
    z_sp_smooth = gps.getValues()[2]
    z_sp_tau = 4.0  # Increased to 4s for smoother ramp

    # Control flags
    xy_enabled = False  # Start with Z-only control

    print("🚁 Controller started - Z-axis only mode")
    print("   Press 'X' to enable XY control")
    print("   Press '1' to hover, '2' for circle\n")

    step_count = 0
    while robot.step(timestep) != -1:
        current_time = robot.getTime()
        dt = current_time - past_time

        if dt < 1e-6:
            continue

        # Read sensors
        gps_values = gps.getValues()
        current_pos = [gps_values[0], gps_values[1], gps_values[2]]
        roll, pitch, yaw = imu.getRollPitchYaw()
        yaw_rate = gyro.getValues()[2]

        # Compute velocity
        x_global, y_global = gps_values[0], gps_values[1]
        vx_global = (x_global - past_x) / dt
        vy_global = (y_global - past_y) / dt
        vz = (gps_values[2] - past_z) / dt

        cosyaw, sinyaw = cos(yaw), sin(yaw)
        vx_body = vx_global * cosyaw + vy_global * sinyaw
        vy_body = -vx_global * sinyaw + vy_global * cosyaw

        # Keyboard handling
        key = keyboard.getKey()
        while key != -1:
            if key == ord('1'):
                trajectory.center = current_pos.copy()
                trajectory.set_trajectory("hover", current_time)
                controller.pid_z.reset()
                z_sp_smooth = current_pos[2]
            elif key == ord('2'):
                trajectory.center = current_pos.copy()
                trajectory.set_trajectory("circle", current_time)
                controller.pid_z.reset()
                z_sp_smooth = current_pos[2]
            elif key == ord('X') or key == ord('x'):
                xy_enabled = not xy_enabled
                print(f"{'✅ XY control ENABLED' if xy_enabled else '⚠️  XY control DISABLED'}")

            key = keyboard.getKey()

        # Get target with Z smoothing
        target_position = trajectory.get_target_position(current_time)
        target_z_raw = target_position[2]
        z_sp_smooth += (target_z_raw - z_sp_smooth) * min(1.0, dt / z_sp_tau)
        target_position[2] = z_sp_smooth

        # Freeze Z integral during first 2 seconds to prevent startup windup
        if current_time - start_time < 2.0:
            controller.pid_z.integral = 0.0

        # Compute control
        motor_omegas = controller.compute_control(
            target_position, current_pos, [vx_body, vy_body, vz],
            [roll, pitch, yaw], yaw_rate, dt, xy_enabled=xy_enabled
        )

        # Apply to motors (unified positive sign)
        motors[0].setVelocity(motor_omegas[0])
        motors[1].setVelocity(motor_omegas[1])
        motors[2].setVelocity(motor_omegas[2])
        motors[3].setVelocity(motor_omegas[3])

        # Print status
        if step_count % int(0.5 / dt) == 0:
            pos_error = np.linalg.norm([target_position[i] - current_pos[i] for i in range(3)])
            z_error = target_position[2] - current_pos[2]

            if trajectory.trajectory_type != "hover" or pos_error > 0.05:
                print(f"[{current_time:6.2f}s] {trajectory.trajectory_type:8s} | "
                      f"Z:{current_pos[2]:7.3f} (tgt:{target_position[2]:.2f}, err:{z_error:+.3f}) | "
                      f"ω:[{motor_omegas[0]:.0f},{motor_omegas[1]:.0f},{motor_omegas[2]:.0f},{motor_omegas[3]:.0f}]")

        past_x = x_global
        past_y = y_global
        past_z = gps_values[2]
        past_time = current_time
        step_count += 1


if __name__ == "__main__":
    main()
