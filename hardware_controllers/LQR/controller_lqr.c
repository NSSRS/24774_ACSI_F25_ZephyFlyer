/**
 * @file controller_lqr.c
 * @brief LQR Hover Controller Implementation for Crazyflie
 * 
 * This module implements a 12-state LQR controller with integral action
 * for hover and position tracking on the Crazyflie 2.1.
 * 
 * @author Converted from Python by Claude
 * @date 2025
 */

#include "controller_lqr.h"
#include <math.h>
#include <string.h>

// ============================================================================
// Private Constants
// ============================================================================

#define PI_F 3.14159265358979323846f
#define SQRT2_INV 0.707106781f  // 1/sqrt(2)

// ============================================================================
// Pre-computed LQR Gain Matrix
// ============================================================================

/**
 * Pre-computed LQR gain matrix K (4x12)
 * 
 * Computed offline using CARE with:
 *   Q = diag([500, 500, 20, 40, 40, 15, 30, 30, 2, 6, 6, 3])
 *   R = diag([800, 1500, 1500, 20000])
 * 
 * State order: [px, py, pz, vx, vy, vz, roll, pitch, yaw, p, q, r]
 * Input order: [thrust, roll_moment, pitch_moment, yaw_moment]
 * 
 * Note: These gains should be recomputed if physical parameters change.
 * Use MATLAB/Python to solve the continuous-time algebraic Riccati equation.
 */
static const float DEFAULT_K[LQR_NU][LQR_NX] = {
    // Row 0: Thrust gains
    //  px        py        pz        vx        vy        vz       roll     pitch      yaw        p          q          r
    { 0.0000f,  0.0000f,  0.0250f,  0.0000f,  0.0000f,  0.0137f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f},
    
    // Row 1: Roll moment gains (controls Y position via roll → ay = -g*roll)
    //  px        py        pz        vx        vy        vz       roll     pitch      yaw        p          q          r
    { 0.0000f, -0.0183f,  0.0000f,  0.0000f, -0.0163f,  0.0000f, -0.0141f,  0.0000f,  0.0000f, -0.0020f,  0.0000f,  0.0000f},
    
    // Row 2: Pitch moment gains (controls X position via pitch → ax = g*pitch)
    //  px        py        pz        vx        vy        vz       roll     pitch      yaw        p          q          r
    { 0.0183f,  0.0000f,  0.0000f,  0.0163f,  0.0000f,  0.0000f,  0.0000f,  0.0141f,  0.0000f,  0.0000f,  0.0020f,  0.0000f},
    
    // Row 3: Yaw moment gains (disabled)
    { 0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0000f,  0.0001f,  0.0000f,  0.0000f,  0.0002f}
};

// ============================================================================
// Private Helper Functions
// ============================================================================

/**
 * @brief Clamp a float value to a range
 */
static inline float clampf(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/**
 * @brief Wrap angle to [-pi, pi]
 */
static inline float wrapAngle(float angle) {
    while (angle > PI_F) angle -= 2.0f * PI_F;
    while (angle < -PI_F) angle += 2.0f * PI_F;
    return angle;
}

/**
 * @brief Matrix-vector multiplication: y = K * x
 * 
 * @param K 4x12 matrix (row-major)
 * @param x 12-element input vector
 * @param y 4-element output vector
 */
static void matVecMult(const float K[LQR_NU][LQR_NX], 
                       const float x[LQR_NX], 
                       float y[LQR_NU]) {
    for (int i = 0; i < LQR_NU; i++) {
        y[i] = 0.0f;
        for (int j = 0; j < LQR_NX; j++) {
            y[i] += K[i][j] * x[j];
        }
    }
}

// ============================================================================
// Public API Implementation
// ============================================================================

void lqrControllerInit(lqr_controller_t* ctrl, float dt, float hover_height) {
    if (ctrl == NULL) return;
    
    // Clear structure
    memset(ctrl, 0, sizeof(lqr_controller_t));
    
    // Physical parameters
    ctrl->mass = LQR_MASS;
    ctrl->gravity = LQR_GRAVITY;
    ctrl->Ixx = LQR_IXX;
    ctrl->Iyy = LQR_IYY;
    ctrl->Izz = LQR_IZZ;
    
    // Timestep
    ctrl->dt = dt;
    
    // Copy default gains
    memcpy(ctrl->K, DEFAULT_K, sizeof(DEFAULT_K));
    
    // Reference state (hover at specified height)
    memset(ctrl->x_ref, 0, sizeof(ctrl->x_ref));
    ctrl->x_ref[2] = hover_height;  // pz
    
    // Velocity filter
    ctrl->alpha_v = 0.4f;
    ctrl->vx_filtered = 0.0f;
    ctrl->vy_filtered = 0.0f;
    ctrl->vz_filtered = 0.0f;
    
    // Integral gains and limits
    ctrl->Ki_z = 0.003f;
    ctrl->Ki_xy = 0.002f;
    ctrl->int_limit_z = 0.05f;
    ctrl->int_limit_xy = 0.05f;
    
    // Input constraints (normalized)
    ctrl->u_min[0] = -0.05f;  // thrust
    ctrl->u_min[1] = -0.02f;  // roll moment
    ctrl->u_min[2] = -0.02f;  // pitch moment
    ctrl->u_min[3] = -0.05f;  // yaw moment
    
    ctrl->u_max[0] = 0.05f;
    ctrl->u_max[1] = 0.02f;
    ctrl->u_max[2] = 0.02f;
    ctrl->u_max[3] = 0.05f;
    
    // Mixer parameters
    ctrl->arm_length = LQR_ARM_LENGTH;
    ctrl->thrust2torque = LQR_THRUST2TORQUE;
    ctrl->motor_to_thrust = LQR_MOTOR_TO_THRUST;
    
    ctrl->initialized = true;
}

void lqrControllerReset(lqr_controller_t* ctrl) {
    if (ctrl == NULL) return;
    
    // Reset state
    memset(ctrl->x, 0, sizeof(ctrl->x));
    
    // Reset velocity filter
    ctrl->vx_filtered = 0.0f;
    ctrl->vy_filtered = 0.0f;
    ctrl->vz_filtered = 0.0f;
    
    // Reset integrators
    ctrl->int_ez = 0.0f;
    ctrl->int_ex = 0.0f;
    ctrl->int_ey = 0.0f;
    
    // Reset previous position
    memset(ctrl->prev_pos, 0, sizeof(ctrl->prev_pos));
    ctrl->prev_time = 0.0f;
    
    // Reset outputs
    memset(ctrl->u, 0, sizeof(ctrl->u));
    memset(ctrl->motors, 0, sizeof(ctrl->motors));
}

void lqrResetIntegrators(lqr_controller_t* ctrl) {
    if (ctrl == NULL) return;
    ctrl->int_ez = 0.0f;
    ctrl->int_ex = 0.0f;
    ctrl->int_ey = 0.0f;
}

void lqrSetTarget(lqr_controller_t* ctrl, float x, float y, float z) {
    if (ctrl == NULL) return;
    
    ctrl->x_ref[0] = x;
    ctrl->x_ref[1] = y;
    ctrl->x_ref[2] = z;
    
    // Reset integrators on target change
    lqrResetIntegrators(ctrl);
}

void lqrSetSetpoint(lqr_controller_t* ctrl, const lqr_setpoint_t* setpoint) {
    if (ctrl == NULL || setpoint == NULL) return;
    
    ctrl->x_ref[0] = setpoint->x;
    ctrl->x_ref[1] = setpoint->y;
    ctrl->x_ref[2] = setpoint->z;
    ctrl->x_ref[8] = setpoint->yaw;
    
    lqrResetIntegrators(ctrl);
}

/**
 * @brief Estimate velocity from position using low-pass filtered differentiation
 */
static void estimateVelocity(lqr_controller_t* ctrl, 
                             const float pos[3], 
                             float timestamp) {
    float dt = timestamp - ctrl->prev_time;
    
    if (dt > 1e-6f && ctrl->prev_time > 0.0f) {
        // Raw velocity estimate
        float vx_raw = (pos[0] - ctrl->prev_pos[0]) / dt;
        float vy_raw = (pos[1] - ctrl->prev_pos[1]) / dt;
        float vz_raw = (pos[2] - ctrl->prev_pos[2]) / dt;
        
        // Low-pass filter
        ctrl->vx_filtered = (1.0f - ctrl->alpha_v) * ctrl->vx_filtered + 
                            ctrl->alpha_v * vx_raw;
        ctrl->vy_filtered = (1.0f - ctrl->alpha_v) * ctrl->vy_filtered + 
                            ctrl->alpha_v * vy_raw;
        ctrl->vz_filtered = (1.0f - ctrl->alpha_v) * ctrl->vz_filtered + 
                            ctrl->alpha_v * vz_raw;
    }
    
    // Update previous values
    ctrl->prev_pos[0] = pos[0];
    ctrl->prev_pos[1] = pos[1];
    ctrl->prev_pos[2] = pos[2];
    ctrl->prev_time = timestamp;
}

/**
 * @brief Construct state vector from sensor data
 */
static void constructState(lqr_controller_t* ctrl, 
                          const lqr_sensor_data_t* sensors) {
    // Position
    ctrl->x[0] = sensors->position[0];  // px
    ctrl->x[1] = sensors->position[1];  // py
    ctrl->x[2] = sensors->position[2];  // pz
    
    // Velocity (filtered)
    ctrl->x[3] = ctrl->vx_filtered;
    ctrl->x[4] = ctrl->vy_filtered;
    ctrl->x[5] = ctrl->vz_filtered;
    
    // Attitude
    ctrl->x[6] = sensors->attitude[0];   // roll
    ctrl->x[7] = sensors->attitude[1];   // pitch
    ctrl->x[8] = sensors->attitude[2];   // yaw
    
    // Angular rates
    ctrl->x[9] = sensors->angular_rate[0];   // p
    ctrl->x[10] = sensors->angular_rate[1];  // q
    ctrl->x[11] = sensors->angular_rate[2];  // r
}

/**
 * @brief Compute LQR control with integral action
 */
static void computeControl(lqr_controller_t* ctrl) {
    // Compute state error: e = x - x_ref
    float e[LQR_NX];
    for (int i = 0; i < LQR_NX; i++) {
        e[i] = ctrl->x[i] - ctrl->x_ref[i];
    }
    
    // Wrap yaw error to [-pi, pi]
    e[8] = wrapAngle(e[8]);
    
    // Update integrators with anti-windup
    ctrl->int_ez += e[2] * ctrl->dt;
    ctrl->int_ex += e[0] * ctrl->dt;
    ctrl->int_ey += e[1] * ctrl->dt;
    
    ctrl->int_ez = clampf(ctrl->int_ez, -ctrl->int_limit_z, ctrl->int_limit_z);
    ctrl->int_ex = clampf(ctrl->int_ex, -ctrl->int_limit_xy, ctrl->int_limit_xy);
    ctrl->int_ey = clampf(ctrl->int_ey, -ctrl->int_limit_xy, ctrl->int_limit_xy);
    
    // LQR feedback: u = -K * e
    matVecMult(ctrl->K, e, ctrl->u);
    for (int i = 0; i < LQR_NU; i++) {
        ctrl->u[i] = -ctrl->u[i];
    }
    
    // Add integral compensation
    ctrl->u[0] += -ctrl->Ki_z * ctrl->int_ez;   // Thrust integral
    ctrl->u[1] += -ctrl->Ki_xy * ctrl->int_ey;  // Roll (controls Y position)
    ctrl->u[2] += -ctrl->Ki_xy * ctrl->int_ex;  // Pitch (controls X position)
    ctrl->u[3] = 0.0f;  // Yaw disabled
    
    // Input constraints
    for (int i = 0; i < LQR_NU; i++) {
        ctrl->u[i] = clampf(ctrl->u[i], ctrl->u_min[i], ctrl->u_max[i]);
    }
}

/**
 * @brief Convert control output to motor commands using Crazyflie mixer
 * 
 * Implements the power_distribution_quadrotor.c mixer from Crazyflie firmware.
 */
static void controlToMotors(lqr_controller_t* ctrl) {
    float u_thrust = ctrl->u[0];
    float u_roll = ctrl->u[1];
    float u_pitch = ctrl->u[2];
    float u_yaw = ctrl->u[3];
    
    // Convert normalized input to physical units
    float hover_thrust = ctrl->mass * ctrl->gravity;
    float total_thrust = hover_thrust + u_thrust * LQR_THRUST_SCALE;
    if (total_thrust < 0.0f) total_thrust = 0.0f;
    
    float tau_x = u_roll * LQR_MOMENT_SCALE;
    float tau_y = u_pitch * LQR_MOMENT_SCALE;
    float tau_z = 0.0f;  // Yaw disabled
    
    // Crazyflie X-configuration mixer
    // Effective arm length for X config: arm / sqrt(2)
    float arm = SQRT2_INV * ctrl->arm_length;
    
    float thrust_part = 0.25f * total_thrust;
    float roll_part = 0.25f / arm * tau_x;
    float pitch_part = 0.25f / arm * tau_y;
    float yaw_part = 0.25f * tau_z / ctrl->thrust2torque;
    
    // Motor forces (N) - Crazyflie X configuration
    // Motor layout:
    //     M2 (CCW)
    //      |
    // M3 --+-- M1 (CW)
    //      |
    //     M4 (CW)
    float F1 = thrust_part - roll_part - pitch_part - yaw_part;  // Front-right
    float F2 = thrust_part - roll_part + pitch_part + yaw_part;  // Front-left
    float F3 = thrust_part + roll_part + pitch_part - yaw_part;  // Rear-right
    float F4 = thrust_part + roll_part - pitch_part + yaw_part;  // Rear-left
    
    // Convert force to motor command and clamp
    ctrl->motors[0] = clampf(F1 / ctrl->motor_to_thrust, 0.0f, 600.0f);
    ctrl->motors[1] = clampf(F2 / ctrl->motor_to_thrust, 0.0f, 600.0f);
    ctrl->motors[2] = clampf(F3 / ctrl->motor_to_thrust, 0.0f, 600.0f);
    ctrl->motors[3] = clampf(F4 / ctrl->motor_to_thrust, 0.0f, 600.0f);
}

bool lqrControllerUpdate(lqr_controller_t* ctrl, 
                         const lqr_sensor_data_t* sensors,
                         lqr_motor_output_t* output) {
    if (ctrl == NULL || sensors == NULL || output == NULL) {
        return false;
    }
    
    if (!ctrl->initialized) {
        return false;
    }
    
    // Step 1: Estimate velocity from position
    estimateVelocity(ctrl, sensors->position, sensors->timestamp);
    
    // Step 2: Construct state vector
    constructState(ctrl, sensors);
    
    // Step 3: Compute LQR control
    computeControl(ctrl);
    
    // Step 4: Convert to motor commands
    controlToMotors(ctrl);
    
    // Step 5: Copy to output
    output->m1 = ctrl->motors[0];
    output->m2 = ctrl->motors[1];
    output->m3 = ctrl->motors[2];
    output->m4 = ctrl->motors[3];
    
    return true;
}

void lqrGetState(const lqr_controller_t* ctrl, float* state) {
    if (ctrl == NULL || state == NULL) return;
    memcpy(state, ctrl->x, sizeof(ctrl->x));
}

void lqrGetControlOutput(const lqr_controller_t* ctrl, float* u) {
    if (ctrl == NULL || u == NULL) return;
    memcpy(u, ctrl->u, sizeof(ctrl->u));
}

float lqrGetPositionError(const lqr_controller_t* ctrl) {
    if (ctrl == NULL) return 0.0f;
    
    float ex = ctrl->x[0] - ctrl->x_ref[0];
    float ey = ctrl->x[1] - ctrl->x_ref[1];
    float ez = ctrl->x[2] - ctrl->x_ref[2];
    
    return sqrtf(ex*ex + ey*ey + ez*ez);
}

bool lqrIsInitialized(const lqr_controller_t* ctrl) {
    return (ctrl != NULL) && ctrl->initialized;
}

void lqrSetGains(lqr_controller_t* ctrl, const float K[LQR_NU][LQR_NX]) {
    if (ctrl == NULL || K == NULL) return;
    memcpy(ctrl->K, K, sizeof(ctrl->K));
}

void lqrSetIntegralGains(lqr_controller_t* ctrl, float Ki_z, float Ki_xy) {
    if (ctrl == NULL) return;
    ctrl->Ki_z = Ki_z;
    ctrl->Ki_xy = Ki_xy;
}

void lqrSetInputConstraints(lqr_controller_t* ctrl, 
                           const float u_min[LQR_NU], 
                           const float u_max[LQR_NU]) {
    if (ctrl == NULL) return;
    if (u_min != NULL) memcpy(ctrl->u_min, u_min, sizeof(ctrl->u_min));
    if (u_max != NULL) memcpy(ctrl->u_max, u_max, sizeof(ctrl->u_max));
}

void lqrSetVelocityFilter(lqr_controller_t* ctrl, float alpha) {
    if (ctrl == NULL) return;
    ctrl->alpha_v = clampf(alpha, 0.0f, 1.0f);
}
