/**
 * @file controller_lqr.h
 * @brief LQR Hover Controller for Crazyflie
 * 
 * 12-State LQR controller with integral action for position hold.
 * Compatible with Crazyflie firmware architecture.
 * 
 * State vector (12 states):
 *   x = [px, py, pz, vx, vy, vz, roll, pitch, yaw, p, q, r]
 * 
 * Control input (4 inputs):
 *   u = [thrust_delta, roll_moment, pitch_moment, yaw_moment]
 * 
 * @author Converted from Python by Claude
 * @date 2025
 */

#ifndef __CONTROLLER_LQR_H__
#define __CONTROLLER_LQR_H__

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Configuration
// ============================================================================

/** Number of states in the state vector */
#define LQR_NX 12

/** Number of control inputs */
#define LQR_NU 4

/** Default control loop frequency (Hz) */
#define LQR_DEFAULT_FREQ 500

// ============================================================================
// Physical Constants (Crazyflie 2.1)
// ============================================================================

/** Mass in kg */
#define LQR_MASS 0.027f

/** Gravity in m/s^2 */
#define LQR_GRAVITY 9.81f

/** Moment of inertia about X axis (kg·m²) */
#define LQR_IXX 1.6e-5f

/** Moment of inertia about Y axis (kg·m²) */
#define LQR_IYY 1.6e-5f

/** Moment of inertia about Z axis (kg·m²) */
#define LQR_IZZ 2.9e-5f

/** Arm length in m (motor to center) */
#define LQR_ARM_LENGTH 0.046f

/** Thrust to torque ratio (k_m / k_f) */
#define LQR_THRUST2TORQUE 0.005022f

/** Motor command to thrust conversion (N per unit) */
#define LQR_MOTOR_TO_THRUST 1.63e-3f

// ============================================================================
// Scaling Constants
// ============================================================================

/** Thrust scaling: normalized input → N */
#define LQR_THRUST_SCALE 1.0f

/** Moment scaling: normalized input → N·m */
#define LQR_MOMENT_SCALE 2.0e-4f

// ============================================================================
// Data Structures
// ============================================================================

/**
 * @brief LQR Controller State
 */
typedef struct {
    // Physical parameters
    float mass;
    float gravity;
    float Ixx, Iyy, Izz;
    
    // Control timestep
    float dt;
    
    // LQR gain matrix K (4x12)
    float K[LQR_NU][LQR_NX];
    
    // Reference state
    float x_ref[LQR_NX];
    
    // Current state estimate
    float x[LQR_NX];
    
    // Velocity filter state
    float vx_filtered;
    float vy_filtered;
    float vz_filtered;
    float alpha_v;  // Low-pass filter coefficient
    
    // Integral terms (anti-windup)
    float int_ez;  // Z position integral
    float int_ex;  // X position integral
    float int_ey;  // Y position integral
    float Ki_z;    // Z integral gain
    float Ki_xy;   // XY integral gain
    float int_limit_z;   // Z integral limit
    float int_limit_xy;  // XY integral limit
    
    // Input constraints
    float u_min[LQR_NU];
    float u_max[LQR_NU];
    
    // Mixer parameters
    float arm_length;
    float thrust2torque;
    float motor_to_thrust;
    
    // Last control output
    float u[LQR_NU];
    
    // Last motor commands
    float motors[4];
    
    // Previous position (for velocity estimation)
    float prev_pos[3];
    float prev_time;
    
    // Initialization flag
    bool initialized;
    
} lqr_controller_t;

/**
 * @brief Sensor input structure
 */
typedef struct {
    float position[3];      // [x, y, z] in m
    float attitude[3];      // [roll, pitch, yaw] in rad
    float angular_rate[3];  // [p, q, r] in rad/s
    float timestamp;        // Current time in seconds
} lqr_sensor_data_t;

/**
 * @brief Motor output structure
 */
typedef struct {
    float m1;  // Front-right motor
    float m2;  // Front-left motor
    float m3;  // Rear-right motor
    float m4;  // Rear-left motor
} lqr_motor_output_t;

/**
 * @brief Setpoint structure
 */
typedef struct {
    float x;
    float y;
    float z;
    float yaw;  // Optional yaw reference (rad)
} lqr_setpoint_t;

// ============================================================================
// API Functions
// ============================================================================

/**
 * @brief Initialize the LQR controller
 * 
 * @param ctrl Pointer to controller structure
 * @param dt Control timestep in seconds
 * @param hover_height Default hover height in meters
 */
void lqrControllerInit(lqr_controller_t* ctrl, float dt, float hover_height);

/**
 * @brief Reset controller state and integrators
 * 
 * @param ctrl Pointer to controller structure
 */
void lqrControllerReset(lqr_controller_t* ctrl);

/**
 * @brief Reset only the integral terms
 * 
 * @param ctrl Pointer to controller structure
 */
void lqrResetIntegrators(lqr_controller_t* ctrl);

/**
 * @brief Set the target hover position
 * 
 * @param ctrl Pointer to controller structure
 * @param x Target X position (m)
 * @param y Target Y position (m)
 * @param z Target Z position (m)
 */
void lqrSetTarget(lqr_controller_t* ctrl, float x, float y, float z);

/**
 * @brief Set the target position with yaw
 * 
 * @param ctrl Pointer to controller structure
 * @param setpoint Pointer to setpoint structure
 */
void lqrSetSetpoint(lqr_controller_t* ctrl, const lqr_setpoint_t* setpoint);

/**
 * @brief Update controller with new sensor data and compute motor commands
 * 
 * @param ctrl Pointer to controller structure
 * @param sensors Pointer to sensor data
 * @param output Pointer to motor output structure
 * @return true if control update successful
 */
bool lqrControllerUpdate(lqr_controller_t* ctrl, 
                         const lqr_sensor_data_t* sensors,
                         lqr_motor_output_t* output);

/**
 * @brief Get the current state estimate
 * 
 * @param ctrl Pointer to controller structure
 * @param state Output array (must be at least LQR_NX elements)
 */
void lqrGetState(const lqr_controller_t* ctrl, float* state);

/**
 * @brief Get the last control output
 * 
 * @param ctrl Pointer to controller structure
 * @param u Output array (must be at least LQR_NU elements)
 */
void lqrGetControlOutput(const lqr_controller_t* ctrl, float* u);

/**
 * @brief Get the current position error magnitude
 * 
 * @param ctrl Pointer to controller structure
 * @return Position error in meters
 */
float lqrGetPositionError(const lqr_controller_t* ctrl);

/**
 * @brief Check if controller is initialized
 * 
 * @param ctrl Pointer to controller structure
 * @return true if initialized
 */
bool lqrIsInitialized(const lqr_controller_t* ctrl);

// ============================================================================
// Advanced Functions (for tuning/debugging)
// ============================================================================

/**
 * @brief Set custom LQR gain matrix
 * 
 * @param ctrl Pointer to controller structure
 * @param K Pointer to 4x12 gain matrix (row-major)
 */
void lqrSetGains(lqr_controller_t* ctrl, const float K[LQR_NU][LQR_NX]);

/**
 * @brief Set integral gains
 * 
 * @param ctrl Pointer to controller structure
 * @param Ki_z Z position integral gain
 * @param Ki_xy XY position integral gain
 */
void lqrSetIntegralGains(lqr_controller_t* ctrl, float Ki_z, float Ki_xy);

/**
 * @brief Set input constraints
 * 
 * @param ctrl Pointer to controller structure
 * @param u_min Minimum input array (4 elements)
 * @param u_max Maximum input array (4 elements)
 */
void lqrSetInputConstraints(lqr_controller_t* ctrl, 
                           const float u_min[LQR_NU], 
                           const float u_max[LQR_NU]);

/**
 * @brief Set velocity filter coefficient
 * 
 * @param ctrl Pointer to controller structure
 * @param alpha Filter coefficient (0-1, higher = less filtering)
 */
void lqrSetVelocityFilter(lqr_controller_t* ctrl, float alpha);

#endif // __CONTROLLER_LQR_H__
