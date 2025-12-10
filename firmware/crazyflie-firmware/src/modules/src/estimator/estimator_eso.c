#include "estimator_eso.h"
#include "math.h"
#include "log.h"
#include <stdint.h>
#include "static_mem.h"   // if not already included

// Global float outputs (API stays the same)
eso_t esoOutput;

// ESO log group with INT32, small and consistent with motor group style
eso_log_int_t esoLogInt;

// ========================
// Config / constants
// ========================

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG2RAD ( (float)M_PI / 180.0f )

// Sample time (stabilizer loop ~1kHz)
static const float ESO_TS   = 0.001f;   // [s]
static const float ESO_MASS = 0.027f;   // [kg]
static const float ESO_G    = 9.81f;    // [m/s^2]

// Pitch clamp for Euler kinematics (same as Python)
static const float ESO_PITCH_LIMIT = (float)M_PI / 2.0f - 0.2f;

// ========================
// Observer internal state
// ========================

// z = [ x,y,z, vx,vy,vz, roll,pitch,yaw, dx,dy,dz ]
static float eso_z[12];

// L: 12x6 observer gain matrix (discrete-time), computed from Python design_L.py
// Max gain: 0.022360, Stability: max eigenvalue = -0.57
static float ESO_L[12][6] = {
    {   0.012198f,    0.000000f,   -0.000001f,   -0.000000f,    0.000279f,    0.000004f},
    {   0.000000f,    0.012198f,    0.000001f,   -0.000279f,   -0.000000f,    0.000004f},
    {  -0.000001f,    0.000001f,    0.016624f,   -0.000002f,   -0.000002f,   -0.000000f},
    {   0.014407f,    0.000000f,   -0.000011f,   -0.000001f,    0.009627f,    0.000094f},
    {   0.000000f,    0.014407f,    0.000011f,   -0.009628f,   -0.000000f,    0.000094f},
    {  -0.000014f,    0.000014f,    0.018174f,   -0.000096f,   -0.000096f,   -0.000000f},
    {  -0.000000f,   -0.000111f,   -0.000001f,    0.022360f,    0.000000f,    0.000000f},
    {   0.000111f,   -0.000000f,   -0.000001f,    0.000000f,    0.022360f,   -0.000000f},
    {   0.000001f,    0.000001f,   -0.000000f,    0.000000f,   -0.000000f,    0.014142f},
    {   0.004472f,   -0.000000f,   -0.000000f,    0.000000f,   -0.000056f,   -0.000001f},
    {  -0.000000f,    0.004472f,    0.000000f,    0.000056f,    0.000000f,   -0.000001f},
    {   0.000000f,   -0.000000f,    0.006325f,    0.000001f,    0.000001f,    0.000000f}
};

static bool esoInitialized = false;

// ========================
// Small helpers
// ========================

static float clampf(float x, float minVal, float maxVal)
{
  if (x < minVal) return minVal;
  if (x > maxVal) return maxVal;
  return x;
}

// Rotation matrix body->world (ZYX Euler convention)
static void esoRotationMatrix(float roll, float pitch, float yaw, float R[3][3])
{
  float cr = cosf(roll);
  float sr = sinf(roll);
  float cp = cosf(pitch);
  float sp = sinf(pitch);
  float cy = cosf(yaw);
  float sy = sinf(yaw);

  R[0][0] = cp * cy;
  R[0][1] = sr*sp*cy - cr*sy;
  R[0][2] = cr*sp*cy + sr*sy;

  R[1][0] = cp * sy;
  R[1][1] = sr*sp*sy + cr*cy;
  R[1][2] = cr*sp*sy - sr*cy;

  R[2][0] = -sp;
  R[2][1] = sr * cp;
  R[2][2] = cr * cp;
}

// Euler kinematics matrix: body rates -> Euler rate
static void esoEulerKinematics(float roll, float pitch, float yaw, float E[3][3])
{
  (void)yaw; // not used explicitly in formula

  float cr = cosf(roll);
  float sr = sinf(roll);

  // Clamp pitch to avoid singularity at ±90°
  float pitchSafe = clampf(pitch, -ESO_PITCH_LIMIT, ESO_PITCH_LIMIT);
  float cp = cosf(pitchSafe);
  float sp = sinf(pitchSafe);
  float tp = sp / cp; // tan(pitchSafe)

  E[0][0] = 1.0f;  E[0][1] = sr * tp;   E[0][2] = cr * tp;
  E[1][0] = 0.0f;  E[1][1] = cr;        E[1][2] = -sr;
  E[2][0] = 0.0f;  E[2][1] = sr / cp;   E[2][2] = cr / cp;
}

// f_continuous(z, u, omega_body): dz/dt = f(z, u, omega)
// Inputs:
//   z[12]         : state
//   u_thrust      : scalar thrust [N]
//   omega_body[3] : gyro [rad/s]
// Output:
//   dzdt[12]
static void esoFContinuous(const float z[12],
                           float u_thrust,
                           const float omega_body[3],
                           float dzdt[12])
{
  float roll  = z[6];
  float pitch = z[7];
  float yaw   = z[8];

  // Position dynamics: dp = v
  dzdt[0] = z[3];
  dzdt[1] = z[4];
  dzdt[2] = z[5];

  // Velocity dynamics: dv = -g e3 + R*[0,0,T]/m + d_f
  float R[3][3];
  esoRotationMatrix(roll, pitch, yaw, R);

  // thrust_world = R * [0, 0, T] is just T * column 2 of R
  float thrust_world_x = R[0][2] * u_thrust;
  float thrust_world_y = R[1][2] * u_thrust;
  float thrust_world_z = R[2][2] * u_thrust;

  // Python: dv = -[0,0,g] + thrust_world/m + d_f
  // where d_f = z[9:12] is the 3D disturbance vector
  dzdt[3] = 0.0f + thrust_world_x / ESO_MASS + z[9];      // x: no gravity
  dzdt[4] = 0.0f + thrust_world_y / ESO_MASS + z[10];     // y: no gravity
  dzdt[5] = -ESO_G + thrust_world_z / ESO_MASS + z[11];   // z: has -g

  // Attitude dynamics: d(attitude) = E(roll,pitch,yaw) * omega_body
  float E[3][3];
  esoEulerKinematics(roll, pitch, yaw, E);

  float p = omega_body[0];
  float q = omega_body[1];
  float r = omega_body[2];

  dzdt[6] = E[0][0]*p + E[0][1]*q + E[0][2]*r;  // d(roll)
  dzdt[7] = E[1][0]*p + E[1][1]*q + E[1][2]*r;  // d(pitch)
  dzdt[8] = E[2][0]*p + E[2][1]*q + E[2][2]*r;  // d(yaw)

  // Disturbance dynamics: assumed constant
  dzdt[9]  = 0.0f;
  dzdt[10] = 0.0f;
  dzdt[11] = 0.0f;
}

// Initialize ESO state from first measurement [x,y,z, roll,pitch,yaw]
static void esoInitializeFromMeasurement(const float y_meas[6])
{
  for (int i = 0; i < 12; i++) {
    eso_z[i] = 0.0f;
  }

  // Position
  eso_z[0] = y_meas[0];
  eso_z[1] = y_meas[1];
  eso_z[2] = y_meas[2];

  // Attitude
  eso_z[6] = y_meas[3];
  eso_z[7] = y_meas[4];
  eso_z[8] = y_meas[5];

  // Velocity and disturbances remain zero
}

// One ESO step: mirror of Python step(...)
static void esoStep(const float y_meas[6],
                    float u_thrust,
                    const float omega_body[3])
{
  float dzdt[12];
  float z_pred[12];
  float y_pred[6];
  float r[6];

  // PREDICT: z_pred = z + Ts * f(z, u, omega)
  esoFContinuous(eso_z, u_thrust, omega_body, dzdt);
  for (int i = 0; i < 12; i++) {
    z_pred[i] = eso_z[i] + ESO_TS * dzdt[i];
  }

  // Predict measurements:
  //   y_pred = [x,y,z, roll,pitch,yaw] from z_pred
  y_pred[0] = z_pred[0];
  y_pred[1] = z_pred[1];
  y_pred[2] = z_pred[2];
  y_pred[3] = z_pred[6];
  y_pred[4] = z_pred[7];
  y_pred[5] = z_pred[8];

  // INNOVATION: r = y_meas - y_pred
  for (int i = 0; i < 6; i++) {
    r[i] = y_meas[i] - y_pred[i];
  }

  // CORRECT: z = z_pred + L * r
  for (int i = 0; i < 12; i++) {
    float corr = 0.0f;
    for (int j = 0; j < 6; j++) {
      corr += ESO_L[i][j] * r[j];
    }
    eso_z[i] = z_pred[i] + corr;
  }
}

// ========================
// Public API
// ========================

void esoInit(void)
{
  for (int i = 0; i < 12; i++) {
    eso_z[i] = 0.0f;
  }
  esoInitialized = false;
}

eso_log_int_t esoLogInt;

// This runs every stabilizer loop
void esoUpdate(eso_t* eso,
               const state_t* state,
               const sensorData_t* sensors,
               const control_t* prevControl)
{
  // Build measurement vector y_meas = [x,y,z, roll,pitch,yaw]
  float y_meas[6];

  // Positions: already in meters (world frame)
  y_meas[0] = state->position.x;
  y_meas[1] = state->position.y;
  y_meas[2] = state->position.z;

  // Attitude: convert from deg to rad
  y_meas[3] = state->attitude.roll  * DEG2RAD;
  y_meas[4] = state->attitude.pitch * DEG2RAD;
  y_meas[5] = state->attitude.yaw   * DEG2RAD;

  if (!esoInitialized) {
    esoInitializeFromMeasurement(y_meas);
    esoInitialized = true;
  }

  // Input thrust [N]
  float u_thrust = 0.0f;
  if (prevControl != NULL) {
    // Prefer SI thrust if used by your controller
    u_thrust = prevControl->thrustSi;
  }

  // Body rates from gyro [deg/s] -> [rad/s]
  float omega_body[3];
  omega_body[0] = sensors->gyro.x * DEG2RAD;
  omega_body[1] = sensors->gyro.y * DEG2RAD;
  omega_body[2] = sensors->gyro.z * DEG2RAD;

  // Run ESO step
  esoStep(y_meas, u_thrust, omega_body);

  // Map ESO state to output structure.
  // z[9:12] = disturbance ACCELERATION in world frame [m/s²]
  // (Note: Python comments say "force" but the dynamics equation shows it's acceleration)
  eso->d_accel_x = eso_z[9];
  eso->d_accel_y = eso_z[10];
  eso->d_accel_z = eso_z[11];
  eso->d_thrust  = eso_z[11] * ESO_MASS;


  // ======================================================
  // DEBUG MODE — FORCE CONSTANT VALUES TO VERIFY LOGGING PIPELINE
  // ======================================================
  // eso->d_accel_x = 10.0f;
  // eso->d_accel_y = 10.0f;
  // eso->d_accel_z = 10.0f;
  // eso->d_thrust  = 10.0f;

  // ======================================================
  // COPY TO PUBLIC FLOAT OUTPUT STRUCT
  // ======================================================
  esoOutput.d_accel_x = eso->d_accel_x;
  esoOutput.d_accel_y = eso->d_accel_y;
  esoOutput.d_accel_z = eso->d_accel_z;
  esoOutput.d_thrust  = eso->d_thrust;

  // ======================================================
  // COMPACT INT32 LOGGING STRUCT  (scaled by 1000)
  // ======================================================
  esoLogInt.x = (int32_t)(esoOutput.d_accel_x * 1000.0f);
  esoLogInt.y = (int32_t)(esoOutput.d_accel_y * 1000.0f);
  esoLogInt.z = (int32_t)(esoOutput.d_accel_z * 1000.0f);
  esoLogInt.t = (int32_t)(esoOutput.d_thrust  * 1000.0f);

}

void esoApplyDisturbanceCompensation(control_t* control,
                                     const eso_t* eso)
{
  // Apply ONLY vertical thrust compensation
  // Horizontal forces should be handled by outer-loop position controller
  // by adjusting desired attitude, NOT by generating fake torques
  
  // Uncomment when ready to test:
  //
  // control->thrustSi -= eso->d_thrust;
  //
  // Start with partial compensation for safety:
  // float compensation_gain = 0.5f;  // 50% compensation
  // control->thrustSi -= compensation_gain * eso->d_thrust;
  
  (void)control;
  (void)eso;
}