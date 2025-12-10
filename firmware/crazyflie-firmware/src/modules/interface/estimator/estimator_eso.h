#ifndef __ESTIMATOR_ESO_H__
#define __ESTIMATOR_ESO_H__

#include "stabilizer_types.h"
#include "sensors.h"

// =======================================================
// ESO output: disturbance accelerations + thrust disturbance
// =======================================================
typedef struct {
    float d_accel_x;   // world-frame disturbance acceleration X [m/s^2]
    float d_accel_y;   // world-frame disturbance acceleration Y [m/s^2]
    float d_accel_z;   // world-frame disturbance acceleration Z [m/s^2]
    float d_thrust;    // vertical disturbance force [N]
} eso_t;

// GLOBAL output used by log system
extern eso_t esoOutput;


// =======================================================
// Compact INT32 log struct for bandwidth-safe logging
// =======================================================
typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t t;
} eso_log_int_t;

// Exported global instance
extern eso_log_int_t esoLogInt;


// =======================================================
// Public API
// =======================================================
void esoInit(void);

void esoUpdate(eso_t* eso,
               const state_t* state,
               const sensorData_t* sensors,
               const control_t* prevControl);

void esoApplyDisturbanceCompensation(control_t* control,
                                     const eso_t* eso);

#endif // __ESTIMATOR_ESO_H__
