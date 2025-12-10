#include "controller.h"
#include "stabilizer_types.h"

void controllerForceTorqueInit(void) {}

bool controllerForceTorqueTest(void) {
return true;
}

void controllerForceTorque(control_t *control,
const setpoint_t *setpoint,
const sensorData_t *sensors,
const state_t *state,
const stabilizerStep_t step)
{
// Pass thrust directly
control->thrust = setpoint->thrust;

// Pass torques through the existing attitude channels
control->roll = setpoint->attitude.roll; // τx
control->pitch = setpoint->attitude.pitch; // τy
control->yaw = setpoint->attitude.yaw; // τz
}