#ifndef CONTROLLER_FORCETORQUE_H
#define CONTROLLER_FORCETORQUE_H

#include "stabilizer_types.h"

void controllerForceTorqueInit(void);
bool controllerForceTorqueTest(void);
void controllerForceTorque(control_t *control,
const setpoint_t *setpoint,
const sensorData_t *sensors,
const state_t *state,
const stabilizerStep_t step);

#endif