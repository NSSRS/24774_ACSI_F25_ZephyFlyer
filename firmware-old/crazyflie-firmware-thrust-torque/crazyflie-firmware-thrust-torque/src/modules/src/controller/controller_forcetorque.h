#ifndef __CONTROLLER_FORCETORQUE_H__
#define __CONTROLLER_FORCETORQUE_H__

#include <stdbool.h>
#include "stabilizer_types.h"

void controllerForceTorqueInit(void);

bool controllerForceTorqueTest(void);

void controllerForceTorque(control_t *control,
                           const setpoint_t *setpoint,
                           const sensorData_t *sensors,
                           const state_t *state,
                           const stabilizerStep_t step);

#endif // __CONTROLLER_FORCETORQUE_H__
