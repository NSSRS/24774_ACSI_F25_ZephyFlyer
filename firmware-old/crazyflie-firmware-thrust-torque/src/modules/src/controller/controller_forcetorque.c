#include "controller.h"
#include "stabilizer_types.h"

void controllerForceTorqueInit(void) {}

bool controllerForceTorqueTest(void) { return true; }

void controllerForceTorque(control_t *control,
                           const setpoint_t *setpoint,
                           const sensorData_t *sensors,
                           const state_t *state,
                           const stabilizerStep_t step)
{
  // Use SI thrust
  control->thrustSi = setpoint->thrust;

  // Use SI torques
  control->torqueX = setpoint->torqueX;
  control->torqueY = setpoint->torqueY;
  control->torqueZ = setpoint->torqueZ;

  control->controlMode = controlModeForceTorque;
}

controller_t controllerForceTorque = {
  .init = controllerForceTorqueInit,
  .test = controllerForceTorqueTest,
  .control = controllerForceTorque,
};
