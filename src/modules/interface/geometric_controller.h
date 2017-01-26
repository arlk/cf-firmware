#ifndef GEOMETRIC_CONTROLLER_H_
#define GEOMETRIC_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>
#include "commander.h"
#include "stabilizer.h"
#include "stabilizer_types.h"


void geometricControllerInit();

void attitudeControllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired);

bool geometricControllerTest();

void geometricMomentController(const rotation_t* rotation, const sensorData_t *sensors, rotation_t* rotationDes);

void geometricControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw);

#endif /* GEOMETRIC_CONTROLLER_H_ */
