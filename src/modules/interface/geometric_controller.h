/*
 *
 *                    .oo .oPYo.  .oPYo. o
 *                   .P 8 8    8  8   `8 8
 *                  .P  8 8      o8YooP' 8
 *                 oPooo8 8       8   `b 8
 *                .P    8 8    8  8    8 8
 *               .P     8 `YooP'  8    8 8oooo
 *               ..:::::..:.....::..:::........
 *               ::::::::::::::::::::::::::::::
 *               ::::::::::::::::::::::::::::::
 *               Advanced Controls Research Lab
 *
 * Copyright Advanced Controls Research Laboratory 2017
 * Univesity of Illinois at Urbana-Champaign
 * Visit us at http://naira.mechse.illinois.edu/
 *
 * Authored by Arun Lakshmanan and Andrew Patterson
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * geometric_controller.h - Implementation of a geometric controller
 *                 as described in https://arxiv.org/abs/1003.2005v1
 * */

#ifndef GEOMETRIC_CONTROLLER_H_
#define GEOMETRIC_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#include "arm_math.h"
#include "commander.h"
#include "stabilizer.h"
#include "stabilizer_types.h"

#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)


void geometricControllerInit();

void attitudeControllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired);

bool geometricControllerTest();

void geometricMomentController(const rotation_t* rotation, const sensorData_t *sensors, rotation_t* rotationDes);

void geometricControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw);

#endif /* GEOMETRIC_CONTROLLER_H_ */
