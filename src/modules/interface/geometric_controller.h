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
 * */

#ifndef GEOMETRIC_CONTROLLER_H_
#define GEOMETRIC_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "torque_estimator.h"

#include "arm_math.h"
#include "commander.h"
#include "stabilizer.h"
#include "stabilizer_types.h"

#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)
#define GRAVITY 9.80665f

#define GEOMETRIC_UPDATE_DT  (float)(1.0f/ATTITUDE_RATE)
#define MAIN_LOOP_DT  (float)(1.0f/RATE_MAIN_LOOP)

void geometricControllerInit();

bool geometricControllerTest();
/*
bool vehicleEnqueuePitchStates(xQueueHandle* queue, void *pitchStates);

bool vehicleGetQueuePitchStates(float *pitchStates);
*/
void geometricControllerGetAttitudeDesired(const state_t* state,
    setpoint_t* setpoint);

void geometricControllerGetOmegaDesired(setpoint_t* setpoint);

void geometricControllerGetThrustDesired(const state_t* state, setpoint_t* setpoint);

void geometricMomentController(state_t* state,
    const sensorData_t* sensors, setpoint_t* setpoint);

void geometricControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw);

void geometricControllerGetThrustOutput(float* thrust);

#endif /* GEOMETRIC_CONTROLLER_H_ */
