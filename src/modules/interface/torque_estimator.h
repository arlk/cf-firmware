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
 * Authored by Robert Mitchell Jones
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
 * torque_estimator.c - servo motor controller for serial manipulator
 * */
#ifndef TORQUE_ESTIMATOR_H_
#define TORQUE_ESTIMATOR_H_

#include <stdint.h>
#include "stabilizer_types.h"
#include "serial_manipulator.h"

#define K_VIS 65.0f

#define IZ_1 28.741e-7f
#define IZ_2 189.073E-6f
#define M_1 9.30e-3f
#define M_2 30.7e-3f
#define L_1 0.100f
#define L_2 0.130f
#define R_1 43.48e-3f
#define R_2 67.77e-3f

#define SERVO_ACC_MAX 50.0f

void servoControllerInit(const float updateDt);

bool servoControllerTest();

int servoControllerUpdatePID(float servoPosActual, float servoPosDesired);

void servoControllerResetAllPID(void);

void servoGetCmd(int* targetAll, const state_t* state, setpoint_t* setpoint);

void servoEstUpdate(float ts, int servoNumber, servoStates_t* servoStates, int* targetAll);

float lagrangeDynamics(float payloadMass, servoStates_t* servoStates);

float pwm2rad(float target);

#endif /* PID_H_ */