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

#include <stdint>

#define K_VIS 65.0f

void servoControllerInit(const float updateDt);

bool servoControllerTest();

void servoControllerUpdatePID(float servoPosActual, float servoPosDesired);

void servoControllerResetAllPID(void);

void servoEstUpdate(ts,target);

void lagrangeDynamics(servoStates,manipStates,payloadMass);

void pwm2rad(target);

#endif /* PID_H_ */