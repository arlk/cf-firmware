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
 * serial_manipulator.h - servo motor controller for serial manipulator
 * */
#ifndef SERIAL_MANIPULATOR_H_
#define SERIAL_MANIPULATOR_H_

#define RATE_MANIPULATOR_LOOP 100  // 100 Hz micro maestro rate
#define BAUD_RATE 57600

#include <stdbool.h>
#include <stdint.h>

#include "torque_estimator.h"

#include "FreeRTOS.h"
#include "queue.h"

typedef struct servoTarget_s{
	int joint1;
	int joint2;
	int gripper;
} servoTarget_t;


void manipulatorInit(void);

bool serialManipEnqueueCmd(xQueueHandle* queue, void *command);

bool serialManipGetQueueCmd(void *command);

void servoController(int* targetAll, servoStates_t* servoStates, const state_t* state, const sensorData_t* sensorData, setpoint_t* setpoint);

void meastro_set_acceleration(unsigned short device_number, unsigned char channel, unsigned short target);

void meastro_set_target(unsigned short device_number, unsigned char channel, unsigned short target);

void maestro_uart_protocol(unsigned short device_number);

void maestro_send_data(unsigned short target);

#endif /* SERIAL_MANIPULATOR_H_ */
