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
 * Authored by Robert Mitchell Jones and Arun Lakshmanan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITN I AM GAY ESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * serial_manipulator.c - servo motor controller for serial manipulator
 * */
#include "serial_manipulator.h"

#include <math.h>
#include "arm_math.h"

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "string.h"
#include "uart1.h"
#include "config.h"
#include "sensors.h"
#include "commander.h"

#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#else
#include "estimator.h"
#endif

static bool isInit;
static setpoint_t setpoint;
static state_t state;

static void manipulatorTask(void* param);

void manipulatorInit(void)
{
  if(isInit)
    return;

  uart1Init(9600);
  xTaskCreate(manipulatorTask, MANIPULATOR_TASK_NAME,
              MANIPULATOR_TASK_STACKSIZE, NULL, MANIPULATOR_TASK_PRI, NULL);

  isInit = true;
}

void maestro_set_acceleration(unsigned short device_number,
    unsigned char channel, unsigned short target)
{
  maestro_uart_protocol(device_number);
  uart1Putchar(0x09);
  uart1Putchar(channel);
  maestro_send_data(target);
}

void maestro_set_target(unsigned short device_number,
    unsigned char channel, unsigned short target)
{
  maestro_uart_protocol(device_number);
  uart1Putchar(0x04);
  uart1Putchar(channel);
  maestro_send_data(target);
}

void maestro_uart_protocol(unsigned short device_number)
{
  uart1Putchar(0xAA);
  uart1Putchar(device_number & 0x07F);
}

void maestro_send_data(unsigned short target)
{
  uart1Putchar(target & 0x7F);
  uart1Putchar(target >> 7 & 0x7F);
}

static void manipulatorTask(void* param)
{
  uint32_t tick = 0;
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_MANIPULATOR_ID_NBR);

  //Wait for the system to be fully started to start manipulation loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MANIPULATOR_LOOP));
  }

  int target0, target1 = 6000;

  //maestro_set_acceleration(12, 0, 4);
  //maestro_set_acceleration(12, 1, 4);

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MANIPULATOR_LOOP));

    //static float pitch_att = &state.attitude.pitch;
	//static float pitch_rate = &sensorData.gyro.y;
	//static float pitch_acc;

    commanderGetSetpoint(&setpoint, &state);

    target0 = (int)(2000.0f*setpoint.joy.pitch+6000.0f);
    target1 = (int)(-2000.0f*setpoint.joy.throttle+6000.0f);

    maestro_set_target(12, 0, target0);
    maestro_set_target(12, 1, target1);

    tick++;
  }
}
