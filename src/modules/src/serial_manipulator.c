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
#include "torque_estimator.h"

#include <math.h>
#include "arm_math.h"

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"
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

int targetAll[3];

static xQueueHandle manipCmdQueue;
#define MANIP_CMD_QUEUE_LENGTH 10

void manipulatorInit(void)
{
  if(isInit){
  	xQueueReset(manipCmdQueue);
  	return;
  }

  uart1Init(BAUD_RATE);
  xTaskCreate(manipulatorTask, MANIPULATOR_TASK_NAME,
              MANIPULATOR_TASK_STACKSIZE, NULL, MANIPULATOR_TASK_PRI, NULL);

  manipCmdQueue = xQueueCreate(MANIP_CMD_QUEUE_LENGTH,sizeof(targetAll));

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

bool serialManipEnqueueCmd(xQueueHandle* queue, void *command)
{
  portBASE_TYPE result;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(queue, command, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD();
    }
  } else {
    result = xQueueSend(queue, command, 0);
  }
  return (result==pdTRUE);
}


bool serialManipGetQueueCmd(float *command)
{
	portBASE_TYPE result;
	result = xQueueReceive(manipCmdQueue, command, 0);
	return (result==pdTRUE);
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

  int target0 = 6000, target1 = 6000, target2 = 6000;

  //maestro_set_acceleration(12, 0, 4);
  //maestro_set_acceleration(12, 1, 4);

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MANIPULATOR_LOOP));

    //static float pitch_att = &state.attitude.pitch;
	//static float pitch_rate = &sensorData.gyro.y;
	//static float pitch_acc;

    commanderGetSetpoint(&setpoint, &state);

    /*
    target0 = (int)(2000.0f*setpoint.joy.pitch+6000.0f);
    target1 = (int)(-2000.0f*setpoint.joy.throttle+6000.0f);
    target2 = (int)(-3000.0f*(float)setpoint.joy.trigger+7000.0f);
	*/

    
    if ((float)setpoint.joy.trigger > 0.5f)
    {
    	// coad
    	target0 = (int)(2000.0f*(float)state.attitude.pitch+6000.0f);
    	target1 = (int)(2000.0f*(float)state.attitude.pitch+6000.0f);
    	target2 = (int)(4000.0f);
    }
    else
    {
    	// coad
    	target0 = 5000, target1 = 5000, target2 = 5000;
    }
    
    targetAll[0] = target0;
    targetAll[1] = target1;
    targetAll[2] = target2;

    maestro_set_target(12, 0, target0);
    maestro_set_target(12, 1, target1);
    maestro_set_target(12, 2, target2);

    serialManipEnqueueCmd(manipCmdQueue, (void *)targetAll);

    tick++;
  }
}
