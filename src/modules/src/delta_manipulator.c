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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * delta_manipulator.c - servo motor controller for delta manipulator
 * */
#include "delta_manipulator.h"

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

static bool isInit;
static setpoint_t setpoint;
static state_t state;

static void manipulatorTask(void* param);

typedef struct {
  uint8_t hdr[2];
  uint8_t mID;
  uint8_t len;
  uint8_t inst;
  uint8_t addr;
  uint8_t data[3];
} __attribute__((packed)) dynamixelPacket_s;


 int delta_calcAngleYZ(float x0, float y0, float z0, float* theta) {
     float y1 = -0.5f * 0.57735f * delta_f; // f/2 * tg 30
     y0 -= 0.5f * 0.57735f    * delta_e;    // shift center to edge
     // z = a + b*y
     float a = (x0*x0 + y0*y0 + z0*z0 +delta_rf*delta_rf - delta_re*delta_re - y1*y1)/(2*z0);
     float b = (y1-y0)/z0;
     // discriminant
     float d = -(a+b*y1)*(a+b*y1)+delta_rf*(b*b*delta_rf+delta_rf); 
     if (d < 0) return -1; // non-existing point
     float yj = (y1 - a*b - (float)sqrt(d))/(b*b + 1); // choosing outer point
     float zj = a + b*yj;
     *theta = 180.0f*(float)atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0f:0.0f);
     return 0;
 }
 
 // inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
 // returned status: 0=OK, -1=non-existing position
 int delta_calcInverse(float x0, float y0, float z0, float* theta1, float* theta2, float* theta3) {
     *theta1 = *theta2 = *theta3 = 0;
     int status = delta_calcAngleYZ(x0, y0, z0, theta1);
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
     return status;
 }



void manipulatorInit(void)
{
  if(isInit)
    return;

  uart1Init(BAUD_RATE);
  xTaskCreate(manipulatorTask, MANIPULATOR_TASK_NAME,
              MANIPULATOR_TASK_STACKSIZE, NULL, MANIPULATOR_TASK_PRI, NULL);

  isInit = true;
}

void calcChksum(uint8_t* msg)
{
  int i = 0;
  uint16_t sum = 0;
  uint8_t len;
  len = msg[3];

  for (i = 2; i <= len+2; i++)
  {
    sum += msg[i];
  }

   msg[len+3] = ~sum & 0xFF;
}

void dynamixelEnableTorque(uint8_t motorID, const uint8_t torqueState)
{
  /* torqueState: 0 <--> 1 */

  dynamixelPacket_s msg = (dynamixelPacket_s) {
    .hdr = {0xFF, 0xFF},
    .mID = motorID,
    .len = DELTA_SINGLE_MSG_LEN,
    .inst = DELTA_WRITE_DATA,
    .addr = DELTA_ADDRESS_TORQUE_ENABLE,
    .data = {0}
  };

  memcpy(&msg.data, &torqueState, sizeof(torqueState));
  calcChksum((uint8_t*)&msg);

  uart1SendData(msg.len + DELTA_STD_LEN, (uint8_t*)&msg);
}

void dynamixelSetLED(uint8_t motorID, uint8_t ledState)
{
  /* ledState: 0 <--> 1 */

  dynamixelPacket_s msg = (dynamixelPacket_s) {
    .hdr = {0xFF, 0xFF},
    .mID = motorID,
    .len = DELTA_SINGLE_MSG_LEN,
    .inst = DELTA_WRITE_DATA,
    .addr = DELTA_ADDRESS_LED,
    .data = {0}
  };

  memcpy(&msg.data, &ledState, sizeof(ledState));
  calcChksum((uint8_t*)&msg);

  uart1SendData(msg.len + DELTA_STD_LEN, (uint8_t*)&msg);
}

void dynamixelMovePosn(uint8_t motorID, uint16_t position)
{
  /* position: 0 <--> 1023 */

  dynamixelPacket_s msg = (dynamixelPacket_s) {
    .hdr = {0xFF, 0xFF},
    .mID = motorID,
    .len = DELTA_DOUBLE_MSG_LEN,
    .inst = DELTA_WRITE_DATA,
    .addr = DELTA_ADDRESS_GOAL_POSITION,
    .data = {0}
  };

  memcpy(&msg.data, &position, sizeof(position));
  calcChksum((uint8_t*)&msg);

  uart1SendData(msg.len + DELTA_STD_LEN, (uint8_t*)&msg);
}

static void manipulatorTask(void* param)
{
  uint32_t tick = 0;
  uint32_t lastWakeTime;
  uint8_t ledState = 0;

  vTaskSetApplicationTaskTag(0, (void*)TASK_MANIPULATOR_ID_NBR);

  //Wait for the system to be fully started to start manipulation loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MANIPULATOR_LOOP));
  }

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(5));

    commanderGetSetpoint(&setpoint, &state);

    ledState = !ledState;

    if (ledState) {
      dynamixelSetLED(2, 1);
      vTaskDelayUntil(&lastWakeTime, F2T(100));
      dynamixelMovePosn(1, 255);
      vTaskDelayUntil(&lastWakeTime, F2T(100));
      dynamixelMovePosn(2, 255);
      vTaskDelayUntil(&lastWakeTime, F2T(100));
      dynamixelMovePosn(3, 255);
    }
    else {
      dynamixelSetLED(2, 0);
      vTaskDelayUntil(&lastWakeTime, F2T(100));
      dynamixelMovePosn(1, 400);
      vTaskDelayUntil(&lastWakeTime, F2T(100));
      dynamixelMovePosn(2, 400);
      vTaskDelayUntil(&lastWakeTime, F2T(100));
      dynamixelMovePosn(3, 400);
    }

    tick++;
  }
}
