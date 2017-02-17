/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2017 Bitcraze AB
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
 *
 */
#include <math.h>
#include <stdbool.h>

#include "crtp_commander.h"

#include "arm_math.h"
#include "commander.h"
#include "crtp.h"
#include "param.h"
#include "FreeRTOS.h"
#include "num.h"

#define MIN_THRUST  1000
#define MAX_THRUST  60000
#define DEG_TO_RAD (PI/180.0f)

static mode_t mode;

/**
 * CRTP commander rpyt packet format
 */
struct CommanderCrtpLegacyValues
{
  float roll;       // deg
  float pitch;      // deg
  float yaw;        // deg
  uint16_t thrust;
} __attribute__((packed));

static bool thrustLocked = true;

void crtpCommanderRpytDecodeSetpoint(setpoint_t *setpoint, CRTPPacket *pk)
{
  struct CommanderCrtpLegacyValues *values = (struct CommanderCrtpLegacyValues*)pk->data;

  if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
    thrustLocked = true;
  }
  if (values->thrust == 0) {
    thrustLocked = false;
  }

  // Thrust
  uint16_t rawThrust = values->thrust;

  if (thrustLocked || (rawThrust < MIN_THRUST)) {
    setpoint->thrust = 0;
  } else {
    setpoint->thrust = min(rawThrust, MAX_THRUST);
  }

  switch (mode) {
    case velMode:
      /* Velocity Mode */
      setpoint->mode = velMode;
      setpoint->velocity.x = -values->pitch;
      setpoint->velocity.y = values->roll;
      setpoint->position.z = values->thrust/1000.0f;
      setpoint->attitudeRate.yaw = values->yaw;
      break;

    case posMode:
      /* Position Mode */
      setpoint->mode = posMode;
      setpoint->position.x = -values->pitch;
      setpoint->position.y = values->roll;
      setpoint->position.z = values->thrust/1000.0f;
      setpoint->attitudeRate.yaw = values->yaw;
      break;

    case simpleTraj:
      /* Simple Trajectories */
      setpoint->mode = simpleTraj;
      setpoint->joy.roll = values->roll;
      setpoint->joy.pitch = values->pitch;
      setpoint->joy.yaw = values->yaw;
      setpoint->joy.throttle = values->thrust/1000.0f;
      break;

    case genericTraj:
      setpoint->mode = genericTraj;
      break;

    default:
      setpoint->mode = angleMode;
      setpoint->attitude.roll = -values->roll*DEG_TO_RAD;
      setpoint->attitude.pitch = values->pitch*DEG_TO_RAD;
      setpoint->attitudeRate.yaw = values->yaw*DEG_TO_RAD;
  }
}


PARAM_GROUP_START(flightmode)
PARAM_ADD(PARAM_UINT8, mode, &mode)
PARAM_GROUP_STOP(flightmode)
