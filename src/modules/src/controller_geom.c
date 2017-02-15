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
 * controller_geom.c - Calls geometric controller functions
 * */

#include "FreeRTOS.h"
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "arm_math.h"
#include "geometric_controller.h"
#include "simple_trajectories.h"
#include "flight_math.h"
#include "sensfusion6.h"

#include "log.h"
#include "param.h"

static attitude_t attitudeDesired;
static float actuatorThrust;

static bool circMode = false;

void stateControllerInit(void)
{
  geometricControllerInit();
}

bool stateControllerTest(void)
{
  bool pass = true;

  pass &= geometricControllerTest();

  return pass;
}

void stateController(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {

    switch (setpoint->mode) {
      case 1:
        /* Velocity Mode */
        setpoint->velocity.x = values->pitch;
        setpoint->velocity.y = values->roll;
        setpoint->position.z = values->thrust/1000.0f;
        setpoint->attitudeRate.yaw = values->yaw;
        break;

      case 2:
        /* Position Mode */
        setpoint->position.x = -values->pitch;
        setpoint->position.y = values->roll;
        setpoint->position.z = values->thrust/1000.0f;
        setpoint->attitude.yaw = values->yaw;
        break;

      default:
        /* Angle Mode */
        setpoint->attitude.roll = values->roll;
        setpoint->attitude.pitch = values->pitch;
        setpoint->attitudeRate.yaw = values->yaw;
        setpoint->thrust = values->thrust;
    }

    // Manual control (joystick)
    if (circMode)
    {
      trajectoryInit(tick);
      updateTrajectory(&attitudeDesired, setpoint, tick);
      geometricControllerGetAttitudeDesired(state, &attitudeDesired, setpoint);
      geometricControllerGetThrustDesired(state, setpoint);
      geometricControllerGetThrustOutput(&actuatorThrust);
    }
    else if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll*DEG_TO_RAD;
      attitudeDesired.pitch = -setpoint->attitude.pitch*DEG_TO_RAD;
      eulerToRotationZYX(&attitudeDesired, &setpoint->rotation);
      actuatorThrust = setpoint->thrust;
    }
    // Crane mode (joystick velocities)
    else if (setpoint->mode.x == modeVelocity &&
        setpoint->mode.y == modeVelocity &&
        setpoint->mode.z == modeAbs) {
      setpoint->position.x = state->position.x;
      setpoint->position.y = state->position.y;
      geometricControllerGetAttitudeDesired(state, &attitudeDesired, setpoint);
      geometricControllerGetThrustDesired(state, setpoint);
      geometricControllerGetThrustOutput(&actuatorThrust);
    }
    // Position Set
    else if (setpoint->mode.x == modeAbs &&
        setpoint->mode.y == modeAbs &&
        setpoint->mode.z == modeAbs) {
      geometricControllerGetAttitudeDesired(state, &attitudeDesired, setpoint);
      geometricControllerGetThrustDesired(state, setpoint);
      geometricControllerGetThrustOutput(&actuatorThrust);
    }
    geometricMomentController(&state->rotation, sensors, &setpoint->rotation);

    geometricControllerGetActuatorOutput(&control->roll,
                                         &control->pitch,
                                         &control->yaw);

    control->yaw = -control->yaw;
  }

  // Thrust
  control->thrust = actuatorThrust;

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    // Reset the integrated values if there is no thrust input
    attitudeDesired.yaw = state->attitude.yaw*DEG_TO_RAD;
  }
}


LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
LOG_GROUP_STOP(controller)

PARAM_GROUP_START(trajmode)
PARAM_ADD(PARAM_UINT8, circMode, &circMode)
PARAM_GROUP_STOP(trajmode)
