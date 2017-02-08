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
#include "flight_math.h"
#include "sensfusion6.h"

#include "log.h"
#include "param.h"

#define GEOMETRIC_UPDATE_DT  (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static float actuatorThrust;

static float max_position_x = 1.0;
static float max_position_y = 1.0;
static float max_position_z = 1.0;
static float min_position_x = -1.0;
static float min_position_y = -1.0;
static float min_position_z = -1.0;

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

    // Yaw input (default: rate)
    if (setpoint->mode.yaw == modeVelocity) {
      attitudeDesired.yaw -= setpoint->attitudeRate.yaw*DEG_TO_RAD*GEOMETRIC_UPDATE_DT;
      while (attitudeDesired.yaw > PI)
        attitudeDesired.yaw -= 2*PI;
      while (attitudeDesired.yaw < -PI)
        attitudeDesired.yaw += 2*PI;
    }
    // Yaw input (angle)
    else {
      attitudeDesired.yaw = setpoint->attitude.yaw*DEG_TO_RAD;
    }


    // Manual control (joystick)
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll*DEG_TO_RAD;
      attitudeDesired.pitch = -setpoint->attitude.pitch*DEG_TO_RAD;
      eulerToRotationZYX(&attitudeDesired, &setpoint->rotation);
      actuatorThrust = setpoint->thrust;
    }
    // Crane mode (joystick velocities)
    else if (setpoint->mode.x == modeVelocity &&
        setpoint->mode.y == modeVelocity &&
        setpoint->mode.z == modeVelocity) {
      setpoint->position.x += setpoint->velocity.x*GEOMETRIC_UPDATE_DT;
      setpoint->position.z += setpoint->velocity.y*GEOMETRIC_UPDATE_DT;
      setpoint->position.y += setpoint->velocity.z*GEOMETRIC_UPDATE_DT;

      if (setpoint->position.x > max_position_x) {
        setpoint->position.x = max_position_x;
      } else if (setpoint->position.x < min_position_x) {
        setpoint->position.x = min_position_x;
      }

      if (setpoint->position.y > max_position_y) {
        setpoint->position.y = max_position_y;
      } else if (setpoint->position.y < min_position_y) {
        setpoint->position.y = min_position_y;
      }

      if (setpoint->position.z > max_position_z) {
        setpoint->position.z = max_position_z;
      } else if (setpoint->position.z < min_position_z) {
        setpoint->position.z = min_position_z;
      }

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
    //FIXME arun
    /* setpoint->position.x = state->position.x; */
    /* setpoint->position.y = state->position.y; */
    /* setpoint->position.z = state->position.z; */
  }
}


LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
LOG_GROUP_STOP(controller)

LOG_GROUP_START(posnsat)
LOG_ADD(LOG_FLOAT, max_x, &max_position_x)
LOG_ADD(LOG_FLOAT, max_y, &max_position_y)
LOG_ADD(LOG_FLOAT, max_z, &max_position_z)
LOG_ADD(LOG_FLOAT, min_x, &min_position_x)
LOG_ADD(LOG_FLOAT, min_y, &min_position_y)
LOG_ADD(LOG_FLOAT, min_z, &min_position_z)
LOG_GROUP_STOP(posnsat)
