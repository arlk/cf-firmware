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

#include "stabilizer.h"
#include "stabilizer_types.h"

#include "geometric_controller.h"
#include "flight_math.h"
#include "sensfusion6.h"

#include "log.h"
#include "param.h"

#define GEOMETRIC_UPDATE_DT  (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static float actuatorThrust;

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

void updateAttitudeDesired(const state_t *state, attitude_t* attitudeDesired,
    setpoint_t* setpoint)
{
  static float errRotation[3];
  errPosition[0] = setpoint->positionDes.x - state->position.x;
  errPosition[1] = setpoint->positionDes.y - state->position.y;
  errPosition[2] = setpoint->positionDes.z - state->position.z;

  static float errVelocity[3];
  errVelocity[0] = setpoint->velocityDes.x - state->velocity.x;
  errVelocity[1] = setpoint->velocityDes.y - state->velocity.y;
  errVelocity[2] = setpoint->velocityDes.z - state->velocity.z;

  static float forceMagnitude =
    (k_pos*errPosition[0] + k_vel*errVelocity[0] + mass*stateDes->acc.x)*state->rotation.vals[0][2]
  + (k_pos*errPosition[1] + k_vel*errVelocity[1] + mass*stateDes->acc.y)*state->rotation.vals[1][2]
  + (k_pos*errPosition[2] + k_vel*errVelocity[2] + mass*(GRAVITY + stateDes->acc.z))*state->rotation.vals[2][2];

  setpoint->rotation.vals[0][2] = (k_pos*errPosition[0] + k_vel*errVelocity[0]
      + mass*setpoint->acc.x)*state->rotation.vals[0][2]/forceMagnitude;
  setpoint->rotation.vals[1][2] = (k_pos*errPosition[1] + k_vel*errVelocity[1]
      + mass*setpoint->acc.y)*state->rotation.vals[1][2]/forceMagnitude;
  setpoint->rotation.vals[2][2] = (k_pos*errPosition[2] + k_vel*errVelocity[2]
      + mass*(GRAVITY + setpoint->acc.z))*state->rotation.vals[2][2]/forceMagnitude;

  static float xInterDesired[3];
  xInterDesired[0] = arm_cos_f32(attitudeDesired.yaw);
  xInterDesired[1] = arm_sin_f32(attitudeDesired.yaw);
  xInterDesired[2] = 0;

  setpoint->rotation.vals[0][1] = -setpoint->rotation.vals[2][2]*xInterDesired[1]
    + setpoint->rotation[1][2]*xInterDesired[2];
  setpoint->rotation.vals[1][1] =  setpoint->rotation.vals[2][2]*xInterDesired[0]
    - setpoint->rotation[0][2]*xInterDesired[2];
  setpoint->rotation.vals[2][1] = -setpoint->rotation.vals[1][2]*xInterDesired[0]
    + setpoint->rotation[0][2]*xInterDesired[1];

  setpoint->rotation.vals[0][0] = -setpoint->rotation.vals[2][1]*setpoint->rotation.vals[1][2]
    + setpoint->rotation[1][1]*setpoint->rotation.vals[2][2];
  setpoint->rotation.vals[1][0] = setpoint->rotation.vals[2][1]*setpoint->rotation.vals[0][2]
    - setpoint->rotation[0][1]*setpoint->rotation.vals[2][2];
  setpoint->rotation.vals[2][0] = -setpoint->rotation.vals[1][1]*setpoint->rotation.vals[0][2]
    + setpoint->rotation[0][1]*setpoint->rotation.vals[1][2];
}

void stateController(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Yaw control (default: rate)
    if (setpoint->mode.yaw == modeVelocity) {
       attitudeDesired.yaw -= setpoint->attitudeRate.yaw*DEG_TO_RAD/500.0f;
      while (attitudeDesired.yaw > PI)
        attitudeDesired.yaw -= 2*PI;
      while (attitudeDesired.yaw < -PI)
        attitudeDesired.yaw += 2*PI;
    } else {
      attitudeDesired.yaw = setpoint->attitude.yaw*DEG_TO_RAD;
    }

    // Switch between manual and automatic position control
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll*DEG_TO_RAD;
      attitudeDesired.pitch = -setpoint->attitude.pitch*DEG_TO_RAD;
      eulerToRotationZYX(&attitudeDesired, &setpoint->rotation);
    } else if (setpoint->mode.x == modeVelocity && setpoint->mode.y == modeVelocity) {
      updateAttitudeDesired(&state, &attitudeDesired, &setpoint);
    }

    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust;
    }
    /* } else if (setpoint->mode.z == modeAbs) { */
    /*   geometricControllerGetActuatorOutput(&actuatorThrust); */

    /* geometricControllerTest(&state->attitudeRotation, sensors, &setpoint->rotation); */

    geometricMomentController(&state->attitudeRotation, sensors, &setpoint->rotation);

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

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw*DEG_TO_RAD;
  }
}


LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
LOG_GROUP_STOP(controller)
