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

static float actuatorThrust;

static void yawFromDesiredYawRate(setpoint_t *setpoint) {
  setpoint->attitude.yaw -= setpoint->attitudeRate.yaw*GEOMETRIC_UPDATE_DT;

  while (setpoint->attitude.yaw > PI)
    setpoint->attitude.yaw -= 2*PI;
  while (setpoint->attitude.yaw < -PI)
    setpoint->attitude.yaw += 2*PI;
}

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
      case velMode:
        /* Velocity Mode */
        setpoint->position.x = state->position.x;
        setpoint->position.y = state->position.y;
        yawFromDesiredYawRate(setpoint);
        geometricControllerGetAttitudeDesired(state, setpoint);
        geometricControllerGetThrustDesired(state, setpoint);
        geometricControllerGetThrustOutput(&actuatorThrust);
        break;

      case posMode:
        /* Position Mode */
        trajectoryDeinit();
        yawFromDesiredYawRate(setpoint);
        geometricControllerGetAttitudeDesired(state, setpoint);
        geometricControllerGetThrustDesired(state, setpoint);
        geometricControllerGetThrustOutput(&actuatorThrust);
        break;

      case simpleTraj:
        /* Simple Trajectories */
        trajectoryInit(tick);
        updateTrajectory(setpoint, tick);
        geometricControllerGetAttitudeDesired(state, setpoint);
        geometricControllerGetOmegaDesired(setpoint);
        geometricControllerGetThrustDesired(state, setpoint);
        geometricControllerGetThrustOutput(&actuatorThrust);
        break;

      case genericTraj:
        /* Generic Trajectories */
        break;

      default:
        /* Angle Mode */
        setpoint->attitude.roll = setpoint->attitude.roll;
        setpoint->attitude.pitch = -setpoint->attitude.pitch;
        yawFromDesiredYawRate(setpoint);
        eulerToRotationZYX(&setpoint->attitude, &setpoint->rotation);
        actuatorThrust = setpoint->thrust;
    }

    geometricMomentController(state, sensors, setpoint);

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

    // Reset if there is no thrust input
    setpoint->attitude.yaw = state->attitude.yaw;
  }
}


LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
LOG_GROUP_STOP(controller)
