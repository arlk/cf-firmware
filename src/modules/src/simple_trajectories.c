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
 * University of Illinois at Urbana-Champaign
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
 * geometric_controller.c - Implementation of a geometric controller
 * */

#include <stdbool.h>

#include "FreeRTOS.h"

#include "simple_trajectories.h"

static bool isInit;
static uint32_t startTick = 0;
static uint32_t nowTick = 0;

static float circRad = 0.8f;
static float circFreq = 0.2f;
static float circAlt = 2.0f;

static float omg = 0;
static float amp = 0;
static float phase = 0;

#define BETA 0.999400359f

typedef enum traj_e {
  noneTraj = 0,
  circTraj
} traj_t;

static traj_t traj = 0;

void trajectoryInit(const uint32_t tick)
{
  if(isInit)
    return;

  isInit = true;
  startTick = tick;
}

void circleUpdate(setpoint_t* setpoint, const uint32_t tick)
{
    float amp_des;
    float speed = setpoint->joy.throttle/60.0f;
    float size = setpoint->joy.throttle/20.0f;
    size = (size > 0.20f) ? 1.0f : 0.0f;

    phase = (omg - speed*2.0f*PI*circFreq)*tick*GEOMETRIC_UPDATE_DT + phase;
    omg = speed*2.0f*PI*circFreq;

    amp_des = size*circRad;
    amp = (1.0f-BETA)*amp_des + BETA*amp;

    setpoint->position.x =
       amp*arm_cos_f32(omg*(float)tick*GEOMETRIC_UPDATE_DT + phase);
    setpoint->position.y =
       amp*arm_sin_f32(omg*(float)tick*GEOMETRIC_UPDATE_DT + phase);
    setpoint->position.z = circAlt;

    setpoint->velocity.x =
      -amp*omg*arm_sin_f32(omg*(float)tick*GEOMETRIC_UPDATE_DT + phase);
    setpoint->velocity.y =
       amp*omg*arm_cos_f32(omg*(float)tick*GEOMETRIC_UPDATE_DT + phase);
    setpoint->velocity.z = 0.0f;

    setpoint->acc.x =
      -amp*omg*omg*arm_cos_f32(omg*(float)tick*GEOMETRIC_UPDATE_DT + phase);
    setpoint->acc.y =
      -amp*omg*omg*arm_sin_f32(omg*(float)tick*GEOMETRIC_UPDATE_DT + phase);
    setpoint->acc.z = 0.0f;

    setpoint->jerk.x =
       amp*omg*omg*omg*arm_sin_f32(omg*(float)tick*GEOMETRIC_UPDATE_DT +
           phase);
    setpoint->jerk.y =
      -amp*omg*omg*omg*arm_cos_f32(omg*(float)tick*GEOMETRIC_UPDATE_DT +
          phase);
    setpoint->jerk.z = 0.0f;

    setpoint->snap.x =
       amp*omg*omg*omg*omg*arm_cos_f32(omg*(float)tick*GEOMETRIC_UPDATE_DT +
           phase);
    setpoint->snap.y =
       amp*omg*omg*omg*omg*arm_sin_f32(omg*(float)tick*GEOMETRIC_UPDATE_DT +
           phase);
    setpoint->snap.z = 0.0f;

    setpoint->attitude.yaw = omg*(float)tick*GEOMETRIC_UPDATE_DT + phase;
    setpoint->attitudeRate.yaw = omg;
    setpoint->attitudeAcc.yaw = 0.0f;
}

void updateTrajectory(setpoint_t* setpoint, const uint32_t tick)
{
    nowTick = tick - startTick;

    switch (traj) {
      case circTraj:
        circleUpdate(setpoint, nowTick);
        break;

      case noneTraj:

      default:
        break;
    }
}

PARAM_GROUP_START(circTraj)
PARAM_ADD(PARAM_FLOAT, circRad, &circRad)
PARAM_ADD(PARAM_FLOAT, circFreq, &circFreq)
PARAM_ADD(PARAM_FLOAT, circAlt, &circAlt)
PARAM_GROUP_STOP(circTraj)

PARAM_GROUP_START(trajmode)
PARAM_ADD(PARAM_UINT8, traj, &traj)
PARAM_GROUP_STOP(trajmode)
