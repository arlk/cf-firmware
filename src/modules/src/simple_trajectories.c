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

static float circRad = 0.8f;
static float circFreq = 0.2f;
static float circAlt = 2.0f;
static float circLisa = 1.0;
static float circLisb = 2.0;
static float circLisA = 2.0;
static float circLisB = 1.0;

static bool isInit;
static uint32_t startTick = 0;
static uint32_t nowTick = 0;

void trajectoryInit(const uint32_t tick)
{
  if(isInit)
    return;

  isInit = true;
  startTick = tick;
}

void updateTrajectory(attitude_t* attitudeDesired, setpoint_t* setpoint, const uint32_t tick)
{
    nowTick = tick - startTick;

    float Lissa_a = circLisa*2*PI*circFreq;
    float Lissa_b = circLisb*2*PI*circFreq;
    float Lissa_A = circLisA*circRad;
    float Lissa_B = circLisB*circRad;

    setpoint->position.x =
      Lissa_A*arm_cos_f32(Lissa_a*(float)tick*GEOMETRIC_UPDATE_DT);
    setpoint->position.y =
      Lissa_B*arm_sin_f32(Lissa_b*(float)tick*GEOMETRIC_UPDATE_DT);
    setpoint->position.z = circAlt;

    setpoint->velocity.x =
      -Lissa_A*Lissa_a*arm_sin_f32(Lissa_a*(float)tick*GEOMETRIC_UPDATE_DT);
    setpoint->velocity.y =
      Lissa_B*Lissa_b*arm_cos_f32(Lissa_b*(float)tick*GEOMETRIC_UPDATE_DT);
    setpoint->velocity.z = 0;

    setpoint->acc.x =
      -Lissa_A*Lissa_a*Lissa_a*arm_cos_f32(Lissa_a*(float)tick*GEOMETRIC_UPDATE_DT);
    setpoint->acc.y =
      -Lissa_B*Lissa_b*Lissa_b*arm_sin_f32(Lissa_b*(float)tick*GEOMETRIC_UPDATE_DT);
    setpoint->acc.z = 0;

    /* attitudeDesired->yaw = atan2f(setpoint->velocity.y, setpoint->velocity.x); */
    attitudeDesired->yaw = Lissa_a*(float)tick*GEOMETRIC_UPDATE_DT;
}

PARAM_GROUP_START(circTraj)
PARAM_ADD(PARAM_FLOAT, circRad, &circRad)
PARAM_ADD(PARAM_FLOAT, circFreq, &circFreq)
PARAM_ADD(PARAM_FLOAT, circAlt, &circAlt)
PARAM_ADD(PARAM_FLOAT, circLisa, &circLisa)
PARAM_ADD(PARAM_FLOAT, circLisb, &circLisb)
PARAM_ADD(PARAM_FLOAT, circLisA, &circLisA)
PARAM_ADD(PARAM_FLOAT, circLisB, &circLisB)
PARAM_GROUP_STOP(circTraj)
