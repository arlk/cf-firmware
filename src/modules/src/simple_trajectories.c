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

/* Thiago's Bezier variables*/
static traj P1;
static traj P2;
static traj P3;
static traj P4;
static traj P_control;
static traj V_control;
static traj J_control;
static traj A_control;


/* End thiago */

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
    float P_vec[3];
    float V_vec[3];
    float A_vec[3];
    float J_vec[3];

    compBezier(&P_control,(float)tick*GEOMETRIC_UPDATE_DT,P_vec);
    compBezier(&V_control,(float)tick*GEOMETRIC_UPDATE_DT,V_vec);
    compBezier(&A_control,(float)tick*GEOMETRIC_UPDATE_DT,A_vec);
    compBezier(&J_control,(float)tick*GEOMETRIC_UPDATE_DT,J_vec);


    setpoint->position.x = P_vec[0];
    setpoint->position.y = P_vec[1];
    setpoint->position.z = circAlt;

    setpoint->velocity.x = V_vec[0];
    setpoint->velocity.y = V_vec[1];
    setpoint->velocity.z = 0.0f;

    setpoint->acc.x = A_vec[0];
    setpoint->acc.y = A_vec[1];
    setpoint->acc.z = 0.0f;

    setpoint->jerk.x = J_vec[0];
    setpoint->jerk.y = J_vec[1];
    setpoint->jerk.z = 0.0f;

    setpoint->snap.x = 0.0f;
    setpoint->snap.y = 0.0f;
    setpoint->snap.z = 0.0f;

    setpoint->attitude.yaw = 0.0f;
    setpoint->attitudeRate.yaw = 0.0f;
    setpoint->attitudeAcc.yaw = 0.0f;
}

void updateTrajectory(setpoint_t* setpoint, const uint32_t tick)
{
    nowTick = tick - startTick;

    if(nowTick >= P_control.total_time/GEOMETRIC_UPDATE_DT){
      nowTick = 0;
		  P_control = *P_control.next;
		  diffBezier(&P_control, &V_control);
		  diffBezier(&V_control, &A_control);
		  diffBezier(&A_control, &J_control);
    }

    switch (traj) {
      case circTraj:
        circleUpdate(setpoint, nowTick);
        break;

      case noneTraj:

      default:
        break;
    }
}

/* Init thiago mod bezier trajectory */

float nchoosek (float N, float K){
    int i;
    float result = 1.0;
    for (i = 1; i <= K; i++)
    {
        result *= N - (K - i);
        result /= i;
    }
    return result;
}

void compBezier (traj* P, float time, float* vec){
    int i;
    float n;
    float coef;
    float t;

    vec[0] = 0.0;
    vec[1] = 0.0;
    vec[2] = 0.0;
    t = time / P->total_time;
    n = (float)P->n;
            for(i=0;i<=n;i++){
				        coef = nchoosek(n, i);*pow((1.0 - t), n - (float)i)*(pow(t,(float)i));
                vec[0] = vec[0] + coef*P->Cx[i];
                vec[1] = vec[1] + coef*P->Cy[i];
                vec[2] = vec[2] + coef*P->Cz[i];
            }
}

void diffBezier(traj* P, traj* V) {
	float n = (float)P->n;
	V->n = P->n - 1;
	for (int i = 0; i <= n-1; i++) {

		V->Cx[i] = n*(P->Cx[i + 1] - P->Cx[i]);
		V->Cy[i] = n*(P->Cy[i + 1] - P->Cy[i]);
		V->Cz[i] = n*(P->Cz[i + 1] - P->Cz[i]);
	}


void initBezierTraj(void)
{

  float Cx1[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.103511 , 0.254562 , 0.472964 , 0.730878 , 1.000000 };
  float Cy1[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.458360 , 0.936816 , 1.434853 , 1.807018 , 2.000000 };
  float Cz1[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

  float Cx2[10] = { 1.000000 , 1.269122 , 1.549454 , 1.812693 , 2.030072 , 2.169468 , 2.221992 , 2.192790 , 2.106922 , 2.000000 };
  float Cy2[10] = { 2.000000 , 2.192982 , 2.206781 , 1.988085 , 1.575724 , 1.064367 , 0.667420 , 0.509421 , 0.636384 , 1.000000 };
  float Cz2[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

  float Cx3[10] = { 2.000000 , 1.893078 , 1.765104 , 1.651691 , 1.599009 , 1.646206 , 1.838047 , 2.152899 , 2.559023 , 3.000000 };
  float Cy3[10] = { 1.000000 , 1.363616 , 1.963885 , 2.752496 , 3.586815 , 4.278114 , 4.563007 , 4.403225 , 3.830464 , 3.000000 };
  float Cz3[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

  float Cx4[10] = { 3.000000 , 3.440977 , 3.916808 , 4.371074 , 4.722677 , 5.000000 , 5.000000 , 5.000000 , 5.000000 , 5.000000 };
  float Cy4[10] = { 3.000000 , 2.169536 , 1.081370 , -0.109220 , -1.123373 , -2.000000 , -2.000000 , -2.000000 , -2.000000 , -2.000000 };
  float Cz4[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

  P1.n = 9;
  P2.n = 9;
  P3.n = 9;
  P4.n = 9;

  P1.total_time = 10.0;
  P2.total_time = 10.0;
  P3.total_time = 10.0;
  P4.total_time = 10.0;

  P1.next = &P2;
  P2.next = &P3;
  P3.next = &P4;
  P4.next = &P1;

  for (int i = 0; i <= P1.n; i++) {
		  P1.Cx[i] = Cx1[i];
		  P1.Cy[i] = Cy1[i];
		  P1.Cz[i] = Cz1[i];
	  }

  for (int i = 0; i <= P1.n; i++) {
      P2.Cx[i] = Cx2[i];
      P2.Cy[i] = Cy2[i];
      P2.Cz[i] = Cz2[i];
    }

  for (int i = 0; i <= P1.n; i++) {
      P3.Cx[i] = Cx3[i];
      P3.Cy[i] = Cy3[i];
      P3.Cz[i] = Cz3[i];
    }

  for (int i = 0; i <= P1.n; i++) {
      P4.Cx[i] = Cx4[i];
      P4.Cy[i] = Cy4[i];
      P4.Cz[i] = Cz4[i];
    }


      P_control = P1;
   	  diffBezier(&P_control, &V_control);
   	  diffBezier(&V_control, &A_control);
   	  diffBezier(&A_control, &J_control);

}
/* end thiago's mods*/

PARAM_GROUP_START(circTraj)
PARAM_ADD(PARAM_FLOAT, circRad, &circRad)
PARAM_ADD(PARAM_FLOAT, circFreq, &circFreq)
PARAM_ADD(PARAM_FLOAT, circAlt, &circAlt)
PARAM_GROUP_STOP(circTraj)

PARAM_GROUP_START(trajmode)
PARAM_ADD(PARAM_UINT8, traj, &traj)
PARAM_GROUP_STOP(trajmode)
