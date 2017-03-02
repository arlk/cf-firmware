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

static float nchosk[5][11] = {{1,6,15,20,15,6,1,0,0,0},{1,7,21,35,35,21,7,1,0,0},{1,8,28,56,70,56,28,8,1,0},{1,9,36,84,126,126,84,36,9,1}};

/* static float omg = 0; */
/* static float amp = 0; */
/* static float phase = 0; */

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

static traj_t traj_state = 0;

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
    float pre_comp_coef[9];
    compt_coef (pre_comp_coef,(float)tick*GEOMETRIC_UPDATE_DT,P_control.n,P_control.total_time);
    compBezier(&P_control,(float)tick*GEOMETRIC_UPDATE_DT,P_vec,pre_comp_coef);
    //compBezier(&V_control,(float)tick*GEOMETRIC_UPDATE_DT,V_vec,pre_comp_coef);
   // compBezier(&A_control,(float)tick*GEOMETRIC_UPDATE_DT,A_vec,pre_comp_coef);
   // compBezier(&J_control,(float)tick*GEOMETRIC_UPDATE_DT,J_vec,pre_comp_coef);


    setpoint->position.x = P_vec[0] + sin(tick*GEOMETRIC_UPDATE_DT);
    setpoint->position.y = P_vec[1];
    setpoint->position.z = circAlt;

    setpoint->velocity.x = 0*V_vec[0] + cos(tick*GEOMETRIC_UPDATE_DT);
   // setpoint->velocity.y = V_vec[1];
    setpoint->velocity.z = 0.0f;

setpoint->acc.x = -sin(tick*GEOMETRIC_UPDATE_DT);
   // setpoint->acc.x = A_vec[0];
   // setpoint->acc.y = A_vec[1];
    setpoint->acc.z = 0.0f;
setpoint->jerk.x =-cos(tick*GEOMETRIC_UPDATE_DT);
    //setpoint->jerk.x = J_vec[0];
    //setpoint->jerk.y = J_vec[1];
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

    if(nowTick >= P_control.total_time/GEOMETRIC_UPDATE_DT)
    {
      startTick = tick;
		//  P_control = *P_control.next;
		//  diffBezier(&P_control, &V_control);
		//  diffBezier(&V_control, &A_control);
		 // diffBezier(&A_control, &J_control);
    }

    switch (traj_state) {
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

void compBezier (traj* P, float time, float* vec, float* c){
    int i,n;
    float coef;
    float t;
    t = time/P->total_time;
    n = P->n;
    vec[0] = 0.0;
    vec[1] = 0.0;
    vec[2] = 0.0;
            for(i=0;i<=P->n;i++){
                coef =  nchosk[P->n-6][i]*power2int((1.0f - t), n - i)*c[i];
                vec[0] = vec[0] + coef*P->Cx[i];
                vec[1] = vec[1] + coef*P->Cy[i];
               // vec[2] = vec[2] + coef*P->Cz[i];
            }
}

void diffBezier(traj* P, traj* V) {
	float n = (float)P->n;
	V->n = P->n - 1;
	V-> total_time = P->total_time;
	for (int i = 0; i <= n-1; i++) {
		V->Cx[i] = n*(P->Cx[i + 1] - P->Cx[i])/V-> total_time;
		V->Cy[i] = n*(P->Cy[i + 1] - P->Cy[i])/V-> total_time;
		//V->Cz[i] = n*(P->Cz[i + 1] - P->Cz[i])/V-> total_time;
	}
}

void compt_coef (float *coef,float time, int n, float ttime){
    float t;

    t = time /ttime;

    for(int i=0;i<=n;i++){
     coef[i] =   (power2int(t,i));
    //    coef[i] = powf((1.0f - t), m - (float)i)* (powf(t,(float)i));
    }
}

float power2int (float x, int n){
    float number = 1.0f;
   while(n){
       if(n & 1)
           number *=x;
       n >>=1;
       x *=x;
    }
    return(number);
}

void initBezierTraj(void)
{

  float Cx1[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.046966 , 0.167899 , 0.429725 , 0.735613 , 1.000000 };
  float Cy1[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.010024 , -0.217828 , -0.264585 , -0.175490 , 0.000000 };
  float Cz1[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

  float Cx2[10] = { 1.000000 , 1.130960 , 1.251738 , 1.351935 , 1.421830 , 1.446633 , 1.432744 , 1.337164 , 1.184861 , 1.000000 };
  float Cy2[10] = { 0.000000 , 0.086927 , 0.195051 , 0.318363 , 0.450598 , 0.584525 , 0.719483 , 0.835800 , 0.930266 , 1.000000 };
  float Cz2[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

  float Cx3[10] = { 1.000000 , 0.886831 , 0.761461 , 0.629433 , 0.496180 , 0.368284 , 0.247258 , 0.146770 , 0.064990 , 0.000000 };
  float Cy3[10] = { 1.000000 , 1.042690 , 1.076111 , 1.099603 , 1.112550 , 1.114005 , 1.104423 , 1.080949 , 1.045473 , 1.000000 };
  float Cz3[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

  float Cx4[10] = { 0.000000 , -0.178878 , -0.230554 , -0.194989 , -0.117090 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };
  float Cy4[10] = { 1.000000 , 0.874841 , 0.673957 , 0.439204 , 0.219229 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };
  float Cz4[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

  P1.n = 9;
  P2.n = 9;
  P3.n = 9;
  P4.n = 9;

  P1.total_time = 15.0520;
  P2.total_time = 7.6000 ;
  P3.total_time = 4.5880;
  P4.total_time = 12.7600;

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
   	//  diffBezier(&P_control, &V_control);
   	 // diffBezier(&V_control, &A_control);
   	  //diffBezier(&A_control, &J_control);

}
/* end thiago's mods*/

PARAM_GROUP_START(circTraj)
PARAM_ADD(PARAM_FLOAT, circRad, &circRad)
PARAM_ADD(PARAM_FLOAT, circFreq, &circFreq)
PARAM_ADD(PARAM_FLOAT, circAlt, &circAlt)
PARAM_GROUP_STOP(circTraj)

PARAM_GROUP_START(trajmode)
PARAM_ADD(PARAM_UINT8, trajState, &traj_state)
PARAM_GROUP_STOP(trajmode)
