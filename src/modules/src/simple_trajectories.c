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
static traj P5;
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
  initBezierTraj();
  isInit = true;
  startTick = tick;
}

void circleUpdate(setpoint_t* setpoint, const uint32_t tick)
{
    float P_vec[3];
     float V_vec[3];
     float A_vec[3];
     float J_vec[3];
     float pre_comp[9];
     float pre_comp_n[9];
    // compt_coef (pre_comp,pre_comp_n,(float)tick*GEOMETRIC_UPDATE_DT,P_control.n,P_control.total_time);
     compBezier(&P_control,P_vec,pre_comp,pre_comp_n,0,(float)tick*GEOMETRIC_UPDATE_DT);
    compBezier(&V_control,V_vec,pre_comp,pre_comp_n,1,(float)tick*GEOMETRIC_UPDATE_DT);
    compBezier(&A_control,A_vec,pre_comp,pre_comp_n,2,(float)tick*GEOMETRIC_UPDATE_DT);
    compBezier(&J_control,J_vec,pre_comp,pre_comp_n,3,(float)tick*GEOMETRIC_UPDATE_DT);


    setpoint->position.x = P_vec[0] ;
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

    if(nowTick >= P_control.total_time/GEOMETRIC_UPDATE_DT)
    {
      startTick = tick;
      P_control = *P_control.next;
      diffBezier(&P_control, &V_control);
      diffBezier(&V_control, &A_control);
	 diffBezier(&A_control, &J_control);
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

void compBezier (traj* P, float* vec, float* c,float*cn,int bias,float t){
    int i;
    float coef;
    float time = t/P->total_time;
    vec[0] = 0.0;
    vec[1] = 0.0;
    vec[2] = 0.0;
            for(i=0;i<=P->n;i++){

                coef =  nchosk[P->n-6][i];//*cn[i+bias]*c[i];
                coef = coef*powf(time,(float)i)*powf((1.0f - time), P->n - (float)i);
                vec[0] = vec[0] + coef*P->Cx[i];
                vec[1] = vec[1] + coef*P->Cy[i];
                //vec[2] = vec[2] + coef*P->Cz[i];
            }
}

void diffBezier(traj* P, traj* V) {
	float n = (float)P->n;
	V->n = P->n - 1;
	V-> total_time = P->total_time;
	for (int i = 0; i <= n-1; i++) {
		V->Cx[i] = n*(P->Cx[i + 1] - P->Cx[i])/(V-> total_time);
		V->Cy[i] = n*(P->Cy[i + 1] - P->Cy[i])/(V-> total_time);
		//V->Cz[i] = n*(P->Cz[i + 1] - P->Cz[i])/V-> total_time;
	}
}

void compt_coef (float *coef1,float *coef2,float time, int n, float ttime){
    float t;
    t = time /ttime;
    for(int i=0;i<=n;i++){
     coef1[i] =   (power2int(t,i));
     coef2[i] = power2int((1.0f - t), n - i);
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

  float Cx1[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.642351 , 1.245854 , 1.785221 , 2.052672 , 2.000000 };
     float Cy1[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.288092 , 0.649037 , 1.110233 , 1.582029 , 2.000000 };
     float Cz1[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

     float Cx2[10] = { 2.000000 , 1.955098 , 1.677554 , 1.137502 , 0.389356 , -0.466283 , -1.235185 , -1.782885 , -2.036581 , -2.000000 };
     float Cy2[10] = { 2.000000 , 2.356314 , 2.673511 , 2.911679 , 3.044226 , 3.046066 , 2.929275 , 2.702962 , 2.386361 , 2.000000 };
     float Cz2[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

     float Cx3[10] = { -2.000000 , -1.965693 , -1.676078 , -1.134227 , -0.402412 , 0.402414 , 1.134229 , 1.676079 , 1.965693 , 2.000000 };
     float Cy3[10] = { 2.000000 , 1.637659 , 1.213962 , 0.745845 , 0.251246 , -0.251247 , -0.745845 , -1.213962 , -1.637659 , -2.000000 };
     float Cz3[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

     float Cx4[10] = { 2.000000 , 2.036581 , 1.782884 , 1.235184 , 0.466283 , -0.389356 , -1.137502 , -1.677553 , -1.955098 , -2.000000 };
     float Cy4[10] = { -2.000000 , -2.386360 , -2.702962 , -2.929274 , -3.046065 , -3.044225 , -2.911678 , -2.673510 , -2.356313 , -2.000000 };
     float Cz4[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

     float Cx5[10] = { -2.000000 , -2.052672 , -1.785222 , -1.245854 , -0.642352 , -0.000000 , -0.000000 , -0.000000 , -0.000000 , -0.000000 };
     float Cy5[10] = { -2.000000 , -1.582029 , -1.110233 , -0.649037 , -0.288092 , -0.000000 , -0.000000 , -0.000000 , -0.000000 , -0.000000 };
     float Cz5[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

  P1.n = 9;
  P2.n = 9;
  P3.n = 9;
  P4.n = 9;
  P5.n = 9;

  float opt_time_stretch = 8.2766;
  P1.total_time = opt_time_stretch*0.2220f;
  P2.total_time = opt_time_stretch*0.1893f;
  P3.total_time = opt_time_stretch*0.1775f;
  P4.total_time = opt_time_stretch*0.1893f;
  P5.total_time = opt_time_stretch*0.2220f;

  P1.next = &P2;
  P2.next = &P3;
  P3.next = &P4;
  P4.next = &P5;
  P5.next = &P1;

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
    for (int i = 0; i <= P1.n; i++) {
        P5.Cx[i] = Cx5[i];
        P5.Cy[i] = Cy5[i];
        P5.Cz[i] = Cz5[i];
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
PARAM_ADD(PARAM_UINT8, trajState, &traj_state)
PARAM_GROUP_STOP(trajmode)
