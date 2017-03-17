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

  float Cx1[10] = { 0.000000 , -0.000000 , 0.000000 , -0.000000 , 0.000000 , -0.029007 , 1.435693 , 1.844059 , 1.572610 , 1.000000 };
  float Cy1[10] = { 0.000000 , -0.000000 , 0.000000 , -0.000000 , 0.000000 , 0.056000 , 0.054601 , 0.311392 , 0.673186 , 1.000000 };
  float Cz1[10] = { 0.000000 , -0.000000 , 0.000000 , -0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

  float Cx2[10] = { 1.000000 , 0.738483 , 0.414148 , 0.063067 , -0.278595 , -0.559320 , -0.806906 , -0.842265 , -0.721759 , -0.500000 };
  float Cy2[10] = { 1.000000 , 1.149260 , 1.291223 , 1.412556 , 1.500496 , 1.536493 , 1.527404 , 1.419222 , 1.235687 , 1.000000 };
  float Cz2[10] = { -0.000000 , 0.000000 , -0.000000 , 0.000000 , -0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

  float Cx3[10] = { -0.500000 , -0.307071 , -0.037504 , 0.272737 , 0.588690 , 0.863022 , 1.102999 , 1.184279 , 1.139321 , 1.000000 };
  float Cy3[10] = { 1.000000 , 0.794953 , 0.550433 , 0.281715 , 0.003767 , -0.265012 , -0.521360 , -0.725721 , -0.883487 , -1.000000 };
  float Cz3[10] = { -0.000000 , 0.000000 , -0.000000 , 0.000000 , -0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

  float Cx4[10] = { 1.000000 , 0.856021 , 0.611263 , 0.300907 , -0.040530 , -0.369951 , -0.682796 , -0.879524 , -0.978490 , -1.000000 };
  float Cy4[10] = { -1.000000 , -1.120408 , -1.196758 , -1.234944 , -1.240800 , -1.221171 , -1.178539 , -1.125493 , -1.064896 , -1.000000 };
  float Cz4[10] = { -0.000000 , 0.000000 , -0.000000 , 0.000000 , -0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

  float Cx5[10] = { -1.000000 , -1.033505 , -0.879084 , -0.613475 , -0.324888 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };
  float Cy5[10] = { -1.000000 , -0.898917 , -0.787404 , -0.677752 , -0.584537 , -0.500000 , -0.500000 , -0.500000 , -0.500000 , -0.500000 };
  float Cz5[10] = { -0.000000 , 0.000000 , -0.000000 , 0.000000 , -0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

  P1.n = 9;
  P2.n = 9;
  P3.n = 9;
  P4.n = 9;
  P5.n = 9;

  P1.total_time = 6.8860  ;
  P2.total_time = 3.1460;
  P3.total_time = 2.7360 ;
  P4.total_time = 2.8280  ;
  P5.total_time =  4.4040;
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
