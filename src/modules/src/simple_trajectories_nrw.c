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

#include "ledring12.h"

static bool isInit;
static uint32_t startTick = 0;
static uint32_t nowTick = 0;

static float circRad = 0.8f;
static float circFreq = 0.2f;
static float circAlt = 2.0f;
/* static float opt_time_stretch = 8.2766; */
static float opt_time_stretch = 12.5f;

static float nchosk[5][11] = {{1,6,15,20,15,6,1,0,0,0},{1,7,21,35,35,21,7,1,0,0},{1,8,28,56,70,56,28,8,1,0},{1,9,36,84,126,126,84,36,9,1}};

/* static float omg = 0; */
/* static float amp = 0; */
/* static float phase = 0; */

#define BETA 0.999400359f

/* Thiago's Bezier variables*/
static traj P[16];
static traj P_control;
static traj V_control;
static traj A_control;
static traj J_control;
static traj S_control;

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

void trajectoryDeinit(void)
{
  if(!isInit)
    return;
  isInit = false;
  startTick = 0;
}

void circleUpdate(setpoint_t* setpoint, const uint32_t tick)
{
     float P_vec[3];
     float V_vec[3];
     float A_vec[3];
     float J_vec[3];
     float S_vec[3];
     float pre_comp[9];
     float pre_comp_n[9];
    // compt_coef (pre_comp,pre_comp_n,(float)tick*MAIN_LOOP_DT,P_control.n,P_control.total_time);
    compBezier(&P_control,P_vec,pre_comp,pre_comp_n,0,(float)tick*MAIN_LOOP_DT);
    compBezier(&V_control,V_vec,pre_comp,pre_comp_n,1,(float)tick*MAIN_LOOP_DT);
    compBezier(&A_control,A_vec,pre_comp,pre_comp_n,2,(float)tick*MAIN_LOOP_DT);
    compBezier(&J_control,J_vec,pre_comp,pre_comp_n,3,(float)tick*MAIN_LOOP_DT);
    compBezier(&S_control,S_vec,pre_comp,pre_comp_n,4,(float)tick*MAIN_LOOP_DT);


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

    setpoint->snap.x = S_vec[0];
    setpoint->snap.y = S_vec[1];
    setpoint->snap.z = 0.0f;

    setpoint->attitude.yaw = 0.0f;
    setpoint->attitudeRate.yaw = 0.0f;
    setpoint->attitudeAcc.yaw = 0.0f;
}

void updateTrajectory(setpoint_t* setpoint, const uint32_t tick)
{
    nowTick = tick - startTick;

    if(nowTick >= P_control.total_time/MAIN_LOOP_DT)
    {
      startTick = tick;
      P_control = *P_control.next;
      diffBezier(&P_control, &V_control);
      diffBezier(&V_control, &A_control);
      diffBezier(&A_control, &J_control);
      diffBezier(&J_control, &S_control);
      ledWriteQueue(&P_control.led_state);
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

void compBezier1D (traj* P, float* vec, float* c,float*cn,int bias,float t){
    int i;
    float coef;
    float time = t/P->total_time;
    *vec = 0.0;
            for(i=0;i<=P->n;i++){

                coef =  nchosk[P->n-6][i];//*cn[i+bias]*c[i];
                coef = coef*powf(time,(float)i)*powf((1.0f - time), P->n - (float)i);
                *vec = *vec + coef*P->Cx[i];
            }
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

void diffBezier1D(traj* P, traj* V) {
	float n = (float)P->n;
	V->n = P->n - 1;
	V-> total_time = P->total_time;
	for (int i = 0; i <= n-1; i++) {
		V->Cx[i] = n*(P->Cx[i + 1] - P->Cx[i])/(V-> total_time);
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

     float Cx[16][10] = {{ -2.000000 , -2.000000 , -2.000000 , -2.000000 , -2.000000 , -2.026735 , -2.037715 , -2.025470 , -1.978966 , -1.900000 },{ -1.900000 , -1.840312 , -1.762078 , -1.666072 , -1.557259 , -1.442741 , -1.333928 , -1.237922 , -1.159688 , -1.100000 },{ -1.100000 , -1.021034 , -0.974530 , -0.962285 , -0.973265 , -1.000000 , -1.000000 , -1.000000 , -1.000000 , -1.000000 },{ -1.000000 , -1.000000 , -1.000000 , -1.000000 , -1.000000 , -0.500000 , -0.500000 , -0.500000 , -0.500000 , -0.500000 },{ -0.500000 , -0.500000 , -0.500000 , -0.500000 , -0.500000 , -0.399291 , -0.268701 , -0.097394 , 0.083548 , 0.250000 },{ 0.250000 , 0.413311 , 0.562673 , 0.675300 , 0.734850 , 0.726808 , 0.660398 , 0.546385 , 0.403535 , 0.250000 },{ 0.250000 , 0.192060 , 0.132599 , 0.072591 , 0.013001 , -0.045252 , -0.101302 , -0.154472 , -0.204173 , -0.250000 },{ -0.250000 , -0.419403 , -0.535865 , -0.578938 , -0.562831 , -0.500000 , -0.500000 , -0.500000 , -0.500000 , -0.500000 },{ -0.500000 , -0.500000 , -0.500000 , -0.500000 , -0.500000 , 0.250000 , 0.250000 , 0.250000 , 0.250000 , 0.250000 },{ 0.250000 , 0.250000 , 0.250000 , 0.250000 , 0.250000 , 0.500000 , 0.500000 , 0.500000 , 0.500000 , 0.500000 },{ 0.500000 , 0.500000 , 0.500000 , 0.500000 , 0.500000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 },{ 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.045194 , 1.102323 , 1.176878 , 1.256394 , 1.333000 },{ 1.333000 , 1.388777 , 1.443011 , 1.492664 , 1.535987 , 1.571324 , 1.600188 , 1.624122 , 1.645574 , 1.667000 },{ 1.667000 , 1.688426 , 1.709826 , 1.733656 , 1.762374 , 1.797540 , 1.840694 , 1.890196 , 1.944309 , 2.000000 },{ 2.000000 , 2.076488 , 2.155951 , 2.230524 , 2.287714 , 2.333000 , 2.333000 , 2.333000 , 2.333000 , 2.333000 },{ 2.333000 , 2.333000 , 2.333000 , 2.333000 , 2.333000 , -2.000000 , -2.000000 , -2.000000 , -2.000000 , -2.000000 }};

     float Cy[16][10] = {{ -1.000000 , -1.000000 , -1.000000 , -1.000000 , -1.000000 , -0.388484 , 0.177094 , 0.689367 , 0.971422 , 1.000000 },{ 1.000000 , 1.021601 , 0.898385 , 0.620306 , 0.227479 , -0.227479 , -0.620306 , -0.898385 , -1.021601 , -1.000000 },{ -1.000000 , -0.971422 , -0.689367 , -0.177094 , 0.388484 , 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 },{ 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 },{ 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.053295 , 1.090900 , 1.104343 , 1.074282 , 1.000000 },{ 1.000000 , 0.927120 , 0.811672 , 0.652981 , 0.467629 , 0.283192 , 0.138387 , 0.049510 , 0.010277 , -0.000000 },{ 0.000000 , -0.003878 , -0.003633 , -0.000375 , 0.004490 , 0.009486 , 0.012771 , 0.013091 , 0.009115 , -0.000000 },{ 0.000000 , -0.033693 , -0.137595 , -0.354202 , -0.634459 , -1.000000 , -1.000000 , -1.000000 , -1.000000 , -1.000000 },{ -1.000000 , -1.000000 , -1.000000 , -1.000000 , -1.000000 , -0.000000 , -0.000000 , -0.000000 , -0.000000 , -0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , -1.000000 , -1.000000 , -1.000000 , -1.000000 , -1.000000 },{ -1.000000 , -1.000000 , -1.000000 , -1.000000 , -1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 },{ 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 , 0.382471 , -0.187946 , -0.700815 , -0.978867 , -1.000000 },{ -1.000000 , -1.015387 , -0.894573 , -0.629028 , -0.253829 , 0.179203 , 0.570925 , 0.856977 , 1.000002 , 1.000000 },{ 1.000000 , 0.999998 , 0.856974 , 0.570926 , 0.179210 , -0.253816 , -0.629011 , -0.894558 , -1.015378 , -1.000000 },{ -1.000000 , -0.978879 , -0.700835 , -0.187966 , 0.382459 , 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 },{ 1.000000 , 1.000000 , 1.000000 , 1.000000 , 1.000000 , -1.000000 , -1.000000 , -1.000000 , -1.000000 , -1.000000 }};

     float Cz[16][10] = {{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 }};


  opt_time_stretch = 8.0f;
  P[0].total_time = opt_time_stretch*0.3629f;
  P[1].total_time = opt_time_stretch*0.2743f;
  P[2].total_time = opt_time_stretch*0.3629f;
  P[0].led_state = 7;
  P[1].led_state = 7;
  P[2].led_state = 7;

  P[3].total_time = 5.0f;

  opt_time_stretch = 5.5f;
  P[4].total_time = opt_time_stretch*0.2688f;
  P[5].total_time = opt_time_stretch*0.2637f;
  P[6].total_time = opt_time_stretch*0.0995f;
  P[7].total_time = opt_time_stretch*0.3679f;
  P[4].led_state = 7;
  P[5].led_state = 7;
  P[6].led_state = 7;
  P[7].led_state = 7;

  P[8].total_time = 5.0f;

  P[9].total_time = 3.0f;
  P[9].led_state = 7;

  P[10].total_time = 5.0f;

  opt_time_stretch = 10.0f;
  P[11].total_time = opt_time_stretch*0.2893f;
  P[12].total_time = opt_time_stretch*0.2107f;
  P[13].total_time = opt_time_stretch*0.2107f;
  P[14].total_time = opt_time_stretch*0.2893f;
  P[11].led_state = 7;
  P[12].led_state = 7;
  P[13].led_state = 7;
  P[14].led_state = 7;

  P[15].total_time = 10.0f;

  for (int j = 0; j <= 15; j++) {
    P[j].n = 9;
    if (j < 15) {
      P[j].next = &P[j+1];
    } else {
      P[j].next = &P[0];
    }
    for (int i = 0; i <= P[j].n; i++) {
        P[j].Cx[i] = Cx[j][i];
        P[j].Cy[i] = Cy[j][i];
        P[j].Cz[i] = Cz[j][i];
      }
  }

  P_control = P[1];
  ledWriteQueue(&P_control.led_state);

  diffBezier(&P_control, &V_control);
  diffBezier(&V_control, &A_control);
  diffBezier(&A_control, &J_control);
  diffBezier(&J_control, &S_control);
}
/* end thiago's mods*/

PARAM_GROUP_START(circTraj)
PARAM_ADD(PARAM_FLOAT, circRad, &circRad)
PARAM_ADD(PARAM_FLOAT, circFreq, &circFreq)
PARAM_ADD(PARAM_FLOAT, circAlt, &circAlt)
PARAM_ADD(PARAM_FLOAT, timeStr, &opt_time_stretch)
PARAM_GROUP_STOP(circTraj)

PARAM_GROUP_START(trajmode)
PARAM_ADD(PARAM_UINT8, trajState, &traj_state)
PARAM_GROUP_STOP(trajmode)
