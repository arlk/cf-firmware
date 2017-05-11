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

const uint8_t uiuc_blue[] = {0, 0, 70};
const uint8_t uiuc_orange[] = {255, 30, 0};

/* static float omg = 0; */
/* static float amp = 0; */
/* static float phase = 0; */

#define BETA 0.999400359f

/* Thiago's Bezier variables*/
static traj P[12];
static traj P_control;
static traj V_control;
static traj A_control;
static traj J_control;
static traj S_control;

static traj Y[12];
static traj Y_control;
static traj YD_control;
static traj YDD_control;

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

    float Y_vec;
    float YD_vec;
    float YDD_vec;
    compBezier1D(&Y_control,&Y_vec,pre_comp,pre_comp_n,0,(float)tick*MAIN_LOOP_DT);
    compBezier1D(&YD_control,&YD_vec,pre_comp,pre_comp_n,1,(float)tick*MAIN_LOOP_DT);
    compBezier1D(&YDD_control,&YDD_vec,pre_comp,pre_comp_n,2,(float)tick*MAIN_LOOP_DT);


    setpoint->position.x = P_vec[0];
    setpoint->position.y = P_vec[1];
    setpoint->position.z = P_vec[2];

    setpoint->velocity.x = V_vec[0];
    setpoint->velocity.y = V_vec[1];
    setpoint->velocity.z = V_vec[2];

    setpoint->acc.x = A_vec[0];
    setpoint->acc.y = A_vec[1];
    setpoint->acc.z = A_vec[2];

    setpoint->jerk.x = J_vec[0];
    setpoint->jerk.y = J_vec[1];
    setpoint->jerk.z = J_vec[2];

    setpoint->snap.x = S_vec[0];
    setpoint->snap.y = S_vec[1];
    setpoint->snap.z = S_vec[2];

    setpoint->attitude.yaw = Y_vec;
    setpoint->attitudeRate.yaw = YD_vec;
    setpoint->attitudeAcc.yaw = YDD_vec;
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

      Y_control = *Y_control.next;
      diffBezier1D(&Y_control, &YD_control);
      diffBezier1D(&YD_control, &YDD_control);
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
                vec[2] = vec[2] + coef*P->Cz[i];
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
    V->Cz[i] = n*(P->Cz[i + 1] - P->Cz[i])/V-> total_time;
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
     float Cx[12][10] = {{ 1.500000 , 1.500000 , 1.500000 , 1.500000 , 1.500000 , 1.040673 , 0.473236 , -0.238618 , -0.933934 , -1.500000 },{ -1.500000 , -1.853226 , -2.156125 , -2.381312 , -2.508718 , -2.518509 , -2.412005 , -2.194083 , -1.882123 , -1.500000 },{ -1.500000 , -1.212150 , -0.884487 , -0.527216 , -0.152636 , 0.225232 , 0.592229 , 0.934625 , 1.240431 , 1.500000 },{ 1.500000 , 1.785913 , 2.015727 , 2.176547 , 2.258928 , 2.258030 , 2.174111 , 2.012584 , 1.783416 , 1.500000 },{ 1.500000 , 1.231265 , 0.913756 , 0.558894 , 0.180868 , -0.204230 , -0.579682 , -0.929558 , -1.239986 , -1.500000 },{ -1.500000 , -1.764790 , -1.977298 , -2.125943 , -2.202269 , -2.201880 , -2.124899 , -1.975962 , -1.763743 , -1.500000 },{ -1.500000 , -1.236257 , -0.920991 , -0.565960 , -0.186009 , 0.201883 , 0.580025 , 0.931592 , 1.242034 , 1.500000 },{ 1.500000 , 1.753314 , 1.956028 , 2.097395 , 2.169644 , 2.168819 , 2.095194 , 1.953239 , 1.751159 , 1.500000 },{ 1.500000 , 1.235121 , 0.915655 , 0.554557 , 0.168298 , -0.224306 , -0.603873 , -0.952356 , -1.254742 , -1.500000 },{ -1.500000 , -1.722660 , -1.898234 , -2.018468 , -2.077813 , -2.074069 , -2.008422 , -1.885586 , -1.712965 , -1.500000 },{ -1.500000 , -1.217287 , -0.863477 , -0.460654 , -0.037225 , 0.377290 , 0.751872 , 1.069226 , 1.318405 , 1.500000 },{ 1.500000 , 1.791017 , 1.908460 , 1.854751 , 1.708378 , 1.500000 , 1.500000 , 1.500000 , 1.500000 , 1.500000 }};

     float Cy[12][10] = {{ 1.500000 , 1.500000 , 1.500000 , 1.500000 , 1.500000 , 1.708378 , 1.854751 , 1.908461 , 1.791017 , 1.500000 },{ 1.500000 , 1.318405 , 1.069225 , 0.751872 , 0.377290 , -0.037226 , -0.460655 , -0.863478 , -1.217288 , -1.500000 },{ -1.500000 , -1.712965 , -1.885585 , -2.008421 , -2.074068 , -2.077812 , -2.018467 , -1.898233 , -1.722659 , -1.500000 },{ -1.500000 , -1.254742 , -0.952356 , -0.603873 , -0.224306 , 0.168298 , 0.554557 , 0.915655 , 1.235121 , 1.500000 },{ 1.500000 , 1.751158 , 1.953238 , 2.095194 , 2.168818 , 2.169643 , 2.097394 , 1.956027 , 1.753313 , 1.500000 },{ 1.500000 , 1.242034 , 0.931592 , 0.580025 , 0.201883 , -0.186009 , -0.565960 , -0.920991 , -1.236257 , -1.500000 },{ -1.500000 , -1.763743 , -1.975962 , -2.124899 , -2.201880 , -2.202269 , -2.125943 , -1.977298 , -1.764790 , -1.500000 },{ -1.500000 , -1.239985 , -0.929557 , -0.579681 , -0.204230 , 0.180868 , 0.558894 , 0.913757 , 1.231265 , 1.500000 },{ 1.500000 , 1.783415 , 2.012583 , 2.174109 , 2.258028 , 2.258926 , 2.176545 , 2.015726 , 1.785913 , 1.500000 },{ 1.500000 , 1.240432 , 0.934626 , 0.592229 , 0.225232 , -0.152636 , -0.527216 , -0.884487 , -1.212150 , -1.500000 },{ -1.500000 , -1.882123 , -2.194083 , -2.412005 , -2.518509 , -2.508718 , -2.381312 , -2.156125 , -1.853226 , -1.500000 },{ -1.500000 , -0.933934 , -0.238619 , 0.473236 , 1.040673 , 1.500000 , 1.500000 , 1.500000 , 1.500000 , 1.500000 }};

     float Cz[12][10] = {{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 }};

     float Cphi[12][10] = {{ 2.715700 , 2.715700 , 2.715700 , 2.715700 , 2.715700 , 2.764666 , 2.867862 , 3.050819 , 3.306462 , 3.616460 },{ 3.616460 , 3.809899 , 4.024502 , 4.255816 , 4.497681 , 4.742200 , 4.982665 , 5.214096 , 5.434930 , 5.646214 },{ 5.646214 , 5.805372 , 5.959111 , 6.107878 , 6.252961 , 6.396364 , 6.540333 , 6.686867 , 6.837291 , 6.992193 },{ 6.992193 , 7.162817 , 7.338872 , 7.521143 , 7.709323 , 7.901887 , 8.096566 , 8.291044 , 8.483579 , 8.673175 },{ 8.673175 , 8.852951 , 9.030084 , 9.203726 , 9.373632 , 9.540300 , 9.704781 , 9.868352 , 10.032172 , 10.197123 },{ 10.197123 , 10.365103 , 10.534255 , 10.705508 , 10.879487 , 11.056333 , 11.235693 , 11.416853 , 11.598913 , 11.780972 },{ 11.780972 , 11.963032 , 12.145091 , 12.326251 , 12.505612 , 12.682457 , 12.856437 , 13.027690 , 13.196842 , 13.364822 },{ 13.364822 , 13.529773 , 13.693593 , 13.857164 , 14.021646 , 14.188313 , 14.358220 , 14.531862 , 14.708996 , 14.888771 },{ 14.888771 , 15.078367 , 15.270901 , 15.465379 , 15.660058 , 15.852622 , 16.040801 , 16.223071 , 16.399127 , 16.569750 },{ 16.569750 , 16.724652 , 16.875077 , 17.021611 , 17.165580 , 17.308984 , 17.454066 , 17.602834 , 17.756573 , 17.915732 },{ 17.915732 , 18.127016 , 18.347850 , 18.579281 , 18.819746 , 19.064265 , 19.306129 , 19.537444 , 19.752047 , 19.945486 },{ 19.945486 , 20.255483 , 20.511126 , 20.694083 , 20.797279 , 20.846245 , 20.846245 , 20.846245 , 20.846245 , 20.846245 }};

  opt_time_stretch = 21.0f;
  P[0].total_time = opt_time_stretch*0.1388f;
  P[1].total_time = opt_time_stretch*0.0866f;
  P[2].total_time = opt_time_stretch*0.0652f;
  P[3].total_time = opt_time_stretch*0.0719f;
  P[4].total_time = opt_time_stretch*0.0681f;
  P[5].total_time = opt_time_stretch*0.0694f;
  P[6].total_time = opt_time_stretch*0.0694f;
  P[7].total_time = opt_time_stretch*0.0681f;
  P[8].total_time = opt_time_stretch*0.0719f;
  P[9].total_time = opt_time_stretch*0.0652f;
  P[10].total_time = opt_time_stretch*0.0866f;
  Y[11].total_time = opt_time_stretch*0.1388f;
  Y[0].total_time = opt_time_stretch*0.1388f;
  Y[1].total_time = opt_time_stretch*0.0866f;
  Y[2].total_time = opt_time_stretch*0.0652f;
  Y[3].total_time = opt_time_stretch*0.0719f;
  Y[4].total_time = opt_time_stretch*0.0681f;
  Y[5].total_time = opt_time_stretch*0.0694f;
  Y[6].total_time = opt_time_stretch*0.0694f;
  Y[7].total_time = opt_time_stretch*0.0681f;
  Y[8].total_time = opt_time_stretch*0.0719f;
  Y[9].total_time = opt_time_stretch*0.0652f;
  Y[10].total_time = opt_time_stretch*0.0866f;
  Y[11].total_time = opt_time_stretch*0.1388f;

  for (int j = 0; j <= 11; j++) {
    P[j].n = 9;
    if (j < 15) {
      P[j].next = &P[j+1];
    } else {
      P[j].next = &P[0];
    }
    for (int i = 0; i <= P[j].n; i++) {
        P[j].Cx[i] = Cx[j][i];
        P[j].Cy[i] = Cy[j][i];
        P[j].Cz[i] = Cz[j][i] + 1.75f;
      }
  }

  for (int j = 0; j <= 11; j++) {
    Y[j].n = 9;
    if (j < 15) {
      Y[j].next = &Y[j+1];
    } else {
      Y[j].next = &Y[0];
    }
    for (int i = 0; i <= P[j].n; i++) {
        Y[j].Cx[i] = Cphi[j][i];
      }
  }

  P_control = P[0];
  Y_control = Y[0];

  diffBezier(&P_control, &V_control);
  diffBezier(&V_control, &A_control);
  diffBezier(&A_control, &J_control);
  diffBezier(&J_control, &S_control);

  diffBezier1D(&Y_control, &YD_control);
  diffBezier1D(&YD_control, &YDD_control);
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
