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
static traj P[15];
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

    float Y_vec;
    float YD_vec;
    float YDD_vec;
    compBezier1D(&P_control,&Y_vec,pre_comp,pre_comp_n,0,(float)tick*MAIN_LOOP_DT);
    compBezier1D(&V_control,&YD_vec,pre_comp,pre_comp_n,1,(float)tick*MAIN_LOOP_DT);
    compBezier1D(&A_control,&YDD_vec,pre_comp,pre_comp_n,2,(float)tick*MAIN_LOOP_DT);

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
                *vec = *vec + coef*P->Cphi[i];
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

void diffBezier(traj* P, traj* V) {
	float n = (float)P->n;
	V->n = P->n - 1;
	V-> total_time = P->total_time;
	for (int i = 0; i <= n-1; i++) {
		V->Cx[i] = n*(P->Cx[i + 1] - P->Cx[i])/(V->total_time);
		V->Cy[i] = n*(P->Cy[i + 1] - P->Cy[i])/(V->total_time);
    V->Cz[i] = n*(P->Cz[i + 1] - P->Cz[i])/(V->total_time);
    V->Cphi[i] = n*(P->Cphi[i + 1] - P->Cphi[i])/(V->total_time);
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

     float Cx[15][10] = {{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.401108 , 0.805060 , 1.203017 , 1.453186 , 1.500000 },{ 1.500000 , 1.535252 , 1.455192 , 1.236092 , 0.881951 , 0.410621 , -0.117296 , -0.646365 , -1.121105 , -1.500000 },{ -1.500000 , -1.767585 , -1.987367 , -2.144724 , -2.228507 , -2.231956 , -2.154237 , -1.999607 , -1.777257 , -1.500000 },{ -1.500000 , -1.236327 , -0.922995 , -0.571024 , -0.194396 , 0.190822 , 0.567861 , 0.920637 , 1.235083 , 1.500000 },{ 1.500000 , 1.775857 , 1.998010 , 2.153816 , 2.234006 , 2.233745 , 2.153131 , 1.997178 , 1.775248 , 1.500000 },{ 1.500000 , 1.234305 , 0.918929 , 0.565256 , 0.187529 , -0.198040 , -0.574573 , -0.925960 , -1.238148 , -1.500000 },{ -1.500000 , -1.767466 , -1.982415 , -2.132977 , -2.210462 , -2.210312 , -2.132577 , -1.981910 , -1.767080 , -1.500000 },{ -1.500000 , -1.237113 , -0.923605 , -0.570839 , -0.193133 , 0.193133 , 0.570839 , 0.923605 , 1.237113 , 1.500000 },{ 1.500000 , 1.767080 , 1.981910 , 2.132577 , 2.210311 , 2.210462 , 2.132977 , 1.982415 , 1.767466 , 1.500000 },{ 1.500000 , 1.238148 , 0.925960 , 0.574573 , 0.198040 , -0.187529 , -0.565256 , -0.918929 , -1.234305 , -1.500000 },{ -1.500000 , -1.775248 , -1.997178 , -2.153132 , -2.233745 , -2.234007 , -2.153817 , -1.998011 , -1.775857 , -1.500000 },{ -1.500000 , -1.235083 , -0.920637 , -0.567861 , -0.190822 , 0.194396 , 0.571024 , 0.922995 , 1.236327 , 1.500000 },{ 1.500000 , 1.777258 , 1.999608 , 2.154238 , 2.231957 , 2.228508 , 2.144725 , 1.987368 , 1.767585 , 1.500000 },{ 1.500000 , 1.121105 , 0.646366 , 0.117297 , -0.410620 , -0.881950 , -1.236091 , -1.455191 , -1.535252 , -1.500000 },{ -1.500000 , -1.453187 , -1.203018 , -0.805061 , -0.401109 , 0.000000 , 0.000000 , 0.000000 , -0.000000 , -0.000000 }};

     float Cy[15][10] = {{ 1.500000 , 1.774067 , 2.035318 , 2.256137 , 2.407755 , 2.458146 , 2.390541 , 2.198042 , 1.892671 , 1.500000 },{ 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.163714 , 0.405014 , 0.749491 , 1.136046 , 1.500000 },{ 1.500000 , 1.222686 , 0.901832 , 0.546444 , 0.168902 , -0.215471 , -0.590043 , -0.938276 , -1.245501 , -1.500000 },{ -1.500000 , -1.742030 , -1.936373 , -2.072953 , -2.144108 , -2.145487 , -2.076557 , -1.940762 , -1.745224 , -1.500000 },{ -1.500000 , -1.244650 , -0.935427 , -0.583686 , -0.204234 , 0.185584 , 0.567387 , 0.923477 , 1.238402 , 1.500000 },{ 1.500000 , 1.752519 , 1.955349 , 2.097553 , 2.171052 , 2.171501 , 2.098743 , 1.956836 , 1.753646 , 1.500000 },{ 1.500000 , 1.240916 , 0.929190 , 0.576361 , 0.197193 , -0.191269 , -0.571135 , -0.925284 , -1.238813 , -1.500000 },{ -1.500000 , -1.757087 , -1.963462 , -2.107947 , -2.182350 , -2.182350 , -2.107947 , -1.963462 , -1.757087 , -1.500000 },{ -1.500000 , -1.238813 , -0.925284 , -0.571135 , -0.191269 , 0.197193 , 0.576361 , 0.929190 , 1.240916 , 1.500000 },{ 1.500000 , 1.753645 , 1.956836 , 2.098743 , 2.171501 , 2.171052 , 2.097553 , 1.955348 , 1.752518 , 1.500000 },{ 1.500000 , 1.238402 , 0.923477 , 0.567387 , 0.185583 , -0.204235 , -0.583686 , -0.935427 , -1.244650 , -1.500000 },{ -1.500000 , -1.745223 , -1.940761 , -2.076557 , -2.145486 , -2.144108 , -2.072953 , -1.936373 , -1.742029 , -1.500000 },{ -1.500000 , -1.245501 , -0.938277 , -0.590043 , -0.215471 , 0.168902 , 0.546444 , 0.901832 , 1.222686 , 1.500000 },{ 1.500000 , 1.892671 , 2.198042 , 2.390540 , 2.458145 , 2.407754 , 2.256136 , 2.035318 , 1.774067 , 1.500000 },{ 1.500000 , 1.136046 , 0.749492 , 0.405014 , 0.163714 , -0.000000 , -0.000000 , -0.000000 , -0.000000 , -0.000000 }};

     float Cphi[15][10] = {{ 0.387517 , 0.387517 , 0.387517 , 0.387517 , 0.387517 , 0.484944 , 0.629970 , 0.848591 , 1.124638 , 1.442873 },{ 1.442873 , 1.682513 , 1.946076 , 2.227055 , 2.519246 , 2.813787 , 3.107084 , 3.394400 , 3.673822 , 3.944843 },{ 3.944843 , 4.136245 , 4.323456 , 4.506298 , 4.684941 , 4.859964 , 5.032099 , 5.202291 , 5.371487 , 5.540560 },{ 5.540560 , 5.701349 , 5.862026 , 6.023344 , 6.185991 , 6.350510 , 6.517232 , 6.686217 , 6.857269 , 7.029999 },{ 7.029999 , 7.209861 , 7.391543 , 7.574606 , 7.758389 , 7.942092 , 8.124958 , 8.306448 , 8.486364 , 8.664800 },{ 8.664800 , 8.837043 , 9.007909 , 9.177483 , 9.346113 , 9.514372 , 9.682906 , 9.852252 , 10.022691 , 10.194258 },{ 10.194258 , 10.369504 , 10.545927 , 10.723563 , 10.902176 , 11.081260 , 11.260195 , 11.438457 , 11.615773 , 11.792126 },{ 11.792126 , 11.965710 , 12.138361 , 12.310059 , 12.481025 , 12.651716 , 12.822682 , 12.994380 , 13.167031 , 13.340615 },{ 13.340615 , 13.516968 , 13.694285 , 13.872546 , 14.051481 , 14.230565 , 14.409179 , 14.586814 , 14.763237 , 14.938483 },{ 14.938483 , 15.110050 , 15.280489 , 15.449835 , 15.618369 , 15.786628 , 15.955258 , 16.124832 , 16.295697 , 16.467941 },{ 16.467941 , 16.646377 , 16.826293 , 17.007784 , 17.190649 , 17.374353 , 17.558136 , 17.741199 , 17.922881 , 18.102743 },{ 18.102743 , 18.275473 , 18.446525 , 18.615510 , 18.782231 , 18.946750 , 19.109397 , 19.270714 , 19.431392 , 19.592181 },{ 19.592181 , 19.761253 , 19.930449 , 20.100642 , 20.272777 , 20.447800 , 20.626443 , 20.809285 , 20.996497 , 21.187898 },{ 21.187898 , 21.458919 , 21.738342 , 22.025657 , 22.318954 , 22.613495 , 22.905685 , 23.186665 , 23.450227 , 23.689867 },{ 23.689867 , 24.008102 , 24.284149 , 24.502771 , 24.647797 , 24.745224 , 24.745224 , 24.745224 , 24.745224 , 24.745224 }};



  opt_time_stretch = 60.0f;
  P[0].total_time = opt_time_stretch*0.1087f;
  P[1].total_time = opt_time_stretch*0.0818f;
  P[2].total_time = opt_time_stretch*0.0578f;
  P[3].total_time = opt_time_stretch*0.0550f;
  P[4].total_time = opt_time_stretch*0.0572f;
  P[5].total_time = opt_time_stretch*0.0553f;
  P[6].total_time = opt_time_stretch*0.0564f;
  P[7].total_time = opt_time_stretch*0.0556f;
  P[8].total_time = opt_time_stretch*0.0564f;
  P[9].total_time = opt_time_stretch*0.0553f;
  P[10].total_time = opt_time_stretch*0.0572f;
  P[11].total_time = opt_time_stretch*0.0550f;
  P[12].total_time = opt_time_stretch*0.0578f;
  P[13].total_time = opt_time_stretch*0.0818f;
  P[14].total_time = opt_time_stretch*0.1087f;

  for (int j = 0; j <= 14; j++) {
    P[j].n = 9;
    if (j < 14) {
      P[j].next = &P[j+1];
    } else {
      P[j].next = &P[0];
    }
    for (int i = 0; i <= P[j].n; i++) {
        P[j].Cx[i] = Cx[j][i];
        P[j].Cy[i] = Cy[j][i];
        P[j].Cz[i] = 1.25f;
        P[j].Cphi[i] = Cphi[j][i];
      }
  }

  P_control = P[0];

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
