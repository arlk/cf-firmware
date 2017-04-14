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
static traj P6;
static traj P7;
static traj P8;
static traj P9;
static traj P10;
static traj P_control;
static traj V_control;
static traj A_control;
static traj J_control;
static traj S_control;

static traj Y1;
static traj Y2;
static traj Y3;
static traj Y4;
static traj Y5;
static traj Y6;
static traj Y7;
static traj Y8;
static traj Y9;
static traj Y10;
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

  float Cx1[10] = { -2.000000 , -2.000000 , -2.000000 , -2.000000 , -2.000000 , -2.062372 , -2.092786 , -2.077188 , -1.989812 , -1.833333 };
     float Cy1[10] = { -2.000000 , -2.000000 , -2.000000 , -2.000000 , -2.000000 , -1.824769 , -1.643204 , -1.445896 , -1.282764 , -1.166667 };
     float Cz1[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

     float Cx2[10] = { -1.833333 , -1.773160 , -1.702769 , -1.622311 , -1.532560 , -1.434591 , -1.330264 , -1.221560 , -1.110723 , -1.000000 };
     float Cy2[10] = { -1.166667 , -1.122022 , -1.084333 , -1.054331 , -1.031936 , -1.016982 , -1.007969 , -1.003442 , -1.001463 , -1.000000 };
     float Cz2[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

     float Cx3[10] = { -1.000000 , -0.886816 , -0.773752 , -0.663210 , -0.557598 , -0.459179 , -0.369824 , -0.290902 , -0.223150 , -0.166667 };
     float Cy3[10] = { -1.000000 , -0.998504 , -0.997547 , -0.994957 , -0.988457 , -0.975802 , -0.955102 , -0.924908 , -0.884386 , -0.833333 };
     float Cz3[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

     float Cx4[10] = { -0.166667 , -0.107197 , -0.060220 , -0.025851 , -0.003427 , 0.008514 , 0.012110 , 0.009981 , 0.004991 , -0.000000 };
     float Cy4[10] = { -0.833333 , -0.779581 , -0.714155 , -0.636814 , -0.548088 , -0.449311 , -0.342501 , -0.230173 , -0.115087 , -0.000000 };
     float Cz4[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

     float Cx5[10] = { -0.000000 , -0.004991 , -0.009981 , -0.012110 , -0.008514 , 0.003426 , 0.025851 , 0.060220 , 0.107197 , 0.166667 };
     float Cy5[10] = { 0.000000 , 0.115087 , 0.230173 , 0.342501 , 0.449311 , 0.548088 , 0.636814 , 0.714155 , 0.779581 , 0.833333 };
     float Cz5[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

     float Cx6[10] = { 0.166667 , 0.223150 , 0.290902 , 0.369824 , 0.459179 , 0.557598 , 0.663210 , 0.773752 , 0.886816 , 1.000000 };
     float Cy6[10] = { 0.833333 , 0.884386 , 0.924908 , 0.955102 , 0.975802 , 0.988457 , 0.994957 , 0.997547 , 0.998504 , 1.000000 };
     float Cz6[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

     float Cx7[10] = { 1.000000 , 1.110723 , 1.221560 , 1.330264 , 1.434591 , 1.532560 , 1.622311 , 1.702769 , 1.773160 , 1.833333 };
     float Cy7[10] = { 1.000000 , 1.001463 , 1.003442 , 1.007969 , 1.016982 , 1.031936 , 1.054331 , 1.084333 , 1.122022 , 1.166667 };
     float Cz7[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

     float Cx8[10] = { 1.833333 , 1.989813 , 2.077188 , 2.092786 , 2.062372 , 2.000000 , 2.000000 , 2.000000 , 2.000000 , 2.000000 };
     float Cy8[10] = { 1.166667 , 1.282764 , 1.445896 , 1.643204 , 1.824769 , 2.000000 , 2.000000 , 2.000000 , 2.000000 , 2.000000 };
     float Cz8[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

     float Cx9[10] = { 2.000000 , 2.000000 , 2.000000 , 2.000000 , 2.000000 , 1.333333 , 0.629469 , -0.153140 , -0.827778 , -1.333333 };
     float Cy9[10] = { 2.000000 , 2.000000 , 2.000000 , 2.000000 , 2.000000 , 2.222222 , 2.281643 , 2.175362 , 1.838889 , 1.333333 };
     float Cz9[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

     float Cx10[10] = { -1.333333 , -1.838889 , -2.175362 , -2.281643 , -2.222222 , -2.000000 , -2.000000 , -2.000000 , -2.000000 , -2.000000 };
     float Cy10[10] = { 1.333333 , 0.827778 , 0.153140 , -0.629469 , -1.333333 , -2.000000 , -2.000000 , -2.000000 , -2.000000 , -2.000000 };
     float Cz10[10] = { 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };


     float Cphi1[10] = { 1.912756 , 1.912756 , 1.912756 , 1.912756 , 1.912756 , 1.767529 , 1.557368 , 1.260519 , 0.935435 , 0.638318 };
     float Cphi2[10] = { 0.638318 , 0.524064 , 0.413945 , 0.311158 , 0.218850 , 0.140472 , 0.078681 , 0.035951 , 0.013890 , 0.013214 };
     float Cphi3[10] = { 0.013214 , 0.012523 , 0.034177 , 0.078940 , 0.146599 , 0.235732 , 0.343657 , 0.466355 , 0.598703 , 0.734943 };
     float Cphi4[10] = { 0.734943 , 0.878385 , 1.026142 , 1.171493 , 1.306932 , 1.424788 , 1.518204 , 1.582156 , 1.614132 , 1.614132 };
     float Cphi5[10] = { 1.614132 , 1.614132 , 1.582156 , 1.518204 , 1.424787 , 1.306931 , 1.171492 , 1.026142 , 0.878385 , 0.734943 };
     float Cphi6[10] = { 0.734943 , 0.598702 , 0.466355 , 0.343657 , 0.235732 , 0.146599 , 0.078941 , 0.034178 , 0.012523 , 0.013214 };
     float Cphi7[10] = { 0.013214 , 0.013890 , 0.035951 , 0.078681 , 0.140472 , 0.218849 , 0.311158 , 0.413945 , 0.524063 , 0.638318 };
     float Cphi8[10] = { 0.638318 , 0.935434 , 1.260519 , 1.557368 , 1.767530 , 1.912757 , 1.912757 , 1.912757 , 1.912757 , 1.912757 };
     float Cphi9[10] = { 1.9128, 1.9128, 1.9128, 1.9128, 1.9128, 2.5842, 3.1607, 3.6716, 3.9270, 3.9270 };
     float Cphi10[10] = { 3.9270, 3.9270, 3.6716, 3.1607, 2.5842, 1.9128, 1.9128, 1.9128, 1.9128, 1.9128 };

  P1.n = 9;
  P2.n = 9;
  P3.n = 9;
  P4.n = 9;
  P5.n = 9;
  P6.n = 9;
  P7.n = 9;
  P8.n = 9;
  P9.n = 9;
  P10.n = 9;
  Y1.n = 9;
  Y2.n = 9;
  Y3.n = 9;
  Y4.n = 9;
  Y5.n = 9;
  Y6.n = 9;
  Y7.n = 9;
  Y8.n = 9;
  Y9.n = 9;
  Y10.n = 9;

  P1.total_time = opt_time_stretch*0.2282f;
  P2.total_time = opt_time_stretch*0.0877f;
  P3.total_time = opt_time_stretch*0.0897f;
  P4.total_time = opt_time_stretch*0.0944f;
  P5.total_time = opt_time_stretch*0.0944f;
  P6.total_time = opt_time_stretch*0.0897f;
  P7.total_time = opt_time_stretch*0.0877f;
  P8.total_time = opt_time_stretch*0.2282f;
  P9.total_time = opt_time_stretch*0.5f;
  P10.total_time = opt_time_stretch*0.5f;
  Y1.total_time = opt_time_stretch*0.2282f;
  Y2.total_time = opt_time_stretch*0.0877f;
  Y3.total_time = opt_time_stretch*0.0897f;
  Y4.total_time = opt_time_stretch*0.0944f;
  Y5.total_time = opt_time_stretch*0.0944f;
  Y6.total_time = opt_time_stretch*0.0897f;
  Y7.total_time = opt_time_stretch*0.0877f;
  Y8.total_time = opt_time_stretch*0.2282f;
  Y9.total_time = opt_time_stretch*0.5f;
  Y10.total_time = opt_time_stretch*0.5f;

  P1.next = &P2;
  P2.next = &P3;
  P3.next = &P4;
  P4.next = &P5;
  P5.next = &P6;
  P6.next = &P7;
  P7.next = &P8;
  P8.next = &P9;
  P9.next = &P10;
  P10.next = &P1;
  Y1.next = &Y2;
  Y2.next = &Y3;
  Y3.next = &Y4;
  Y4.next = &Y5;
  Y5.next = &Y6;
  Y6.next = &Y7;
  Y7.next = &Y8;
  Y8.next = &Y9;
  Y9.next = &Y10;
  Y10.next = &Y1;

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
  for (int i = 0; i <= P1.n; i++) {
      P6.Cx[i] = Cx6[i];
      P6.Cy[i] = Cy6[i];
      P6.Cz[i] = Cz6[i];
    }
  for (int i = 0; i <= P1.n; i++) {
      P7.Cx[i] = Cx7[i];
      P7.Cy[i] = Cy7[i];
      P7.Cz[i] = Cz7[i];
    }
  for (int i = 0; i <= P1.n; i++) {
      P8.Cx[i] = Cx8[i];
      P8.Cy[i] = Cy8[i];
      P8.Cz[i] = Cz8[i];
    }
  for (int i = 0; i <= P1.n; i++) {
      P9.Cx[i] = Cx9[i];
      P9.Cy[i] = Cy9[i];
      P9.Cz[i] = Cz9[i];
    }
  for (int i = 0; i <= P1.n; i++) {
      P10.Cx[i] = Cx10[i];
      P10.Cy[i] = Cy10[i];
      P10.Cz[i] = Cz10[i];
    }

  for (int i = 0; i <= Y1.n; i++) {
		  Y1.Cx[i] = Cphi1[i];
	  }
  for (int i = 0; i <= Y1.n; i++) {
      Y2.Cx[i] = Cphi2[i];
    }
  for (int i = 0; i <= Y1.n; i++) {
      Y3.Cx[i] = Cphi3[i];
    }
  for (int i = 0; i <= Y1.n; i++) {
      Y4.Cx[i] = Cphi4[i];
    }
  for (int i = 0; i <= Y1.n; i++) {
      Y5.Cx[i] = Cphi5[i];
    }
  for (int i = 0; i <= Y1.n; i++) {
      Y6.Cx[i] = Cphi6[i];
    }
  for (int i = 0; i <= Y1.n; i++) {
      Y7.Cx[i] = Cphi7[i];
    }
  for (int i = 0; i <= Y1.n; i++) {
      Y8.Cx[i] = Cphi8[i];
    }
  for (int i = 0; i <= Y1.n; i++) {
      Y9.Cx[i] = Cphi9[i];
    }
  for (int i = 0; i <= Y1.n; i++) {
      Y10.Cx[i] = Cphi10[i];
    }

  P_control = P1;
  Y_control = Y1;

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
