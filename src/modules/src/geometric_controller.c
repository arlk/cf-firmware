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
 * Univesity of Illinois at Urbana-Champaign
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
 *                 as described in https://arxiv.org/abs/1003.2005v1
 * */

#include <stdbool.h>

#include "FreeRTOS.h"

#include "geometric_controller.h"

#include "arm_math.h"
#include "flight_math.h"
#include "param.h"
#include "log.h"

static float k_rot_xy = 1.5;
static float k_rot_z = 5.0;
static float k_omg_xy = 0.23;
static float k_omg_z = 0.6;
static float j_xx = 1.39e-5;
static float j_yy = 1.39e-5;
static float j_zz = 2.173e-5;

//FIXME(arun)
static float mom_gain = 30000;


static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

static int16_t rollOutput;
static int16_t pitchOutput;
static int16_t yawOutput;
static float rollMoment;
static float pitchMoment;
static float yawMoment;

static bool isInit;

static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
static inline void mat_sub(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_sub_f32(pSrcA, pSrcB, pDst)); }

void geometricControllerInit()
{
  if(isInit)
    return;

  isInit = true;
}

bool geometricControllerTest()
{
  return isInit;
}

void geometricMomentController(const rotation_t* rotation, const sensorData_t *sensors, rotation_t* rotationDes)
{
  arm_matrix_instance_f32 Rwb = {3, 3, (float *)rotation->vals};
  arm_matrix_instance_f32 Rwd = {3, 3, (float *)rotationDes->vals};

  static float rotationDesTransp[3][3];
  static arm_matrix_instance_f32 Rwdt = {3, 3, (float *)rotationDesTransp};

  static float desOrientation[3][3];
  static arm_matrix_instance_f32 Rdb = {3, 3, (float *)desOrientation};

  static float desOrientationTransp[3][3];
  static arm_matrix_instance_f32 Rdbt = {3, 3, (float *)desOrientationTransp};

  static float orientationErr[3][3];
  static arm_matrix_instance_f32 eR = {3, 3, (float *)orientationErr};

  mat_trans(&Rwd, &Rwdt);
  mat_mult(&Rwdt, &Rwb, &Rdb);
  mat_trans(&Rdb, &Rdbt);
  mat_sub(&Rdb, &Rdbt, &eR);

  static float errRotation[3];
  vee_map(eR.pData, errRotation);


  rollMoment  = (-k_rot_xy*0.5f*errRotation[0] -k_omg_xy*sensors->gyro.x*DEG_TO_RAD)
              + (-j_yy + j_zz)*sensors->gyro.y*sensors->gyro.z;
  pitchMoment = -(-k_rot_xy*0.5f*errRotation[1] -k_omg_xy*sensors->gyro.y*DEG_TO_RAD)
              + (j_xx - j_zz)*sensors->gyro.x*sensors->gyro.z;
  yawMoment   = (-k_rot_z*0.5f*errRotation[2] -k_omg_z*sensors->gyro.z*DEG_TO_RAD)
              + (-j_xx + j_yy)*sensors->gyro.x*sensors->gyro.y;

  rollOutput  = saturateSignedInt16(mom_gain*rollMoment);
  pitchOutput = saturateSignedInt16(mom_gain*pitchMoment);
  yawOutput   = saturateSignedInt16(mom_gain*yawMoment);
}

void geometricControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = rollOutput;
  *pitch = pitchOutput;
  *yaw = yawOutput;
}

LOG_GROUP_START(geom_ctl_out)
LOG_ADD(LOG_FLOAT, roll_moment, &rollMoment)
LOG_ADD(LOG_FLOAT, pitch_moment, &pitchMoment)
LOG_ADD(LOG_FLOAT, yaw_moment, &yawMoment)
LOG_GROUP_STOP(geom_ctl_out)

PARAM_GROUP_START(geom_ctl_gains)
PARAM_ADD(PARAM_FLOAT, kr_xy, &k_rot_xy)
PARAM_ADD(PARAM_FLOAT, kr_z, &k_rot_z)
PARAM_ADD(PARAM_FLOAT, ko_xy, &k_omg_xy)
PARAM_ADD(PARAM_FLOAT, ko_z, &k_omg_z)
PARAM_ADD(PARAM_FLOAT, jxx, &j_xx)
PARAM_ADD(PARAM_FLOAT, jyy, &j_yy)
PARAM_ADD(PARAM_FLOAT, jzz, &j_zz)
PARAM_ADD(PARAM_FLOAT, moment_gain, &mom_gain)
PARAM_GROUP_STOP(geom_ctl_gains)
