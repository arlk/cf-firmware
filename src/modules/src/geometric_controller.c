/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * attitude_pid_controller.c: Attitude controler using PID correctors
 */
#include <stdbool.h>

#include "FreeRTOS.h"

#include "geometric_controller.h"

#include "arm_math.h"
#include "flight_math.h"
#include "param.h"
#include "log.h"

// FIXME
#define K_ROT_XY (float)1.5
#define K_ROT_Z (float)5.0
#define K_OMG_XY (float)0.23
#define K_OMG_Z (float)0.6
#define J_XX (float)1.39e-5
#define J_YY (float)1.39e-5
#define J_ZZ (float)2.173e-5
#define MOM_GAIN (float)30000


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


  rollMoment = (0.5f*(-K_ROT_XY*errRotation[0]) -K_OMG_XY*sensors->gyro.x*3.141f/180.0f)
              + (-J_YY + J_ZZ)*sensors->gyro.y*sensors->gyro.z;
  pitchMoment = -(0.5f*(-K_ROT_XY*errRotation[1]) -K_OMG_XY*sensors->gyro.y*3.141f/180.0f)
              + (J_XX - J_ZZ)*sensors->gyro.x*sensors->gyro.z;
  yawMoment = (0.5f*(-K_ROT_Z*errRotation[2]) -K_OMG_Z*sensors->gyro.z*3.141f/180.0f)
              + (-J_XX + J_YY)*sensors->gyro.x*sensors->gyro.y;

  rollOutput = saturateSignedInt16(MOM_GAIN*rollMoment);
  pitchOutput = saturateSignedInt16(MOM_GAIN*pitchMoment);
  yawOutput = saturateSignedInt16(MOM_GAIN*yawMoment);
}

void geometricControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll = rollOutput;
  *pitch = pitchOutput;
  *yaw = yawOutput;
}

LOG_GROUP_START(geometric_ctl)
LOG_ADD(LOG_FLOAT, roll_moment, &rollMoment)
LOG_ADD(LOG_FLOAT, pitch_moment, &pitchMoment)
LOG_ADD(LOG_FLOAT, yaw_moment, &yawMoment)
LOG_GROUP_STOP(geometric_ctl)

/* PARAM_GROUP_START(pid_attitude) */
/* PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRoll.kp) */
/* PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRoll.ki) */
/* PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRoll.kd) */
/* PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitch.kp) */
/* PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitch.ki) */
/* PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitch.kd) */
/* PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYaw.kp) */
/* PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYaw.ki) */
/* PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYaw.kd) */
/* PARAM_GROUP_STOP(pid_attitude) */
/*  */
/* PARAM_GROUP_START(pid_rate) */
/* PARAM_ADD(PARAM_FLOAT, rpRateLimit,  &rpRateLimit) */
/* PARAM_ADD(PARAM_FLOAT, yawRateLimit, &yawRateLimit) */
/* PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRollRate.kp) */
/* PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRollRate.ki) */
/* PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRollRate.kd) */
/* PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitchRate.kp) */
/* PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitchRate.ki) */
/* PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitchRate.kd) */
/* PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYawRate.kp) */
/* PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYawRate.ki) */
/* PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYawRate.kd) */
/* PARAM_GROUP_STOP(pid_rate) */
