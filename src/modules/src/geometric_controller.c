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
 * */

#include <stdbool.h>

#include "FreeRTOS.h"

#include "geometric_controller.h"

#include "arm_math.h"
#include "flight_math.h"
#include "param.h"
#include "log.h"

static float k_pos_xy = 0.85f;
static float k_pos_z = 0.65f;
static float k_vel_xy = 0.45f;
static float k_vel_z = 0.25f;
static float thr_gain = 1.0e5f;

static float k_rot_xy = 1.0f;
static float k_rot_z = 5.0f;
static float k_omg_xy = 0.25f;
static float k_omg_z = 0.6f;
static float j_xx = 1.39e-5f;
static float j_yy = 1.39e-5f;
static float j_zz = 2.173e-5f;
static float mom_gain = 30000.0f;

static int16_t rollOutput;
static int16_t pitchOutput;
static int16_t yawOutput;
static float thrustOutput;

static float rollMoment;
static float pitchMoment;
static float yawMoment;
static float thrustForce;

static float errPosition[3];
static float errVelocity[3];

static bool isInit;

static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
static inline void mat_sub(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_sub_f32(pSrcA, pSrcB, pDst)); }
static inline float arm_sqrt(float32_t in)
{ float pOut = 0; arm_status result = arm_sqrt_f32(in, &pOut); configASSERT(ARM_MATH_SUCCESS == result); return pOut; }


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

void geometricControllerGetAttitudeDesired(const state_t* state,
    attitude_t* attitudeDesired, setpoint_t* setpoint)
{
  errPosition[0] = setpoint->position.x - state->position.x;
  errPosition[1] = setpoint->position.y - state->position.y;
  errPosition[2] = setpoint->position.z - state->position.z;

  errVelocity[0] = setpoint->velocity.x - state->velocity.x;
  errVelocity[1] = setpoint->velocity.y - state->velocity.y;
  errVelocity[2] = setpoint->velocity.z - state->velocity.z;

  setpoint->rotation.vals[0][2] = k_pos_xy*errPosition[0] + k_vel_xy*errVelocity[0]
      + MASS*setpoint->acc.x;
  setpoint->rotation.vals[1][2] = k_pos_xy*errPosition[1] + k_vel_xy*errVelocity[1]
      + MASS*setpoint->acc.y;
  setpoint->rotation.vals[2][2] = k_pos_z*errPosition[2] + k_vel_z*errVelocity[2]
      + MASS*(GRAVITY + setpoint->acc.z);

  static float forceMagnitude = 0;
  forceMagnitude = arm_sqrt(
    setpoint->rotation.vals[0][2] * setpoint->rotation.vals[0][2] +
    setpoint->rotation.vals[1][2] * setpoint->rotation.vals[1][2] +
    setpoint->rotation.vals[2][2] * setpoint->rotation.vals[2][2]);

  setpoint->rotation.vals[0][2] = setpoint->rotation.vals[0][2]/forceMagnitude;
  setpoint->rotation.vals[1][2] = setpoint->rotation.vals[1][2]/forceMagnitude;
  setpoint->rotation.vals[2][2] = setpoint->rotation.vals[2][2]/forceMagnitude;

  static float xInterDesired[3];
  xInterDesired[0] = arm_cos_f32(attitudeDesired->yaw);
  xInterDesired[1] = arm_sin_f32(attitudeDesired->yaw);
  xInterDesired[2] = 0;

  setpoint->rotation.vals[0][1] = -setpoint->rotation.vals[2][2]*xInterDesired[1]
    + setpoint->rotation.vals[1][2]*xInterDesired[2];
  setpoint->rotation.vals[1][1] =  setpoint->rotation.vals[2][2]*xInterDesired[0]
    - setpoint->rotation.vals[0][2]*xInterDesired[2];
  setpoint->rotation.vals[2][1] = -setpoint->rotation.vals[1][2]*xInterDesired[0]
    + setpoint->rotation.vals[0][2]*xInterDesired[1];

  setpoint->rotation.vals[0][0] = -setpoint->rotation.vals[2][1]*setpoint->rotation.vals[1][2]
    + setpoint->rotation.vals[1][1]*setpoint->rotation.vals[2][2];
  setpoint->rotation.vals[1][0] = setpoint->rotation.vals[2][1]*setpoint->rotation.vals[0][2]
    - setpoint->rotation.vals[0][1]*setpoint->rotation.vals[2][2];
  setpoint->rotation.vals[2][0] = -setpoint->rotation.vals[1][1]*setpoint->rotation.vals[0][2]
    + setpoint->rotation.vals[0][1]*setpoint->rotation.vals[1][2];
}

void geometricControllerGetThrustDesired(const state_t* state, setpoint_t* setpoint)
{
  thrustForce = (k_pos_xy*errPosition[0] + k_vel_xy*errVelocity[0]
                + MASS*setpoint->acc.x)*state->rotation.vals[0][2]
         + (k_pos_xy*errPosition[1] + k_vel_xy*errVelocity[1]
                + MASS*setpoint->acc.y)*state->rotation.vals[1][2]
         + (k_pos_z*errPosition[2] + k_vel_z*errVelocity[2]
                + MASS*(GRAVITY + setpoint->acc.z))*state->rotation.vals[2][2];

  thrustOutput = thr_gain*thrustForce;
}

void geometricMomentController(const rotation_t* rotation,
    const sensorData_t *sensors, rotation_t* rotationDes)
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

void geometricControllerGetThrustOutput(float* thrust)
{
  *thrust = thrustOutput;
}

LOG_GROUP_START(geom_ctl_out)
LOG_ADD(LOG_FLOAT, roll_moment, &rollMoment)
LOG_ADD(LOG_FLOAT, pitch_moment, &pitchMoment)
LOG_ADD(LOG_FLOAT, yaw_moment, &yawMoment)
LOG_ADD(LOG_FLOAT, thrust_force, &thrustForce)
LOG_GROUP_STOP(geom_ctl_out)

PARAM_GROUP_START(geom_ctl_moment)
PARAM_ADD(PARAM_FLOAT, kr_xy, &k_rot_xy)
PARAM_ADD(PARAM_FLOAT, kr_z, &k_rot_z)
PARAM_ADD(PARAM_FLOAT, ko_xy, &k_omg_xy)
PARAM_ADD(PARAM_FLOAT, ko_z, &k_omg_z)
PARAM_ADD(PARAM_FLOAT, jxx, &j_xx)
PARAM_ADD(PARAM_FLOAT, jyy, &j_yy)
PARAM_ADD(PARAM_FLOAT, jzz, &j_zz)
PARAM_ADD(PARAM_FLOAT, moment_gain, &mom_gain)
PARAM_GROUP_STOP(geom_ctl_moment)

PARAM_GROUP_START(geom_ctl_thrust)
PARAM_ADD(PARAM_FLOAT, kp_xy, &k_pos_xy)
PARAM_ADD(PARAM_FLOAT, kp_z, &k_pos_z)
PARAM_ADD(PARAM_FLOAT, kv_xy, &k_vel_xy)
PARAM_ADD(PARAM_FLOAT, kv_z, &k_vel_z)
PARAM_ADD(PARAM_FLOAT, thrust_gain, &thr_gain)
PARAM_GROUP_STOP(geom_ctl_thrust)
