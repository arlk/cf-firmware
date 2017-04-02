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
#include "queue.h"
#include "stm32f4xx.h"

#include "geometric_controller.h"


#include "arm_math.h"
#include "flight_math.h"
#include "param.h"
#include "log.h"
#include "torque_estimator.h"

#ifdef SERIAL_MANIP

static float manip_mom_gain = 30000.0f;
static float test_manip_rollMoment; // Nm
static float test_manip_pitchMoment = 0.1f; // Nm
static float test_manip_yawMoment; // Nm

#endif /* SERIAL_MANIP */

static float k_pos_xy = 0.85f;
static float k_pos_z = 0.65f;
static float k_vel_xy = 0.45f;
static float k_vel_z = 0.25f;
static float mass = 0.028f;
static float thr_gain = 60000.0f;

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

//static float vehiclePitchStates[3];

//static xQueueHandle vehicleStatesQueue;
//#define VEH_STATES_QUEUE_LENGTH 10

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
    //xQueueReset(vehicleStatesQueue);
    return;

  //vehicleStatesQueue = xQueueCreate(VEH_STATES_QUEUE_LENGTH,sizeof(vehiclePitchStates));
  isInit = true;
}

bool geometricControllerTest()
{
  return isInit;
}
/*
bool vehicleEnqueuePitchStates(xQueueHandle* queue, void *pitchStates)
{
  portBASE_TYPE result;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(queue, pitchStates, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD();
    }
  } else {
    result = xQueueSend(queue, pitchStates, 0);
  }
  return (result==pdTRUE);
}


bool vehicleGetQueuePitchStates(float *pitchStates)
{
  portBASE_TYPE result;
  result = xQueueReceive(vehicleStatesQueue, pitchStates, 0);
  return (result==pdTRUE);
}
*/
void geometricControllerGetOmegaDesired(setpoint_t* setpoint)
{
  static float desThrust[3];
  desThrust[0] = setpoint->acc.x;
  desThrust[1] = setpoint->acc.y;
  desThrust[2] = setpoint->acc.z + GRAVITY;

  static float invForceMagnitude = 0;
  invForceMagnitude = invSqrt(
    powf(desThrust[0], 2) +
    powf(desThrust[1], 2) +
    powf(desThrust[2], 2));

  static float z_b[3];
  z_b[0] = desThrust[0]*invForceMagnitude;
  z_b[1] = desThrust[1]*invForceMagnitude;
  z_b[2] = desThrust[2]*invForceMagnitude;

  static float x_w[3];
  x_w[0] = arm_cos_f32(setpoint->attitude.yaw);
  x_w[1] = arm_sin_f32(setpoint->attitude.yaw);
  x_w[2] = 0;

  static float y_b[3];
  y_b[0] = -z_b[2]*x_w[1] + z_b[1]*x_w[2];
  y_b[1] =  z_b[2]*x_w[0] - z_b[0]*x_w[2];
  y_b[2] = -z_b[1]*x_w[0] + z_b[0]*x_w[1];

  static float x_b[3];
  x_b[0] = -y_b[2]*z_b[1] + y_b[1]*z_b[2];
  x_b[1] =  y_b[2]*z_b[0] - y_b[0]*z_b[2];
  x_b[2] = -y_b[1]*z_b[0] + y_b[0]*z_b[1];

  static float forceDot;
  forceDot = z_b[0]*setpoint->jerk.x
        + z_b[1]*setpoint->jerk.y
        + z_b[2]*setpoint->jerk.z;

  static float h_w[3];
  h_w[0] = (setpoint->jerk.x - forceDot*z_b[0])*invForceMagnitude;
  h_w[1] = (setpoint->jerk.y - forceDot*z_b[1])*invForceMagnitude;
  h_w[2] = (setpoint->jerk.z - forceDot*z_b[2])*invForceMagnitude;

  setpoint->attitudeRate.roll = -h_w[0]*y_b[0]
        - h_w[1]*y_b[1]
        - h_w[2]*y_b[2];

  setpoint->attitudeRate.pitch = h_w[0]*x_b[0]
        + h_w[1]*x_b[1]
        + h_w[2]*x_b[2];

  setpoint->attitudeRate.yaw = setpoint->attitudeRate.yaw*z_b[2];

  static float crossTemp[3];
  crossTemp[0] = (-setpoint->attitudeRate.yaw*z_b[1] + setpoint->attitudeRate.pitch*z_b[2])/invForceMagnitude;
  crossTemp[1] = (setpoint->attitudeRate.yaw*z_b[0] - setpoint->attitudeRate.roll*z_b[2])/invForceMagnitude;
  crossTemp[2] = (-setpoint->attitudeRate.pitch*z_b[0] + setpoint->attitudeRate.roll*z_b[1])/invForceMagnitude;

  static float crossTempA[3];
  crossTempA[0] = -setpoint->attitudeRate.yaw*crossTemp[1] + setpoint->attitudeRate.pitch*crossTemp[2];
  crossTempA[1] = setpoint->attitudeRate.yaw*crossTemp[0] - setpoint->attitudeRate.roll*crossTemp[2];
  crossTempA[2] = -setpoint->attitudeRate.pitch*crossTemp[0] + setpoint->attitudeRate.roll*crossTemp[1];

  static float forceDDot;
  forceDDot = z_b[0]*setpoint->snap.x
        + z_b[1]*setpoint->snap.y
        + z_b[2]*setpoint->snap.z
        - z_b[0]*crossTempA[0]
        - z_b[1]*crossTempA[1]
        - z_b[2]*crossTempA[2];

  static float crossTempB[3];
  crossTempB[0] = (-setpoint->attitudeRate.yaw*z_b[1] + setpoint->attitudeRate.pitch*z_b[2])*forceDot;
  crossTempB[1] = (setpoint->attitudeRate.yaw*z_b[0] - setpoint->attitudeRate.roll*z_b[2])*forceDot;
  crossTempB[2] = (-setpoint->attitudeRate.pitch*z_b[0] + setpoint->attitudeRate.roll*z_b[1])*forceDot;

  static float h_a[3];
  h_a[0] = (setpoint->snap.x - forceDDot*z_b[0] - 2*crossTempB[0] - crossTempA[0])*invForceMagnitude;
  h_a[1] = (setpoint->snap.y - forceDDot*z_b[1] - 2*crossTempB[1] - crossTempA[1])*invForceMagnitude;
  h_a[2] = (setpoint->snap.z - forceDDot*z_b[2] - 2*crossTempB[2] - crossTempA[2])*invForceMagnitude;

  setpoint->attitudeAcc.roll = -h_a[0]*y_b[0]
        - h_a[1]*y_b[1]
        - h_a[2]*y_b[2];

  setpoint->attitudeAcc.pitch = h_a[0]*x_b[0]
        + h_a[1]*x_b[1]
        + h_a[2]*x_b[2];

  setpoint->attitudeAcc.yaw = setpoint->attitudeAcc.yaw*z_b[2];
}

void geometricControllerGetAttitudeDesired(const state_t* state,
    setpoint_t* setpoint)
{
  errPosition[0] = setpoint->position.x - state->position.x;
  errPosition[1] = setpoint->position.y - state->position.y;
  errPosition[2] = setpoint->position.z - state->position.z;

  errVelocity[0] = setpoint->velocity.x - state->velocity.x;
  errVelocity[1] = setpoint->velocity.y - state->velocity.y;
  errVelocity[2] = setpoint->velocity.z - state->velocity.z;

  setpoint->rotation.vals[0][2] = k_pos_xy*errPosition[0] + k_vel_xy*errVelocity[0]
      + mass*setpoint->acc.x;
  setpoint->rotation.vals[1][2] = k_pos_xy*errPosition[1] + k_vel_xy*errVelocity[1]
      + mass*setpoint->acc.y;
  setpoint->rotation.vals[2][2] = k_pos_z*errPosition[2] + k_vel_z*errVelocity[2]
      + mass*(GRAVITY + setpoint->acc.z);

  static float invForceMagnitude = 0;
  invForceMagnitude = invSqrt(
    powf(setpoint->rotation.vals[0][2], 2) +
    powf(setpoint->rotation.vals[1][2], 2) +
    powf(setpoint->rotation.vals[2][2], 2));

  setpoint->rotation.vals[0][2] = setpoint->rotation.vals[0][2]*invForceMagnitude;
  setpoint->rotation.vals[1][2] = setpoint->rotation.vals[1][2]*invForceMagnitude;
  setpoint->rotation.vals[2][2] = setpoint->rotation.vals[2][2]*invForceMagnitude;

  static float xInterDesired[3];
  xInterDesired[0] = arm_cos_f32(setpoint->attitude.yaw);
  xInterDesired[1] = arm_sin_f32(setpoint->attitude.yaw);
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
                + mass*setpoint->acc.x)*state->rotation.vals[0][2]
         + (k_pos_xy*errPosition[1] + k_vel_xy*errVelocity[1]
                + mass*setpoint->acc.y)*state->rotation.vals[1][2]
         + (k_pos_z*errPosition[2] + k_vel_z*errVelocity[2]
                + mass*(GRAVITY + setpoint->acc.z))*state->rotation.vals[2][2];

  #ifdef SERIAL_MANIP

  thrustOutput = thr_gain*setpoint->joy.throttle;

  #else

  thrustOutput = thr_gain*thrustForce;

  #endif /* SERIAL_MANIP */
}

void geometricMomentController(const rotation_t* rotation,
    const sensorData_t *sensors, setpoint_t* setpoint, const state_t* state)
{
  arm_matrix_instance_f32 Rwb = {3, 3, (float *)rotation->vals};
  arm_matrix_instance_f32 Rwd = {3, 3, (float *)setpoint->rotation.vals};

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

  static float errOmega[3];
  errOmega[0] = sensors->gyro.x*DEG_TO_RAD
    - desOrientationTransp[0][0]*setpoint->attitudeRate.roll
    - desOrientationTransp[0][1]*setpoint->attitudeRate.pitch
    - desOrientationTransp[0][2]*setpoint->attitudeRate.yaw;
  errOmega[1] = sensors->gyro.y*DEG_TO_RAD
    - desOrientationTransp[1][0]*setpoint->attitudeRate.roll
    - desOrientationTransp[1][1]*setpoint->attitudeRate.pitch
    - desOrientationTransp[1][2]*setpoint->attitudeRate.yaw;
  errOmega[2] = sensors->gyro.z*DEG_TO_RAD
    - desOrientationTransp[2][0]*setpoint->attitudeRate.roll
    - desOrientationTransp[2][1]*setpoint->attitudeRate.pitch
    - desOrientationTransp[2][2]*setpoint->attitudeRate.yaw;

  static float hatOmega[3][3] = {{0}};
  static arm_matrix_instance_f32 hO = {3, 3, (float *)hatOmega};
  hatOmega[0][1] = -sensors->gyro.z*DEG_TO_RAD;
  hatOmega[0][2] = sensors->gyro.y*DEG_TO_RAD;
  hatOmega[1][0] = sensors->gyro.z*DEG_TO_RAD;
  hatOmega[1][2] = -sensors->gyro.x*DEG_TO_RAD;
  hatOmega[2][0] = -sensors->gyro.y*DEG_TO_RAD;
  hatOmega[2][1] = sensors->gyro.x*DEG_TO_RAD;

  static float tempOrientation[3][3];
  static arm_matrix_instance_f32 tR = {3, 3, (float *)tempOrientation};
  mat_mult(&hO, &Rdbt, &tR);

  static float orientFeedFwd[3];
  orientFeedFwd[0] = j_xx*(
      tempOrientation[0][0]*setpoint->attitudeRate.roll
    + tempOrientation[0][1]*setpoint->attitudeRate.pitch
    + tempOrientation[0][2]*setpoint->attitudeRate.yaw
    - desOrientationTransp[0][0]*setpoint->attitudeAcc.roll
    - desOrientationTransp[0][1]*setpoint->attitudeAcc.pitch
    - desOrientationTransp[0][2]*setpoint->attitudeAcc.yaw);
  orientFeedFwd[1] = j_yy*(
      tempOrientation[1][0]*setpoint->attitudeRate.roll
    + tempOrientation[1][1]*setpoint->attitudeRate.pitch
    + tempOrientation[1][2]*setpoint->attitudeRate.yaw
    - desOrientationTransp[1][0]*setpoint->attitudeAcc.roll
    - desOrientationTransp[1][1]*setpoint->attitudeAcc.pitch
    - desOrientationTransp[1][2]*setpoint->attitudeAcc.yaw);
  orientFeedFwd[2] = j_zz*(
      tempOrientation[2][0]*setpoint->attitudeRate.roll
    + tempOrientation[2][1]*setpoint->attitudeRate.pitch
    + tempOrientation[2][2]*setpoint->attitudeRate.yaw
    - desOrientationTransp[2][0]*setpoint->attitudeAcc.roll
    - desOrientationTransp[2][1]*setpoint->attitudeAcc.pitch
    - desOrientationTransp[2][2]*setpoint->attitudeAcc.yaw);

  rollMoment  = (-k_rot_xy*0.5f*errRotation[0] -k_omg_xy*errOmega[0])
              + (-j_yy + j_zz)*sensors->gyro.y*sensors->gyro.z
              /* - orientFeedFwd[0]; */
              - 0.0f*orientFeedFwd[0];
  pitchMoment = -(-k_rot_xy*0.5f*errRotation[1] -k_omg_xy*errOmega[1])
              + (j_xx - j_zz)*sensors->gyro.x*sensors->gyro.z
              /* - orientFeedFwd[1]; */
              - 0.0f*orientFeedFwd[1];
  yawMoment   = (-k_rot_z*0.5f*errRotation[2] -k_omg_z*errOmega[2])
              + (-j_xx + j_yy)*sensors->gyro.x*sensors->gyro.y
              /* - orientFeedFwd[2]; */
              - 0.0f*orientFeedFwd[2];

  /*
  vehiclePitchStates[0] = 0.0f; //pitch attitude
  vehiclePitchStates[1] = 0.0f; //pitch rate
  vehiclePitchStates[2] = 0.0f; //pitch acceleration
  
  vehicleEnqueuePitchStates(vehicleStatesQueue, (void *)vehiclePitchStates);
  */

  #ifdef SERIAL_MANIP
  servoStates_t servoStates;
  int targetAll[3];

  servoController(targetAll, &servoStates, state, setpoint);

  rollOutput  = saturateSignedInt16(/*mom_gain*rollMoment +*/ manip_mom_gain*test_manip_rollMoment);
  //pitchOutput = saturateSignedInt16(/*mom_gain*pitchMoment +*/ manip_mom_gain*test_manip_pitchMoment);
  pitchOutput = saturateSignedInt16( mom_gain*pitchMoment + manip_mom_gain*lagrangeDynamics(0.0f, &servoStates));
  //pitchOutput = saturateSignedInt16( (setpoint->joy.throttle*60000.0f) *0.1f ); // TEST: 0.1 Nm desired output
  yawOutput   = saturateSignedInt16(/*mom_gain*yawMoment +*/ manip_mom_gain*test_manip_yawMoment);

  #else

  rollOutput  = saturateSignedInt16(mom_gain*rollMoment);
  pitchOutput = saturateSignedInt16(mom_gain*pitchMoment);
  yawOutput   = saturateSignedInt16(mom_gain*yawMoment);

  #endif /* SERIAL_MANIP */
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

LOG_GROUP_START(geomOut)
LOG_ADD(LOG_FLOAT, roll_moment, &rollMoment)
LOG_ADD(LOG_FLOAT, pitch_moment, &pitchMoment)
LOG_ADD(LOG_FLOAT, yaw_moment, &yawMoment)
LOG_ADD(LOG_FLOAT, thrust_force, &thrustForce)
LOG_GROUP_STOP(geomOut)

PARAM_GROUP_START(geomMoment)
PARAM_ADD(PARAM_FLOAT, kr_xy, &k_rot_xy)
PARAM_ADD(PARAM_FLOAT, kr_z, &k_rot_z)
PARAM_ADD(PARAM_FLOAT, ko_xy, &k_omg_xy)
PARAM_ADD(PARAM_FLOAT, ko_z, &k_omg_z)
PARAM_ADD(PARAM_FLOAT, jxx, &j_xx)
PARAM_ADD(PARAM_FLOAT, jyy, &j_yy)
PARAM_ADD(PARAM_FLOAT, jzz, &j_zz)
PARAM_ADD(PARAM_FLOAT, moment_gain, &mom_gain)
PARAM_GROUP_STOP(geomMoment)

PARAM_GROUP_START(geomThrust)
PARAM_ADD(PARAM_FLOAT, kp_xy, &k_pos_xy)
PARAM_ADD(PARAM_FLOAT, kp_z, &k_pos_z)
PARAM_ADD(PARAM_FLOAT, kv_xy, &k_vel_xy)
PARAM_ADD(PARAM_FLOAT, kv_z, &k_vel_z)
PARAM_ADD(PARAM_FLOAT, thrust_gain, &thr_gain)
PARAM_ADD(PARAM_FLOAT, mass, &mass)
PARAM_GROUP_STOP(geomThrust)

#ifdef SERIAL_MANIP

PARAM_GROUP_START(feedforward)
PARAM_ADD(PARAM_FLOAT, manip_gain, &manip_mom_gain)
PARAM_ADD(PARAM_FLOAT, manip_roll, &test_manip_rollMoment)
PARAM_ADD(PARAM_FLOAT, manip_pitch, &test_manip_pitchMoment)
PARAM_ADD(PARAM_FLOAT, manip_yaw, &test_manip_yawMoment)
PARAM_GROUP_STOP(feedforward)

#endif /* SERIAL_MANIP */
