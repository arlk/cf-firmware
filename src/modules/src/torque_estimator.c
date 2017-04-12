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
 * Authored by Robert Mitchell Jones
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
 * torque_estimator.c - manipulator joint torque estimator
 * */
#include <math.h>
#include "arm_math.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "pid.h"
#include "torque_estimator.h"
#include "serial_manipulator.h"
#include "sensors.h"
#include "geometric_controller.h"

#include "complementary_angacc_estimator.h"

#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#else
#include "estimator.h"
#endif

static float servoPidCmd;
static bool isInit = false;

PidObject pidServo;


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

///// SERVO ESTIMATOR PID /////
void servoControllerInit(const float updateDt){

	if(isInit)
      return;

	pidInit(&pidServo, 0, PID_SERVO_KP, PID_SERVO_KI, PID_SERVO_KD, updateDt, 1/updateDt, 1, false);
	pidSetIntegralLimit(&pidServo, PID_SERVO_INTEGRATION_LIMIT);

	isInit = true;
}


bool servoControllerTest()
{
  return isInit;
}


int servoControllerUpdatePID(float servoPosActual, float servoPosDesired)
{
  pidSetDesired(&pidServo, servoPosDesired);
  servoPidCmd = pidUpdate(&pidServo, servoPosActual, true);
  return servoPidCmd;
}


void servoControllerResetAllPID(void)
{
  pidReset(&pidServo);
}


float servoAccSat(float servoAcc, float servoAccMax)
{
	float servoAccSat;
	// coad
	if (servoAcc > servoAccMax)
	{
		servoAccSat = servoAccMax;
	}
	else if (servoAcc < -servoAccMax)
	{
		servoAccSat = -servoAccMax;
	}
	else
	{
		servoAccSat = servoAcc;
	}
	return servoAccSat;
}


void servoGetCmd(int* targetAll, const state_t* state, setpoint_t* setpoint){

	
    targetAll[0] = (int)(2000.0f*setpoint->joy.pitch+6000.0f);
    targetAll[1] = (int)(-2000.0f*setpoint->joy.throttle+6000.0f);
    targetAll[2] = 4000;
    //targetAll[2] = (int)(-3000.0f*(float)setpoint->joy.trigger+7000.0f);
	

    /*
    if ((float)setpoint->joy.trigger > 0.5f && fabsf(state->attitude.pitch) < 1.0f )
    {
    	// coad
    	targetAll[0] = (int)(-2000.0f*state->attitude.pitch+6000.0f);
    	targetAll[1] = (int)(2000.0f*state->attitude.pitch+6000.0f);
    	targetAll[2] = (int)(4000.0f);
    }
    else
    {
    	// coad
    	targetAll[0] = 6000, targetAll[1] = 6000, targetAll[2] = 6000;
    }
	*/
}

///// SERVO ESTIMATOR LOOP /////
void servoEstUpdate(float ts, int servoNumber, servoStates_t* servoStates, const state_t* state, const sensorData_t* sensorData, int* targetAll){
	float avis;

	avis = -K_VIS*servoStates->vel[servoNumber];
	servoStates->acc[servoNumber] = avis + servoControllerUpdatePID(servoStates->pos[servoNumber], pwm2rad((float)targetAll[servoNumber]) )
		+ lagrangeDynamics(0.0f, servoStates, state, sensorData);
	servoStates->acc[servoNumber] = servoAccSat(servoStates->acc[servoNumber],SERVO_ACC_MAX);
	servoStates->vel[servoNumber] += servoStates->acc[servoNumber]*ts;
	servoStates->pos[servoNumber] += servoStates->vel[servoNumber]*ts;
}


///// MANIPULATOR DYNAMICS LOOP /////
float lagrangeDynamics(float payloadMass, servoStates_t* servoStates, const state_t* state, const sensorData_t* sensorData){
	static float c1;
	static float c2;
	static float s2;
	static float c12;
	static float alpha;
	static float beta;
	static float delta;
	static float moment1;
	static float moment2;

	static float theta1;
	static float theta2;
	static float theta1Dot;
	static float theta2Dot;
	static float theta1DDot;
	static float theta2DDot;


	theta1 = servoStates->pos[0] + state->attitude.pitch;
	theta2 = servoStates->pos[1] + state->attitude.pitch;
	theta1Dot = servoStates->vel[0] + DEG_TO_RAD*sensorData->gyro.y;
	theta2Dot = servoStates->vel[1] + DEG_TO_RAD*sensorData->gyro.y;
	theta1DDot = servoStates->acc[0] + state->angAcc.y;
	theta2DDot = servoStates->acc[1] + state->angAcc.y;

	c1 = arm_cos_f32(theta1);
	c2 = arm_cos_f32(theta2 - theta1);
	s2 = arm_sin_f32(theta2 - theta1);
	c12 = arm_cos_f32(theta2);



	alpha = IZ_1 + IZ_2 + M_1*powf(R_1,2.0f) + M_2*(powf(L_1,2.0f) + powf(L_2,2.0f));
	beta = M_2*L_1*R_2;
	delta = IZ_2 + M_2*powf(R_2,2.0f);

	moment1 = (alpha + 2.0f*beta*c2)*theta1DDot + (delta + beta*c2)*theta2DDot + (-beta*s2*theta2Dot)*theta1Dot
					+ (-beta*s2*(theta1Dot + theta2Dot))*theta2Dot + (-GRAVITY*c1*(M_1*R_1 + M_2*L_2) + M_2*(-GRAVITY)*R_2*c12);

	moment2 = (delta + beta*c2)*theta1DDot + delta*theta2DDot + (beta*s2*theta1Dot)*theta1Dot;

	return moment1 + moment2;  /// Total manipulator moment ///

}


///// TOOLS /////
float pwm2rad(float target){
	//code
	float target_rad = ((target-4000.0f)*0.00025f)*(PI);
	return target_rad;
}

