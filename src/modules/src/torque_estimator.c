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
#include "task.h"

#include "pid.h"
#include "torque_estimator.h"
#include "manipulator.h"
#include "sensors.h"

#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#else
#include "estimator.h"
#endif

//double pitch_att = &state.attitude.pitch;
//double pitch_rate = &sensorData.gyro.y;
//double pitch_acc;

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


void servoControllerUpdatePID(float servoPosActual, float servoPosDesired)
{
  pidSetDesired(&pidServo, servoPosDesired);
  servoPidCmd = saturateSignedInt16(
  pidUpdate(&pidServo, servoPosActual, true));
}


void servoControllerResetAllPID(void)
{
  pidReset(&pidServo);
}



///// SERVO ESTIMATOR LOOP /////
void servoEstUpdate(float ts,float target){
	static float avis = 0.0f;
	static float servoAcc = 0.0f;
	static float servoVel = 0.0f;
	static float servoPosActual;

	avis = -K_VIS*servoVel;
	servoAcc = servoPidCmd + avis;// + lagrangeDynamics(float servoStates, float manipStates);
	servoAcc = servoAccSat(servoAcc,servoAccMax);
	servoVel += servoAcc*ts;
	servoPosActual += servoVel*ts;
}


///// MANIPULATOR DYNAMICS LOOP /////
    float lagrangeDynamics(float servoStates, float manipStates, float payloadMass){
	static float c1;
	static float c2;
	static float s2;
	static float c12;
	static float alpha;
	static float beta;
	static float delta;
	static float moment1;
	static float moment2;
	
	c1 = arm_cos_f32(theta1);
	c2 = arm_cos_f32(theta2 - theta1);
	s2 = arm_sin_f32(theta2 - theta1);
	c12 = arm_cos_f32(theta2);
	
	alpha = IZ_1 + IZ_2 + M_1*powf(R_1,2.0f) + M_2*(powf(L_1,2.0f) + powf(L_2,2.0f));
	beta = M_2*L_1*R_2;
	delta = IZ_2 + M_2*powf(R_2,2.0f);
	
	moment1 = (alpha + 2*beta*c2)*theta1DDot + (delta + beta*c2)*theta2DDot + (-beta*s2*theta2Dot)*theta1Dot
					+ (-beta*s2*(theta1Dot + theta2Dot))*theta2Dot + (GRAVITY*c1*(M_1*R_1 + M_2*L_2) + M_2*GRAVITY*R_2*c12);

	moment2 = (delta + beta*c2)*theta1DDot + delta*theta2DDot + (beta*s2*theta1Dot)*theta1Dot;

	return moment1 + moment2;  /// Total manipulator moment ///

}


///// TOOLS /////
void pwm2rad(float target){
	//code
	double target_rad = ((target-500.0)*0.0900)*(PI/180.0);
}

