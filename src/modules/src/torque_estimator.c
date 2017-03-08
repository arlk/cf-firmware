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

#include "FreeRTOS.h"
#include "task.h"

#include "pid.h"
#include "manipulator.h"
#include "sensors.h"

#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#else
#include "estimator.h"
#endif

double pitch_att = &state.attitude.pitch;
double pitch_rate = &sensorData.gyro.y;
double pitch_acc;

static float servoPidCmd;


PidObject pidServo;

///// SERVO ESTIMATOR PID /////
void servoControllerInit(const float updateDt){

	if(isInit)
      return;

	pidInit(&pidServoPos, 0, PID_SERVO_KP, PID_SERVO_KI, PID_SERVO_KD, updateDt);
	pidSetIntegralLimit(&pidServoPos, PID_SERVO_INTEGRATION_LIMIT);

	isInit = true;
}


bool servoControllerTest()
{
  return isInit;
}


void servoControllerUpdatePID(float servoPosActual, float servoPosDesired)
{
  pidSetDesired(&pidServo, servoPosDesired);
  servoPidCmd = saturateSignedInt16(pidUpdate(&pidServo, servoPosActual, true));
}


void servoControllerResetAllPID(void)
{
  pidReset(&pidServo);
}



///// SERVO ESTIMATOR LOOP /////
void servoEstUpdate(ts,target){
	static float avis = 0.0f;
	static float servoAcc = 0.0f;
	static float servoVel = 0.0f;
	static float servoPosActual

	avis = -K_VIS*servoVel;
	servoAcc = servoPidCmd + avis + Marm;
	servoAcc = servoAccSat(servoAcc,servoAccMax);
	servoVel += servoAcc*ts;
	servoPosActual += servoVel*ts;
	// more coad
}


/////MANIPULATOR DYNAMICS LOOP /////
void lagrangeDynamics(servoStates,manipStates,payloadMass){
	// coad
}


///// TOOLS /////
void pwm2rad(target){
	//code
	double target_rad = ((target-500.0)*0.0900)*(PI/180.0);
}

