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
 * complementary_angacc_estimator.c - estimates angular velocities of vehicle
 * */

#include "complementary_angacc_estimator.h"

#include "math.h"
#include "arm_math.h"
#include "sensors.h"
#include "pid.h"

#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#else
#include "estimator.h"
#endif

/*
#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT 1.0/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT 1.0/POS_UPDATE_RATE
*/
PidObject pidComplementaryAcc;

static float complementaryHsPidCmd;
static bool isInit;

///// SERVO ESTIMATOR PID /////
void complementaryHsInit(const float updateDt){

  if(isInit)
      return;

  pidInit(&pidComplementaryAcc, 0, PID_COMPLEMENTARY_ESTIMATOR_KP, PID_COMPLEMENTARY_ESTIMATOR_KI,
    PID_COMPLEMENTARY_ESTIMATOR_KD, updateDt, 1/updateDt, 1, false);
  pidSetIntegralLimit(&pidComplementaryAcc, PID_SERVO_INTEGRATION_LIMIT);

  isInit = true;
}


bool complementaryHsTest()
{
  return isInit;
}


int complementaryHsUpdatePID(float gyroActual, float omegaDotDesired)
{
  pidSetDesired(&pidComplementaryAcc, omegaDotDesired);
  complementaryHsPidCmd = pidUpdate(&pidComplementaryAcc, gyroActual, true);
  return complementaryHsPidCmd;
}


void complementaryHsResetAllPID(void)
{
  pidReset(&pidComplementaryAcc);
}


float modelPredictiveEstimatorPitchAcc(float Mhat, const state_t *state, const sensorData_t *sensorData){
  // coad
  float qDotHat;
  qDotHat = ((I_ZZ - I_XX) * sensorData->gyro.x * sensorData->gyro.y + I_ZX * (powf(sensorData->gyro.x, 2.0f)
    - powf(sensorData->gyro.y, 2.0f)) ) / I_YY + Mhat / I_YY;
  return qDotHat;
}


float complementaryAngAccEstimator(state_t *state, const sensorData_t *sensorData)
{
  // coad
  float omegaDotHat;

  omegaDotHat = modelPredictiveEstimatorPitchAcc(0.0f, states, sensorData) + complementaryHsUpdatePID(omegaDotHat, sensorData->gyro.y);

  return omegaDotHat;
}  
