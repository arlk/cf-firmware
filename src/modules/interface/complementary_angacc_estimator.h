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
 * complementary_angacc_estimator.h - header for complementary_angacc_estimator.c
 * */
#ifndef COMPLEMENTARY_ANGACC_ESTIMATOR_H_
#define COMPLEMENTARY_ANGACC_ESTIMATOR_H_

#include <stdint.h>
#include "stabilizer_types.h"
#include "pid.h"

#define I_XX 0.0270f
#define I_YY 0.0270f
#define I_ZZ 0.0460f
#define I_ZX 0.0000f

void complementaryHsInit(const float updateDt);

bool complementaryHsTest();

float complementaryHsUpdatePID(float gyroActual, float omegaDotDesired);

void complementaryHsResetAllPID(void);

float modelPredictiveEstimatorPitchAcc(const state_t *state, const sensorData_t *sensorData);

void complementaryAngAccEstimator(state_t *state, const sensorData_t *sensorData);

#endif /* COMPLEMENTARY_ANGACC_ESTIMATOR_H_ */