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

#include "arm_math.h"
#include "flight_math.h"
#include "geometric_controller.h"
#include "param.h"
#include "log.h"
#include "math.h" //Thaigo

/* Thiago added funtions and definitions*/
float nchoosek (float N, float K);

typedef struct traj traj;
struct traj{
    float Cx[10];
    float Cy[10];
    float Cz[10];
    int n;
    float total_time;
    traj *next;
};
void compBezier (traj* P, float t, float* vec);
void diffBezier(traj* P, traj* V);
void initBezierTraj(void);

/* end of Thiago added stuff*/

void trajectoryInit(const uint32_t tick);

void updateTrajectory(setpoint_t* setpoint, const uint32_t tick);
