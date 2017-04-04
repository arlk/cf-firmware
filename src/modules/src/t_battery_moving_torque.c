
 /*                    .oo .oPYo.  .oPYo. o
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

#include "t_battery_moving_torque.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "arm_math.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "pid.h"


#include "sensors.h"

//Assume function called 500Hz (0.002s)


static float f_x_btry=0;
static float f_mpl_esti=0;
static float f_fdbk=0;
static float f_e_old=0;


float mplesti(float t1, float t2, float xbtry, float TQ_need)
{
	float m1 = 12.38e-3; 
	float m2 = 35.29e-3; 
	float l1 = 0.150; 
	float l2 = 0.165;
	float r1 = 66.82e-3;
	float r2 = 85e-3; 

	float mpl_esti;

	mpl_esti = (TQ_need / 9.81f + 0.02f*0.124f - (r1*arm_cos_f32(t1) - 0.02f)*m1 - (l1*arm_cos_f32(t1) + r2*arm_cos_f32(t2) - 0.02f)*m2 + xbtry*0.063f * 2.0f) / (l1*arm_cos_f32(t1) + l2*arm_cos_f32(t2) - 0.02f);

	if (mpl_esti >= 0.3f)
	{
		mpl_esti = 0.3f;
	}
	if (mpl_esti <= 0.0f)
	{
		mpl_esti = 0.0f;
	}
	return mpl_esti;
}

float linear_act(float TQ_need, float fdbk, float e_old)
{
	float mbtry, x1, r, e, v,kp,kd,result;
	mbtry = 2.0f * 0.063f*9.81f;
	x1 = TQ_need / mbtry;
	r = x1 - 0.05f;
	e = r - (fdbk - 0.05f);
	v = (e - e_old) / 0.004f;
	f_e_old = e;


	kp = 20.0f;
	kd = 1.0f;
	result = kp*r + kd*v + 0.05f;

	if ((result - fdbk) >= 0.002f*0.2f)
	{
		result = fdbk + 0.002f*0.2f;
	}
	if ((result - fdbk) <= -0.002f*0.2f)
	{
		result = fdbk - 0.002f*0.2f;
	}
	if (result >= 1.1f)
	{
		result = 1.1f;
	}
	if (result <= 0.5f)
	{
		result = 0.5f;
	}
	
	f_fdbk = result;
	return result;
}

float tq_est_static(float t1, float t2, float mpl_esti)
{

	float m1 = 12.38e-3;
	float m2 = 35.29e-3;
	float l1 = 0.150;
	float l2 = 0.165;
	float r1 = 66.82e-3;
	float r2 = 85e-3;

	float tq_esti;
	tq_esti = (r1*arm_cos_f32(t1)*m1 + (l1*arm_cos_f32(t1) + r2*arm_cos_f32(t2))*m2 + (l1*arm_cos_f32(t1) + l2*arm_cos_f32(t2))*mpl_esti)*(9.81f);
	return tq_esti;
}


int updateBatteryPosition(float tq_get,float t1,float t2)//dont know how to get tq_get(geometric pitch output)
{

	static float TQ_need;

	static float feedforward_tq;
	feedforward_tq = tq_est_static(t1, t2, f_mpl_esti);//compute feedforward result
	
	TQ_need = tq_get - feedforward_tq;
	f_x_btry = linear_act(TQ_need, f_fdbk, f_e_old);
	f_mpl_esti = mplesti(t1, t2, f_x_btry, TQ_need);

	int f_batteryPos=(int)((f_x_btry-0.5f)/0.6f*4000.0f+4000.0f);
	return f_batteryPos;
}