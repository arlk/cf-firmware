
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "geometric_controller.h"
#include "flight_math.h"
#include "sensfusion6.h"

#include "log.h"
#include "param.h"

#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)
#define GEOMETRIC_UPDATE_DT  (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static rotation_t rotationDesired;
static float actuatorThrust;
//FIXME
static float yawDesired = 0;

void stateControllerInit(void)
{
  geometricControllerInit();
}

bool stateControllerTest(void)
{
  bool pass = true;

  pass &= geometricControllerTest();

  return pass;
}

void stateController(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
       yawDesired -= 10*setpoint->attitudeRate.yaw/500.0f;
      while (yawDesired > 180.0f)
        yawDesired -= 360.0f;
      while (yawDesired < -180.0f)
        yawDesired += 360.0f;
    } else {
      yawDesired = setpoint->attitude.yaw;
    }
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    }

    attitudeDesired.yaw = yawDesired*3.141f/180.0f;
    attitudeDesired.roll = attitudeDesired.roll*3.141f/180.0f;
    attitudeDesired.pitch = -attitudeDesired.pitch*3.141f/180.0f;

    eulerToRotationZYX(&rotationDesired, &attitudeDesired);

    geometricMomentController(&state->attitudeRotation, sensors, &rotationDesired);

    geometricControllerGetActuatorOutput(&control->roll,
                                         &control->pitch,
                                         &control->yaw);

    control->yaw = -control->yaw;
  }

  // Thrust
  control->thrust = actuatorThrust;

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    // Reset the calculated YAW angle for rate control
    /* attitudeDesired.yaw = state->attitude.yaw; */
  }
}


/* LOG_GROUP_START(controller) */
/* LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust) */
/* LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll) */
/* LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch) */
/* LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw) */
/* LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll) */
/* LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch) */
/* LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw) */
/* LOG_GROUP_STOP(controller) */
/*  */
/* PARAM_GROUP_START(controller) */
/* PARAM_ADD(PARAM_UINT8, tiltComp, &tiltCompensationEnabled) */
/* PARAM_GROUP_STOP(controller) */
