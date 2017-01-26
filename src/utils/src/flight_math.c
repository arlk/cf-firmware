#include "flight_math.h"

#include "math.h"
#include "arm_math.h"

void eulerToRotationZYX(rotation_t* rotation, const attitude_t* euler)
{
  rotation->vals[0][0] = arm_cos_f32(euler->yaw)*arm_cos_f32(euler->pitch);
  rotation->vals[0][1] = arm_cos_f32(euler->yaw)*arm_sin_f32(euler->pitch)*arm_sin_f32(euler->roll)
    - arm_cos_f32(euler->roll)*arm_sin_f32(euler->yaw);
  rotation->vals[0][2] = arm_sin_f32(euler->yaw)*arm_sin_f32(euler->roll)
    + arm_cos_f32(euler->yaw)*arm_cos_f32(euler->roll)*arm_sin_f32(euler->pitch);

  rotation->vals[1][0] = arm_cos_f32(euler->pitch)*arm_sin_f32(euler->yaw);
  rotation->vals[1][1] = arm_sin_f32(euler->yaw)*arm_sin_f32(euler->roll)*arm_sin_f32(euler->pitch)
    + arm_cos_f32(euler->yaw)*arm_cos_f32(euler->roll);
  rotation->vals[1][2] = -arm_cos_f32(euler->yaw)*arm_sin_f32(euler->roll)
    + arm_cos_f32(euler->roll)*arm_sin_f32(euler->yaw)*arm_sin_f32(euler->pitch);

  rotation->vals[2][0] = -arm_sin_f32(euler->pitch);
  rotation->vals[2][1] = arm_cos_f32(euler->pitch)*arm_sin_f32(euler->roll);
  rotation->vals[2][2] = arm_cos_f32(euler->pitch)*arm_cos_f32(euler->roll);
}

void quatToRotationZYX(rotation_t* rotation, const quaternion_t* q)
{
  rotation->vals[0][0] = q->q0 * q->q0 + q->q1 * q->q1 - q->q2 * q->q2 - q->q3 * q->q3;
  rotation->vals[0][1] = 2 * q->q1 * q->q2 - 2 * q->q0 * q->q3;
  rotation->vals[0][2] = 2 * q->q1 * q->q3 + 2 * q->q0 * q->q2;

  rotation->vals[1][0] = 2 * q->q1 * q->q2 + 2 * q->q0 * q->q3;
  rotation->vals[1][1] = q->q0 * q->q0 - q->q1 * q->q1 + q->q2 * q->q2 - q->q3 * q->q3;
  rotation->vals[1][2] = 2 * q->q2 * q->q3 - 2 * q->q0 * q->q1;

  rotation->vals[2][0] = 2 * q->q1 * q->q3 - 2 * q->q0 * q->q2;
  rotation->vals[2][1] = 2 * q->q2 * q->q3 + 2 * q->q0 * q->q1;
  rotation->vals[2][2] = q->q0 * q->q0 - q->q1 * q->q1 - q->q2 * q->q2 + q->q3 * q->q3;
}

void vee_map(float* matrix, float* vec)
{
  vec[0] = -matrix[5];
  vec[1] =  matrix[2];
  vec[2] = -matrix[1];
}
