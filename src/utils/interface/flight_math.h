#ifndef FLIGHT_MATH_H_
#define FLIGHT_MATH_H_

#include <stdbool.h>
#include <stdint.h>

#include "stabilizer_types.h"

void eulerToRotationZYX(rotation_t* rotation, const attitude_t* euler);

void quatToRotationZYX(rotation_t* rotation, const quaternion_t* q);

void vee_map(float* matrix, float* vec);

#endif // FLIGHT_MATH_H_
