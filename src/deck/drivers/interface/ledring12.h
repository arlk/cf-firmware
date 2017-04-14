#ifndef __LEDRING12_H__
#define __LEDRING12_H__

#include <stdint.h>
#include <stdbool.h>

#define NBR_LEDS  12

extern uint8_t ledringmem[NBR_LEDS * 2];

bool ledWriteQueue(uint32_t *ledEffect);

#endif //__LEDRING12_H__
