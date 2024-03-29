#ifndef __LEDRING12_H__
#define __LEDRING12_H__

#include <stdint.h>
#include <stdbool.h>

#define NBR_LEDS  12

extern uint8_t ledringmem[NBR_LEDS * 2];

bool ledEffectEnqueue(uint32_t *ledEffect);

bool ledRGBEnqueue(uint8_t *ledRGB);

#endif //__LEDRING12_H__
