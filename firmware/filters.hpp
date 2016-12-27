/*
   filters.hpp : filter function declarations

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef __arm__
extern "C" {
#else
#include <stdio.h>
#endif

// complementary filter
float complementaryFilter(float a, float b, float c);

// deadband filter
int32_t deadbandFilter(int32_t value, int32_t deadband);

/* this holds the data required to update samples thru a filter */
typedef struct biquad_s {
    float a0, a1, a2, a3, a4;
    float x1, x2, y1, y2;
} biquad_t;

typedef struct filterStatePt1_s {
	float state;
	float RC;
	float constdT;
} filterStatePt1_t;

float filterApplyPt1(float input, filterStatePt1_t *filter, uint8_t f_cut, float dt);
float applyBiQuadFilter(float sample, biquad_t *state);
void  BiQuadNewLpf(uint8_t filterCutFreq, biquad_t *newState, float refreshRate);

#ifdef __arm__
} // extern "C"
#endif

