/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float state;
    float state1;
    float k;
} pt2Filter_t;

typedef struct {
    float state;
    float state1;
    float state2;
    float k;
} pt3Filter_t;

typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    float weight;
} biquadFilter_t;

enum {
    FILTER_LPF1 = 0,
    FILTER_LPF2
};

typedef enum {
    FILTER_LPF,    // 2nd order Butterworth section
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;

typedef enum {
    FILTER_PT1 = 0,
    FILTER_BIQUAD,
    FILTER_PT2,
    FILTER_PT3,
} lowpassFilterType_e;

struct filter_s;
typedef struct filter_s filter_t;
typedef float (*filterApplyFnPtr)(filter_t *filter, float input);
