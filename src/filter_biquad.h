/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "filter.h"
#include "maths.h"
#include "utils.h"

static void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight)
{
    // setup variables
    const float omega = 2.0f * M_PI * filterFreq * refreshRate * 0.000001f;
    const float sn = sin_approx(omega);
    const float cs = cos_approx(omega);
    const float alpha = sn / (2.0f * Q);

    switch (filterType) {
        case FILTER_LPF:
            // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
            // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
            filter->b1 = 1 - cs;
            filter->b0 = filter->b1 * 0.5f;
            filter->b2 = filter->b0;
            filter->a1 = -2 * cs;
            filter->a2 = 1 - alpha;
            break;
        case FILTER_NOTCH:
            filter->b0 = 1;
            filter->b1 = -2 * cs;
            filter->b2 = 1;
            filter->a1 = filter->b1;
            filter->a2 = 1 - alpha;
            break;
        case FILTER_BPF:
            filter->b0 = alpha;
            filter->b1 = 0;
            filter->b2 = -alpha;
            filter->a1 = -2 * cs;
            filter->a2 = 1 - alpha;
            break;
    }

    const float a0 = 1 + alpha;

    // precompute the coefficients
    filter->b0 /= a0;
    filter->b1 /= a0;
    filter->b2 /= a0;
    filter->a1 /= a0;
    filter->a2 /= a0;

    // update weight
    filter->weight = weight;
}

static void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight)
{
    biquadFilterUpdate(filter, filterFreq, refreshRate, Q, filterType, weight);

    // zero initial samples
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

// --------------------------------------------------------------------------------------------------

// Computes a biquadFilter_t filter in direct form 2 on a sample (higher
// precision but can't handle changes in coefficients
static float biquadFilterApply(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->x1;

    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;

    return result;
}

// sets up a biquad filter as a 2nd order butterworth LPF
static void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate)
{
    biquadFilterInit(filter, filterFreq, refreshRate, BIQUAD_Q, FILTER_LPF, 1.0f);
}


