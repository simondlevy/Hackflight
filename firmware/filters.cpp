/*
   filters.cpp : filter function implementations

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

#ifdef __arm__
extern "C" {
#endif

#include "hackflight.hpp"

// complementary filter
float complementaryFilter(float a, float b, float c) 
{
    return a * c + b * (1 - c);
}

// deadband filter
int32_t deadbandFilter(int32_t value, int32_t deadband)
{
    if (abs(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

// PT1 Low Pass filter (when no dT specified it will be calculated from the cycleTime)
float filterApplyPt1(float input, filterStatePt1_t *filter, uint8_t f_cut, float dT) {

	// Pre calculate and store RC
	if (!filter->RC) {
		filter->RC = 1.0f / ( 2.0f * (float)M_PI * f_cut );
	}

    filter->state = filter->state + dT / (filter->RC + dT) * (input - filter->state);

    return filter->state;
}

#define M_LN2_FLOAT	0.69314718055994530942f
#define M_PI_FLOAT	3.14159265358979323846f


#define BIQUAD_BANDWIDTH 1.9f     /* bandwidth in octaves */

#define GYRO_SAMPLE_PERIOD     1000  // XXX should be determined empirically
#define GYRO_SYNC_DENOMINATOR  4     // Sample every 4th gyro measurement 2khz

/* sets up a biquad Filter */
void BiQuadNewLpf(uint8_t filterCutFreq, biquad_t *newState, float refreshRate)
{
    uint32_t targetLooptime = GYRO_SYNC_DENOMINATOR * GYRO_SAMPLE_PERIOD;

	float samplingRate;
    samplingRate = 1 / (targetLooptime * 0.000001f);

    if (!refreshRate) {
    	samplingRate = 1 / (targetLooptime * 0.000001f);
    } else {
    	samplingRate = refreshRate;
    }

    float omega, sn, cs, alpha;
    float a0, a1, a2, b0, b1, b2;

    /* setup variables */
    omega = 2 * M_PI_FLOAT * (float) filterCutFreq / samplingRate;
    sn = sinf(omega);
    cs = cosf(omega);
    alpha = sn * sinf(M_LN2_FLOAT /2 * BIQUAD_BANDWIDTH * omega /sn);

    b0 = (1 - cs) /2;
    b1 = 1 - cs;
    b2 = (1 - cs) /2;
    a0 = 1 + alpha;
    a1 = -2 * cs;
    a2 = 1 - alpha;

    /* precompute the coefficients */
    newState->a0 = b0 /a0;
    newState->a1 = b1 /a0;
    newState->a2 = b2 /a0;
    newState->a3 = a1 /a0;
    newState->a4 = a2 /a0;

    /* zero initial samples */
    newState->x1 = newState->x2 = 0;
    newState->y1 = newState->y2 = 0;

}

/* Computes a biquad_t filter on a sample */
float applyBiQuadFilter(float sample, biquad_t *state)
{
    float result;

    /* compute result */
    result = state->a0 * sample + state->a1 * state->x1 + state->a2 * state->x2 -
        state->a3 * state->y1 - state->a4 * state->y2;

    /* shift x1 to x2, sample to x1 */
    state->x2 = state->x1;
    state->x1 = sample;

    /* shift y1 to y2, result to y1 */
    state->y2 = state->y1;
    state->y1 = result;

    return result;
}
#ifdef __arm__
} // extern "C"
#endif
