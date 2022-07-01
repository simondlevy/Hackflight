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

#include "accel.h"
#include "arming.h"
#include "datatypes.h"
#include "maths.h"

// static const uint16_t CALIBRATING_ACC_CYCLES = 400;
static const uint16_t LPF_CUTOFF_HZ = 10;

// quality factor - 2nd order Butterworth = 1/sqrt(2)
static const float BIQUAD_Q = 0.7071067811865475;     

/*
   static void calibrate(accel_t * acc, float adc[3])
   {
   static int32_t a[3];

   for (int axis = 0; axis < 3; axis++) {

// Reset a[axis] at start of calibration
if (calibrating == CALIBRATING_ACC_CYCLES) {
a[axis] = 0;
}

// Sum up CALIBRATING_ACC_CYCLES readings
a[axis] += adc[axis];

// Reset global variables to prevent other code from using un-calibrated data
adc[axis] = 0;
acc->trims->raw[axis] = 0;
}

if (calibrating == 1) {
// Calculate average, shift Z down by acc_1G and store values in EEPROM
// at end of calibration
acc->trims->raw[0] = (a[0] + (CALIBRATING_ACC_CYCLES / 2)) /
CALIBRATING_ACC_CYCLES;
acc->trims->raw[1] = (a[1] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
acc->trims->raw[2] = (a[2] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES -
ACCEL_1G;

}

calibrating--;
}
 */

#if defined(__cplusplus)
extern "C" {
#endif


// Computes a biquadFilter_t filter in direct form 2 on a sample (higher
// precision but can't handle changes in coefficients
static float biquadFilterApply(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->x1;

    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;

    return result;
}

static void biquadFilterUpdate(
        biquadFilter_t *filter,
        float filterFreq,
        uint32_t refreshRate,
        float Q,
        biquadFilterType_e filterType,
        float weight)
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

static void biquadFilterInit(biquadFilter_t *filter,
        float filterFreq,
        uint32_t refreshRate,
        float Q,
        biquadFilterType_e filterType,
        float weight)
{
    biquadFilterUpdate(filter, filterFreq, refreshRate, Q, filterType, weight);

    // zero initial samples
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

// sets up a biquad filter as a 2nd order butterworth LPF
static void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate)
{
    biquadFilterInit(filter, filterFreq, refreshRate, BIQUAD_Q, FILTER_LPF, 1.0f);
}



// ============================================================================

void accelUpdate(imu_align_fun align, imu_sensor_t * accum)
{
    static axes_t adc;
    static bool initialized;
    static biquadFilter_t filter[3];
    // static flightDynamicsTrims_t * trims;

    // the calibration is done is the main loop. Calibrating decreases at each
    // cycle down to 0, then we enter in a normal mode.
    // uint16_t calibrating;      

    if (!initialized) {
        const uint32_t accSampleTimeUs = 1e6 / ACCEL_RATE;
        biquadFilterInitLPF(&filter[0], LPF_CUTOFF_HZ, accSampleTimeUs); 
        biquadFilterInitLPF(&filter[1], LPF_CUTOFF_HZ, accSampleTimeUs); 
        biquadFilterInitLPF(&filter[2], LPF_CUTOFF_HZ, accSampleTimeUs); 
    }
    initialized = true;

    if (!accelIsReady()) {
        return;
    }

    adc.x = 0;//accelRead(0);
    adc.y = 0;//accelRead(1);
    adc.z = 0;//accelRead(2);

    adc.x = biquadFilterApply(&filter[0], adc.x);
    adc.y = biquadFilterApply(&filter[1], adc.y);
    adc.z = biquadFilterApply(&filter[2], adc.z);

    align(&adc);

    // if (calibrating != 0) {
    //     calibrate(acc, adc);
    // }

    // adc[0] -= acc->trims->raw[0];
    // adc[1] -= acc->trims->raw[1];
    // adc[2] -= acc->trims->raw[2];

    accum->values.x += adc.x;
    accum->values.y += adc.y;
    accum->values.z += adc.z;
    accum->count++;
}

#if defined(__cplusplus)
}
#endif


