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
#include "filter.h"
#include "maths.h"
#include "macros.h"

// static const uint16_t CALIBRATING_ACC_CYCLES = 400;
static const uint16_t LPF_CUTOFF_HZ = 10;

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

// ============================================================================

void accelUpdate(imu_sensor_t * accum)
{
    static float adc[3];
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

    for (int k = 0; k < 3; k++) {
        adc[k] = accelRead(k);
    }

    for (int k = 0; k < 3; k++) {
        adc[k] = biquadFilterApply(&filter[k], adc[k]);
    }

    alignSensorViaRotation(adc);

    // if (calibrating != 0) {
    //     calibrate(acc, adc);
    // }

    // adc[0] -= acc->trims->raw[0];
    // adc[1] -= acc->trims->raw[1];
    // adc[2] -= acc->trims->raw[2];

    accum->values.x += adc[0];
    accum->values.y += adc[1];
    accum->values.z += adc[2];
    accum->count++;
}
