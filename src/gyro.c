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

#include "core_dt.h"
#include "datatypes.h"
#include "debug.h"
#include "pt1_filter.h"
#include "gyro.h"
#include "imu.h"
#include "maths.h"
#include "stats.h"
#include "system.h"
#include "time.h"

static const uint32_t CALIBRATION_DURATION           = 1250000;
static const uint16_t LPF1_DYN_MIN_HZ                = 250;
static const uint8_t  MOVEMENT_CALIBRATION_THRESHOLD = 48;
static const uint16_t LPF2_STATIC_HZ                 = 500;

#if defined(__cplusplus)
extern "C" {
#endif

static uint32_t calculateCalibratingCycles(void)
{
    return CALIBRATION_DURATION / CORE_PERIOD();
}

static float nullFilterApply(filter_t *filter, float input)
{
    (void)filter;
    return input;
}

static void initLowpassFilter( gyro_t * gyro, int slot, uint16_t lpfHz)
{
    filterApplyFnPtr *lowpassFilterApplyFn;
    gyroLowpassFilter_t *lowpassFilter = NULL;

    switch (slot) {
        case FILTER_LPF1:
            lowpassFilterApplyFn = &gyro->lowpassFilterApplyFn;
            lowpassFilter = gyro->lowpassFilter;
            break;

        case FILTER_LPF2:
            lowpassFilterApplyFn = &gyro->lowpass2FilterApplyFn;
            lowpassFilter = gyro->lowpass2Filter;
            break;

        default:
            return;
    }

    const float gain = pt1FilterGain(lpfHz, CORE_DT());

    // Dereference the pointer to null before checking valid cutoff and filter
    // type. It will be overridden for positive cases.
    *lowpassFilterApplyFn = nullFilterApply;

    // If lowpass cutoff has been specified
    *lowpassFilterApplyFn = (filterApplyFnPtr) pt1FilterApply;
    for (int axis = 0; axis < 3; axis++) {
        pt1FilterInit(&lowpassFilter[axis].pt1FilterState, gain);
    }
}

static void setCalibrationCycles(gyro_t * gyro)
{
    gyro->calibrationCyclesRemaining = (int32_t)calculateCalibratingCycles();
}

static void calibrateAxis(gyro_t * gyro, uint8_t axis)
{
    // Reset g[axis] at start of calibration
    if (gyro->calibrationCyclesRemaining == (int32_t)calculateCalibratingCycles()) {
        gyro->calibrationSum[axis] = 0.0f;
        devClear(&gyro->calibrationVariance[axis]);
        // zero is set to zero until calibration complete
        gyro->zero[axis] = 0.0f;
    }

    // Sum up CALIBRATING_GYRO_TIME_US readings
    gyro->calibrationSum[axis] += gyroReadRaw(axis);
    devPush(&gyro->calibrationVariance[axis], gyroReadRaw(axis));

    if (gyro->calibrationCyclesRemaining == 1) {
        const float stddev = devStandardDeviation(&gyro->calibrationVariance[axis]);

        // check deviation and startover in case the model was moved
        if (MOVEMENT_CALIBRATION_THRESHOLD && stddev >
                MOVEMENT_CALIBRATION_THRESHOLD) {
            setCalibrationCycles(gyro);
            return;
        }

        // please take care with exotic boardalignment !!
        gyro->zero[axis] =
            gyro->calibrationSum[axis] / calculateCalibratingCycles();
    }
}

static void calibrate(gyro_t * gyro)
{
    for (int axis = 0; axis < 3; axis++) {
        calibrateAxis(gyro, axis);
    }

    --gyro->calibrationCyclesRemaining;
}

// ============================================================================

void gyroInit(hackflight_t * hf)
{
    gyro_t * gyro = &hf->gyro;

    initLowpassFilter(gyro, FILTER_LPF1, LPF1_DYN_MIN_HZ);

    gyro->downsampleFilterEnabled = true;

    initLowpassFilter( gyro, FILTER_LPF2, LPF2_STATIC_HZ);

    setCalibrationCycles(gyro); // start calibrating
}

void gyroReadScaled(hackflight_t * hf, vehicleState_t * vstate)
{
    if (!gyroIsReady()) return;

    gyro_t * gyro = &hf->gyro;

    bool calibrationComplete = gyro->calibrationCyclesRemaining <= 0;

    if (calibrationComplete) {

        axes_t adc = {
            gyroReadRaw(0) - gyro->zero[0],
            gyroReadRaw(1) - gyro->zero[1],
            gyroReadRaw(2) - gyro->zero[2]
        };

        hf->imuAlignFun(&adc);

        gyro->dps[0] = adc.x * (gyroScaleDps() / 32768.);
        gyro->dps[1] = adc.y * (gyroScaleDps() / 32768.);
        gyro->dps[2] = adc.z * (gyroScaleDps() / 32768.);

    } else {

        calibrate(gyro);
    }

    if (gyro->downsampleFilterEnabled) {
        // using gyro lowpass 2 filter for downsampling
        gyro->sampleSum[0] = gyro->lowpass2FilterApplyFn(
                (filter_t *)&gyro->lowpass2Filter[0], gyro->dps[0]);
        gyro->sampleSum[1] = gyro->lowpass2FilterApplyFn(
                (filter_t *)&gyro->lowpass2Filter[1], gyro->dps[1]);
        gyro->sampleSum[2] = gyro->lowpass2FilterApplyFn(
                (filter_t *)&gyro->lowpass2Filter[2], gyro->dps[2]);
    } else {
        // using simple averaging for downsampling
        gyro->sampleSum[0] += gyro->dps[0];
        gyro->sampleSum[1] += gyro->dps[1];
        gyro->sampleSum[2] += gyro->dps[2];
        gyro->sampleCount++;
    }

    for (int axis = 0; axis < 3; axis++) {

        // downsample the individual gyro samples
        float dps_filtered = 0;
        if (gyro->downsampleFilterEnabled) {
            // using gyro lowpass 2 filter for downsampling
            dps_filtered = gyro->sampleSum[axis];
        } else {
            // using simple average for downsampling
            if (gyro->sampleCount) {
                dps_filtered = gyro->sampleSum[axis] / gyro->sampleCount;
            }
            gyro->sampleSum[axis] = 0;
        }

        // apply static notch filters and software lowpass filters
        dps_filtered =
            gyro->lowpassFilterApplyFn((filter_t *)&gyro->lowpassFilter[axis],
                    dps_filtered);

        gyro->dps_filtered[axis] = dps_filtered;
    }

    gyro->sampleCount = 0;


    // Used for quaternion filter; stubbed otherwise
    imuAccumulateGyro(gyro);

    vstate->dphi   = gyro->dps_filtered[0];
    vstate->dtheta = gyro->dps_filtered[1];
    vstate->dpsi   = gyro->dps_filtered[2];

    gyro->isCalibrating = !calibrationComplete;
}

#if defined(__cplusplus)
}
#endif
