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

static void initLowpassFilterAxis(
        gyroLowpassFilter_t * lowpassFilter,
        uint16_t lpfHz, 
        uint8_t axis)
{
    pt1FilterInit(
            &lowpassFilter[axis].pt1FilterState, 
            pt1FilterGain(lpfHz, CORE_DT()));
}

static void initLowpassFilter(
        gyroLowpassFilter_t * lowpassFilter,
        uint16_t lpfHz)
{
    initLowpassFilterAxis(lowpassFilter, lpfHz, 0);
    initLowpassFilterAxis(lowpassFilter, lpfHz, 1);
    initLowpassFilterAxis(lowpassFilter, lpfHz, 2);
}

static void setCalibrationCycles(gyro_t * gyro)
{
    gyro->calibrationCyclesRemaining = (int32_t)calculateCalibratingCycles();
}

static void calibrateAxis(gyro_t * gyro, uint8_t axis)
{
    static float calibrationSum[3];
    static stdev_t calibrationVariance[3];

    // Reset g[axis] at start of calibration
    if (gyro->calibrationCyclesRemaining ==
            (int32_t)calculateCalibratingCycles()) {
        calibrationSum[axis] = 0.0f;
        devClear(&calibrationVariance[axis]);
        // zero is set to zero until calibration complete
        gyro->zero[axis] = 0.0f;
    }

    // Sum up CALIBRATING_GYRO_TIME_US readings
    calibrationSum[axis] += gyroReadRaw(axis);
    devPush(&calibrationVariance[axis], gyroReadRaw(axis));

    if (gyro->calibrationCyclesRemaining == 1) {

        float stddev = devStandardDeviation(&calibrationVariance[axis]);

        // check deviation and startover in case the model was moved
        if (MOVEMENT_CALIBRATION_THRESHOLD && stddev >
                MOVEMENT_CALIBRATION_THRESHOLD) {
            setCalibrationCycles(gyro);
            return;
        }

        // please take care with exotic boardalignment !!
        gyro->zero[axis] = calibrationSum[axis] / calculateCalibratingCycles();
    }
}

static void calibrate(gyro_t * gyro)
{
    calibrateAxis(gyro, 0);
    calibrateAxis(gyro, 1);
    calibrateAxis(gyro, 2);

    --gyro->calibrationCyclesRemaining;
}

static void computeDpsFilteredAxis(
        gyro_t * gyro,
        float sampleSum[3],
        uint8_t axis)
{
    // using gyro lowpass 2 filter for downsampling
    float dpsFiltered = sampleSum[axis];

    // apply static notch filters and software lowpass filters
    dpsFiltered =
        //gyro->lowpassFilterApplyFn((filter_t *)&gyro->lowpassFilter[axis],
        pt1FilterApply((pt1Filter_t *)&gyro->lowpassFilter[axis], dpsFiltered);

    gyro->dpsFiltered[axis] = dpsFiltered;
}

// ============================================================================

void gyroInit(hackflight_t * hf)
{
    gyro_t * gyro = &hf->gyro;

    initLowpassFilter(gyro->lowpassFilter, LPF1_DYN_MIN_HZ);

    initLowpassFilter(gyro->lowpass2Filter, LPF2_STATIC_HZ);

    // Start calibrating
    setCalibrationCycles(gyro);
}

void gyroReadScaled(hackflight_t * hf, vehicleState_t * vstate)
{
    if (!gyroIsReady()) return;

    gyro_t * gyro = &hf->gyro;

    bool calibrationComplete = gyro->calibrationCyclesRemaining <= 0;

    static float dps[3];
    static float sampleSum[3];

    if (calibrationComplete) {

        axes_t adc = {
            gyroReadRaw(0) - gyro->zero[0],
            gyroReadRaw(1) - gyro->zero[1],
            gyroReadRaw(2) - gyro->zero[2]
        };

        hf->imuAlignFun(&adc);

        dps[0] = adc.x * (gyroScaleDps() / 32768.);
        dps[1] = adc.y * (gyroScaleDps() / 32768.);
        dps[2] = adc.z * (gyroScaleDps() / 32768.);

    } else {

        calibrate(gyro);
    }

    // using gyro lowpass 2 filter for downsampling
    sampleSum[0] =
        pt1FilterApply((pt1Filter_t *)&gyro->lowpass2Filter[0], dps[0]);
    sampleSum[1] =
        pt1FilterApply((pt1Filter_t *)&gyro->lowpass2Filter[1], dps[1]);
    sampleSum[2] =
        pt1FilterApply((pt1Filter_t *)&gyro->lowpass2Filter[2], dps[2]);

    computeDpsFilteredAxis(gyro, sampleSum, 0);
    computeDpsFilteredAxis(gyro, sampleSum, 1);
    computeDpsFilteredAxis(gyro, sampleSum, 2);

    gyro->sampleCount = 0;

    // Used for quaternion filter; stubbed otherwise
    imuAccumulateGyro(gyro);

    vstate->dphi   = gyro->dpsFiltered[0];
    vstate->dtheta = gyro->dpsFiltered[1];
    vstate->dpsi   = gyro->dpsFiltered[2];

    gyro->isCalibrating = !calibrationComplete;
}

#if defined(__cplusplus)
}
#endif
