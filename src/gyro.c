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

static void initLowpassFilter(
        gyroLowpassFilter_t * lowpassFilter,
        uint16_t lpfHz)
{
    pt1FilterInit(
            &lowpassFilter->pt1FilterState, pt1FilterGain(lpfHz, CORE_DT()));
}

static void setCalibrationCycles(gyro_t * gyro)
{
    gyro->calibrationCyclesRemaining = (int32_t)calculateCalibratingCycles();
}

static void calibrateAxis(
        gyro_t * gyro,
        gyroAxis_t * axis,
        uint8_t index)
{
    static float calibrationSum[3];
    static stdev_t calibrationVariance[3];

    if (gyro->calibrationCyclesRemaining ==
            (int32_t)calculateCalibratingCycles()) {
        calibrationSum[index] = 0;
        devClear(&calibrationVariance[index]);
        axis->zero = 0;
    }

    // Sum up CALIBRATING_GYRO_TIME_US readings
    calibrationSum[index] += gyroReadRaw(index);
    devPush(&calibrationVariance[index], gyroReadRaw(index));

    if (gyro->calibrationCyclesRemaining == 1) {

        float stddev = devStandardDeviation(&calibrationVariance[index]);

        // check deviation and startover in case the model was moved
        if (MOVEMENT_CALIBRATION_THRESHOLD && stddev >
                MOVEMENT_CALIBRATION_THRESHOLD) {
            setCalibrationCycles(gyro);
            return;
        }

        // please take care with exotic boardalignment !!
        axis->zero = calibrationSum[index] / calculateCalibratingCycles();
    }
}

static void calibrate(gyro_t * gyro)
{
    calibrateAxis(gyro, &gyro->x, 0);
    calibrateAxis(gyro, &gyro->y, 1);
    calibrateAxis(gyro, &gyro->z, 2);

    --gyro->calibrationCyclesRemaining;
}

static void computeDpsFilteredAxis(
        gyro_t * gyro,
        float sampleSum,
        gyroAxis_t * axis)
{
    axis->dpsFiltered =
        pt1FilterApply((pt1Filter_t *)&axis->lowpassFilter, sampleSum);
}

// ============================================================================

void gyroInit(hackflight_t * hf)
{
    gyro_t * gyro = &hf->gyro;

    initLowpassFilter(&gyro->x.lowpassFilter, LPF1_DYN_MIN_HZ);
    initLowpassFilter(&gyro->y.lowpassFilter, LPF1_DYN_MIN_HZ);
    initLowpassFilter(&gyro->z.lowpassFilter, LPF1_DYN_MIN_HZ);

    initLowpassFilter(&gyro->x.lowpass2Filter, LPF2_STATIC_HZ);
    initLowpassFilter(&gyro->y.lowpass2Filter, LPF2_STATIC_HZ);
    initLowpassFilter(&gyro->z.lowpass2Filter, LPF2_STATIC_HZ);

    // Start calibrating
    setCalibrationCycles(gyro);
}

void gyroReadScaled(hackflight_t * hf, vehicleState_t * vstate)
{
    if (!gyroIsReady()) return;

    gyro_t * gyro = &hf->gyro;

    bool calibrationComplete = gyro->calibrationCyclesRemaining <= 0;

    static axes_t _dps;
    static axes_t _sampleSum;

    if (calibrationComplete) {

        axes_t adc = {
            gyroReadRaw(0) - gyro->x.zero,
            gyroReadRaw(1) - gyro->y.zero,
            gyroReadRaw(2) - gyro->z.zero
        };

        hf->imuAlignFun(&adc);

        _dps.x = adc.x * (gyroScaleDps() / 32768.);
        _dps.y = adc.y * (gyroScaleDps() / 32768.);
        _dps.z = adc.z * (gyroScaleDps() / 32768.);

    } else {

        calibrate(gyro);
    }

    // using gyro lowpass 2 filter for downsampling
    _sampleSum.x =
        pt1FilterApply((pt1Filter_t *)&gyro->x.lowpass2Filter, _dps.x);
    _sampleSum.y =
        pt1FilterApply((pt1Filter_t *)&gyro->y.lowpass2Filter, _dps.y);
    _sampleSum.z =
        pt1FilterApply((pt1Filter_t *)&gyro->z.lowpass2Filter, _dps.z);

    computeDpsFilteredAxis(gyro, _sampleSum.x, &gyro->x);
    computeDpsFilteredAxis(gyro, _sampleSum.y, &gyro->y);
    computeDpsFilteredAxis(gyro, _sampleSum.z, &gyro->z);

    gyro->sampleCount = 0;

    // Used for quaternion filter; stubbed otherwise
    imuAccumulateGyro(gyro);

    vstate->dphi   = gyro->x.dpsFiltered;
    vstate->dtheta = gyro->y.dpsFiltered;
    vstate->dpsi   = gyro->z.dpsFiltered;

    gyro->isCalibrating = !calibrationComplete;
}

#if defined(__cplusplus)
}
#endif
