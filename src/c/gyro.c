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

#include "datatypes.h"
#include "filter.h"
#include "filter_pt1.h"
#include "filter_impl.h"
#include "gyro.h"
#include "imu.h"
#include "maths.h"
#include "stats.h"

#if defined(__cplusplus)
extern "C" {
#endif

static const uint16_t CALIBRATION_DURATION           = 125;
static const uint16_t LPF1_DYN_MIN_HZ                = 250;
static const uint8_t  MOVEMENT_CALIBRATION_THRESHOLD = 48;
static const uint16_t LPF2_STATIC_HZ                 = 500;

enum {
    FILTER_LPF1 = 0,
    FILTER_LPF2
};

typedef union {
    pt1Filter_t pt1FilterState;
    biquadFilter_t biquadFilterState;
    pt2Filter_t pt2FilterState;
    pt3Filter_t pt3FilterState;
} gyroLowpassFilter_t;

typedef struct {
    float sum[3];
    stdev_t var[3];
    int32_t cyclesRemaining;
} gyroCalibration_t;

typedef struct {

    float dps[3];         // aligned, calibrated, scaled, but unfiltered data from sensor
    float dps_filtered[3];        // filtered gyro data
    float dps_filtered_prev[3];
    uint8_t sampleCount;  // gyro sensor sample counter
    float sampleSum[3];   // summed samples used for downsampling

    // if true then downsample using gyro lowpass 2, otherwise use averaging
    bool downsampleFilterEnabled;      

    gyroCalibration_t calibration;

    // lowpass gyro soft filter
    filterApplyFnPtr lowpassFilterApplyFn;
    gyroLowpassFilter_t lowpassFilter[3];

    // lowpass2 gyro soft filter
    filterApplyFnPtr lowpass2FilterApplyFn;
    gyroLowpassFilter_t lowpass2Filter[3];

    float zero[3];

} gyro_t;

static int32_t gyroCalculateCalibratingCycles(void)
{
    return (CALIBRATION_DURATION * 10000) / GYRO_PERIOD();
}

static float nullFilterApply(filter_t *filter, float input)
{
    (void)filter;
    return input;
}

static bool initLowpassFilterLpf(gyro_t * gyro, int slot, uint16_t lpfHz, uint32_t looptime)
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
            return false;
    }

    bool ret = false;

    // Establish some common constants
    const float gyroDt = looptime * 1e-6f;

    // Gain could be calculated a little later as it is specific to the
    // pt1/bqrcf2/fkf branches
    const float gain = pt1FilterGain(lpfHz, gyroDt);

    // Dereference the pointer to null before checking valid cutoff and filter
    // type. It will be overridden for positive cases.
    *lowpassFilterApplyFn = nullFilterApply;

    // If lowpass cutoff has been specified
    if (lpfHz) {
        *lowpassFilterApplyFn = (filterApplyFnPtr) pt1FilterApply;
        for (int axis = 0; axis < 3; axis++) {
            pt1FilterInit(&lowpassFilter[axis].pt1FilterState, gain);
        }
        ret = true;
    }
    return ret;
}

static void setCalibrationCycles(gyro_t * gyro)
{
    gyro->calibration.cyclesRemaining = gyroCalculateCalibratingCycles();
}

static void performGyroCalibration(gyro_t * gyro)
{
    for (int axis = 0; axis < 3; axis++) {
        // Reset g[axis] at start of calibration
        if (gyro->calibration.cyclesRemaining == gyroCalculateCalibratingCycles()) {
            gyro->calibration.sum[axis] = 0.0f;
            devClear(&gyro->calibration.var[axis]);
            // zero is set to zero until calibration complete
            gyro->zero[axis] = 0.0f;
        }

        // Sum up CALIBRATING_GYRO_TIME_US readings
        gyro->calibration.sum[axis] += gyroReadRaw(axis);
        devPush(&gyro->calibration.var[axis], gyroReadRaw(axis));

        if (gyro->calibration.cyclesRemaining == 1) {
            const float stddev = devStandardDeviation(&gyro->calibration.var[axis]);

            // check deviation and startover in case the model was moved
            if (MOVEMENT_CALIBRATION_THRESHOLD && stddev > MOVEMENT_CALIBRATION_THRESHOLD) {
                setCalibrationCycles(gyro);
                return;
            }

            // please take care with exotic boardalignment !!
            gyro->zero[axis] =
                gyro->calibration.sum[axis] / gyroCalculateCalibratingCycles();
        }
    }

    --gyro->calibration.cyclesRemaining;
}

static gyro_t _gyro;

static void gyroUpdate(vehicle_state_t * state, bool * is_calibrating)
{
    bool calibrationComplete = _gyro.calibration.cyclesRemaining <= 0;

    static float _adc[3];

    if (calibrationComplete) {
        // move 16-bit gyro data into floats to avoid overflows in calculations

        _adc[0] = gyroReadRaw(0) - _gyro.zero[0];
        _adc[1] = gyroReadRaw(1) - _gyro.zero[1];
        _adc[2] = gyroReadRaw(2) - _gyro.zero[2];

        alignSensorViaRotation(_adc);
    } else {
        performGyroCalibration(&_gyro);
    }

    if (calibrationComplete) {
        _gyro.dps[0] = _adc[0] * gyroScale();
        _gyro.dps[1] = _adc[1] * gyroScale();
        _gyro.dps[2] = _adc[2] * gyroScale();
    }

    if (_gyro.downsampleFilterEnabled) {
        // using gyro lowpass 2 filter for downsampling
        _gyro.sampleSum[0] =
            _gyro.lowpass2FilterApplyFn((filter_t *)&_gyro.lowpass2Filter[0], _gyro.dps[0]);
        _gyro.sampleSum[1] =
            _gyro.lowpass2FilterApplyFn((filter_t *)&_gyro.lowpass2Filter[1], _gyro.dps[1]);
        _gyro.sampleSum[2] =
            _gyro.lowpass2FilterApplyFn((filter_t *)&_gyro.lowpass2Filter[2], _gyro.dps[2]);
    } else {
        // using simple averaging for downsampling
        _gyro.sampleSum[0] += _gyro.dps[0];
        _gyro.sampleSum[1] += _gyro.dps[1];
        _gyro.sampleSum[2] += _gyro.dps[2];
        _gyro.sampleCount++;
    }


    for (int axis = 0; axis < 3; axis++) {

        // downsample the individual gyro samples
        float dps_filtered = 0;
        if (_gyro.downsampleFilterEnabled) {
            // using gyro lowpass 2 filter for downsampling
            dps_filtered = _gyro.sampleSum[axis];
        } else {
            // using simple average for downsampling
            if (_gyro.sampleCount) {
                dps_filtered = _gyro.sampleSum[axis] / _gyro.sampleCount;
            }
            _gyro.sampleSum[axis] = 0;
        }

        // apply static notch filters and software lowpass filters
        dps_filtered =
            _gyro.lowpassFilterApplyFn((filter_t *)&_gyro.lowpassFilter[axis],
                    dps_filtered);

        _gyro.dps_filtered[axis] = dps_filtered;
    }

    _gyro.sampleCount = 0;


    // Used for quaternion filter; stubbed otherwise
    imuAccumulateGyro(_gyro.dps_filtered);

    state->dphi   = _gyro.dps_filtered[0];
    state->dtheta = _gyro.dps_filtered[1];
    state->dpsi   = _gyro.dps_filtered[2];

    *is_calibrating = !calibrationComplete;
}

// ============================================================================

void gyroInit(void)
{
    initLowpassFilterLpf(&_gyro, FILTER_LPF1, LPF1_DYN_MIN_HZ, GYRO_PERIOD());

    _gyro.downsampleFilterEnabled = initLowpassFilterLpf(
            &_gyro,
            FILTER_LPF2,
            LPF2_STATIC_HZ,
            GYRO_PERIOD()
            );

    setCalibrationCycles(&_gyro); // start calibrating
}


void gyroReadScaled(vehicle_state_t * state, bool * isCalibrating)
{
    if (gyroIsReady()) {
        gyroUpdate(state, isCalibrating);
    }
}

#if defined(__cplusplus)
}
#endif
