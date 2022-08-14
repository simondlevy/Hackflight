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

#pragma once

#include <stdlib.h>

#include "core_rate.h"
#include "datatypes.h"
#include "filters/pt1.h"
#include "imu.h"
#include "maths.h"
#include "stats.h"
#include "system.h"
#include "time.h"

#if defined(__cplusplus)
extern "C" {
#endif

    void     gyroDevInit(void);
    uint32_t gyroDevInterruptCount(void);
    bool     gyroDevIsReady(void);
    int16_t  gyroDevReadRaw(uint8_t k);
    uint16_t gyroDevScaleDps(void);

    void     gyroInit(gyro_t * gyro);
    void     gyroReadScaled(gyro_t *gyro, imu_align_fun align, vehicle_state_t * vstate);
    uint32_t gyroSyncTime(void);

#if defined(__cplusplus)
}
#endif

class Gyro {

    private:

        static const uint32_t CALIBRATION_DURATION           = 1250000;
        static const uint16_t LPF1_DYN_MIN_HZ                = 250;
        static const uint8_t  MOVEMENT_CALIBRATION_THRESHOLD = 48;
        static const uint16_t LPF2_STATIC_HZ                 = 500;

        imu_sensor_t m_accum;
        float m_dps[3];            // aligned, calibrated, scaled, unfiltered
        float  m_dps_filtered[3];  // filtered 
        uint8_t m_sampleCount;     // sample counter
        float m_sampleSum[3];      // summed samples used for downsampling
        bool  m_isCalibrating;

        // if true then downsample using gyro lowpass 2, otherwise use averaging
        bool m_downsampleFilterEnabled;      

        gyroCalibration_t m_calibration;

        // lowpass gyro soft filter
        filterApplyFnPtr m_lowpassFilterApplyFn;
        gyroLowpassFilter_t m_lowpassFilter[3];

        // lowpass2 gyro soft filter
        filterApplyFnPtr m_lowpass2FilterApplyFn;
        gyroLowpassFilter_t m_lowpass2Filter[3];

        float m_zero[3];

        static uint32_t calculateCalibratingCycles(void)
        {
            return CALIBRATION_DURATION / CORE_PERIOD();
        }

        static float nullFilterApply(filter_t *filter, float input)
        {
            (void)filter;
            return input;
        }

        bool initLowpassFilterLpf(
                int slot,
                uint16_t lpfHz,
                uint32_t looptime)
        {
            filterApplyFnPtr *lowpassFilterApplyFn;
            gyroLowpassFilter_t *lowpassFilter = NULL;

            switch (slot) {
                case FILTER_LPF1:
                    lowpassFilterApplyFn = &m_lowpassFilterApplyFn;
                    lowpassFilter = m_lowpassFilter;
                    break;

                case FILTER_LPF2:
                    lowpassFilterApplyFn = &m_lowpass2FilterApplyFn;
                    lowpassFilter = m_lowpass2Filter;
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

            // Dereference the pointer to null before checking valid cutoff and
            // filter type. It will be overridden for positive cases.
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

        void setCalibrationCycles(void)
        {
            m_calibration.cyclesRemaining = (int32_t)calculateCalibratingCycles();
        }

        void calibrate(void)
        {
            for (int axis = 0; axis < 3; axis++) {
                // Reset g[axis] at start of calibration
                if (m_calibration.cyclesRemaining ==
                        (int32_t)calculateCalibratingCycles()) {
                    m_calibration.sum[axis] = 0.0f;
                    devClear(&m_calibration.var[axis]);
                    // zero is set to zero until calibration complete
                    m_zero[axis] = 0.0f;
                }

                // Sum up CALIBRATING_GYRO_TIME_US readings
                m_calibration.sum[axis] += gyroDevReadRaw(axis);
                devPush(&m_calibration.var[axis], gyroDevReadRaw(axis));

                if (m_calibration.cyclesRemaining == 1) {
                    const float stddev =
                        devStandardDeviation(&m_calibration.var[axis]);

                    // check deviation and startover in case the model was moved
                    if (MOVEMENT_CALIBRATION_THRESHOLD && stddev >
                            MOVEMENT_CALIBRATION_THRESHOLD) {
                        setCalibrationCycles();
                        return;
                    }

                    // please take care with exotic boardalignment !!
                    m_zero[axis] =
                        m_calibration.sum[axis] / calculateCalibratingCycles();
                }
            }

            --m_calibration.cyclesRemaining;
        }

        void accumulate(void)
        {
            static float _adcf[3];

            // integrate using trapezium rule to avoid bias
            m_accum.values.x +=0.5f * (_adcf[0] + m_dps_filtered[0]) * CORE_PERIOD();
            m_accum.values.y += 0.5f * (_adcf[1] + m_dps_filtered[1]) * CORE_PERIOD();
            m_accum.values.z += 0.5f * (_adcf[2] + m_dps_filtered[2]) * CORE_PERIOD();

            m_accum.count++;

            for (int axis = 0; axis < 3; axis++) {
                _adcf[axis] = m_dps_filtered[axis];
            }
        }


    public:

        Gyro(void)
        {
            initLowpassFilterLpf(FILTER_LPF1, LPF1_DYN_MIN_HZ, CORE_PERIOD());

            m_downsampleFilterEnabled = initLowpassFilterLpf(
                    FILTER_LPF2,
                    LPF2_STATIC_HZ,
                    CORE_PERIOD()
                    );

            setCalibrationCycles(); // start calibrating
        }

        void readScaled(imu_align_fun align, vehicle_state_t * vstate)
        {
            if (!gyroDevIsReady()) return;

            bool calibrationComplete = m_calibration.cyclesRemaining <= 0;

            static axes_t _adc;

            if (calibrationComplete) {

                // move 16-bit gyro data into floats to avoid overflows in
                // calculations

                _adc.x = gyroDevReadRaw(0) - m_zero[0];
                _adc.y = gyroDevReadRaw(1) - m_zero[1];
                _adc.z = gyroDevReadRaw(2) - m_zero[2];

                align(&_adc);

            } else {
                calibrate();
            }

            if (calibrationComplete) {
                m_dps[0] = _adc.x * (gyroDevScaleDps() / 32768.);
                m_dps[1] = _adc.y * (gyroDevScaleDps() / 32768.);
                m_dps[2] = _adc.z * (gyroDevScaleDps() / 32768.);
            }

            if (m_downsampleFilterEnabled) {
                // using gyro lowpass 2 filter for downsampling
                m_sampleSum[0] = m_lowpass2FilterApplyFn(
                        (filter_t *)&m_lowpass2Filter[0], m_dps[0]);
                m_sampleSum[1] = m_lowpass2FilterApplyFn(
                        (filter_t *)&m_lowpass2Filter[1], m_dps[1]);
                m_sampleSum[2] = m_lowpass2FilterApplyFn(
                        (filter_t *)&m_lowpass2Filter[2], m_dps[2]);
            } else {
                // using simple averaging for downsampling
                m_sampleSum[0] += m_dps[0];
                m_sampleSum[1] += m_dps[1];
                m_sampleSum[2] += m_dps[2];
                m_sampleCount++;
            }

            for (int axis = 0; axis < 3; axis++) {

                // downsample the individual gyro samples
                float dps_filtered = 0;
                if (m_downsampleFilterEnabled) {
                    // using gyro lowpass 2 filter for downsampling
                    dps_filtered = m_sampleSum[axis];
                } else {
                    // using simple average for downsampling
                    if (m_sampleCount) {
                        dps_filtered = m_sampleSum[axis] / m_sampleCount;
                    }
                    m_sampleSum[axis] = 0;
                }

                // apply static notch filters and software lowpass filters
                dps_filtered =
                    m_lowpassFilterApplyFn((filter_t *)&m_lowpassFilter[axis],
                            dps_filtered);

                m_dps_filtered[axis] = dps_filtered;
            }

            m_sampleCount = 0;

            // Used for fusion with accelerometer
            accumulate();

            vstate->dphi   = m_dps_filtered[0];
            vstate->dtheta = m_dps_filtered[1];
            vstate->dpsi   = m_dps_filtered[2];

            m_isCalibrating = !calibrationComplete;
        }

        static int32_t getSkew(
                uint32_t nextTargetCycles,
                int32_t desiredPeriodCycles)
        {
            int32_t skew = cmpTimeCycles(nextTargetCycles, gyroSyncTime()) %
                desiredPeriodCycles;

            if (skew > (desiredPeriodCycles / 2)) {
                skew -= desiredPeriodCycles;
            }

            return skew;
        }

}; // class Gyro

