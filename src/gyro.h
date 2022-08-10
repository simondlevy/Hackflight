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
#include "pt1_filter.h"
#include "imu.h"
#include "maths.h"
#include "stats.h"
#include "system.h"
#include "time.h"

#if defined(__cplusplus)
extern "C" {
#endif

    void     gyroDevInit(void);
    int32_t  gyroGetSkew(uint32_t nextTargetCycles, int32_t desiredPeriodCycles);
    void     gyroInit(gyro_t * gyro);
    uint32_t gyroInterruptCount(void);
    bool     gyroIsReady(void);
    int16_t  gyroReadRaw(uint8_t k);
    void     gyroReadScaled(gyro_t *gyro, imu_align_fun align, vehicle_state_t * vstate);
    uint16_t gyroScaleDps(void);
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
                m_calibration.sum[axis] += gyroReadRaw(axis);
                devPush(&m_calibration.var[axis], gyroReadRaw(axis));

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

}; // class Gyro

