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

#include "datatypes.h"

#if defined(__cplusplus)
extern "C" {
#endif

    void     gyroDevInit(void);
    int32_t  gyroGetSkew(uint32_t nextTargetCycles, int32_t desiredPeriodCycles);
    void     gyroInit(gyro_t * gyro);
    uint32_t gyroInterruptCount(void);
    bool     gyroIsReady(void);
    int16_t  gyroReadRaw(uint8_t k);
    void gyroReadScaled(gyro_t *gyro, imu_align_fun align, vehicle_state_t * vstate);
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
        float m_dps[3];          // aligned, calibrated, scaled, unfiltered
        float  m_dps_filtered[3]; // filtered 
        uint8_t m_sampleCount;     // sample counter
        float m_sampleSum[3];    // summed samples used for downsampling
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

        float zero[3];

}; // class Gyro

