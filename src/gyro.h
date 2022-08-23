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

#include "clock.h"
#include "datatypes.h"
#include "filters/pt1.h"
#include "gyro_device.h"
#include "imu.h"
#include "constrain.h"
#include "stats.h"
#include "system.h"
#include "time.h"

class Gyro {

    private:

        static const uint32_t CALIBRATION_DURATION           = 1250000;
        static const uint16_t LPF1_DYN_MIN_HZ                = 250;
        static const uint8_t  MOVEMENT_CALIBRATION_THRESHOLD = 48;
        static const uint16_t LPF2_STATIC_HZ                 = 500;

        typedef struct {
            float sum[3];
            Stats var[3];
            int32_t cyclesRemaining;
        } calibration_t;

        float m_dps[3];            // aligned, calibrated, scaled, unfiltered
        float  m_dps_filtered[3];  // filtered 
        uint8_t m_sampleCount;     // sample counter
        float m_sampleSum[3];      // summed samples used for downsampling
        bool  m_isCalibrating;

        calibration_t m_calibration;

        Pt1Filter m_lowpassFilter1[3] = {
            Pt1Filter(LPF1_DYN_MIN_HZ),
            Pt1Filter(LPF1_DYN_MIN_HZ),
            Pt1Filter(LPF1_DYN_MIN_HZ)
        };

        Pt1Filter m_lowpassFilter2[3] = {
            Pt1Filter(LPF2_STATIC_HZ),
            Pt1Filter(LPF2_STATIC_HZ),
            Pt1Filter(LPF2_STATIC_HZ)
        };

        float m_zero[3];

        static uint32_t calculateCalibratingCycles(void)
        {
            return CALIBRATION_DURATION / Clock::PERIOD();
        }

        static float nullFilterApply(filter_t *filter, float input)
        {
            (void)filter;
            return input;
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
                    m_calibration.var[axis].clear();
                    // zero is set to zero until calibration complete
                    m_zero[axis] = 0.0f;
                }

                // Sum up CALIBRATING_GYRO_TIME_US readings
                m_calibration.sum[axis] += gyroDevReadRaw(axis);
                m_calibration.var[axis].push(gyroDevReadRaw(axis));

                if (m_calibration.cyclesRemaining == 1) {
                    const float stddev = m_calibration.var[axis].stdev();

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

    public:

        Gyro(void)
        {
            setCalibrationCycles(); // start calibrating
        }

        void readScaled(Imu * imu, Imu::align_fun align, vehicle_state_t * vstate)
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

            // using gyro lowpass 2 filter for downsampling
            m_sampleSum[0] = m_lowpassFilter2[0].apply(m_dps[0]);
            m_sampleSum[1] = m_lowpassFilter2[1].apply(m_dps[1]);
            m_sampleSum[2] = m_lowpassFilter2[2].apply(m_dps[2]);

            for (int axis = 0; axis < 3; axis++) {

                // apply static notch filters and software lowpass filters
                m_dps_filtered[axis] =
                    m_lowpassFilter1[axis].apply(m_sampleSum[axis]);
            }

            m_sampleCount = 0;

            // Used for fusion with accelerometer
            imu->accumulateGyro(
                    m_dps_filtered[0], m_dps_filtered[1], m_dps_filtered[2]);

            vstate->dphi   = m_dps_filtered[0];
            vstate->dtheta = m_dps_filtered[1];
            vstate->dpsi   = m_dps_filtered[2];

            m_isCalibrating = !calibrationComplete;
        }

        bool isCalibrating(void)
        {
            return m_isCalibrating;
        }

        static int32_t getSkew(
                uint32_t nextTargetCycles,
                int32_t desiredPeriodCycles)
        {
            int32_t skew = cmpTimeCycles(nextTargetCycles, gyroDevSyncTime()) %
                desiredPeriodCycles;

            if (skew > (desiredPeriodCycles / 2)) {
                skew -= desiredPeriodCycles;
            }

            return skew;
        }

}; // class Gyro

