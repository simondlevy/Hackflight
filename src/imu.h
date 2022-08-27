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

#if defined(__cplusplus)
extern "C" {
#endif

    // Called externally
    uint32_t imuDevGyroInterruptCount(void);
    int16_t  imuDevReadRawGyro(uint8_t k);
    uint32_t imuDevGyroSyncTime(void);

#if defined(__cplusplus)
}
#endif

#if defined(__cplusplus)

#include "arming.h"
#include "clock.h"
#include "constrain.h"
#include "state.h"
#include "filters/pt1.h"
#include "imu.h"
#include "stdev.h"
#include "system.h"
#include "time.h"

class Imu {

    private:

        static const uint32_t GYRO_CALIBRATION_DURATION      = 1250000;
        static const uint16_t GYRO_GYRO_LPF1_DYN_MIN_HZ      = 250;
        static const uint16_t GYRO_LPF2_STATIC_HZ            = 500;
        static const uint8_t  MOVEMENT_CALIBRATION_THRESHOLD = 48;

        typedef struct {
            float sum[3];
            Stdev var[3];
            int32_t cyclesRemaining;
        } calibration_t;

        uint8_t m_interruptPin;
        float m_dps[3];            // aligned, calibrated, scaled, unfiltered
        float  m_dps_filtered[3];  // filtered 
        uint8_t m_sampleCount;     // sample counter
        float m_sampleSum[3];      // summed samples used for downsampling
        bool  m_isCalibrating;
        calibration_t m_calibration;
        float m_zero[3];
        
        uint16_t m_gyroScale;

        Pt1Filter m_lowpassFilter1[3] = {
            Pt1Filter(GYRO_GYRO_LPF1_DYN_MIN_HZ),
            Pt1Filter(GYRO_GYRO_LPF1_DYN_MIN_HZ),
            Pt1Filter(GYRO_GYRO_LPF1_DYN_MIN_HZ)
        };

        Pt1Filter m_lowpassFilter2[3] = {
            Pt1Filter(GYRO_LPF2_STATIC_HZ),
            Pt1Filter(GYRO_LPF2_STATIC_HZ),
            Pt1Filter(GYRO_LPF2_STATIC_HZ)
        };

        static uint32_t calculateCalibratingCycles(void)
        {
            return GYRO_CALIBRATION_DURATION / Clock::PERIOD();
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
                m_calibration.sum[axis] += imuDevReadRawGyro(axis);
                m_calibration.var[axis].push(imuDevReadRawGyro(axis));

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

    protected:

        Imu(uint8_t interruptPin, uint16_t gyroScale) 
        {
            m_interruptPin = interruptPin;

            m_gyroScale = gyroScale;

            setCalibrationCycles(); // start calibrating
        }

    public:

        typedef struct {
            float x;
            float y;
            float z;
        } axes_t;

        typedef struct {
            float w;
            float x;
            float y;
            float z;
        } quaternion_t;

        typedef struct {
            float r20;
            float r21;
            float r22;
        } rotation_t;

        typedef struct {
            uint32_t quietPeriodEnd;
            uint32_t resetTimeEnd;
            bool resetCompleted;
        } gyro_reset_t;

        typedef struct {
            uint32_t time;
            quaternion_t quat;
            rotation_t rot;
            gyro_reset_t gyroReset;
        } fusion_t;

        typedef void (*align_fun)(axes_t * axes);

        virtual void accumulateGyro(float gx, float gy, float gz)
        {
            (void)gx;
            (void)gy;
            (void)gz;
        }

        virtual void getEulerAngles(
                fusion_t * fusionPrev,
                Arming * arming,
                uint32_t time,
                State * vstate) = 0;

        void begin(void)
        {
            devInit(m_interruptPin);
        }

        void readScaledGyro(Imu * imu, Imu::align_fun align, State * vstate)
        {
            if (!devGyroIsReady()) return;

            bool calibrationComplete = m_calibration.cyclesRemaining <= 0;

            static axes_t _adc;

            if (calibrationComplete) {

                // move 16-bit gyro data into floats to avoid overflows in
                // calculations

                _adc.x = imuDevReadRawGyro(0) - m_zero[0];
                _adc.y = imuDevReadRawGyro(1) - m_zero[1];
                _adc.z = imuDevReadRawGyro(2) - m_zero[2];

                align(&_adc);

            } else {
                calibrate();
            }

            if (calibrationComplete) {
                m_dps[0] = _adc.x * (m_gyroScale / 32768.);
                m_dps[1] = _adc.y * (m_gyroScale / 32768.);
                m_dps[2] = _adc.z * (m_gyroScale / 32768.);
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

        bool gyroIsCalibrating(void)
        {
            return m_isCalibrating;
        }

        int32_t getGyroSkew(
                uint32_t nextTargetCycles,
                int32_t desiredPeriodCycles)
        {
            int32_t skew = cmpTimeCycles(nextTargetCycles, imuDevGyroSyncTime()) %
                desiredPeriodCycles;

            if (skew > (desiredPeriodCycles / 2)) {
                skew -= desiredPeriodCycles;
            }

            return skew;
        }

        virtual bool devGyroIsReady(void) = 0;
        virtual void devInit(uint8_t interruptPin) = 0;

        //virtual uint32_t devGyroInterruptCount(void) = 0;
        //virtual uint32_t devGyroSyncTime(void) = 0;
        //virtual int16_t  devReadRawGyro(uint8_t k) = 0;

}; // class Imu

#endif


