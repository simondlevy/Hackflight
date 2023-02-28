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

#include "board.h"
#include "logic/core/axes.h"
#include "logic/core/constrain.h"
#include "logic/core/filters/pt1.h"
#include "logic/core/pid.h"
#include "logic/core/utils.h"
#include "logic/core/vstate.h"

class Imu {

    private:

        static const uint32_t GYRO_CALIBRATION_DURATION      = 1250000;
        static const uint16_t GYRO_LPF1_DYN_MIN_HZ           = 250;
        static const uint16_t GYRO_LPF2_STATIC_HZ            = 500;
        static const uint8_t  MOVEMENT_CALIBRATION_THRESHOLD = 48;

        static float rad2deg(float rad)
        {
            return 180 * rad / M_PI;
        }

        static int16_t rad2degi(float rad)
        {
            return (int16_t)rad2deg(rad);
        }

        class Stats {

            private:

                float  m_oldM;
                float  m_newM;
                float  m_oldS;
                float  m_newS;
                int32_t m_n;

                float variance(void)
                {
                    return ((m_n > 1) ? m_newS / (m_n - 1) : 0.0f);
                }

            public:

                void stdevClear(void)
                {
                    m_n = 0;
                }

                void stdevPush(float x)
                {
                    m_n++;

                    if (m_n == 1) {
                        m_oldM = m_newM = x;
                        m_oldS = 0.0f;
                    } else {
                        m_newM = m_oldM + (x - m_oldM) / m_n;
                        m_newS = m_oldS + (x - m_oldM) * (x - m_newM);
                        m_oldM = m_newM;
                        m_oldS = m_newS;
                    }
                }

                float stdevCompute(void)
                {
                    return sqrtf(variance());
                }
        }; 

        typedef struct {
            float sum[3];
            Stats stats[3];
        } calibration_t;

        typedef struct {

            float dps;           // aligned, calibrated, scaled, unfiltered
            float dpsFiltered;   // filtered 
            float sampleSum;     // summed samples used for downsampling
            float zero;

            Pt1Filter lowpassFilter1 = Pt1Filter(GYRO_LPF1_DYN_MIN_HZ);
            Pt1Filter lowpassFilter2 = Pt1Filter(GYRO_LPF2_STATIC_HZ);

        } gyroAxis_t;

        calibration_t m_gyroCalibration;

        int32_t  m_gyroCalibrationCyclesRemaining;
        float    m_gyroScale;
        bool     m_gyroIsCalibrating;

        gyroAxis_t m_gyroX;
        gyroAxis_t m_gyroY;
        gyroAxis_t m_gyroZ;

        static uint32_t calculateGyroCalibratingCycles(void)
        {
            return GYRO_CALIBRATION_DURATION / PidController::PERIOD;
        }

        void calibrateGyroAxis(int16_t rawGyro[3], gyroAxis_t & axis, const uint8_t index)
        {
            // Reset at start of calibration
            if (m_gyroCalibrationCyclesRemaining ==
                    (int32_t)calculateGyroCalibratingCycles()) {
                m_gyroCalibration.sum[index] = 0.0f;
                m_gyroCalibration.stats[index].stdevClear();
                // zero is set to zero until calibration complete
                axis.zero = 0.0f;
            }

            // Sum up CALIBRATING_GYRO_TIME_US readings
            m_gyroCalibration.sum[index] += rawGyro[index];
            m_gyroCalibration.stats[index].stdevPush(rawGyro[index]);

            if (m_gyroCalibrationCyclesRemaining == 1) {
                const float stddev = m_gyroCalibration.stats[index].stdevCompute();

                // check deviation and startover in case the model was moved
                if (MOVEMENT_CALIBRATION_THRESHOLD && stddev >
                        MOVEMENT_CALIBRATION_THRESHOLD) {
                    setGyroCalibrationCycles();
                    return;
                }

                axis.zero = m_gyroCalibration.sum[index] / calculateGyroCalibratingCycles();
            }
        }

        void calibrateGyro(int16_t rawGyro[3])
        {
            calibrateGyroAxis(rawGyro, m_gyroX, 0);
            calibrateGyroAxis(rawGyro, m_gyroY, 1);
            calibrateGyroAxis(rawGyro, m_gyroZ, 2);

            --m_gyroCalibrationCyclesRemaining;
        }

        void applyGyroLpf1(gyroAxis_t & axis)
        {
            axis.dpsFiltered = axis.lowpassFilter1.apply(axis.sampleSum);
        }

        void applyGyroLpf2(gyroAxis_t & axis)
        {
            axis.sampleSum = axis.lowpassFilter2.apply(axis.dps);
        }

        void scaleGyro(gyroAxis_t & axis, const float adc)
        {
            axis.dps = adc * m_gyroScale; 
        }

        float readCalibratedGyro(int16_t rawGyro[3], gyroAxis_t & axis, uint8_t index)
        {
            return rawGyro[index] - axis.zero;
        }

    protected:

        typedef Axes (*rotateFun_t)(Axes & axes);

        uint32_t m_gyroSyncTime;

        rotateFun_t m_rotateFun;

        void setGyroCalibrationCycles(void)
        {
            m_gyroCalibrationCyclesRemaining = (int32_t)calculateGyroCalibratingCycles();
        }

        static auto quat2euler(
                const float qw, const float qx, const float qy, const float qz) -> Axes 
        {
            const auto phi = atan2(2.0f*(qw*qx+qy*qz), qw*qw-qx*qx-qy*qy+qz*qz);
            const auto theta = asin(2.0f*(qx*qz-qw*qy));
            const auto psi = atan2(2.0f*(qx*qy+qw*qz), qw*qw+qx*qx-qy*qy-qz*qz);

            // Convert heading from [-pi,+pi] to [0,2*pi]
            return Axes(phi, theta, psi + (psi < 0 ? 2*M_PI : 0)); 
        }

        Imu(const rotateFun_t rotateFun, const uint16_t gyroScale)
        {
            m_rotateFun = rotateFun;
            m_gyroScale = gyroScale / 32768.;
        }

        // For software quaternion
        virtual void accumulateGyro(float x, float y, float z)
        {
            (void)x;
            (void)y;
            (void)z;
        }

    public:

        virtual void handleInterrupt(const uint32_t cycleCounter) = 0;

        virtual auto gyroRawToFilteredDps(int16_t rawGyro[3]) -> Axes
        {
            accumulateGyro(m_gyroX.dpsFiltered, m_gyroY.dpsFiltered, m_gyroZ.dpsFiltered);

            const auto calibrationComplete = m_gyroCalibrationCyclesRemaining <= 0;

            static Axes _adc;

            if (calibrationComplete) {

                // move 16-bit gyro data into floats to avoid overflows in
                // calculations

                _adc.x = readCalibratedGyro(rawGyro, m_gyroX, 0);
                _adc.y = readCalibratedGyro(rawGyro, m_gyroY, 1);
                _adc.z = readCalibratedGyro(rawGyro, m_gyroZ, 2);

                _adc = m_rotateFun(_adc);

                scaleGyro(m_gyroX, _adc.x);
                scaleGyro(m_gyroY, _adc.y);
                scaleGyro(m_gyroZ, _adc.z);
            } 
            
            else {
                calibrateGyro(rawGyro);
            }

            // Use gyro lowpass 2 filter for downsampling
            applyGyroLpf2(m_gyroX);
            applyGyroLpf2(m_gyroY);
            applyGyroLpf2(m_gyroZ);

            // Then apply lowpass 1
            applyGyroLpf1(m_gyroX);
            applyGyroLpf1(m_gyroY);
            applyGyroLpf1(m_gyroZ);

            m_gyroIsCalibrating = !calibrationComplete;

            return Axes(m_gyroX.dpsFiltered, m_gyroY.dpsFiltered, m_gyroZ.dpsFiltered);
        }

        virtual bool gyroIsCalibrating(void)
        {
            return m_gyroIsCalibrating;
        }

        static float deg2rad(float deg)
        {
            return deg * M_PI / 180;
        }

        virtual void begin(const uint32_t clockSpeed) = 0;

        virtual auto getEulerAngles(const uint32_t time) -> Axes = 0;

        virtual void updateAccelerometer(const int16_t rawAccel[3])
        {
            (void)rawAccel;
        }

        virtual int32_t getGyroSkew(
                const uint32_t nextTargetCycles,
                const int32_t desiredPeriodCycles)
        {
            const auto skew =
                intcmp(nextTargetCycles, m_gyroSyncTime) % desiredPeriodCycles;

            return skew > (desiredPeriodCycles / 2) ? skew - desiredPeriodCycles : skew;
        }

        static void getEulerAngles(const VehicleState * vstate, int16_t angles[3])
        {
            angles[0] = (int16_t)(10 * rad2degi(vstate->phi));
            angles[1] = (int16_t)(10 * rad2degi(vstate->theta));
            angles[2] = (int16_t)rad2degi(vstate->psi);
        }

        static auto rotate0(Axes & axes) -> Axes
        {
            return Axes(axes.x, axes.y, axes.z);
        }

        static auto rotate90(Axes & axes) -> Axes
        {
            return Axes(axes.y, -axes.x, axes.z);
        }

        static auto rotate180(Axes & axes) -> Axes
        {
            return Axes(-axes.x, -axes.y, axes.z);
        }

        static auto rotate270(Axes & axes) -> Axes
        {
            return Axes(-axes.y, axes.x, axes.z);
        }

        static auto rotate0Flip(Axes & axes) -> Axes
        {
            return Axes(-axes.x, axes.y, -axes.z);
        }

        static auto rotate90Flip(Axes & axes) -> Axes
        {
            return Axes(axes.y, axes.x, -axes.z);
        }

        static auto rotate180Flip(Axes & axes) -> Axes
        {
            return Axes(axes.x, -axes.y, -axes.z);
        }

        static auto rotate270Flip(Axes & axes) -> Axes
        {
            return Axes(-axes.y, -axes.x, -axes.z);
        }

}; // class Imu
