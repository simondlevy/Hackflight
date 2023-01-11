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

#include "core/axes.h"
#include "core/clock.h"
#include "core/constrain.h"
#include "core/filters/pt1.h"
#include "core/utils.h"
#include "core/vstate.h"
#include "imu.h"
#include "stats.h"

class RealImu : public Imu {

    public:

        typedef Axes (*rotateFun_t)(Axes & axes);

    private:

        typedef struct {
            float sum[3];
            Stats stats[3];
        } calibration_t;

        rotateFun_t m_rotateFun;

        uint8_t  m_interruptPin;

        static const uint32_t GYRO_CALIBRATION_DURATION      = 1250000;
        static const uint16_t GYRO_LPF1_DYN_MIN_HZ           = 250;
        static const uint16_t GYRO_LPF2_STATIC_HZ            = 500;
        static const uint8_t  MOVEMENT_CALIBRATION_THRESHOLD = 48;

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
        uint32_t m_gyroInterruptCount;
        float    m_gyroScale;
        bool     m_gyroIsCalibrating;

        static uint32_t calculateGyroCalibratingCycles(void)
        {
            return GYRO_CALIBRATION_DURATION / Clock::PERIOD();
        }

        void setGyroCalibrationCycles(void)
        {
            m_gyroCalibrationCyclesRemaining = (int32_t)calculateGyroCalibratingCycles();
        }

        void calibrateGyroAxis(gyroAxis_t & axis, const uint8_t index)
        {
            // Reset at start of calibration
            if (m_gyroCalibrationCyclesRemaining == (int32_t)calculateGyroCalibratingCycles()) {
                m_gyroCalibration.sum[index] = 0.0f;
                m_gyroCalibration.stats[index].stdevClear();
                // zero is set to zero until calibration complete
                axis.zero = 0.0f;
            }

            // Sum up CALIBRATING_GYRO_TIME_US readings
            m_gyroCalibration.sum[index] += readRawGyro(index);
            m_gyroCalibration.stats[index].stdevPush(readRawGyro(index));

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

        void calibrateGyro(void)
        {
            calibrateGyroAxis(m_gyroX, 0);
            calibrateGyroAxis(m_gyroY, 1);
            calibrateGyroAxis(m_gyroZ, 2);

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

        float readCalibratedGyro(gyroAxis_t & axis, uint8_t index)
        {
            return readRawGyro(index) - axis.zero;
        }

    protected:

        gyroAxis_t m_gyroX;
        gyroAxis_t m_gyroY;
        gyroAxis_t m_gyroZ;

        RealImu(const rotateFun_t rotateFun, const uint16_t gyroScale)
        {
            m_rotateFun = rotateFun;
            m_gyroScale = gyroScale / 32768.;

        }

        void begin(void)
        {
            // Start calibrating gyro
            setGyroCalibrationCycles(); 
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


        uint32_t m_gyroSyncTime;

        virtual int16_t readRawGyro(uint8_t k) = 0;

        auto readGyroDps(void) -> Axes
        {
            const auto calibrationComplete = m_gyroCalibrationCyclesRemaining <= 0;

            static Axes _adc;

            if (calibrationComplete) {

                // move 16-bit gyro data into floats to avoid overflows in
                // calculations

                _adc.x = readCalibratedGyro(m_gyroX, 0);
                _adc.y = readCalibratedGyro(m_gyroY, 1);
                _adc.z = readCalibratedGyro(m_gyroZ, 2);

                _adc = m_rotateFun(_adc);

            } else {
                calibrateGyro();
            }

            if (calibrationComplete) {
                scaleGyro(m_gyroX, _adc.x);
                scaleGyro(m_gyroY, _adc.y);
                scaleGyro(m_gyroZ, _adc.z);
            }

            // Use gyro lowpass 2 filter for downsampling
            applyGyroLpf2(m_gyroX);
            applyGyroLpf2(m_gyroY);
            applyGyroLpf2(m_gyroZ);

            // Then apply lowpass 1
            applyGyroLpf1(m_gyroX);
            applyGyroLpf1(m_gyroY);
            applyGyroLpf1(m_gyroZ);

            // Used for fusion with accelerometer
            //accumulateGyro(m_gyroX.dpsFiltered, m_gyroY.dpsFiltered, m_gyroZ.dpsFiltered);

            m_gyroIsCalibrating = !calibrationComplete;

            return Axes(m_gyroX.dpsFiltered, m_gyroY.dpsFiltered, m_gyroZ.dpsFiltered);
        }

        bool gyroIsCalibrating(void)
        {
            return m_gyroIsCalibrating;
        }

        virtual int32_t getGyroSkew(
                const uint32_t nextTargetCycles,
                const int32_t desiredPeriodCycles) override
        {
            const auto skew =
                intcmp(nextTargetCycles, m_gyroSyncTime) % desiredPeriodCycles;

            return skew > (desiredPeriodCycles / 2) ? skew - desiredPeriodCycles : skew;
        }

        virtual uint32_t getGyroInterruptCount(void) override
        {
            return m_gyroInterruptCount;
        }

        void handleInterrupt(void)
        {
            m_gyroInterruptCount++;
        }

    public:

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
