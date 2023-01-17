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

#include <stdint.h>
#include <math.h>

class RealImu : public Imu {

    private:

    protected:

        RealImu(const rotateFun_t rotateFun, const uint16_t gyroScale)
        {
            m_rotateFun = rotateFun;
            m_gyroScale = gyroScale / 32768.;

        }

        void begin(uint32_t clockSpeed) 
        {
            (void)clockSpeed;

            // Start calibrating gyro
            setGyroCalibrationCycles(); 
        }


        virtual auto readGyroDps(void) -> Axes
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

    public:

}; // class RealImu
