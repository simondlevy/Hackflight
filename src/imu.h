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
#include "core/axes.h"
#include "core/clock.h"
#include "core/constrain.h"
#include "core/filters/pt1.h"
#include "core/utils.h"
#include "core/vstate.h"

class Imu {

    friend class Board;

    private:

        static float rad2deg(float rad)
        {
            return 180 * rad / M_PI;
        }

        static int16_t rad2degi(float rad)
        {
            return (int16_t)rad2deg(rad);
        }

    protected:

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

    public:

        typedef Axes (*rotateFun_t)(Axes & axes);

        static float deg2rad(float deg)
        {
            return deg * M_PI / 180;
        }

        typedef void (*align_fun)(Axes * axes);

        virtual void begin(uint32_t clockSpeed) = 0;

        virtual auto getEulerAngles(const uint32_t time) -> Axes = 0;

        virtual uint32_t getGyroInterruptCount(void) = 0;

        virtual int32_t getGyroSkew(
                const uint32_t nextTargetCycles, const int32_t desiredPeriodCycles) = 0;

        virtual bool gyroIsCalibrating(void) = 0;

        virtual bool gyroIsReady(void) = 0;

        virtual auto readGyroDps(void) -> Axes = 0;

        virtual void updateAccelerometer(void)
        {
        }

        static void getEulerAngles(const VehicleState * vstate, int16_t angles[3])
        {
            angles[0] = (int16_t)(10 * rad2degi(vstate->phi));
            angles[1] = (int16_t)(10 * rad2degi(vstate->theta));
            angles[2] = (int16_t)rad2degi(vstate->psi);
        }

}; // class Imu
