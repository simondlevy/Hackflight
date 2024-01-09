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

#include "../task.hpp"
#include "datatypes.h"

class ReceiverTask : public LadybugTask {

    private:

       
        static const uint32_t TIMEOUT_USEC = 30000;

        static const uint8_t   THROTTLE_LOOKUP_TABLE_SIZE = 12;
        static constexpr float THROTTLE_EXPO8 = 0;
        static constexpr float THROTTLE_MID8  = 50;

        float    m_channels[6];
        bool     m_gotNewData;
        int16_t  m_lookupThrottleRc[THROTTLE_LOOKUP_TABLE_SIZE];
        bool     m_lostSignal;
        uint32_t m_previousFrameTimeUs;
        float    m_rawThrottle;
        float    m_rawRoll;
        float    m_rawPitch;
        float    m_rawYaw;

        static float convert(
                const uint16_t value,
                const uint16_t srcmin,
                const uint16_t srcmax,
                const float dstmin=1000,
                const float dstmax=2000)
        {
            return dstmin + (dstmax-dstmin) * ((float)value - srcmin) / (srcmax - srcmin);
        }

        // [1000,2000] => [-1,+1]
        static float rescaleCommand(const float raw, const float sgn)
        {
            const auto tmp = fminf(fabs(raw - 1500), 500);
            const auto cmd = tmp * sgn;
            const auto command = raw < 1500 ? -cmd : cmd;
            return command / 500;
        }

        float lookupThrottle(const int32_t tmp)
        {
            static bool _initializedThrottleTable;

            if (!_initializedThrottleTable) {
                for (auto i = 0; i < THROTTLE_LOOKUP_TABLE_SIZE; i++) {
                    const int16_t tmp2 = 10 * i - THROTTLE_MID8;
                    uint8_t y = tmp2 > 0 ?
                        100 - THROTTLE_MID8 :
                        tmp2 < 0 ?
                        THROTTLE_MID8 :
                        1;
                    m_lookupThrottleRc[i] =
                        10 * THROTTLE_MID8 + tmp2 * (100 - THROTTLE_EXPO8 + (int32_t)
                                THROTTLE_EXPO8 * (tmp2 * tmp2) / (y * y)) / 10;
                    m_lookupThrottleRc[i] = 1000 + m_lookupThrottleRc[i];
                }
            }

            _initializedThrottleTable = true;

            const auto tmp3 = tmp / 100;

            // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
            return (float)(m_lookupThrottleRc[tmp3] + (tmp - tmp3 * 100) *
                    (m_lookupThrottleRc[tmp3 + 1] - m_lookupThrottleRc[tmp3]) / 100);
        }

    public:

        ReceiverTask()
            : LadybugTask(RECEIVER, Clock::RATE_33_HZ)
        {
        }

        bool throttleIsDown(void)
        {
            return getRawThrottle() < 1050;
        }

        bool haveSignal(const uint32_t usec)
        {
            return !m_lostSignal && (usec - m_lastSignaledAtUs) < TIMEOUT_USEC;
        }

        float getRawThrottle(void)
        {
            return m_channels[0];
        }

        float getRawRoll(void)
        {
            return m_channels[1];
        }

        float getRawPitch(void)
        {
            return m_channels[2];
        }

        float getRawYaw(void)
        {
            return m_channels[3];
        }

        float getRawAux1(void)
        {
            return m_channels[4];
        }

        float getRawAux2(void)
        {
            return m_channels[5];
        }

        virtual void run(void)
        {
            m_rawThrottle = getRawThrottle();
            m_rawRoll = getRawRoll();
            m_rawPitch = getRawPitch();
            m_rawYaw = getRawYaw();
        }

        void getDemands(demands_t & demands)
        {
            m_previousFrameTimeUs = m_gotNewData ? 0 : m_previousFrameTimeUs;

            // Throttle [1000,2000] => [1000,2000]
            auto tmp = constrain_f_i32(m_rawThrottle, 1050, 2000);
            auto tmp2 = (uint32_t)(tmp - 1050) * 1000 / 950;
            auto commandThrottle = lookupThrottle(tmp2);

            Axis3f rawSetpoints = m_gotNewData ?

                Axis3f {
                        rescaleCommand(m_rawRoll, +1),
                        rescaleCommand(m_rawPitch, +1),
                        rescaleCommand(m_rawYaw, -1)
                    } :

                    Axis3f{0, 0, 0};

            static Axis3f _axes;

            if (m_gotNewData) {

                _axes.x = rawSetpoints.x;
                _axes.y = rawSetpoints.y;
                _axes.z = rawSetpoints.z;
            }

            m_gotNewData = false;

            demands.thrust = constrain_f((commandThrottle - 1000) / 1000, 0, 1);

            demands.roll = _axes.x;
            demands.pitch = _axes.y;
            demands.yaw = _axes.z;
        }

        void setValues(
                uint16_t channels[],
                const uint32_t usec,
                const bool lostSignal,
                const uint16_t srcMin,
                const uint16_t srcMax)
        {
            for (uint8_t k=0; k<6; ++k) {
                m_channels[k] = convert(channels[k], srcMin, srcMax);
            }

            m_lostSignal = lostSignal;
            m_lastSignaledAtUs = usec;
            m_previousFrameTimeUs = usec;
            m_gotNewData = true;
        }

}; // class ReceiverTask
