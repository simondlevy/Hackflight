/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the
   Free Software Foundation, either version 3 of the License, or (at your
   option) any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along
   with Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdlib.h>

#include "core/utils.h"
#include "receiver.h"

class DsmxReceiver : public Receiver {

    private:

        // Support DSMX2048 only
        static const uint8_t  CHAN_SHIFT = 3;
        static const uint8_t  CHAN_MASK  = 0x07;
        static const uint8_t  MAX_CHANNELS = 8;

        static const uint8_t FRAME_SIZE = 16;

        typedef struct {
            uint8_t  bytes[FRAME_SIZE];
            uint32_t lastTimeUs;
            uint8_t  position;
            bool     done;
        } frameData_t;

        frameData_t m_frameData;

        static float convert(
                const uint16_t value,
                const uint16_t dstmin=1000,
                const uint16_t dstmax=2000)
        {
            return Receiver::convert(value, 0, 2048, dstmin, dstmax);
        }

    protected:

        virtual bool devRead(
                float & throttle,
                float & roll,
                float & pitch,
                float & yaw,
                float & aux1,
                float & aux2,
                uint32_t & frameTimeUs) override
        {
            auto result = false;

            if (m_frameData.done) {

                m_frameData.done = false;

                result = true;

                frameTimeUs = m_frameData.lastTimeUs;

                auto * bytes = m_frameData.bytes;

                uint16_t channelData[MAX_CHANNELS] = {};

                for (auto b=3; b<FRAME_SIZE; b+=2) {

                    const auto channel = 0x0F & (bytes[b - 1] >> CHAN_SHIFT);

                    if (channel < MAX_CHANNELS) {

                        channelData[channel] =
                            ((uint32_t)(bytes[b - 1] & CHAN_MASK) << 8) + bytes[b];
                    }
                }

                throttle = convert(channelData[0]);
                roll     = convert(channelData[1]);
                pitch    = convert(channelData[2]);
                yaw      = convert(channelData[3]);
                aux1     = convert(channelData[4], 0, 1);

                // Ignore channel 6 for now (problems with transmitter)
                aux2     = 0;

            }

            return result;
        }

        void parse(const uint8_t byte, const uint32_t usec) override
        {
            const uint32_t timeInterval = intcmp(usec, m_frameData.lastTimeUs);

            m_frameData.lastTimeUs = usec;

            if (timeInterval > 5000) {
                m_frameData.position = 0;
            }

            if (m_frameData.position < FRAME_SIZE) {
                m_frameData.bytes[m_frameData.position++] = byte;
                if (m_frameData.position < FRAME_SIZE) {
                    m_frameData.done = false;
                } else {
                    m_frameData.lastTimeUs = usec;
                    m_frameData.done = true;
                }
            }
        }

}; // class DsmxReceiver
