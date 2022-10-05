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

#include "serial.h"
#include "tasks/receiver.h"
#include "time.h"

class DsmxReceiver : public Receiver {

    private:

        static const uint8_t FRAME_SIZE = 16;

        // Support DSMX2048 only
        static const uint8_t  CHAN_SHIFT = 3;
        static const uint8_t  CHAN_MASK  = 0x07;
        static const uint8_t  MAX_CHANNELS = 8;

        static const uint16_t FRAME_INTERVAL = 5000;

        typedef struct {
            uint8_t  bytes[FRAME_SIZE];
            uint32_t lastTimeUs;
            uint8_t  position;
            bool     done;
        } frameData_t;

        // Receive ISR callback
        static void dataReceive(uint8_t c, void *data, uint32_t usec)
        {
            frameData_t * frameData = (frameData_t *)data;

            const uint32_t timeInterval =
                cmpTimeUs(usec, frameData->lastTimeUs);

            frameData->lastTimeUs = usec;

            if (timeInterval > FRAME_INTERVAL) {
                frameData->position = 0;
            }

            if (frameData->position < FRAME_SIZE) {
                frameData->bytes[frameData->position++] = (uint8_t)c;
                if (frameData->position < FRAME_SIZE) {
                    frameData->done = false;
                } else {
                    frameData->lastTimeUs = usec;
                    frameData->done = true;
                }
            }
        }

        serialPortIdentifier_e m_port;

        frameData_t m_frameData;

        static float convert(
                const uint16_t value, const uint16_t dstmin=1000, const uint16_t dstmax=2000)
        {
            return Receiver::convert(value, 0, 2048, dstmin, dstmax);
        }

     protected:

        virtual void devStart(void) override
        {
            serialOpenPortDsmx(
                    m_port,
                    dataReceive,
                    &m_frameData);
        }

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

    public:

        DsmxReceiver(serialPortIdentifier_e port=SERIAL_PORT_NONE) 
            : Receiver()
        {
            m_port = port;
        }

}; // class DsmxReceiver
