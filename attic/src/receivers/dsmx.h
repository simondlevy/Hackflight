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

#include "datatypes.h"
#include "receiver.h"
#include "serial.h"
#include "time.h"

class DsmxReceiver : public Receiver {

    private:

        static const uint8_t FRAME_SIZE = 16;

        // Support DSMX2048 only
        static const uint8_t  CHAN_SHIFT = 3;
        static const uint8_t  CHAN_MASK  = 0x07;
        static const uint16_t CHAN_RESOLUTION = 2048;
        static const uint8_t MAX_CHANNELS = 8;

        static const uint16_t FRAME_INTERVAL = 5000;

        typedef struct {
            uint8_t  bytes[FRAME_SIZE];
            bool     done;
            uint32_t lastTimeUs;
            uint8_t  position;
        } frameData_t;

        // Receive ISR callback
        static void dataReceive(uint8_t c, void *data, uint32_t currentTimeUs)
        {
            frameData_t * frameData = (frameData_t *)data;

            const uint32_t timeInterval =
                cmpTimeUs(currentTimeUs, frameData->lastTimeUs);

            frameData->lastTimeUs = currentTimeUs;

            if (timeInterval > FRAME_INTERVAL) {
                frameData->position = 0;
            }

            if (frameData->position < FRAME_SIZE) {
                frameData->bytes[frameData->position++] = (uint8_t)c;
                if (frameData->position < FRAME_SIZE) {
                    frameData->done = false;
                } else {
                    frameData->lastTimeUs = currentTimeUs;
                    frameData->done = true;
                }
            }
        }

        serialPortIdentifier_e m_port;

        frameData_t m_frameData;

     protected:

        virtual void begin(void) override
        {
            serialOpenPortDsmx(
                    m_port,
                    dataReceive,
                    &m_frameData);
        }

        virtual float convert(uint16_t * channelData, uint8_t chan) override
        {
            // Ignore channel 6 for now (problems with transmitter)
            uint16_t chanval = chan == 5 ? 1 : channelData[chan];

            return 1000 * (1 + (chanval - 1) / (float)(CHAN_RESOLUTION-1));
        }

        virtual uint8_t devCheck(uint16_t * channelData, uint32_t * frameTimeUs)
            override
        {
            uint8_t result = Receiver::FRAME_PENDING;

            if (m_frameData.done) {

                m_frameData.done = false;

                *frameTimeUs = m_frameData.lastTimeUs;

                uint8_t * bytes = m_frameData.bytes;

                for (int b=3; b<FRAME_SIZE; b+=2) {

                    const uint8_t channel = 0x0F & (bytes[b - 1] >> CHAN_SHIFT);

                    if (channel < MAX_CHANNELS) {

                        channelData[channel] = ((uint32_t)(bytes[b - 1] &
                                    CHAN_MASK) << 8) + bytes[b];
                    }
                }

                result = Receiver::FRAME_COMPLETE;
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
