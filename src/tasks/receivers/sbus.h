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

#pragma once

#include <stdlib.h>

#include "tasks/receiver.h"
#include "time.h"


class SbusReceiver : public Receiver {

    private:

        static const uint32_t FLAG_SIGNAL_LOSS     = (1 << 2);
        static const uint32_t FLAG_FAILSAFE_ACTIVE = (1 << 3);
        static const uint32_t FLAG_CHANNEL_17      = (1 << 0);
        static const uint32_t FLAG_CHANNEL_18      = (1 << 1);

        static const uint16_t CHANNEL_MIN = 173;
        static const uint16_t CHANNEL_MAX = 1812;

        typedef struct sbusChannels_s {
            // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
            uint16_t chan0 : 11;
            uint16_t chan1 : 11;
            uint16_t chan2 : 11;
            uint16_t chan3 : 11;
            uint16_t chan4 : 11;
            uint16_t chan5 : 11;
            uint16_t chan6 : 11;
            uint16_t chan7 : 11;
            uint16_t chan8 : 11;
            uint16_t chan9 : 11;
            uint16_t chan10 : 11;
            uint16_t chan11 : 11;
            uint16_t chan12 : 11;
            uint16_t chan13 : 11;
            uint16_t chan14 : 11;
            uint16_t chan15 : 11;
            uint8_t flags;
        } __attribute__((__packed__)) sbusChannels_t;

        static const uint8_t FRAME_SIZE = sizeof(sbusChannels_t) + 2;

        struct sbusFrame_s {
            uint8_t syncByte;
            sbusChannels_t channels;
            /**
             * The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2
             * RX's the value indicates the telemetry byte that is sent after
             * every 4th sbus
             * frame.
             *
             * See
             * https://github.com/cleancleanissues/590#issuecomment-101027349
             * and
             * https://github.com/cleancleanissues/590#issuecomment-101706023
             */
            uint8_t endByte;
        } __attribute__ ((__packed__));

        typedef union {
            uint8_t bytes[FRAME_SIZE];
            struct sbusFrame_s frame;
        } frame_t;

        typedef struct {
            frame_t frame;
            uint32_t startAtUs;
            uint8_t position;
            bool done;
        } frameData_t;

        frameData_t m_frameData;

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

                auto channels = &m_frameData.frame.frame.channels;

                // Update frame time only if there are no channel errors (timeout)
                frameTimeUs = channels->flags ? frameTimeUs : m_frameData.startAtUs;
                throttle = convert(channels->chan0);

                roll  = convert(channels->chan1);
                pitch = convert(channels->chan2);
                yaw   = convert(channels->chan3);
                aux1  = convert(channels->chan4, 0, 1);
                aux2  = convert(channels->chan5, 0, 1);
            }

            return result;
        }

    public:

        virtual void parse(const uint8_t c) override
        {
            const uint32_t usec = micros();
            const int32_t timeInterval = cmpTimeUs(usec, m_frameData.startAtUs);

            if (timeInterval > 3500) {
                m_frameData.position = 0;
            }

            if (m_frameData.position == 0) {

                if (c != 0x0F) {
                    return;
                }
                m_frameData.startAtUs = usec;
            }

            if (m_frameData.position < FRAME_SIZE) {
                m_frameData.frame.bytes[m_frameData.position++] = c;
                if (m_frameData.position < FRAME_SIZE) {
                    m_frameData.done = false;
                } else {
                    m_frameData.done = true;
                }
            }
        }

        bool ready(void)
        {
            return m_frameData.done;
        }

        uint16_t readChannel0(void)
        {
            return m_frameData.frame.frame.channels.chan0;
        }

        static float convert(
                const uint16_t value,
                const uint16_t dstmin=1000,
                const uint16_t dstmax=2000)
        {
            return Receiver::convert(value, 172, 1811, dstmin, dstmax);
        }

}; // class SbusReceiver

