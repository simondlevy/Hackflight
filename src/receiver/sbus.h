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

#include "receiver.h"
#include "utils.h"


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
            struct sbusFrame_s sbframe;
        } frame_t;

        frame_t  m_frame;
        uint32_t m_startAtUs;
        uint8_t  m_position;
        bool     m_done;

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

            if (m_done) {

                m_done = false;

                result = true;

                auto channels = &m_frame.sbframe.channels;

                // Update frame time only if there are no channel errors (timeout)
                frameTimeUs = channels->flags ? frameTimeUs : m_startAtUs;
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

        void handleSerialEvent(HardwareSerial & serial) {

            while (serial.available()) {
                parse(serial.read());
            }
        }

        void parse(const uint8_t c)
        {
            const uint32_t usec = micros();
            const int32_t timeInterval = intcmp(usec, m_startAtUs);

            if (timeInterval > 3500) {
                m_position = 0;
            }

            if (m_position == 0) {

                if (c != 0x0F) {
                    return;
                }
                m_startAtUs = usec;
            }

            if (m_position < FRAME_SIZE) {
                m_frame.bytes[m_position++] = c;
                if (m_position < FRAME_SIZE) {
                    m_done = false;
                } else {
                    m_done = true;
                }
            }
        }

        bool ready(void)
        {
            return m_done;
        }

        uint16_t readChannel1(void)
        {
            return m_frame.sbframe.channels.chan0;
        }

        uint16_t readChannel2(void)
        {
            return m_frame.sbframe.channels.chan1;
        }

        uint16_t readChannel3(void)
        {
            return m_frame.sbframe.channels.chan2;
        }

        uint16_t readChannel4(void)
        {
            return m_frame.sbframe.channels.chan3;
        }

        uint16_t readChannel5(void)
        {
            return m_frame.sbframe.channels.chan4;
        }

        uint16_t readChannel6(void)
        {
            return m_frame.sbframe.channels.chan5;
        }

        static float convert(
                const uint16_t value,
                const uint16_t dstmin=1000,
                const uint16_t dstmax=2000)
        {
            return Receiver::convert(value, 172, 1811, dstmin, dstmax);
        }

}; // class SbusReceiver

