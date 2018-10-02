/*
   sbus.hpp : Futaba SBUS receiver support for Arduino flight controllers

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "receiver.hpp"
#include <SBUSRX.h>

static SBUSRX rx;

// Support different UARTs

static HardwareSerial * _hardwareSerial;

void serialEvent1(void)
{
    rx.handleSerialEvent(micros());
}

void serialEvent2(void)
{
    rx.handleSerialEvent(micros());
}

void serialEvent3(void)
{
    rx.handleSerialEvent(micros());
}

uint8_t sbusSerialAvailable(void)
{
    return _hardwareSerial->available();
}

uint8_t sbusSerialRead(void)
{
    return _hardwareSerial->read();
}

namespace hf {

    class SBUS_Receiver : public Receiver {

        private:

            const uint16_t MAX_FAILSAFE = 10;

            // Support different Arduino hardware
            uint16_t _serialConfig;

            // These values must persist between calls to readRawvals()
            float    _channels[16];
            uint16_t _failsafeCount;

        protected:

            void begin(void)
            {
                _hardwareSerial->begin(100000, _serialConfig);
            }

            bool gotNewFrame(void)
            {
                if (rx.gotNewFrame()) {

                    uint8_t failsafe = 0;
                    uint16_t lostFrames = 0;

                    rx.getChannelValuesNormalized(_channels, &failsafe, &lostFrames);

                    // accumulate consecutive failsafe hits
                    if (failsafe) {
                        _failsafeCount++;
                    }
                    else { // reset count
                        _failsafeCount = 0;
                    }

                    return true;
                }

                return false;
            }

            void readRawvals(void)
            {
                memset(rawvals, 0, CHANNELS*sizeof(float));
                memcpy(rawvals, _channels, CHANNELS*sizeof(float));
            }

            bool lostSignal(void)
            {
                return _failsafeCount > MAX_FAILSAFE;
            }

        public:

            SBUS_Receiver(
                    const uint8_t channelMap[6], 
                    uint16_t serialConfig,
                    HardwareSerial * hardwareSerial=&Serial1) 
                :  Receiver(channelMap) 
            { 
                _serialConfig = serialConfig;
                _hardwareSerial = hardwareSerial;
                _failsafeCount = 0;
            }

    }; // class SBUS_Receiver

} // namespace
