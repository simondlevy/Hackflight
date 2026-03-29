/*
   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <hackflight.h>
#include <firmware/timer.hpp>

namespace hf {

    class LED {

        private:

            static constexpr float HEARTBEAT_FREQ = 0.75;
            static constexpr float CALIBRATING_FREQ = 3;
            static constexpr uint32_t PULSE_DURATION_MSEC = 50;

        public:

            LED(const uint8_t pin) : _pin(pin) {}

            void begin()
            {
                pinMode(_pin, OUTPUT); 

                digitalWrite(_pin, HIGH);

                for (int j = 1; j<= 3; j++) {
                    digitalWrite(_pin, LOW);
                    delay(70);
                    digitalWrite(_pin, HIGH);
                    delay(360);
                }
            }

            void blink(const uint32_t msec_curr, const bool calibrated)
            {
                static bool _pulsing;
                static uint32_t _pulse_start;
                static Timer _timer;

                if (_timer.ready(calibrated ? HEARTBEAT_FREQ : CALIBRATING_FREQ)) {
                    digitalWrite(_pin, true);
                    _pulsing = true;
                    _pulse_start = msec_curr;
                }

                else if (_pulsing) {
                    if (millis() - _pulse_start > PULSE_DURATION_MSEC) {
                        digitalWrite(_pin, false);
                        _pulsing = false;
                    }
                }
            }

        private:

            uint8_t _pin;

     };
}
