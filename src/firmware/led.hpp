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

#include <Arduino.h>

#include <hackflight.h>

namespace hf {

    class LED {

        private:

            static constexpr float HEARTBEAT_FREQ_HZ = 0.75;
            static constexpr float FASTBLINK_FREQ_HZ = 3;
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

            void blink(const bool is_imu_calibrated)
            {
                static bool _pulsing;
                static uint32_t _pulse_start;

                const auto ready = is_imu_calibrated ?
                    _heartbeatTimer.ready() : _fastblinkTimer.ready();
                
                if (ready) {
                    digitalWrite(_pin, true);
                    _pulsing = true;
                    _pulse_start = millis();
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

            Timer _heartbeatTimer = Timer(HEARTBEAT_FREQ_HZ);
            Timer _fastblinkTimer = Timer(FASTBLINK_FREQ_HZ);
    };
}
