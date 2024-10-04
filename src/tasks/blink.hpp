/*
   Hackflight LED blinking task

   Copyright (C) 2024 Simon D. Levy

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

namespace hf {

    class BlinkTask {

        public:

            void run(
                    const uint8_t led_pin,
                    const uint32_t usec_curr,
                    const float freq_hz)
            {
                static uint32_t _usec_prev;

                static uint32_t _delay;

                if (usec_curr - _usec_prev > _delay) {

                    static bool _alternate;

                    _usec_prev = usec_curr;

                    digitalWrite(led_pin, _alternate);

                    if (_alternate) {
                        _alternate = false;
                        _delay = UPTIME_USEC;
                    }
                    else {
                        _alternate = true;
                        _delay = freq_hz * 1e6;
                    }
                }
            }

        private:

            static const uint32_t UPTIME_USEC = 100'000;
    };

}
