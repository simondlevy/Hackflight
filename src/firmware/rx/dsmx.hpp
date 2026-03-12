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

#include <dsmrx.hpp>  

static Dsm2048 _dsm2048;

void serialEvent1(void)
{
    while (Serial1.available()) {
        _dsm2048.parse(Serial1.read(), micros());
    }
}

namespace hf {

    class RX {

        private:

            static constexpr float ARMING_SWITCH_MIN = 0;

            static constexpr float THROTTLE_DOWN_MAX = -0.95;

        public:

            float chanvals[6];
            bool is_armed;
            bool is_throttle_down;

            void begin()
            {
                Serial1.begin(115000);
            }

            void read()
            {
                const auto usec_curr = micros();

                if (_dsm2048.timedOut(usec_curr)) {
                    return;
                }

                if (_dsm2048.gotNewFrame()) {
                    _dsm2048.getChannelValues(chanvals, 6);
                }

                is_throttle_down = chanvals[0] < THROTTLE_DOWN_MAX;

                is_armed = 
                    chanvals[4] < ARMING_SWITCH_MIN  ? false :
                    is_throttle_down ? true :
                    is_armed;
            }
    };
}
