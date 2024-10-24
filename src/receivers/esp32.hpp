/*

   ESP32 implementation of Receiver interface

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

#include <rx.hpp>
#include <msp.hpp>

namespace hf {

    class Esp32Receiver : public Receiver {

        public:

            void begin()
            {
                Serial2.begin(115200);
            }

            void read(uint16_t channels[6], bool & gotFailsafe) 
            {
                static Msp _msp;

                static uint32_t _last_received_msec;

                while (Serial2.available() > 0) {

                    const auto msgtype = _msp.parse(Serial2.read());

                    if (msgtype == 200) { // SET_RC message

                        _last_received_msec = millis();

                        channels[0] = _msp.parseShort(0);
                        channels[1] = _msp.parseShort(1);
                        channels[2] = _msp.parseShort(2);
                        channels[3] = _msp.parseShort(3);
                        channels[4] = _msp.parseShort(4);
                        channels[5] = _msp.parseShort(5);
                    }
                }

                if ((millis() - _last_received_msec) > RX_TIMEOUT_MSEC) {
                    gotFailsafe = true;
                }


                if (gotFailsafe) {
                    printf("FAILSAFE!!!\n");
                }
                else {
                    printf("c1=%04d c2=%04d c3=%04d c=%04d c5=%04d c6=%04d\n", 
                            channels[0], channels[1], channels[2],
                            channels[3], channels[4], channels[5]);
                }
            }

            float map(const uint16_t val, const float min, const float max)
            {
                return 0; // XXX
            }

        private:

            static const uint32_t RX_TIMEOUT_MSEC = 100;

    };
}
