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

#include "task/receiver/real.h"
#include "msp/parser.h"

class MspReceiver : public RealReceiver {

    private:

        MspParser _parser;

        static void dump(int16_t val)
        {
            Serial.print(val);
            Serial.print("  ");
        }

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
            // Simulates moving the roll stick

            static float _roll;
            static int8_t dir = +1;

            _roll += .00001 * dir;

            if (_roll >= 1.0) {
                dir = -1;
            }

            if (_roll <= -1.0) {
                dir = +1;
            }

            throttle = 1000;
            roll     = 1500 + _roll * 500;
            pitch    = 1500;
            yaw      = 1500;
            aux1 = 0;
            aux2 = 0;

            (void)frameTimeUs;

            return false;
        }

        virtual void parse(const uint8_t c) override
        {
            if (_parser.parse(c) == 200) {

                uint16_t c1 = _parser.parseShort(0);
                uint16_t c2 = _parser.parseShort(1);
                uint16_t c3 = _parser.parseShort(2);
                uint16_t c4 = _parser.parseShort(3);
                uint16_t c5 = _parser.parseShort(4);
                uint16_t c6 = _parser.parseShort(5);

                dump(c1);
                dump(c2);
                dump(c3);
                dump(c4);
                dump(c5);
                dump(c6);
                Serial.println();
            }
        }

}; // class MspReceiver


