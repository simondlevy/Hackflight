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

#include "tasks/receiver.h"

class MockReceiver : public Receiver {

    protected:

        virtual void devStart(void) override
        {
        }

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

            _roll += .005 * dir;

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

}; // class MockReceiver


