/*
   Hackflight "Lite": receiver, sensors, PID controllers, receiver proxy

   Copyright (c) 2020 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "hackflight.hpp"
#include "rxproxy.hpp"

namespace hf {

    class HackflightLite : public Hackflight {

        private:

            // Out output to the main flight controller
            RXProxy * _proxy;

        public:

            void init(Board * board, Receiver * receiver, RXProxy * proxy) 
            {
                // Do general initialization
                Hackflight::init(board, receiver, proxy);

                _proxy = proxy;
            }

            void update(void)
            {
                // Run common update functions
                Hackflight::update();

                runPidControllers();

                Serial.println(_demands.throttle);

                /*
                // Check change in armed state
                _proxy->setArmedStatus(_state.armed);
                _proxy->run(_demands);

                // Check optional sensors
                checkOptionalSensors();
                */
            }

    }; // class HackflightLite

} // namespace
