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

#include "hackflightbase.hpp"
#include "rxproxy.hpp"

namespace hf {

    class HackflightLite : public HackflightBase {

        private:

            // Out output to the main flight controller
            RXProxy * _proxy;

            // Helps us detect change in armed status
            bool _wasArmed = false;

            void checkArmDisarm(void)
            {
                if (_state.armed) {
                    if (!_wasArmed) {
                        _proxy->setArmedStatus(true);
                    }
                    _wasArmed = true;
                }
                else {
                    if (_wasArmed) {
                        _proxy->setArmedStatus(false);
                    }
                    _wasArmed = false;
                }
            }

        public:

            void init(Board * board, Receiver * receiver, RXProxy * proxy) 
            {
                // Do general initialization
                HackflightBase::init(board, receiver, proxy);

                _proxy = proxy;

                _wasArmed = false;
            }

            void update(void)
            {
                // Grab control signal if available
                checkReceiver();

                // Check change in armed state
                checkArmDisarm();

                // Check optional sensors
                checkOptionalSensors();
            }

    }; // class HackflightLite

} // namespace
