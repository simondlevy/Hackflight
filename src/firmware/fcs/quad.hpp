/*
   Hackflight core flight-control for quadcopter

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

// Third-party libraries
#include <dshot-teensy4.hpp>  

// Hackflight library
#include <hackflight.h>
#include <firmware/fc.hpp>
#include <mixers/bfquadx.hpp>

namespace hf {

    class QuadFC {

        public:

            FC _fc;

            void begin()
            {
                _fc.begin();

                _motors.begin(); 
            }

            auto update(const msp_message_t & message) -> Setpoint
            {
                return _fc.update(message, _mixer.motorvals, 4);
            } 

            auto update(const ReceiverData & rxdata) -> Setpoint
            {
                return _fc.update(rxdata, _mixer.motorvals, 4);
            } 

            void runMotors(const Setpoint & pidSetpoint)
            {
                _mixer = Mixer::run(_mixer, pidSetpoint);

                if (_fc.isSafeToFly()) {
                    _motors.run(_fc.isArmed(), _mixer.motorvals);
                }
             }

            auto getState() -> VehicleState
            {
                return _fc._state;
            }

        private:

            // Computation
            Mixer _mixer;

            // Devices
            DshotTeensy4 _motors = DshotTeensy4({2, 3, 4, 5});

    };
}
