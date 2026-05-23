/*
   Hackflight core firmware for quadcopter

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
#include <firmware/core.hpp>
#include <mixers/bfquadx.hpp>

namespace hf {

    class QuadCore {

        public:

            Core _core;

            void begin()
            {
                _core.begin();

                _motors.begin(); 
            }

            void update(const uint32_t rxMsecPrev, const bool rxRequestedArming)
            {
                _core.update(rxMsecPrev, rxRequestedArming,
                        _mixer.motorvals, 4);
            } 

            void runMotors(const Setpoint & pidSetpoint)
            {
                _mixer = Mixer::run(_mixer, pidSetpoint);

                if (_core.isSafeToFly()) {
                    _motors.run(_core.isArmed(), _mixer.motorvals);
                }
             }

        private:

            // Computation
            Mixer _mixer;

            // Devices
            DshotTeensy4 _motors = DshotTeensy4({2, 3, 4, 5});

    }; // class NewQuadCore

} // namespace hf
