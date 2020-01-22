/*
   Mock motor for testing

   Copyright (c) 2020 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "motor.hpp"

namespace hf {

    class MockMotor : public Motor {

        public:

            MockMotor(void) 
                : Motor(0)
            {
            }

            virtual void init(void) override
            {
            }

            virtual void write(float value) override
            {
                (void)value;
            }

    }; // class MockMotor

} // namespace hf
