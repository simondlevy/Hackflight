/*
   gyrometer.hpp : Support for gyrometer (a.k.a. gyroscoe) 

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

#include <cmath>
#include <math.h>

#include "debug.hpp"
#include "sensor.hpp"
#include "board.hpp"

namespace hf {

    class Gyrometer : public Sensor {

        public:

            Gyrometer(Board * board) 
            {
                _board = board;
            }

        protected:

            virtual void modifyState(State & state, float time) override
            {
                (void)state;
                (void)time;
            }

            virtual bool ready(float time) override
            {
                (void)time;

                return false;
            }

        private:

            Board * _board;

    };  // class Gyrometer

} // namespace
