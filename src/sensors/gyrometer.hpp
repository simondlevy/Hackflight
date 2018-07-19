/*
   gyrometer.hpp : Support for gyrometer (a.k.a. gyroscope) 

   Copyright (c) 2018 Simon D. Levy

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

        friend class Hackflight;

        protected:

            void init(Board * board) 
            {
                _board = board;

                memset(_rates, 0, 3*sizeof(float));
            }

            virtual void modifyState(State & state, float time) override
            {
                (void)time;

                memcpy(&state.angularVelocities, _rates, 3*sizeof(float));
            }

            virtual bool ready(float time) override
            {
                (void)time;

                if (_board->getGyrometer(_rates)) {
                    return true;
                }

                return false;
            }

        private:

            // Gyro is a surface-mount sensor
            Board * _board;

            float _rates[3];

    };  // class Gyrometer

} // namespace
