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

#include <math.h>

#include "sensors/surfacemount.hpp"
#include "board.hpp"

namespace hf {

    class Gyrometer : public SurfaceMountSensor {

        friend class Hackflight;

        private:

            float _x = 0;
            float _y = 0;
            float _z = 0;

        protected:

            virtual void modifyState(state_t & state, float time) override
            {
                (void)time;

                // NB: We negate gyro X, Y to simplify PID controller
                state.angularVel[0] =  _x;
                state.angularVel[1] = -_y;
                state.angularVel[2] = -_z;
            }

            virtual bool ready(float time) override
            {
                (void)time;

                bool result = board->getGyrometer(_x, _y, _z);

                return result;
            }

        public:

            Gyrometer(void)
            {
                _x = 0;
                _y = 0;
                _z = 0;
            }

    };  // class Gyrometer

} // namespace
