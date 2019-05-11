/*
   magnetometer.hpp : Support for magnetometer (compass)

   Hackflight requires your Board implementation to provide the
   quaternion directly, but access to magnetometer could be useful
   for other kinds of sensor fusion. 

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

#include "sensor.hpp"
#include "surfacemount.hpp"
#include "board.hpp"

namespace hf {

    class Magnetometer : public SurfaceMountSensor {

        friend class Hackflight;

        public:

            Magnetometer(void)
            {
                _mx = 0;
                _my = 0;
                _mz = 0;
            }

        protected:

            virtual void modifyState(state_t & state, float time) override
            {
                // Here is where you'd do sensor fusion
                (void)state;
                (void)time;
            }

            virtual bool ready(float time) override
            {
                (void)time;

                return board->getMagnetometer(_uTs);
            }

        private:

            float _mx;
            float _my;
            float _mz;

    };  // class Magnetometer

} // namespace
