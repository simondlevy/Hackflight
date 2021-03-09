/*
   Support for VL53L1X time-of-flight rangefinder

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

#include <VL53L1X.h>
#include "sensors/rangefinder.hpp"

namespace hf {

    class VL53L1X_Rangefinder : public Rangefinder {

        private:

            VL53L1X _distanceSensor;

        protected:

            virtual bool distanceAvailable(float & distance) override
            {
                if (_distanceSensor.newDataReady()) {
                    distance = _distanceSensor.getDistance() / 1000.f; // mm => m
                    return true;
                }
                return false;
            }

        public:

            void begin(void)
            {
                _distanceSensor.begin();
            }

    }; // class VL53L1X_Rangefinder 

} // namespace hf
