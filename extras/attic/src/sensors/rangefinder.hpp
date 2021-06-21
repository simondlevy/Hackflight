/*
   Support for rangefinder sensors (sonar, time-of-flight)

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
#include "filters.hpp"

namespace hf {

    class Rangefinder : public Sensor {

        private:

            static constexpr float UPDATE_HZ = 25; // XXX should be using interrupt!

            static constexpr float UPDATE_PERIOD = 1/UPDATE_HZ;

            float _distance = 0;

            LowPassFilter _lpf = LowPassFilter(20);

        protected:

            virtual void modifyState(state_t & state, float time) override
            {
                // Previous values to support first-differencing
                static float _time;
                static float _altitude;

                // Compensate for effect of pitch, roll on rangefinder reading
                state.location[2] =  _distance * cos(state.rotation[0]) * cos(state.rotation[1]);

                // Use first-differenced, low-pass-filtered altitude as variometer
                state.inertialVel[2] = _lpf.update((state.location[2]-_altitude) / (time-_time));

                // Update first-difference values
                _time = time;
                _altitude = state.location[2];
            }

            virtual bool ready(float time) override
            {
                float newDistance;

                if (distanceAvailable(newDistance)) {

                    static float _time;

                    if (time-_time > UPDATE_PERIOD) {

                        _distance = newDistance;

                        _time = time; 

                        return true;
                    }
                }

                return false; 
            }


            virtual bool distanceAvailable(float & distance) = 0;

        public:

            Rangefinder(void) 
            {
                _lpf.begin();
            }

    };  // class Rangefinder

} // namespace
