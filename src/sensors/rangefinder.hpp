/*
   rangefinder.hpp : Support for rangefinder sensors (sonar, time-of-flight)

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
#include "filters.hpp"

namespace hf {

    class Rangefinder : public Sensor {

        public:

            Rangefinder(void) 
            {
                _lpf.init();
            }

        protected:

            virtual bool modifyState(State & state, float time) override
            {
                // Previous values to support first-differencing
                static float _time;
                static float _altitude;

                // Compensate for effect of pitch, roll on rangefinder reading
                state.altitude =  _distance * cos(state.eulerAngles[0]) * cos(state.eulerAngles[1]);

                // Use first-differenced, low-pass-filtered altitude as variometer
                state.variometer = _lpf.update((state.altitude-_altitude) / (time-_time));

                // Update first-difference values
                _time = time;
                _altitude = state.altitude;

				return true;
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

        private:

            static constexpr float UPDATE_HZ = 25; // XXX should be using interrupt!

            static constexpr float UPDATE_PERIOD = 1/UPDATE_HZ;

            float _distance;

            LowPassFilter _lpf = LowPassFilter(20);

    };  // class Rangefinder

} // namespace
