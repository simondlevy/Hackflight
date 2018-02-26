/* 
    barometer.hpp: Altitude estimation using barometer

    Adapted from

    https://github.com/multiwii/baseflight/blob/master/src/sensors.c

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
    along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "debug.hpp"
#include "filter.hpp"

namespace hf {

    class Barometer {

        private: // constants

            const float NOISE_LPF             = 0.5f;
            const float VELOCITY_BOUND        = 300.f;
            const float VELOCITY_DEADBAND     = 10.f;
            static const uint8_t HISTORY_SIZE = 48;

            float   alt;
            float   history[HISTORY_SIZE];
            uint8_t historyIdx;
            float   groundAltitude;
            float   previousAlt;
            float   pressureSum;

            // Pressure in millibars to altitude in centimeters
            float millibarsToCentimeters(float pa)
            {
                //return (1.0f - powf(pa / 1013.25f, 0.190295f)) * 44330.0f;
                return (1.0f - powf(pa / 1013.25f, 0.190295f)) * 4433000.0f;
            }

        public:

            void init(void)
            {
                pressureSum = 0;
                historyIdx = 0;
                groundAltitude = 0;
                alt = 0;
                previousAlt = 0;

                for (uint8_t k=0; k<HISTORY_SIZE; ++k) {
                    history[k] = 0;
                }
            }

            void calibrate(void)
            {
                static float   groundPressure;

                groundPressure -= groundPressure / 8;
                groundPressure += pressureSum / (HISTORY_SIZE - 1);
                groundAltitude = millibarsToCentimeters(groundPressure/8);

                Debug::printf("****** %+f", groundAltitude);
            }

            void update(float pressure)
            {
                uint8_t indexplus1 = (historyIdx + 1) % HISTORY_SIZE;
                history[historyIdx] = pressure;
                pressureSum += history[historyIdx];
                pressureSum -= history[indexplus1];
                historyIdx = indexplus1;
            }

            float getAltitude(void)
            {
                float alt_tmp = millibarsToCentimeters(pressureSum/(HISTORY_SIZE-1)) - groundAltitude;
                alt = Filter::complementary(alt, alt_tmp, NOISE_LPF);

                return alt;
            }

            float getVelocity(uint32_t currentTime)
            {
                static float previousAlt;
                static uint32_t previousTime;
                float vel = (alt - previousAlt) * 1000000.0f / (currentTime-previousTime);
                previousAlt = alt;
                previousTime = currentTime;
                vel = Filter::constrainAbs(vel, VELOCITY_BOUND); 
                return Filter::deadband(vel, VELOCITY_DEADBAND);
            }


    }; // class Barometer

} // namespace hf
