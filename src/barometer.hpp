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

#include "debug.hpp"

namespace hf {

    class Barometer {

        private: // constants

            const float noiseLpf              = 0.5f;
            const float velocityBound         = 300.0f;
            const float velocityDeadband      = 10.0f;
            static const uint8_t HISTORY_SIZE = 48;

        public:

            void  init(Board * _board);
            void  calibrate(void);
            void  update(void);
            float getAltitude(void);
            float getVelocity(uint32_t dTimeMicros);

        private:

            Board * board;

            float   alt;
            float   history[HISTORY_SIZE];
            uint8_t historyIdx;
            float   groundAltitude;
            float   lastAlt;
            float   pressureSum;

            static float millibarsToMeters(float pa);
    };

    /********************************************* CPP ********************************************************/

    void Barometer::init(Board * _board)
    {
        board = _board;

        pressureSum = 0;
        historyIdx = 0;
        groundAltitude = 0;
        alt = 0;
        lastAlt = 0;

        for (uint8_t k=0; k<HISTORY_SIZE; ++k) {
            history[k] = 0;
        }
    }

    void Barometer::calibrate(void)
    {
        static float   groundPressure;

        groundPressure -= groundPressure / 8;
        groundPressure += pressureSum / (HISTORY_SIZE - 1);
        groundAltitude = millibarsToMeters(groundPressure/8);
    }

    void Barometer::update()
    {
        float pressure = board->extrasGetBaroPressure();

        uint8_t indexplus1 = (historyIdx + 1) % HISTORY_SIZE;
        history[historyIdx] = pressure;
        pressureSum += history[historyIdx];
        pressureSum -= history[indexplus1];
        historyIdx = indexplus1;
    }

    float Barometer::getAltitude(void)
    {
        float alt_tmp = millibarsToMeters(pressureSum/(HISTORY_SIZE-1)); 
        alt_tmp -= groundAltitude;
        alt = alt * noiseLpf + alt_tmp * (1.0f - noiseLpf);

        return alt;
    }

    float Barometer::getVelocity(uint32_t dTimeMicros)
    {
        static float lastAlt;
        float vel = (alt - lastAlt) * 1000000.0f / dTimeMicros;
        lastAlt = alt;
        vel = Filter::constrainAbs(vel, velocityBound); 
        return Filter::deadband(vel, velocityDeadband);
    }


    // Pressure in millibars to altitude in meters
    float Barometer::millibarsToMeters(float pa)
    {
        return (1.0f - powf(pa / 1013.25f, 0.190295f)) * 44330.0f;
    }

} // namespace hf
