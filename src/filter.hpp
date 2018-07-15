/* 
   filter.hpp: Static filtering methods

   Adapted from

    https://github.com/multiwii/baseflight/blob/master/src/imu.c

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

#include <cmath>

#include "debug.hpp"

namespace hf {

    class Filter {

        public:


            static float max(float a, float b)
            {
                return a > b ? a : b;
            }

            static float deadband(float value, float deadband)
            {
                if (fabs(value) < deadband) {
                    value = 0;
                } else if (value > 0) {
                    value -= deadband;
                } else if (value < 0) {
                    value += deadband;
                }
                return value;
            }

            static float complementary(float a, float b, float c)
            {
                return a * c + b * (1 - c);
            }

            static float constrainMinMax(float val, float min, float max)
            {
                return (val<min) ? min : ((val>max) ? max : val);
            }

            static float constrainAbs(float val, float max)
            {
                return constrainMinMax(val, -max, +max);
            }

    }; // class Filter

    class LowPassFilter {

        private:

            float _history[256];
            uint8_t _historySize;
            uint8_t _historyIdx;
            float _sum;

        public:

            LowPassFilter(uint16_t historySize)
            {
                _historySize = historySize;
            }

            void init(void)
            {
                for (uint8_t k=0; k<_historySize; ++k) {
                    _history[k] = 0;
                }
                _historyIdx = 0;
                _sum = 0;
            }

            float update(float value)
            {
                uint8_t indexplus1 = (_historyIdx + 1) % _historySize;
                _history[_historyIdx] = value;
                _sum += _history[_historyIdx];
                _sum -= _history[indexplus1];
                _historyIdx = indexplus1;
                return _sum / _historySize;
            }

    }; // class LowPassFilter

} // namespace hf
