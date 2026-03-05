/**
 * Copyright (C) 2011-2012 Bitcraze AB, 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * filter.h - Filtering functions
 */

#pragma once

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

namespace hf {

    class LPF {

        private:

            float _a1;
            float _a2;
            float _b0;
            float _b1;
            float _b2;
            float _delay_element_1;
            float _delay_element_2;

            void setCutoffFreq(const float sample_freq, const float cutoff_freq)
            {
                float fr = sample_freq/cutoff_freq;
                float ohm = tanf(M_PI/fr);
                float c = 1.0f+2.0f*cosf(M_PI/4.0f)*ohm+ohm*ohm;
                _b0 = ohm*ohm/c;
                _b1 = 2.0f*_b0;
                _b2 = _b0;
                _a1 = 2.0f*(ohm*ohm-1.0f)/c;
                _a2 = (1.0f-2.0f*cosf(M_PI/4.0f)*ohm+ohm*ohm)/c;
                _delay_element_1 = 0.0f;
                _delay_element_2 = 0.0f;
            }

        public:

            /**
             * 2-Pole low pass filter
             */
            void init(const float sample_freq, const float cutoff_freq)
            {
                if (cutoff_freq <= 0) {
                    return;
                }

                setCutoffFreq(sample_freq, cutoff_freq);
            }

            float apply(float sample)
            {
                float delay_element_0 = sample - _delay_element_1 * _a1 - 
                    _delay_element_2 * _a2;

                if (!isfinite(delay_element_0)) {
                    // don't allow bad values to propigate via the filter
                    delay_element_0 = sample;
                }

                float output = delay_element_0 * _b0 + _delay_element_1 * _b1 + 
                    _delay_element_2 * _b2;

                _delay_element_2 = _delay_element_1;
                _delay_element_1 = delay_element_0;
                return output;
            }

    }; // class Lpf

}
