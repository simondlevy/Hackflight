/**
 * Copyright (C) 2011-2022 Bitcraze AB, 2024 Simon D. Levy
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
 */

#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

class Num {

    public:

        /* Half precision floating point **********************************************
         *
         * To not use the GCC implementation, uint16_t is used to carry fp16 values
         *
         * FP16 or Half precision floating points is specified by IEEE 754 as binary 16.
         * (float is specified as binary 32). This implementation is NOT GUARANTEED to
         * be conform to the ieee 754 specification, it is 'just' good enough for the
         * Crazyflie usage. For more info about fp16 see
         * http://en.wikipedia.org/wiki/Half-precision_floating-point_format
         *
         * The current implementation has the following limitation:
         *  * No subnormalized number generation
         *  * Rounding seems to give at least 11 bits precision
         *  * Faster and smaller than the GCC implementation
         */

        static uint16_t single2half(float number)
        {
            uint32_t num = 0;

            memcpy(&num, &number, 4);

            auto s = num >> 31;

            auto e = (num >> 23)&0x0FF;

            return 

                (e==255) && (num&0x007fffff) ?  0x7E00 : // NaN

                e>(127+15) ? s?0xFC00:0x7C00 :  //+/- inf

                e<(127-15) ? 0 : // Don't generate subnormalised representation

                (s<<15) | ((e-127+15)<<10) | (((num>>13)&0x3FF)+((num>>12)&0x01));
        }

        static float half2single(uint16_t number)
        {
            uint32_t s = number>>15;

            uint32_t e = (number>>10)&0x01F;

            //All binary16 can be mapped in a binary32
            if(e==0) {
                e=15-127;
            }

            auto fp32 = 

                e != 0x1F ? (s<<31) | ((e+127-15)<<23) | ((number&0x3ff)<<13) :

                number&0x03FF ?  0x7FC00000 : // NaN

                s?0xFF800000:0x7F800000;  //+/- inf

            float result = 0;

            memcpy(&result, &fp32, 4);

            return result;
        }

        static uint16_t limitUint16(int32_t value)
        {
            return (uint16_t)(
                    value > UINT16_MAX  ? UINT16_MAX :
                    value < 0 ? 0 :
                    value);
        }

        static float fconstrain(float value, const float minVal, const float maxVal)
        {
            return fminf(maxVal, fmaxf(minVal,value));
        }

        static float deadband(float value, const float threshold)
        {
            return 

                fabsf(value) < threshold ? 0 :

                value > 0 ? value - threshold :

                value < 0 ? value + threshold :

                value;
        }

        static float rescale(
                const float value,
                const float oldmin, 
                const float oldmax, 
                const float newmin, 
                const float newmax) 
        {
            return (value - oldmin) / (oldmax - oldmin) * 
                (newmax - newmin) + newmin;
        }

}; // class Num
