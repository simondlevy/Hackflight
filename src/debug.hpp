/*
    debug.hpp : Cross-platform serial debugging support for Hackflight
       
    Provides a single method printf() for formatted printing of debug
    messages.  Your Board implementation should provide and outbuf(char *)
    method that displays the message in an appropriate way.

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

#include <stdarg.h>
#include <stdio.h>

namespace hf {

    class Debug {

        public:

            static void printf(const char * fmt, ...)
            {
                va_list ap;
                va_start(ap, fmt);
                char buf[200];
                vsnprintf(buf, 200, fmt, ap); 
                Board::outbuf(buf);
                va_end(ap);
            }

            // for boards that do not support floating-point vnsprintf
            static void printfloat(float val, uint8_t prec=3)
            {
                uint16_t mul = 1;
                for (uint8_t k=0; k<prec; ++k) {
                    mul *= 10;
                }
                char sgn = '+';
                if (val < 0) {
                    val = -val;
                    sgn = '-';
                }
                uint32_t bigval = (uint32_t)(val*mul);
                Debug::printf("%c%d.%d", sgn, bigval/mul, bigval % mul);
            }

            static void printlnfloat(float val, uint8_t prec=3)
            {
                printfloat(val, prec);
                printf("\n");
            }

     }; // class Debug

} // namespace hf


