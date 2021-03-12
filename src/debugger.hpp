/*
    Cross-platform serial debugging support for Hackflight
       
    Provides printf() and related static methods for formatted printing of
    debug messages.  Your Board implementation should provide and outbuf(char
    method that displays the message in an appropriate way.

    Copyright (c) 2021 Simon D. Levy

    MIT License
*/

#pragma once

#include <stdarg.h>
#include <stdio.h>

#include "board.hpp"

namespace hf {

    class Debugger {

        friend class Hackflight;

        private:

            static constexpr float ADHOC_RATE = 100.f; // Hz

            static constexpr float _adhoc_period = 1.f / ADHOC_RATE;

            Board * _board;

            float _prevTime;

        protected:

            void begin(Board * board)
            {
                _board = board;
            }

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
                Debugger::printf("%c%d.%d", sgn, bigval/mul, bigval % mul);
            }

            static void printlnfloat(float val, uint8_t prec=3)
            {
                printfloat(val, prec);
                printf("\n");
            }

        }; // class Debugger

} // namespace hf
