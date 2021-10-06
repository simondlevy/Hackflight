/*
   Class header for board-specific routines

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <stdarg.h>
#include <stdint.h>

namespace hf {

    class Board {

        public:

            // --------------- Pure functionality ------------------------------
            virtual float getTime(void) = 0;

            // ----------------- For real boards -------------------------------
            virtual void begin(void) { }
            virtual void showArmedStatus(bool armed) { (void)armed; }
            virtual void flashLed(bool shouldflash) { (void)shouldflash; }

    }; // class Board

} // namespace
