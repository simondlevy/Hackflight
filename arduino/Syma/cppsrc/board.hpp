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

        friend class Hackflight;
        friend class Debugger;
        friend class TimerTask;
        friend class PidControlTask;

        protected:

            // --------------- Pure functionality ------------------------------
            virtual float getTime(void) = 0;

            // ----------------- For real boards -------------------------------
            virtual void begin(void) { }
            virtual void flashLed(bool shouldflash) { (void)shouldflash; }

    }; // class Board

} // namespace
