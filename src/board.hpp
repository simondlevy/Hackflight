/*
   Class header for board-specific routines

   Copyright (c) 2018 Simon D. Levy

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
        friend class SerialTask;
        friend class PidTask;

        protected:

            //------------------------------------ Core functionality ----------------------------------------------------
            virtual float getTime(void) = 0;


            //-------------------------------------- For real boards -----------------------------------------------------
            virtual void begin(void) {}

            //------------------------------- Serial communications via MSP ----------------------------------------------
            virtual uint8_t serialAvailableBytes(void) { return 0; }
            virtual uint8_t serialReadByte(void)  { return 1; }
            virtual void    serialWriteByte(uint8_t c) { (void)c; }

            //----------------------------------------- Safety -----------------------------------------------------------
            virtual void showArmedStatus(bool armed) { (void)armed; }
            virtual void flashLed(bool shouldflash) { (void)shouldflash; }

            //--------------------------------------- Debugging ----------------------------------------------------------
            static  void outbuf(char * buf);

    }; // class Board

} // namespace
