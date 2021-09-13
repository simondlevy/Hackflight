/*
   Board subclass for real (hardware) boards

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "../board.hpp"
#include "../debugger.hpp"
#include "../filters.hpp"

namespace hf {

    class RealBoard : public Board {

        friend class SerialTask;

        protected:

            static const uint32_t SERIAL_BAUD = 115200;

            void begin(void)
            {
            }

            float getTime(void)
            {
                return micros() / 1.e6f;
            }

            void error(const char * errmsg) 
            {
                while (true) {
                    Debugger::printf("%s\n", errmsg);
                    delay(100);
                }
            }

            virtual void setLed(bool isOn) = 0;

            virtual uint8_t serialAvailable(void) = 0;

            virtual uint8_t serialRead(void) = 0;

    }; // class RealBoard

} // namespace hf
