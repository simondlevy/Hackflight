/*
   Class for Arduino-style serial comms

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "../realboard.hpp"

namespace hf {

    class ArduinoSerial : public RealBoard {

        protected:

            ArduinoSerial()
            {
            }

            uint8_t serialAvailable()
            {
                return Serial.available();
            }

            uint8_t serialRead(void)
            {
                return Serial.read();
            }

            void begin(void)
            {
                // This will blink the LED
                RealBoard::begin();
            }

    }; // class ArduinoSerial

    void Debugger::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
