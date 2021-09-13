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

            void serialWrite(uint8_t byte)
            {
                Serial.write(byte);
            }

            void begin(void)
            {
                // Start serial communcation for GCS/debugging
                Serial.begin(SERIAL_BAUD);

                // This will blink the LED
                RealBoard::begin();
            }

    }; // class ArduinoSerial

    void Debugger::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
