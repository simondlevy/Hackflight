/*
   TinyPICO implementation of Hackflight Board routines

   Uses EM7180 SENtral Sensor Hub in master mode mode

   Copyright (c) 2019 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>
#include "boards/realboards/arduino.hpp"

#include <TinyPICO.h>

namespace hf {

    class TinyPico : public RealBoard {

        private:

            TinyPICO tp;

        protected:

            void setLed(bool isOn) 
            { 
                tp.DotStar_SetPixelColor(0, isOn?255:0, 0);
            }

            uint8_t serialNormalAvailable(void)
            {
                return Serial.available();
            }

            uint8_t serialNormalRead(void)
            {
                return Serial.read();
            }

            void serialNormalWrite(uint8_t c)
            {
                Serial.write(c);
            }

         public:

            void begin(void) 
            {
                Serial.begin(115200);

                // This will blink the LED
                RealBoard::begin();

                // Hang a bit 
                delay(100);

                // Start I^2C
                Wire.begin();

                // Hang a bit
                delay(100);
            }

    }; // class TinyPico

} // namespace hf
