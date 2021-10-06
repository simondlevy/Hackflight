/*
   Arduino code for brushless motor running on standard ESC

   Copyright (c) 2021 Juan Gallostra Acin, Simon D. Levy, Pep Martí Saumell

   Simon D. Levy
*/

#pragma once

#ifdef ESP32
#include <analogWrite.h>
#endif

#include "../rotary.hpp"

namespace rft {

    class ArduinoBrushlessMotor : public RotaryMotor {

        private:

            static const uint16_t MINVAL = 125;
            static const uint16_t MAXVAL = 250;

            void writeValue(uint16_t value)
            {
                analogWrite(_pin, value);
            }

        public:

            ArduinoBrushlessMotor(uint8_t pin)
                : RotaryMotor(pin)
            {
            }

            virtual void begin(void) override
            {
                pinMode(_pin, OUTPUT);
                writeValue(MINVAL);
            }

            virtual void write(float value) override
            { 
                writeValue((uint16_t)(MINVAL+value*(MAXVAL-MINVAL))); 
            }

    }; // class BrushlessMotor

} // namespace rft
