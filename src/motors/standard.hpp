/*
   Arduino code for brushless motor running on standard ESC

   Copyright (c) 2018 Juan Gallostra Acin, Simon D. Levy, Pep Martí Saumell

   MIT License
 */

#pragma once

#include "motor.hpp"

namespace hf {

    class StandardMotor : public Motor {

        private:

#ifdef ESP32
            static const uint16_t OFFSET = 25;
#else
            static const uint16_t OFFSET = 0;
#endif
            static const uint16_t MINVAL = 125;
            static const uint16_t MAXVAL = 250;

            void writeValue(uint8_t index, uint16_t value)
            {
                analogWrite(_pins[index], value+OFFSET);
            }

        public:

            StandardMotor(const uint8_t pins[], const uint8_t count) 
                : Motor(pins, count)
            {
            }

            virtual void begin(void) override
            {
                for (uint8_t k=0; k<_count; ++k) {
                    pinMode(_pins[k], OUTPUT);
                    writeValue(k, MINVAL);
                }
            }

            virtual void write(uint8_t index, float value) override
            { 
                writeValue(index, (uint16_t)(MINVAL+value*(MAXVAL-MINVAL))); 
            }

    }; // class StandardMotor

} // namespace hf
