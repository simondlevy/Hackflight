/*
   Arduino code for brushed motors

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "motor.hpp"

namespace hf {

    class BrushedMotor : public Motor {

        public:

            BrushedMotor(const uint8_t * pins, const uint8_t count) 
                : Motor(pins, count) 
            {
            }

            void begin(void) override
            {
                for (uint8_t k=0; k<_count; ++k) {
                    analogWriteFrequency(_pins[k], 10000);  
                    analogWrite(_pins[k], 0);  
                }
            }

            virtual void write(uint8_t index, float value) override
            {
                analogWrite(_pins[index], (uint8_t)(value * 255));
            }

    }; // class BrushedMotor

} // namespace hf
