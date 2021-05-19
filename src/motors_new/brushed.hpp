/*
   A class for brushed motors

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "motor_new.hpp"

namespace hf {

    class NewBrushedMotor : public NewMotor {

        public:

            NewBrushedMotor(uint8_t pin)
                : NewMotor(pin)
            {
            }

            virtual void begin(void) override 
            {
                analogWriteFrequency(_pin, 10000);
                analogWrite(_pin, 0);
            }

            virtual void write(float value) override
            {
                analogWrite(_pin, (uint8_t)(value * 255));
            }

    }; // class NewBrushedMotor

} // namespace hf
