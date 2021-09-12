/*
   A class for brushed motors

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#ifdef ESP32
#include <analogWrite.h>
#endif

#include "../rotary.hpp"

namespace hf {

    class ArduinoBrushedMotor : public RotaryMotor {

        public:

            ArduinoBrushedMotor(uint8_t pin)
                : RotaryMotor(pin)
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

    }; // class BrushedMotor

} // namespace hf
