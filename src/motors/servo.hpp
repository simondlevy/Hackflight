/*
   A class for servo motors

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "motor.hpp"

#include <Servo.h>

namespace hf {

    class ServoMotor : public Motor {

        private:

            Servo _servo;

        public:

            ServoMotor(uint8_t pin)
                : Motor(pin)
            {
            }

            virtual void begin(void) override 
            {
                _servo.attach(_pin);
            }

            virtual void write(float value) override
            {
                // [-.5,+.5] => [0,180]
                _servo.write((uint8_t)(180*(value+0.5)));
            }

    }; // class ServoMotor

} // namespace hf
