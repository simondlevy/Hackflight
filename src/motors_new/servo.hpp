/*
   A class for servo motors

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "motor_new.hpp"

#include <Servo.h>

namespace hf {

    class NewServoMotor : public NewMotor {

        private:

            Servo _servo;

        public:

            NewServoMotor(uint8_t pin)
                : NewMotor(pin)
            {
            }

            virtual void begin(void) override 
            {
                _servo.attach(_pin);
            }

            virtual void write(float value) override
            {
                _servo.write((uint8_t)(180*value));
            }

    }; // class NewServoMotor

} // namespace hf
