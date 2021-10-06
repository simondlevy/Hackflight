/*
   A class for servo motors on Arduino

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "../servo.hpp"

#include <Servo.h>

namespace hf {

    class ArduinoServoMotor : public ServoMotor {

        private:

            Servo _servo;

        public:

            ArduinoServoMotor(uint8_t pin)
                : ServoMotor(pin)
            {
            }

            virtual void begin(void) override
            {
                _servo.attach(_pin);
            }

            virtual void write(float value) override
            {
                uint8_t usec = (uint8_t)((value + 1) * 90);
                _servo.write(usec);
            }

    }; // class ArduinoServoMotor

} // namespace hf
