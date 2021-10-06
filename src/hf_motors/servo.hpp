/*
   Abstract class for servo motors

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "../HF_motor.hpp"

namespace hf {

    class ServoMotor : public Motor {

        public:

            ServoMotor(uint8_t pin)
                : Motor(pin)
            {
            }

            virtual float constrainValue(float value) override
            {
                return Filter::constrainMinMax(value, -1, +1);
            }


    }; // class ServoMotor

} // namespace hf
