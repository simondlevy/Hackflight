/*
   motor.h : Support for servos and brushed and brushless motors on STM32F* boards

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

extern "C" {

#pragma once

#include <stdint.h>

class Motor {

    protected:

        void * motor;

        void attach(uint8_t pin, uint32_t motorPwmRate, uint16_t idlePulseUsec);
};

class Servo : public Motor {

    public:

        void attach(uint8_t pin) { Motor::attach(pin, 50, 1500); }

        // 1000 - 2000
        void writeMicroseconds(uint16_t uS);

};

class BrushlessMotor : public Motor {

    public:

        void attach(uint8_t pin) { Motor::attach(pin, 50 /*400*/, 1000); };

        // 1000 - 2000
        void writeMicroseconds(uint16_t uS);

};

class BrushedMotor : public Motor {

    public:

        void attach(uint8_t pin) { Motor::attach(pin, 32000, 0); }

        // 1000 - 2000
        void writeMicroseconds(uint16_t uS);
};

}
