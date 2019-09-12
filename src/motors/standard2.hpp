/*
   standard2.hpp : Arduino code for brushless motor running on
   standard ESC.  Alternate version using Servo

   Copyright (c) 2019 Simon D. Levy

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

#pragma once

#include <Servo.h>

class StandardMotor {

    private:

        static const int MINVAL = 50;
        static const int MAXVAL = 150;

        Servo _servo;

    public:

        StandardMotor(uint8_t pin) 
        {
            _servo.attach(pin);
        }

        void begin(void)
        {
            _servo.write(MAXVAL);
            delay(250);
            _servo.write(MINVAL);
        }

        void set(float val)
        {
            int intval = (int)(MINVAL + val * (MAXVAL-MINVAL));
            _servo.write(intval);
        }
};
