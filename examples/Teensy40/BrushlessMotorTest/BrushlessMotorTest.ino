/*
   Arduino sketch to test motors on Teensy4.0 flight Controller

   DID YOU REMEMOVE THE PROPELLERS FIRST?

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
static StandardMotor motor1(14);
static StandardMotor motor2(15);
static StandardMotor motor3(9);
static StandardMotor motor4(2);

void setup(void)
{
    motor1.begin();
    motor2.begin();
    motor3.begin();
    motor4.begin();

    Serial.begin(115200);

    delay(1000);
}

void loop(void)
{
    static float val;
    static int dir = +1;

    motor1.set(val);
    motor2.set(val);
    motor3.set(val);
    motor4.set(val);

    if (val <= 0) dir = +1;
    if (val >= 1) dir = -1;
    val += dir * .01;

    delay(100);
}
