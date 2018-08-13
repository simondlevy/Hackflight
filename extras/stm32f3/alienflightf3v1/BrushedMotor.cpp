/*
  BrushedMotor

  Runs a brushed motor for one second.

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

#include <motor.h>
#include <f3_board.h>

BrushedMotor motor;

static uint16_t value;
static int16_t  direction;

static F3Board * board;

extern "C" {

void setup() {                

    board = new F3Board();

    // Valid pins for ALIENFLIGHTF3 are 0, 8, 14, 15
    motor.attach(14);

    board->delaySeconds(0.1);

    value = 1100;
    direction = +1;
}

void loop() {

    motor.writeMicroseconds(value);

    value += direction;

    if (value == 1200)
        direction = -1;

    if (value == 1100)
        direction = +1;

    board->delaySeconds(.01);
}

}
