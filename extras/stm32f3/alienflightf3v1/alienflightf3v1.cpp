/*
   alienflightf3v1.cpp 

   Copyright (c) 2018 Simon D. Levy

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

#include "../f3board.h"

namespace hf {

class AlienflightF3V1Board : public F3Board {

}; // class AlienflightF3V1Board

void Board::outbuf(char * buf)
{
    (void)buf;
    //Serial.print(buf);
}

} // namespace hf

static hf::AlienflightF3V1Board  * board;

extern void delay(uint32_t);

void setup() {                

    board = new hf::AlienflightF3V1Board();
}

// the loop routine runs over and over again forever:
void loop() {

    board->ledSet(true);  
    board->delaySeconds(1);  
    board->ledSet(false);
    board->delaySeconds(1);  
}
