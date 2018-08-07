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

#include <main.h>
#include "../f3board.h"

namespace hf {

class AlienflightF3V1Board : public F3Board {

}; // class AlienflightF3V1Board

} // namespace hf

void setup() {                

}

// the loop routine runs over and over again forever:
void loop() {

    ledSet(true);  
    delay(1000);  
    ledSet(false);
    delay(1000); 
}
