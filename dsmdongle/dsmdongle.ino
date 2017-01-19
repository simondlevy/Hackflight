/*
   dsmdongle.ino : Arduino/Teensy sketch for sending channel values from 
   Spektrum DSM receiver to host computer.

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


// https://github.com/simondlevy/SpektrumDSM
#include <SpektrumDSM.h>
static SpektrumDSM2048 rx;

// https://github.com/simondlevy/hackflight/tree/master/parser
#include <MSPPG.h>

void setup(void)
{  
    // Start receiver
    rx.begin();

    // Set up serial communication over USB
    Serial.begin(115200);
}


void loop(void)
{  
    static uint16_t values[8];

    values[0] = rx.getChannelValue(1); // roll
    values[1] = rx.getChannelValue(2); // pitch
    values[2] = rx.getChannelValue(3); // throttle
    values[3] = rx.getChannelValue(0); // yaw
    values[4] = rx.getChannelValue(5); // aux

    // unused
    values[5] = 0;
    values[6] = 0;
    values[7] = 0;

    Serial.printf("%d %d %d %d %d\n", values[0], values[1], values[2], values[3], values[4]);
}


