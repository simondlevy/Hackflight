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
    short r = rx.getChannelValue(1); // roll
    short p = rx.getChannelValue(2); // pitch
    short t = rx.getChannelValue(3); // throttle
    short y = rx.getChannelValue(0); // yaw
    short a = rx.getChannelValue(5); // aux

    MSP_Message msg = MSP_Parser::serialize_RC(r, p, t, y, a, 0, 0, 0);

    for (byte b=msg.start(); msg.hasNext(); b=msg.getNext()) {
        Serial.write(b);
    }
}


