/*
   ReadSpektrum : example of how to read from a Spektrum DSM receiver using Arduino API

   This file is part of BreezySTM32.

   SpektrumDSM is free software: you can redistribute it and/or modify
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


#include <Arduino.h>
#include <SpektrumDSM.h>

SpektrumDSM2048 * rx;

static uint8_t avail;

int serialAvailable(void)
{
    return avail;
}

uint8_t serialRead(void)
{
    avail--;

    return Serial3.read();
}

void serialEvent3()
{
    avail = 1;

    rx->handleSerialEvent(micros());
}

void setup() {
  
  Serial.begin(115200);

  Serial3.begin(115200);

  rx = new SpektrumDSM2048();
}

void loop() {

    if (rx->gotNewFrame()) {

        uint16_t values[8];

        rx->getChannelValues(values);

        for (int k=0; k<8; ++k) {
            Serial.printf("%d ", values[k]);
        }
        Serial.printf("\n");
    }

    // Allow some time between readings
    delay(10);  
}
