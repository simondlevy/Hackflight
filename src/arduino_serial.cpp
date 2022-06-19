/*
Copyright (c) 2022 Simon D. Levy

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
 
#include <serial.h>

serialReceiveCallbackPtr _rxCallback;

void serialEvent1(void)
{
    while (Serial1.available()) {
        _rxCallback(Serial1.read(), NULL, micros());
    }
}

bool serialIsTransmitBufferEmpty(void * port)
{
    // Always use Serial
    return Serial.availableForWrite() > 0;
}

void serialOpenPortSbus(serialPortIdentifier_e identifier, serialReceiveCallbackPtr rxCallback)
{
    // Always use Serial1
    (void)identifier;
    Serial1.begin(115200);

    _rxCallback = rxCallback;
}

void * serialOpenPortUsb(void)
{
    Serial.begin(115200);

    return &Serial;
}

uint8_t serialRead(void  * port)
{
    return port == &Serial ? 
        Serial.read() :
        port == &Serial1 ?
        Serial1.read() :
        0;
}

uint32_t serialBytesAvailable(void * port)
{
    return port == &Serial ? 
        Serial.available() :
        port == &Serial1 ?
        Serial1.available() :
        0;
}

void serialWrite(void * port, uint8_t c)
{
    if (port == &Serial) {
        Serial.write(c);
    }
    else if (port == &Serial1) {
        Serial1.write(c);
    }
 }

void serialWriteBuf(void * port, const uint8_t *data, uint32_t count)
{
    if (port == &Serial) {
        Serial.write(data, count);
    }
    else if (port == &Serial1) {
        Serial1.write(data, count);
    }
}
