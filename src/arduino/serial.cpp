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

static const uint32_t MY_SERIAL_SBUS = SERIAL_8E2 | 0xC000ul;

serialReceiveCallbackPtr _callback;
void * _data;

void serialEvent1(void)
{
    while (Serial1.available()) {
        _callback(Serial1.read(), _data, micros());
    }
}

bool serialIsTransmitBufferEmpty(void * port)
{
    (void)port;

    // Always use Serial
    return Serial.availableForWrite() > 0;
}

void serialOpenPortDsmx(
        serialPortIdentifier_e identifier,
        serialReceiveCallbackPtr callback,
        void * data)
{
    (void)identifier;

    // Always use Serial1
    Serial1.begin(115200);
 
    _callback = callback;
    _data = data;
}

void serialOpenPortSbus(
        serialPortIdentifier_e identifier,
        serialReceiveCallbackPtr callback,
        void * data)
{
    (void)identifier;

    // Always use Serial1
#if defined(TEENSYSUINO)
    Serial1.begin(100000, SERIAL_8E2_RXINV_TXINV);
#elif defined(STM32L496xx) || defined(STM32L476xx) || defined(STM32L433xx) || defined(STM32L432xx)
    Serial1.begin(100000, SERIAL_SBUS);
#elif defined(ESP32)
    Serial1.begin(100000, SERIAL_8E2, rxpin, txpin, true);
#else
    Serial1.begin(100000, SERIAL_8E2);
#endif
 
    _callback = callback;
    _data = data;
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
    if (port == &Serial1) {
        Serial1.write(c);
    }
 }

void serialWriteBuf(void * port, const uint8_t *data, uint32_t count)
{
    if (port == &Serial) {
        Serial.write(data, count);
    }
    if (port == &Serial1) {
        Serial1.write(data, count);
    }
}

// These functions are stubbed, because for Arduino we should be using Serial.print*()

void serialDebugFlush(void)
{
}

void serialDebugPrintf(const char *fmt, ...)
{
    (void)fmt;
}

void serialDebugSetPort(void * p)
{
    (void)p;
}
