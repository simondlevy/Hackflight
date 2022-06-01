/*
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

#include "hackflight.h"
#include "serial.h"

static HardwareSerial * _port;

serialReceiveCallbackPtr _rxCallback;

void serialEvent1(void)
{
    while (_port == &Serial1 && Serial1.available()) {
        _rxCallback(Serial1.read(), NULL, micros());
    }
}

void serialOpenPortSbus(
        serialPortIdentifier_e identifier,
        serialReceiveCallbackPtr rxCallback,
        void *rxCallbackData)
{
    switch (identifier) {
        case SERIAL_PORT_USART1:
            _port = &Serial1;
            break;
        case SERIAL_PORT_USART2:
            _port = &Serial2;
            break;
    }

    _port->begin(100000, SERIAL_SBUS);

    _rxCallback = rxCallback;
}


void * serialOpenPortUsb(void)
{
    Serial.begin(115200);
    return (void *)&Serial;
}

void serialBeginWrite(void * port)
{
    (void)port;
}

void serialEndWrite(void * port)
{
    (void)port;
}

bool serialIsTransmitBufferEmpty(void * port)
{
    ((HardwareSerial *)port)->flush();
    return true;
}

uint8_t serialRead(void  * port)
{
    return ((HardwareSerial *)port)->read();
}

uint32_t serialRxBytesWaiting(void * port)
{
    return ((HardwareSerial *)port)->available();
}

uint32_t serialTxBytesFree(void * port)
{
    (void)port;
    return 0;
}

void serialWaitForPortToFinishTransmitting(void * port)
{
    ((HardwareSerial *)port)->flush();
}

void serialWrite(void * port, uint8_t c)
{
    ((HardwareSerial *)port)->write(c);
}

void serialWriteBuf(void * port, const uint8_t *data, uint32_t count)
{
    ((HardwareSerial *)port)->write(data, count);
}
