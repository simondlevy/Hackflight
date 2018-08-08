/*
f3board.cpp : Support for STM32F3 boards

Copyright (C) 2018 Simon D. Levy 

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

extern "C" {

#include "f3board.h"

#include "platform.h"
#include "system.h"
#include "dma.h"
#include "gpio.h"
#include "timer.h"
#include "serial.h"
#include "serial_uart.h"
#include "exti.h"
#include "bus_spi.h"

extern serialPort_t * serial0;

void F3Board::delaySeconds(float sec)
{
    delay(sec*1000);
}

void F3Board::ledSet(bool is_on)
{ 
    uint16_t gpio_pin = LED0_PIN;

    GPIO_TypeDef * gpio = LED0_GPIO;

    if (is_on) {
        digitalLo(gpio, gpio_pin);
    }
    else {
        digitalHi(gpio, gpio_pin);
    }
}

uint8_t F3Board::serialAvailableBytes(void)
{
    return serialRxBytesWaiting(serial0);
}

uint8_t F3Board::serialReadByte(void)
{
    return serialRead(serial0);
}

void F3Board::serialWriteByte(uint8_t c)
{
    serialWrite(serial0, c);
    while (!isSerialTransmitBufferEmpty(serial0));
}

uint32_t F3Board::getMicroseconds(void)
{
    return micros();
}

bool F3Board::getGyrometer(float gyroRates[3])
{
    (void)gyroRates; // XXX

    static uint32_t _time;
    uint32_t time = micros();
    if (time-_time > 5000) {
        _time = time;
        return true;
    }
    return false;
}

bool F3Board::getQuaternion(float quat[4])
{
    (void)quat; // XXX

    static uint32_t _time;
    uint32_t time = micros();
    if (time-_time > 10000) {
        _time = time;
        return true;
    }
    return false;
}

// Support prototype version where LED is on pin A1
F3Board::F3Board(void)
{
    // Do general real-board initialization
    RealBoard::init();
}

void hf::Board::outbuf(char * buf)
{
    for (char *p=buf; *p; p++)
        serialWrite(serial0, *p);
}

} // extern "C"
