/*
   drv_serial.c : serial support for STM32F103CB

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/drv_serial.c

   This file is part of BreezySTM32.

   BreezySTM32 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BreezySTM32 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BreezySTM32.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include "drv_serial.h"

void serialPrint(serialPort_t *instance, const char *str)
{
    uint8_t ch;
    while ((ch = *(str++)) != 0) {
        serialWrite(instance, ch);
    }
}

uint32_t serialGetBaudRate(serialPort_t *instance)
{
    return instance->baudRate;
}

void serialWrite(serialPort_t *instance, uint8_t ch)
{
    instance->vTable->serialWrite(instance, ch);
}

uint8_t serialTotalBytesWaiting(serialPort_t *instance)
{
    return instance->vTable->serialTotalBytesWaiting(instance);
}

uint8_t serialRead(serialPort_t *instance)
{
    return instance->vTable->serialRead(instance);
}

void serialSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    instance->vTable->serialSetBaudRate(instance, baudRate);
}

bool isSerialTransmitBufferEmpty(serialPort_t *instance)
{
    return instance->vTable->isSerialTransmitBufferEmpty(instance);
}

inline void serialSetMode(serialPort_t *instance, portMode_t mode)
{
    instance->vTable->setMode(instance, mode);
}

