/*
   I^2C support for STM32F3 boards

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

extern "C" {

#include "platform.h"

#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/time.h"
#include "drivers/light_led.h"
#include "drivers/bus_i2c.h"

#include "pg/bus_i2c.h"

#include "target.h"

#include "stm32f30x.h"

static bool _writeRegister(uint8_t address, uint8_t subAddress, uint8_t data)
{
    return i2cWrite(I2CDEV_2, address, subAddress, data);
}

static void _readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    i2cRead(I2CDEV_2, address, subAddress, count, dest);
}

} // extern "C"
 
#include <CrossPlatformI2C_Core.h>

uint8_t cpi2c_open(uint8_t address, uint8_t bus)
{
    (void)bus;
    return address;
}

bool cpi2c_writeRegister(uint8_t address, uint8_t subAddress, uint8_t data)
{
    return _writeRegister(address, subAddress, data);
    
    return false;
}

void cpi2c_readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    _readRegisters(address, subAddress, count, dest);
}
