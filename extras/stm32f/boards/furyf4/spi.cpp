/*
   SPI support for STM32F3 boards

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

// Here we put code that interacts with Cleanflight
extern "C" {

#include "spi.h"

    static busDevice_t _bus;

    void spi_init(SPI_TypeDef * instance, IO_t pin)
    {
        SPIDevice device = SPIINVALID;

        if (instance == SPI1) {
            device = SPIDEV_1;
        }

        else if (instance == SPI2) {
            device = SPIDEV_2;
        }

        else if (instance == SPI3) {
            device = SPIDEV_3;
        }

        spiPinConfigure(spiPinConfig(0));
        spiPreInit();

        spiInit(device);

        spiBusSetInstance(&_bus, instance);

        _bus.busdev_u.spi.csnPin = pin;

        delay(100);

        IOInit(_bus.busdev_u.spi.csnPin, OWNER_MPU_CS, 0);
        IOConfigGPIO(_bus.busdev_u.spi.csnPin, SPI_IO_CS_CFG);
        IOHi(_bus.busdev_u.spi.csnPin);
        spiSetDivisor(_bus.busdev_u.spi.instance, SPI_CLOCK_FAST);
    }

    void spi_write_register(uint8_t subAddress, uint8_t data)
    {
        spiBusWriteRegister(&_bus, subAddress, data);
    }

    void spi_read_registers(uint8_t subAddress, uint8_t count, uint8_t * dest)
    {
        spiBusReadRegisterBuffer(&_bus, subAddress, dest, count);
    }

} // extern "C"
