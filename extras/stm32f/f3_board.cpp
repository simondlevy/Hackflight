/*
   f3_board.cpp : Support for STM32F3 boards
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

#include "f3_board.h"

#include "platform.h"
#include "system.h"
#include "dma.h"
#include "gpio.h"
#include "serial.h"

#include <filters.hpp>

    F3Board::F3Board(void)
    {
        usbInit();

        imuInit();

        motorInit();

        RealBoard::init();
    }

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

    uint32_t F3Board::getMicroseconds(void)
    {
        return micros();
    }

    void F3Board::reboot(void)
    {
        systemResetToBootloader();
    }

    void hf::Board::outbuf(char * buf)
    {
        for (char *p=buf; *p; p++)
            F3Board::outchar(*p);
    }


} // extern "C"
