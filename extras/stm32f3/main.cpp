/*
main.cpp : Entry point for STM32F3 boards

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

#include "platform.h"
#include "system.h"
#include "dma.h"
#include "gpio.h"
#include "timer.h"
#include "serial.h"
#include "serial_uart.h"
#include "exti.h"

#
static void ledInit(void)
{
    GPIO_TypeDef * gpio = LED0_GPIO;

    gpio_config_t cfg;

    cfg.pin = LED0_PIN;
    cfg.mode = Mode_Out_PP;
    cfg.speed = Speed_2MHz;

    gpioInit(gpio, &cfg);
}

int main(void) {

    // Defined in each sketch
    void setup();
    void loop();

    void SetSysClock(void);

    // start fpu
    SCB->CPACR = (0x3 << (10*2)) | (0x3 << (11*2));

    SetSysClock();

    systemInit();

    timerInit();  // timer must be initialized before any channel is allocated

    dmaInit();

    ledInit();

    setup();

    while (true) {

        loop();
    }
} // main

} // extern "C"
