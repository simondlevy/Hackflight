/*
main.cpp : entry for STM32F3 firmware

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

#include <main.h>

#include "platform.h"

#include "system.h"
#include "dma.h"
#include "gpio.h"
#include "timer.h"
#include "serial.h"
#include "serial_uart.h"
#include "exti.h"
#include "bus_spi.h"

extern void setup(void);
extern void loop(void);

// Board-specific
GPIO_TypeDef * gpio_type_from_pin(uint8_t pin);
uint16_t gpio_pin_from_pin(uint8_t pin);
serialPort_t * serial0_open(void);

void SetSysClock(void);

serialPort_t * serial0;

static const uint8_t LED_PIN = 16;

void ledInit(void)
{
    GPIO_TypeDef * gpio = gpio_type_from_pin(LED_PIN);

    gpio_config_t cfg;

    cfg.pin = gpio_pin_from_pin(LED_PIN);
    cfg.mode = Mode_Out_PP;
    cfg.speed = Speed_2MHz;

    gpioInit(gpio, &cfg);
}

void ledSet(bool value)
{
    uint16_t gpio_pin = gpio_pin_from_pin(LED_PIN);

    GPIO_TypeDef * gpio = gpio_type_from_pin(LED_PIN);

    if (value) {
        digitalLo(gpio, gpio_pin);
    }
    else {
        digitalHi(gpio, gpio_pin);
    }
}

void reset(void)
{
    systemReset();
}

void resetToBootloader(void)
{
    systemResetToBootloader();
}

int main(void) 
{
    // start fpu
    SCB->CPACR = (0x3 << (10*2)) | (0x3 << (11*2));

    SetSysClock();

    systemInit();

    timerInit();  // timer must be initialized before any channel is allocated

    serial0 = serial0_open();

    dmaInit();

    setup();

    while (true) {

#ifndef EXTERNAL_DEBUG
        static uint32_t dbg_start_msec;
        // support reboot from host computer
        if (millis()-dbg_start_msec > 100) {
            dbg_start_msec = millis();
            while (serialRxBytesWaiting(serial0)) {
                uint8_t c = serialRead(serial0);
                if (c == 'R') 
                    systemResetToBootloader();
            }
        }
#endif
        loop();
    }

} // main

} // extern "C"
