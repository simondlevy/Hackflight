/*
   fl3board.cpp : STM32F3 implementation of Hackflight Board routines

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

extern void setup(void);
extern void loop(void);

extern "C" { // So we can talk to C support code

#include "f3board.h"

#include "platform.h"

#include "system.h"
#include "dma.h"
#include "gpio.h"
#include "timer.h"
#include "serial.h"
#include "serial_uart.h"
#include "exti.h"

static const uint8_t LED_PIN = 16;

GPIO_TypeDef * gpio_type_from_pin(uint8_t pin);
uint16_t gpio_pin_from_pin(uint8_t pin);
serialPort_t * serial0;

namespace hf {

    void F3Board::delaySeconds(float sec)
    {
        delay(sec*1000);
    }

    void F3Board::ledSet(bool is_on)
    { 
        uint16_t gpio_pin = gpio_pin_from_pin(LED_PIN);

        GPIO_TypeDef * gpio = gpio_type_from_pin(LED_PIN);

        if (is_on) {
            digitalLo(gpio, gpio_pin);
        }
        else {
            digitalHi(gpio, gpio_pin);
        }
    }

    uint8_t F3Board::serialAvailableBytes(void)
    {
        return 0; // XXX
    }

    uint8_t F3Board::serialReadByte(void)
    {
        return 0; // XXX
    }

    void F3Board::serialWriteByte(uint8_t c)
    {
        (void)c; // XXX
    }

    uint32_t F3Board::getMicroseconds(void)
    {
        return micros();
    }

    bool F3Board::getGyrometer(float gyroRates[3])
    {
        (void)gyroRates; // XXX
        return false;
    }

    bool F3Board::getQuaternion(float quat[4])
    {
        (void)quat; // XXX
        return false;
    }

    // Support prototype version where LED is on pin A1
    F3Board::F3Board(void)
    {
        // Do general real-board initialization
        RealBoard::init();
    }

    void Board::outbuf(char * buf)
    {
        for (char *p=buf; *p; p++)
            serialWrite(serial0, *p);
    }

} // namespace hf

static void ledInit(void)
{
    GPIO_TypeDef * gpio = gpio_type_from_pin(LED_PIN);

    gpio_config_t cfg;

    cfg.pin = gpio_pin_from_pin(LED_PIN);
    cfg.mode = Mode_Out_PP;
    cfg.speed = Speed_2MHz;

    gpioInit(gpio, &cfg);
}

serialPort_t * serial0_open(void);

void SetSysClock(void);

int main(void) 
{
    // start fpu
    SCB->CPACR = (0x3 << (10*2)) | (0x3 << (11*2));

    SetSysClock();

    systemInit();

    timerInit();  // timer must be initialized before any channel is allocated

    serial0 = serial0_open();

    dmaInit();

    ledInit();

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
