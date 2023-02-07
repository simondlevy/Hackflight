/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

static const uint8_t LED_PIN = PC13;

static void reboot(void)
{
    __enable_irq();
    HAL_RCC_DeInit();
    HAL_DeInit();
    SysTick->CTRL = SysTick->LOAD = SysTick->VAL = 0;
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

    const uint32_t p = (*((uint32_t *) 0x1FFF0000));
    __set_MSP( p );

    void (*SysMemBootJump)(void);
    SysMemBootJump = (void (*)(void)) (*((uint32_t *) 0x1FFF0004));
    SysMemBootJump();

    NVIC_SystemReset();
}

static void blinkLed(void)
{
    auto msec = millis();
    static uint32_t prev;

    if (msec - prev > 500) {
        static bool on;
        prev = msec;
        on = !on;
        digitalWrite(LED_PIN, on);
    }
}


void setup(void)
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
}

void loop(void)
{
    if (Serial.available() && Serial.read() == 'R') {
        reboot();
    }

    blinkLed();
}
