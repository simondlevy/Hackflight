/*
   drv_gpio.c :  GPIO support for STM32F103CB

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/drv_gpio.c

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

#pragma once

typedef enum {
    Mode_AIN = 0x0,
    Mode_IN_FLOATING = 0x04,
    Mode_IPD = 0x28,
    Mode_IPU = 0x48,
    Mode_Out_OD = 0x14,
    Mode_Out_PP = 0x10,
    Mode_AF_OD = 0x1C,
    Mode_AF_PP = 0x18
} GPIO_Mode;

typedef enum {
    Speed_10MHz = 1,
    Speed_2MHz,
    Speed_50MHz
} GPIO_Speed;

typedef enum {
    Pin_0 = 0x0001,
    Pin_1 = 0x0002,
    Pin_2 = 0x0004,
    Pin_3 = 0x0008,
    Pin_4 = 0x0010,
    Pin_5 = 0x0020,
    Pin_6 = 0x0040,
    Pin_7 = 0x0080,
    Pin_8 = 0x0100,
    Pin_9 = 0x0200,
    Pin_10 = 0x0400,
    Pin_11 = 0x0800,
    Pin_12 = 0x1000,
    Pin_13 = 0x2000,
    Pin_14 = 0x4000,
    Pin_15 = 0x8000,
    Pin_All = 0xFFFF
} GPIO_Pin;

typedef struct {
    uint16_t pin;
    GPIO_Mode mode;
    GPIO_Speed speed;
} gpio_config_t;

#define digitalHi(p, i)     { p->BSRR = i; }
#define digitalLo(p, i)     { p->BRR = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }
#define digitalIn(p, i)     (p->IDR & i)

void gpioInit(GPIO_TypeDef *gpio, gpio_config_t *config);
void gpioExtiLineConfig(uint8_t portsrc, uint8_t pinsrc);
void gpioPinRemapConfig(uint32_t remap, bool enable);
