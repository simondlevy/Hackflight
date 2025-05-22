/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * digital.c - Deck-API digital IO implementation
 */

#include <stm32fxxx.h>

#include "gpio.h"
#include "digital.h"

void pinMode(const uint8_t pin, const uint32_t mode)
{
  const auto mapping = GPIOMappings[pin];

  RCC_AHB1PeriphClockCmd(mapping.periph, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure = {
  
      mapping.pin,
      mode == OUTPUT ? GPIO_Mode_OUT:GPIO_Mode_IN,
      GPIO_Speed_25MHz,
      mode == OUTPUT ? GPIO_OType_PP : GPIO_OType_OD,
      mode == INPUT_PULLUP ? GPIO_PuPd_UP : GPIO_PuPd_DOWN
  };

  GPIO_Init(mapping.port, &GPIO_InitStructure);
}

void digitalWrite(const uint8_t pin, const uint32_t val)
{
    const GPIO_Mapping_t mapping = GPIOMappings[pin];

    GPIO_WriteBit(mapping.port, mapping.pin, val ? Bit_SET : Bit_RESET);
}

int digitalRead(const uint8_t pin)
{
    const auto mapping = GPIOMappings[pin];

    const auto val = GPIO_ReadInputDataBit(mapping.port, mapping.pin);

    return (val == Bit_SET) ? HIGH : LOW;
}
