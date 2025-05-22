/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * exti.c - Unified implementation of the exti interrupts
 */
#include <stdbool.h>

#include <stm32fxxx.h>

#include "exti.h"
#include "nvicconf.h"

static bool didInit;

/* Interruption initialisation */
void extiInit()
{
  static NVIC_InitTypeDef NVIC_InitStructure;

  if (didInit)
    return;

  // This is required for the EXTI interrupt configuration since EXTI
  // lines are set via the SYSCFG peripheral; eg.
  // SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);
  RCC_AHB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); 

  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTI4_PRI;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTI15_10_PRI;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  didInit = true;
}

bool extiTest(void)
{
  return didInit;
}

// NRF interrupt
void __attribute__((used)) EXTI4_IRQHandler(void)
{
  NVIC_ClearPendingIRQ(EXTI4_IRQn);
  EXTI_ClearITPendingBit(EXTI_Line4);
  EXTI4_Callback();
}

void __attribute__((weak)) EXTI4_Callback(void) { }


// Deck/sensor interrupts
void __attribute__((used)) EXTI15_10_IRQHandler(void)
{
  NVIC_ClearPendingIRQ(EXTI15_10_IRQn);

  // Gyro
  if (EXTI_GetITStatus(EXTI_Line14) == SET) {
    EXTI_ClearITPendingBit(EXTI_Line14);
    EXTI14_Callback();
  }
}

void __attribute__((weak)) EXTI14_Callback(void) { }
