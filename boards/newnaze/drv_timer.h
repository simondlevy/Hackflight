/*
   drv_timer.h : timer support for STM32F103CB

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/drv_timer.h

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

typedef void timerCCCallbackPtr(uint8_t port, uint16_t capture);

typedef struct {
    TIM_TypeDef *tim;
    GPIO_TypeDef *gpio;
    uint32_t pin;
    uint8_t channel;
    uint8_t irq;
    uint8_t outputEnable;
} timerHardware_t;

extern const timerHardware_t timerHardware[];

void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz);
void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz);
void timerNVICConfigure(uint8_t irq);

void configureTimerInputCaptureCompareChannel(TIM_TypeDef *tim, const uint8_t channel);
void configureTimerCaptureCompareInterrupt(const timerHardware_t *timerHardwarePtr, uint8_t reference, timerCCCallbackPtr *callback);
void configureTimerChannelCallback(TIM_TypeDef *tim, uint8_t channel, uint8_t reference, timerCCCallbackPtr *callback);
