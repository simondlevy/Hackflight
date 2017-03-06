/*
   Motor.cpp : Support for brushed and brushless motors

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/drv_pwm.c

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

extern "C" {

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "Motor.h"

#include "stm32f10x_conf.h"

#include "drv_gpio.h"
#include "drv_timer.h"

typedef struct {
    volatile uint16_t *ccr;
    volatile uint16_t *cr1;
    volatile uint16_t *cnt;
    uint16_t period;

    // for input only
    uint8_t channel;
    uint8_t state;
    uint16_t rise;
    uint16_t fall;
    uint16_t capture;
} pwmPortData_t;

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t idlePulseUsec)
{
    uint16_t tim_oc_preload;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = idlePulseUsec;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    tim_oc_preload = TIM_OCPreload_Enable;

    switch (channel) {
        case TIM_Channel_1:
            TIM_OC1Init(tim, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(tim, tim_oc_preload);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(tim, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(tim, tim_oc_preload);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(tim, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(tim, tim_oc_preload);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(tim, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(tim, tim_oc_preload);
            break;
    }
}


static void pwmGPIOConfig(GPIO_TypeDef *gpio, uint32_t pin, GPIO_Mode mode)
{
    gpio_config_t cfg;

    cfg.pin = pin;
    cfg.mode = mode;
    cfg.speed = Speed_2MHz;
    gpioInit(gpio, &cfg);
}

void BrushedMotor::setSpeed(uint16_t value)
{
    pwmPortData_t * _motor = (pwmPortData_t *)this->motor;
    *_motor->ccr = (value<1000) ? 0 : (value - 1000) * _motor->period / 1000;
}

void BrushlessMotor::setSpeed(uint16_t value)
{
    pwmPortData_t * _motor = (pwmPortData_t *)this->motor;
    *_motor->ccr = value;
}

void Motor::attach(uint8_t pin, uint32_t motorPwmRate, uint16_t idlePulseUsec)
{
    static pwmPortData_t pwmPorts[14];

    uint32_t mhz = (motorPwmRate > 500) ? 8 : 1;
    uint32_t hz = mhz * 1000000;
    uint16_t period = hz / motorPwmRate;

    // XXX currently support only four motors
    int8_t portFromPin[] = {-1, -1, -1, -1, -1, -1, 10, 11, 8, -1, -1, 9, 0};

    int8_t port = portFromPin[pin];

    if (port < 0)
        while (1)
            ;

    pwmPortData_t *p = &pwmPorts[port];
    configTimeBase(timerHardware[port].tim, period, mhz);
    pwmGPIOConfig(timerHardware[port].gpio, timerHardware[port].pin, Mode_AF_PP);

    pwmOCConfig(timerHardware[port].tim, timerHardware[port].channel, idlePulseUsec);

    // Needed only on TIM1
    if (timerHardware[port].outputEnable)
        TIM_CtrlPWMOutputs(timerHardware[port].tim, ENABLE);
    TIM_Cmd(timerHardware[port].tim, ENABLE);

    p->cr1 = &timerHardware[port].tim->CR1;
    p->cnt = &timerHardware[port].tim->CNT;

    switch (timerHardware[port].channel) {
        case TIM_Channel_1:
            p->ccr = &timerHardware[port].tim->CCR1;
            break;
        case TIM_Channel_2:
            p->ccr = &timerHardware[port].tim->CCR2;
            break;
        case TIM_Channel_3:
            p->ccr = &timerHardware[port].tim->CCR3;
            break;
        case TIM_Channel_4:
            p->ccr = &timerHardware[port].tim->CCR4;
            break;
    }
    p->period = period;

    this->motor = p;
}

} // extern "C"
