/*
   f3_motor.cpp : Support for brushed and brushless motors on STM32F3 boards

   Adapted from https://github.com/cleanflight/cleanflight/blob/master/src/main/drivers/pwm_output.c

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

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "motor.h"

#include <platform.h>

#include "gpio.h"
#include "timer.h"

typedef struct {
    volatile timCCR_t *ccr;
    TIM_TypeDef *tim;
    uint16_t period;
} pwmOutputPort_t;


static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = value;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    switch (channel) {
        case TIM_Channel_1:
            TIM_OC1Init(tim, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(tim, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(tim, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(tim, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);
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

void Servo::writeMicroseconds(uint16_t uS)
{
    pwmOutputPort_t * _servo = (pwmOutputPort_t *)this->motor;
    *_servo->ccr = uS;
}


void BrushedMotor::writeMicroseconds(uint16_t uS)
{
    pwmOutputPort_t * _motor = (pwmOutputPort_t *)this->motor;
    *_motor->ccr = (uS<1000) ? 0 : (uS - 1000) * _motor->period / 1000;
}

void BrushlessMotor::writeMicroseconds(uint16_t uS)
{
    pwmOutputPort_t * _motor = (pwmOutputPort_t *)this->motor;
    *_motor->ccr = uS;
}

void Motor::attach(uint8_t pin, uint32_t motorPwmRate, uint16_t idlePulseUsec)
{
    static pwmOutputPort_t pwmPorts[16];

    uint32_t mhz = (motorPwmRate > 500) ? 8 : 1;
    uint32_t hz = mhz * 1000000;
    uint16_t period = hz / motorPwmRate;

    // XXX currently support only four motors
    int8_t portFromPin[] = {3, -1, -1, -1, -1, -1, -1, -1, 2, -1, -1, -1, -1, -1, 1, 0};

    int8_t port = portFromPin[pin];

    if (port < 0)
        while (1)
            ;
    pwmOutputPort_t *p = &pwmPorts[port];

    configTimeBase(timerHardware[port].tim, period, mhz);
    pwmGPIOConfig(timerHardware[port].gpio, timerHardware[port].pin, Mode_AF_PP);

    pwmOCConfig(timerHardware[port].tim, timerHardware[port].channel, idlePulseUsec);
    if (timerHardware[port].outputEnable)
        TIM_CtrlPWMOutputs(timerHardware[port].tim, ENABLE);
    TIM_Cmd(timerHardware[port].tim, ENABLE);

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
    p->tim = timerHardware[port].tim;

    this->motor = p;

 }

} // extern "C"
