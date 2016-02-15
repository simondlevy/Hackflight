/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#pragma once

#define MAX_SERVOS  8
#define MAX_INPUTS  8
#define PULSE_1MS   (1000)      // 1ms pulse width
#define PULSE_MIN   (750)       // minimum PWM pulse width which is considered valid
#define PULSE_MAX   (2250)      // maximum PWM pulse width which is considered valid

// This indexes into the read-only hardware definition structure in drv_pwm.c, as well as into pwmPorts[] structure with dynamic data.
enum {
    PWM1 = 0,
    PWM2,
    PWM3,
    PWM4,
    PWM5,
    PWM6,
    PWM7,
    PWM8,
    PWM9,
    PWM10,
    PWM11,
    PWM12,
    PWM13,
    PWM14,
    MAX_PORTS
};

void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity);

void pwmInit(uint16_t config_failsafeThreshold, uint8_t config_pwmFilter);

void pwmWriteMotor(uint8_t index, uint16_t value);

uint16_t pwmRead(uint8_t channel);
