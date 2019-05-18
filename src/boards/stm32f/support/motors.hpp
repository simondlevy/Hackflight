/*
   STM32F3 motor support

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

#pragma once

extern "C" {

    // From Cleanflight
#include "platform.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "target.h"

    static const uint16_t BRUSHED_PWM_RATE     = 32000;
    static const uint16_t BRUSHED_IDLE_PULSE   = 0; 

    static const uint16_t BRUSHLESS_PWM_RATE   = 0;
    static const uint16_t BRUSHLESS_IDLE_PULSE = 1000;

    static const float    MOTOR_MIN = 1000;
    static const float    MOTOR_MAX = 2000;

    static void motors_init(
            uint16_t pwm_rate, 
            uint16_t idle_pulse, 
            motorPwmProtocolTypes_e protocol, 
            bool unsynced,
            uint8_t m1, 
            uint8_t m2, 
            uint8_t m3, 
            uint8_t m4)
    {
        motorDevConfig_t dev;

        dev.motorPwmRate = pwm_rate;
        dev.motorPwmProtocol = protocol;
        dev.motorPwmInversion = false;
        dev.useUnsyncedPwm = unsynced;
        dev.useBurstDshot = false;

        dev.ioTags[0] = timerioTagGetByUsage(TIM_USE_MOTOR, m1);
        dev.ioTags[1] = timerioTagGetByUsage(TIM_USE_MOTOR, m2);
        dev.ioTags[2] = timerioTagGetByUsage(TIM_USE_MOTOR, m3);
        dev.ioTags[3] = timerioTagGetByUsage(TIM_USE_MOTOR, m4);

        motorDevInit(&dev, idle_pulse, 4);

        pwmEnableMotors();
    }

    void motor_write(uint8_t index, float value)
    {
        pwmWriteMotor(index, MOTOR_MIN + value*(MOTOR_MAX-MOTOR_MIN));
    }

    void brushed_motors_init(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4)
    {
        motors_init(BRUSHED_PWM_RATE, BRUSHED_IDLE_PULSE, PWM_TYPE_BRUSHED, true, m1, m2, m3, m4);
    }

    void brushless_motors_init(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4)
    {
        // XXX should probably support more than just Oneshot protocol
        motors_init(BRUSHLESS_PWM_RATE, BRUSHLESS_IDLE_PULSE, PWM_TYPE_ONESHOT125, false, m1, m2, m3, m4);

        // Send baseline pulse to initialize
        motor_write(0, 0);
        motor_write(1, 0);
        motor_write(2, 0);
        motor_write(3, 0);
    }

} // extern "C"
