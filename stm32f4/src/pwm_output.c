/*
This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <time.h>

#include "escdev.h"
#include "platform.h"
#include "io.h"
#include "pwm_output.h"
#include "timer.h"

uint16_t  PWM_RATE = 480;

FAST_DATA_ZERO_INIT pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value, uint8_t output)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        TIM_OCInitStructure.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
    } else {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    }
    TIM_OCInitStructure.TIM_Pulse = value;

    timerOCInit(tim, channel, &TIM_OCInitStructure);
    timerOCPreloadConfig(tim, channel, TIM_OCPreload_Enable);
}

void pwmOutConfig(timerChannel_t *channel, const timerHardware_t *timerHardware, uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion)
{
    configTimeBase(timerHardware->tim, period, hz);
    pwmOCConfig(timerHardware->tim,
        timerHardware->channel,
        value,
        inversion ? timerHardware->output ^ TIMER_OUTPUT_INVERTED : timerHardware->output
        );

    TIM_CtrlPWMOutputs(timerHardware->tim, ENABLE);
    TIM_Cmd(timerHardware->tim, ENABLE);

    channel->ccr = timerChCCR(timerHardware);

    channel->tim = timerHardware->tim;

    *channel->ccr = 0;
}

static FAST_DATA_ZERO_INIT escDevice_t motorPwmDevice;

static void pwmWriteUnused(uint8_t index, float value)
{
    UNUSED(index);
    UNUSED(value);
}

static void pwmWriteStandard(uint8_t index, float value)
{
    /* TODO: move value to be a number between 0-1 (i.e. percent throttle from mixer) */
    *motors[index].channel.ccr = lrintf((value * motors[index].pulseScale) + motors[index].pulseOffset);
}

void pwmShutdownPulsesForAllMotors(void)
{
    for (int index = 0; index < motorPwmDevice.count; index++) {
        // Set the compare register to 0, which stops the output pulsing if the timer overflows
        if (motors[index].channel.ccr) {
            *motors[index].channel.ccr = 0;
        }
    }
}

void pwmDisableMotors(void)
{
    pwmShutdownPulsesForAllMotors();
}

static escVTable_t motorPwmVTable;
bool pwmEnableMotors(void)
{
    /* check motors can be enabled */
    return (motorPwmVTable.write != &pwmWriteUnused);
}

bool pwmIsMotorEnabled(uint8_t index)
{
    return motors[index].enabled;
}

static void pwmCompleteOneshotMotorUpdate(void)
{
    for (int index = 0; index < motorPwmDevice.count; index++) {
        if (motors[index].forceOverflow) {
            timerForceOverflow(motors[index].channel.tim);
        }
        // Set the compare register to 0, which stops the output pulsing if the timer overflows before the main loop completes again.
        // This compare register will be set to the output value on the next main loop.
        *motors[index].channel.ccr = 0;
    }
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static escVTable_t motorPwmVTable = {
    .postInit = escPostInitNull,
    .enable = pwmEnableMotors,
    .disable = pwmDisableMotors,
    .isEnabled = pwmIsMotorEnabled,
    .shutdown = pwmShutdownPulsesForAllMotors,
    .convertFromExternal = pwmConvertFromExternal,
    .convertToExternal = pwmConvertToExternal,
};

escDevice_t *motorPwmDevInit(uint16_t idlePulse, uint8_t motorCount, bool useUnsyncedPwm)
{
    motorPwmDevice.vTable = motorPwmVTable;

    float sMin = 0;
    float sLen = 0;
    switch (ESC_PROTOCOL) {
    default:
    case ESC_ONESHOT125:
        sMin = 125e-6f;
        sLen = 125e-6f;
        break;
    case ESC_ONESHOT42:
        sMin = 42e-6f;
        sLen = 42e-6f;
        break;
    case ESC_MULTISHOT:
        sMin = 5e-6f;
        sLen = 20e-6f;
        break;
    case ESC_BRUSHED:
        sMin = 0;
        useUnsyncedPwm = true;
        idlePulse = 0;
        break;
    case ESC_STANDARD:
        sMin = 1e-3f;
        sLen = 1e-3f;
        useUnsyncedPwm = true;
        idlePulse = 0;
        break;
    }

    motorPwmDevice.vTable.write = pwmWriteStandard;
    motorPwmDevice.vTable.updateStart = escUpdateStartNull;
    motorPwmDevice.vTable.updateComplete = useUnsyncedPwm ? escUpdateCompleteNull : pwmCompleteOneshotMotorUpdate;

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        const ioTag_t tag = ESC_IO_TAGS[motorIndex];
        const timerHardware_t *timerHardware = timerAllocate(tag, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

        if (timerHardware == NULL) {
            /* not enough motors initialised for the mixer or a break in the motors */
            motorPwmDevice.vTable.write = &pwmWriteUnused;
            motorPwmDevice.vTable.updateComplete = escUpdateCompleteNull;
            /* TODO: block arming and add reason system cannot arm */
            return NULL;
        }

        motors[motorIndex].io = IOGetByTag(tag);
        IOInit(motors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

        IOConfigGPIOAF(motors[motorIndex].io, IOCFG_AF_PP, timerHardware->alternateFunction);

        /* standard PWM outputs */
        // margin of safety is 4 periods when unsynced
        const unsigned pwmRateHz = useUnsyncedPwm ? PWM_RATE : ceilf(1 / ((sMin + sLen) * 4));

        const uint32_t clock = timerClock(timerHardware->tim);
        /* used to find the desired timer frequency for max resolution */
        const unsigned prescaler = ((clock / pwmRateHz) + 0xffff) / 0x10000; /* rounding up */
        const uint32_t hz = clock / prescaler;
        const unsigned period = useUnsyncedPwm ? hz / pwmRateHz : 0xffff;

        /*
            if brushed then it is the entire length of the period.
            TODO: this can be moved back to periodMin and periodLen
            once mixer outputs a 0..1 float value.
        */
        motors[motorIndex].pulseScale = ((ESC_PROTOCOL == ESC_BRUSHED) ? period : (sLen * hz)) / 1000.0f;
        motors[motorIndex].pulseOffset = (sMin * hz) - (motors[motorIndex].pulseScale * 1000);

        pwmOutConfig(&motors[motorIndex].channel, timerHardware, hz, period, idlePulse, false);

        bool timerAlreadyUsed = false;
        for (int i = 0; i < motorIndex; i++) {
            if (motors[i].channel.tim == motors[motorIndex].channel.tim) {
                timerAlreadyUsed = true;
                break;
            }
        }
        motors[motorIndex].forceOverflow = !timerAlreadyUsed;
        motors[motorIndex].enabled = true;
    }

    return &motorPwmDevice;
}

pwmOutputPort_t *pwmGetMotors(void)
{
    return motors;
}
