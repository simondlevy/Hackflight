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

#include <time.h>
#include <maths.h>
#include <motor.h>

#include "platform.h"
#include "dshot.h" // for DSHOT_ constants in mixerInitEscEndpoints
#include "pwm_output.h" // for PWM_TYPE_* and others
#include "dshot_bitbang.h"
#include "dshot_dpwm.h"
#include "motordev.h"
#include "systemdev.h"

#define CONVERT_PARAMETER_TO_PERCENT(param) (0.01f * param)

static bool motorProtocolEnabled = false;
static bool motorProtocolDshot = false;

float motorGetDigitalIdOffset(void)
{
    uint16_t digitalIdleOffsetValue = 450;
    return CONVERT_PARAMETER_TO_PERCENT(digitalIdleOffsetValue * 0.01f);
}

bool motorIsProtocolDshot(void)
{
    return motorProtocolDshot;
}

bool motorCheckProtocolEnabled(bool *isProtocolDshot)
{
    bool enabled = false;
    bool isDshot = false;

    switch (MOTOR_PWM_PROTOCOL) {
    case PWM_TYPE_STANDARD:
    case PWM_TYPE_ONESHOT125:
    case PWM_TYPE_ONESHOT42:
    case PWM_TYPE_MULTISHOT:
    case PWM_TYPE_BRUSHED:
        enabled = true;

        break;

    case PWM_TYPE_DSHOT150:
    case PWM_TYPE_DSHOT300:
    case PWM_TYPE_DSHOT600:
    case PWM_TYPE_PROSHOT1000:
        enabled = true;
        isDshot = true;

        break;
    default:

        break;
    }

    if (isProtocolDshot) {
        *isProtocolDshot = isDshot;
    }

    return enabled;
}

float motorConvertFromExternal(void * motorDevice_void, uint16_t externalValue)
{
    motorDevice_t * motorDevice = (motorDevice_t *)motorDevice_void;

    return motorDevice->vTable.convertExternalToMotor(externalValue);
}

void motorEnable(void * motorDevice_void)
{
    motorDevice_t * motorDevice = (motorDevice_t *)motorDevice_void;

    if (motorDevice->initialized && motorDevice->vTable.enable()) {
        motorDevice->enabled = true;
        motorDevice->motorEnableTimeMs = millis();
    }
}

uint32_t motorGetEnableTimeMs(void * motorDevice_void)
{
    motorDevice_t * motorDevice = (motorDevice_t *)motorDevice_void;

    return motorDevice->motorEnableTimeMs;
}

motorVTable_t motorGetVTable(void * motorDevice_void)
{
    motorDevice_t * motorDevice = (motorDevice_t *)motorDevice_void;

    return motorDevice->vTable;
}

void * motorInitDshot(uint8_t motorCount) {

    motorProtocolEnabled = motorCheckProtocolEnabled(&motorProtocolDshot);

    memset(motors, 0, sizeof(motors));

    static FAST_DATA_ZERO_INIT motorDevice_t *motorDevice;

    motorDevice = dshotBitbangDevInit(motorCount);

    motorDevice->count = motorCount;
    motorDevice->initialized = true;
    motorDevice->motorEnableTimeMs = 0;
    motorDevice->enabled = false;

    return (void *)motorDevice;
}


bool motorIsEnabled(void * motorDevice_void)
{
    motorDevice_t * motorDevice = (motorDevice_t *)motorDevice_void;

    return motorDevice->enabled;
}

void motorPostInit(void * motorDevice_void)
{
    motorDevice_t * motorDevice = (motorDevice_t *)motorDevice_void;

    motorDevice->vTable.postInit();
}

void motorPostInitNull(void)
{
}

void motorUpdateCompleteNull(void)
{
}

bool motorUpdateStartNull(void)
{
    return true;
}

void motorWrite(void * motorDevice_void, float *values)
{
    motorDevice_t * motorDevice = (motorDevice_t *)motorDevice_void;

    if (motorDevice->enabled) {
        if (!motorDevice->vTable.updateStart()) {
            return;
        }
        for (int i = 0; i < motorDevice->count; i++) {
            motorDevice->vTable.write(i, values[i]);
        }
        motorDevice->vTable.updateComplete();
    }
}

void motorWriteNull(uint8_t index, float value)
{
    UNUSED(index);
    UNUSED(value);
}