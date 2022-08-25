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

#pragma once

#include <motor.h>
#include <time.h>

static const uint8_t  MOTOR_IO_TAGS[8] = {32, 33, 19, 18, 56, 24, 0, 0};

static const uint8_t MOTOR_PWM_PROTOCOL = 7;

typedef enum {
    PWM_TYPE_STANDARD = 0,
    PWM_TYPE_ONESHOT125,
    PWM_TYPE_ONESHOT42,
    PWM_TYPE_MULTISHOT,
    PWM_TYPE_BRUSHED,
    PWM_TYPE_DSHOT150,
    PWM_TYPE_DSHOT300,
    PWM_TYPE_DSHOT600,
    PWM_TYPE_PROSHOT1000,
    PWM_TYPE_DISABLED,
    PWM_TYPE_MAX
} g_motorPwmProtocolTypes_e;


typedef struct motorVTable_s {
    // Common
    void (*postInit)(void);
    float (*convertExternalToMotor)(uint16_t externalValue);
    uint16_t (*convertMotorToExternal)(float motorValue);
    bool (*enable)(void);
    void (*disable)(void);
    bool (*isMotorEnabled)(uint8_t index);
    bool (*updateStart)(void);
    void (*write)(uint8_t index, float value);
    void (*writeInt)(uint8_t index, uint16_t value);
    void (*updateComplete)(void);
    void (*shutdown)(void);

    // Digital commands

} motorVTable_t;

typedef struct motorDevice_s {
    motorVTable_t vTable;
    uint8_t       count;
    bool          initialized;
    bool          enabled;
    uint32_t      motorEnableTimeMs;
} motorDevice_t;

void     motorPostInitNull();
void     motorDevWriteNull(uint8_t index, float value);
bool     motorUpdateStartNull(void);
void     motorUpdateCompleteNull(void);
void     motorPostInit(void * motorDevice);

struct motorDevConfig_s; 

motorVTable_t motorGetVTable(void * motorDevice);
bool          motorCheckProtocolEnabled(bool *protocolIsDshot);
void          motorEnable(void * motorDevice);
bool          motorIsEnabled(void * motorDevice);
uint32_t      motorGetEnableTimeMs(void * motorDevice);

struct motorDevConfig_s;
typedef struct motorDevConfig_s motorDevConfig_t;

float motorGetDigitalIdOffset(void);
