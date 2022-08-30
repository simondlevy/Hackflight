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

#include <esc.h>
#include <time.h>

static const uint8_t ESC_IO_TAGS[8] = {32, 33, 19, 18, 56, 24, 0, 0};

typedef enum {
    ESC_STANDARD = 0,
    ESC_ONESHOT125,
    ESC_ONESHOT42,
    ESC_MULTISHOT,
    ESC_BRUSHED,
    ESC_DSHOT150,
    ESC_DSHOT300,
    ESC_DSHOT600,
    ESC_PROSHOT1000,
    ESC_DISABLED,
    ESC_MAX
} escProtocol_t;

static const escProtocol_t ESC_PROTOCOL = ESC_DSHOT600;

typedef struct {

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

} escVTable_t;

typedef struct {
    escVTable_t vTable;
    uint8_t     count;
    bool        initialized;
    bool        enabled;
    uint32_t    enableTimeMs;
} escDevice_t;

void     motorPostInitNull();
void     escDevWriteNull(uint8_t index, float value);
bool     motorUpdateStartNull(void);
void     motorUpdateCompleteNull(void);
void     motorPostInit(void * escDevice);


escVTable_t   motorGetVTable(void * escDevice);
bool          motorCheckProtocolEnabled(bool *protocolIsDshot);
void          motorEnable(void * escDevice);
bool          motorIsEnabled(void * escDevice);
uint32_t      motorGetEnableTimeMs(void * escDevice);

float motorGetDigitalIdOffset(void);
