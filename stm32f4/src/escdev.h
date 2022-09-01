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

    void     (*disable)(void);
    bool     (*enable)(void);
    bool     (*isEnabled)(uint8_t index);
    void     (*shutdown)(void);
    void     (*updateComplete)(void);
    bool     (*updateStart)(void);
    void     (*write)(uint8_t index, float value);
    void     (*writeInt)(uint8_t index, uint16_t value);

} escVTable_t;

typedef struct {
    uint8_t     count;
    bool        enabled;
    uint32_t    enableTimeMs;
    bool        initialized;
    escVTable_t vTable;
} escDevice_t;

void escDevWriteNull(uint8_t index, float value);
void escPostInitNull();
void escUpdateCompleteNull(void);
bool escUpdateStartNull(void);

bool        escCheckProtocolEnabled(bool *protocolIsDshot);
void        escEnable(void * escDevice);
float       escGetDigitalIdOffset(void);
uint32_t    escGetEnableTimeMs(void * escDevice);
escVTable_t escGetVTable(void * escDevice);
bool        escIsEnabled(void * escDevice);
