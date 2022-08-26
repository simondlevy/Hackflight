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

#include <stdint.h>
#include <stdbool.h>

#include <system.h>

#include "platform.h"

void systemInit(void);

typedef enum {
    FAILURE_DEVELOPER = 0,
    FAILURE_MISSING_ACC,
    FAILURE_ACC_INIT,
    FAILURE_ACC_INCOMPATIBLE,
    FAILURE_INVALID_EEPROM_CONTENTS,
    FAILURE_CONFIG_STORE_FAILURE,
    FAILURE_GYRO_INIT_FAILED,
    FAILURE_FLASH_READ_FAILED,
    FAILURE_FLASH_WRITE_FAILED,
    FAILURE_FLASH_INIT_FAILED, // RESERVED
    FAILURE_EXTERNAL_FLASH_READ_FAILED,  // RESERVED
    FAILURE_EXTERNAL_FLASH_WRITE_FAILED, // RESERVED
    FAILURE_EXTERNAL_FLASH_INIT_FAILED,
    FAILURE_SDCARD_READ_FAILED,
    FAILURE_SDCARD_WRITE_FAILED,
    FAILURE_SDCARD_INITIALISATION_FAILED,
    FAILURE_SDCARD_REQUIRED,
} failureMode_e;

#define WARNING_FLASH_DURATION_MS 50
#define WARNING_FLASH_COUNT 5
#define WARNING_PAUSE_DURATION_MS 500
#define WARNING_CODE_DURATION_LONG_MS 250
#define WARNING_CODE_DURATION_SHORT_MS 50

typedef enum {
    BOOTLOADER_REQUEST_ROM,
    BOOTLOADER_REQUEST_FLASH,
} bootloaderRequestType_e;

#if defined(__cplusplus)
extern "C" {
#endif

// failure
void systemIndicateFailure(failureMode_e mode, int repeatCount);
void systemFailureMode(failureMode_e mode);

// bootloader/IAP
void systemReset(void);
void systemResetToBootloader(bootloaderRequestType_e requestType);
void systemCycleCounterInit(void);
void systemInitialiseMemorySections(void);
void systemInitUnusedPins(void);

void systemEnableGPIOPowerUsageAndNoiseReductions(void);
// current crystal frequency - 8 or 12MHz

extern uint32_t hse_value;
extern uint32_t cachedRccCsrValue;

typedef void extiCallbackHandlerFunc(void);

void delay(uint32_t ms);
uint32_t micros(void);
uint32_t millis(void);

#if defined(__cplusplus)
}
#endif
