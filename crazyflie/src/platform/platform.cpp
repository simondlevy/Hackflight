/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2018 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Generic platform functionality
 *
 */

#include <string.h>
#include <strings.h>

#include <free_rtos.h>
#include <semphr.h>
#include <task.h>

#include <crossplatform.h>

#include <arduino/analog.h>

#include <hal/i2cdev.h>

#include <console.h>
#include <led.h>
#include <radiolink.hpp>
#include <safety.hpp>

#include "platform.h"
#include "platformservice.hpp"

// Define to decrease the nRF51 Tx power to reduce interference
#ifndef PLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM
#define PLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM (-12)
#endif

#define PLATFORM_INFO_OTP_NR_OF_BLOCKS 16
#define PLATFORM_INFO_OTP_BLOCK_LEN 32
#if PLATFORM_DEVICE_TYPE_STRING_MAX_LEN < (PLATFORM_INFO_OTP_BLOCK_LEN + 1)
  #error
#endif


static const char * DEFAULT_PLATFORM_STRING = "0;CF20";

static const auto PLATFORM_TASK_STACK_DEPTH = configMINIMAL_STACK_SIZE;
StackType_t  platformTaskStackBuffer[PLATFORM_TASK_STACK_DEPTH]; 
StaticTask_t platformTaskTaskBuffer;

static PlatformService platformService;

static void platformSrvTask(void * obj)
{
    platformService.task((Safety *)obj);
}

void platformServiceInit(void * obj)
{
    platformService.init();

    xTaskCreateStatic(
            platformSrvTask,
            "PLATFORM-SRV", 
            PLATFORM_TASK_STACK_DEPTH,
            obj,
            0,
            platformTaskStackBuffer,
            &platformTaskTaskBuffer);
}

bool platformServiceTest(void)
{
    return platformService.test();
}

static const platformConfig_t* active_config = 0;

int platformInit(void) 
{
    int nrOfConfigs = 0;

    const platformConfig_t* configs = platformGetListOfConfigurations(&nrOfConfigs);

    int err = platformInitConfiguration(configs, nrOfConfigs);
    if (err != 0)
    {
        // This firmware is not compatible, abort init
        return 1;
    }

    platformInitCrazyflie();

    return 0;
}

int platformParseDeviceTypeString(const char* deviceTypeString, char* deviceType) {
    if (deviceTypeString[0] != '0' || deviceTypeString[1] != ';') {
        return 1;
    }

    const int start = 2;
    const int last = start + PLATFORM_DEVICE_TYPE_MAX_LEN - 1;
    int end = 0;
    for (end = start; end <= last; end++) {
        if (deviceTypeString[end] == '\0' || deviceTypeString[end] == ';') {
            break;
        }
    }

    if (end > last) {
        return 1;
    }

    int length = end - start;
    memcpy(deviceType, &deviceTypeString[start], length);
    deviceType[length] = '\0';
    return 0;
}

int platformInitConfiguration(const platformConfig_t* configs, const int nrOfConfigs) {
#ifndef DEVICE_TYPE_STRING_FORCE
    char deviceTypeString[PLATFORM_DEVICE_TYPE_STRING_MAX_LEN];
    char deviceType[PLATFORM_DEVICE_TYPE_MAX_LEN];

    platformGetDeviceTypeString(deviceTypeString);
    platformParseDeviceTypeString(deviceTypeString, deviceType);
#else
#define xstr(s) str(s)
#define str(s) #s

    char* deviceType = xstr(DEVICE_TYPE_STRING_FORCE);
#endif

    for (int i = 0; i < nrOfConfigs; i++) {
        const platformConfig_t* config = &configs[i];
        if (strcmp(config->deviceType, deviceType) == 0) {
            active_config = config;
            return 0;
        }
    }

    return 1;
}

const char* platformConfigGetDeviceType() {
    return active_config->deviceType;
}

const char* platformConfigGetDeviceTypeName() {
    return active_config->deviceTypeName;
}

SensorImplementation_t platformConfigGetSensorImplementation() {
    return active_config->sensorImplementation;
}

bool platformConfigPhysicalLayoutAntennasAreClose() {
    return active_config->physicalLayoutAntennasAreClose;
}

const MotorPerifDef** platformConfigGetMotorMapping() {
    return active_config->motorMap;
}

#ifndef UNIT_TEST_MODE
static char* getAddressOfOtpMemoryBlock(int blockNr) {
    return (char*)(0x1fff7800 + blockNr * 0x20);
}
#else
// This function is replaced by a mock in unit tests
char* getAddressOfOtpMemoryBlock(const int blockNr);
#endif

void platformGetDeviceTypeString(char* deviceTypeString) {

    char block[100] = {};

    for (int i = 0; i < PLATFORM_INFO_OTP_NR_OF_BLOCKS; i++) {
        char* candidateBlock = getAddressOfOtpMemoryBlock(i);
        if (candidateBlock[0] != 0) {
            strcpy(block, candidateBlock);
            break;
        }
    }

    if (!block[0] || ((unsigned char)block[0]) == 0xff) {
        strcpy(block, DEFAULT_PLATFORM_STRING);
    }

    strncpy(deviceTypeString, block, PLATFORM_INFO_OTP_BLOCK_LEN);

    deviceTypeString[PLATFORM_INFO_OTP_BLOCK_LEN] = '\0';
}
