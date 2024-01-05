/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2020 Bitcraze AB
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
 *
 * storage.c: Key/Buffer persistent storage
 *
 */

#include <string.h>

#include <free_rtos.h>
#include <semphr.h>

#include "storage.h"

#include <dictionary.hpp>

#include <console.h>

// Memory organization

// Low level memory access

// ToDo: Shall we handle partition elsewhere?
#define PARTITION_START (1024)
#define PARTITION_LENGTH (7*1024)

// Shared with params
bool storageStats;
bool reformatValue;

static SemaphoreHandle_t storageMutex;

static Dictionary dictionary;

static size_t readEeprom(size_t address, void* data, size_t length)
{
    bool eepromReadBuffer(uint8_t* buffer, uint16_t readAddr, uint16_t len);

    return length == 0 ?
        0 : 
        eepromReadBuffer((uint8_t *)data, PARTITION_START + address, length) ?
        length :
        0;
}

static size_t writeEeprom(size_t address, const void* data, size_t length)
{
    bool eepromWriteBuffer(const uint8_t* buffer, uint16_t writeAddr, uint16_t len);

    return length == 0 ?
        0 : 
        eepromWriteBuffer((const uint8_t *)data, PARTITION_START + address, length) ?
        length :
        0;
}

static bool didInit = false;

void storageInit()
{
    storageMutex = xSemaphoreCreateMutex();

    dictionary.init(readEeprom, writeEeprom, PARTITION_LENGTH);

    didInit = true;
}

bool storageTest()
{
    xSemaphoreTake(storageMutex, portMAX_DELAY);

    bool pass = dictionary.check();

    xSemaphoreGive(storageMutex);

    consolePrintf("STORAGE: Storage check %s.\n", pass?"[OK]":"[FAIL]");

    if (!pass) {
        pass = storageReformat();
    }

    return pass;
}

bool storageStore(const char* key, const void* buffer, size_t length)
{
    if (!didInit) {
        return false;
    }

    xSemaphoreTake(storageMutex, portMAX_DELAY);

    bool result = dictionary.store(key, buffer, length);

    xSemaphoreGive(storageMutex);

    return result;
}


bool storageForeach(const char *prefix, storageFunc_t func)
{
    if (!didInit) {
        return 0;
    }

    xSemaphoreTake(storageMutex, portMAX_DELAY);

    bool success = dictionary.foreach(prefix, func);

    xSemaphoreGive(storageMutex);

    return success;
}

size_t storageFetch(const char *key, void* buffer, size_t length)
{
    if (!didInit) {
        return 0;
    }

    xSemaphoreTake(storageMutex, portMAX_DELAY);

    size_t result = dictionary.fetch(key, buffer, length);

    xSemaphoreGive(storageMutex);

    return result;
}

bool storageDelete(const char* key)
{
    if (!didInit) {
        return false;
    }

    xSemaphoreTake(storageMutex, portMAX_DELAY);

    bool result = dictionary.remove(key);

    xSemaphoreGive(storageMutex);

    return result;
}

bool storageReformat() {
    consolePrintf("STORAGE: Reformatting storage ...\n");

    xSemaphoreTake(storageMutex, portMAX_DELAY);

    dictionary.format();
    bool pass = dictionary.check();

    xSemaphoreGive(storageMutex);

    consolePrintf("STORAGE: Storage check %s.\n", pass?"[OK]":"[FAIL]");

    if (pass == false) {
        consolePrintf("STORAGE: Error: Cannot format storage!\n");
    }

    return pass;
}

void storagePrintStats()
{
    Dictionary::stats_t stats;

    xSemaphoreTake(storageMutex, portMAX_DELAY);

    dictionary.getStats(&stats);

    xSemaphoreGive(storageMutex);


    consolePrintf("STORAGE: Used storage: %d item stored, %d Bytes/%d Bytes (%d%%)\n", stats.totalItems, stats.itemSize, stats.totalSize, (stats.itemSize*100)/stats.totalSize);
    consolePrintf("STORAGE: Fragmentation: %d%%\n", stats.fragmentation);
    consolePrintf("STORAGE: Efficiency: Data: %d Bytes (%d%%), Keys: %d Bytes (%d%%), Metadata: %d Bytes (%d%%)\n",
            stats.dataSize, (stats.dataSize*100)/stats.totalSize,
            stats.keySize, (stats.keySize*100)/stats.totalSize,
            stats.metadataSize, (stats.metadataSize*100)/stats.totalSize);
}



