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

#include "platform.h"
#include "flash_impl.h"
#include "io.h"
#include "io_types.h"

// 20 MHz max SPI frequency
#define FLASH_MAX_SPI_CLK_HZ 20000000
// 5 MHz max SPI init frequency
#define FLASH_MAX_SPI_INIT_CLK 5000000

static ioTag_t CS_TAG = 44;

static extDevice_t devInstance;
static extDevice_t *dev;

static flashDevice_t flashDevice;
static flashPartitionTable_t flashPartitionTable;
static int flashPartitions = 0;

#define FLASH_INSTRUCTION_RDID 0x9F

void flashPreInit(void)
{
}

bool flashIsReady(void)
{
    return flashDevice.vTable->isReady(&flashDevice);
}

void flashEraseSector(uint32_t address)
{
    flashDevice.callback = NULL;
    flashDevice.vTable->eraseSector(&flashDevice, address);
}

void flashEraseCompletely(void)
{
    flashDevice.callback = NULL;
    flashDevice.vTable->eraseCompletely(&flashDevice);
}

/* The callback, if provided, will receive the totoal number of bytes transfered
 * by each call to flashPageProgramContinue() once the transfer completes.
 */
void flashPageProgramBegin(uint32_t address, void (*callback)(uint32_t length))
{
    flashDevice.vTable->pageProgramBegin(&flashDevice, address, callback);
}

uint32_t flashPageProgramContinue(const uint8_t **buffers, uint32_t *bufferSizes, uint32_t bufferCount)
{
    uint32_t maxBytesToWrite = flashDevice.geometry.pageSize - (flashDevice.currentWriteAddress % flashDevice.geometry.pageSize);

    if (bufferCount == 0) {
        return 0;
    }

    if (bufferSizes[0] >= maxBytesToWrite) {
        bufferSizes[0] = maxBytesToWrite;
        bufferCount = 1;
    } else {
        maxBytesToWrite -= bufferSizes[0];
        if ((bufferCount == 2) && (bufferSizes[1] > maxBytesToWrite)) {
            bufferSizes[1] = maxBytesToWrite;
        }
    }

    return flashDevice.vTable->pageProgramContinue(&flashDevice, buffers, bufferSizes, bufferCount);
}

void flashPageProgramFinish(void)
{
    flashDevice.vTable->pageProgramFinish(&flashDevice);
}

void flashPageProgram(uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uint32_t length))
{
    flashDevice.vTable->pageProgram(&flashDevice, address, data, length, callback);
}

int flashReadBytes(uint32_t address, uint8_t *buffer, uint32_t length)
{
    flashDevice.callback = NULL;
    return flashDevice.vTable->readBytes(&flashDevice, address, buffer, length);
}

void flashFlush(void)
{
    if (flashDevice.vTable->flush) {
        flashDevice.vTable->flush(&flashDevice);
    }
}

static const flashGeometry_t noFlashGeometry = {
    .totalSize = 0,
};

const flashGeometry_t *flashGetGeometry(void)
{
    if (flashDevice.vTable && flashDevice.vTable->getGeometry) {
        return flashDevice.vTable->getGeometry(&flashDevice);
    }

    return &noFlashGeometry;
}

/*
 * Flash partitioning
 *
 * Partition table is not currently stored on the flash, in-memory only.
 *
 * Partitions are required so that Badblock management (inc spare blocks), FlashFS (Blackbox Logging), Configuration and Firmware can be kept separate and tracked.
 *
 * XXX FIXME
 * XXX Note that Flash FS must start at sector 0.
 * XXX There is existing blackbox/flash FS code the relies on this!!!
 * XXX This restriction can and will be fixed by creating a set of flash operation functions that take partition as an additional parameter.
 */

static void flashConfigurePartitions(void)
{

    const flashGeometry_t *flashGeometry = flashGetGeometry();
    if (flashGeometry->totalSize == 0) {
        return;
    }

    flashSector_t startSector = 0;
    flashSector_t endSector = flashGeometry->sectors - 1; // 0 based index

    const flashPartition_t *badBlockPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_BADBLOCK_MANAGEMENT);
    if (badBlockPartition) {
        endSector = badBlockPartition->startSector - 1;
    }

    flashPartitionSet(FLASH_PARTITION_TYPE_FLASHFS, startSector, endSector);
}

flashPartition_t *flashPartitionFindByType(uint8_t type)
{
    for (int index = 0; index < FLASH_MAX_PARTITIONS; index++) {
        flashPartition_t *candidate = &flashPartitionTable.partitions[index];
        if (candidate->type == type) {
            return candidate;
        }
    }

    return NULL;
}

const flashPartition_t *flashPartitionFindByIndex(uint8_t index)
{
    if (index >= flashPartitions) {
        return NULL;
    }

    return &flashPartitionTable.partitions[index];
}

void flashPartitionSet(uint8_t type, uint32_t startSector, uint32_t endSector)
{
    flashPartition_t *entry = flashPartitionFindByType(type);

    if (!entry) {
        if (flashPartitions == FLASH_MAX_PARTITIONS - 1) {
            return;
        }
        entry = &flashPartitionTable.partitions[flashPartitions++];
    }

    entry->type = type;
    entry->startSector = startSector;
    entry->endSector = endSector;
}

void flashInit(void)
{
    memset(&flashPartitionTable, 0x00, sizeof(flashPartitionTable));
    flashPartitions = 0;

    // Read chip identification and send it to device detect
    dev = &devInstance;
    dev->busType_u.spi.csnPin = IOGetByTag(CS_TAG);
    IOIsFreeOrPreinit(dev->busType_u.spi.csnPin);

    flashConfigurePartitions();
}

int flashPartitionCount(void)
{
    return flashPartitions;
}
