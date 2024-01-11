/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2020 Bitcraze AB
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
 */

#include <assert.h>
#include <console.h>

#include "onewire.h"

#ifndef ASSERT
#define ASSERT(x) 
#endif

static bool didInit;
static xSemaphoreHandle waitForReply;
static xSemaphoreHandle lockCmdBuf;
static StaticSemaphore_t lockCmdBufBuffer;
static OwCommand owCmdBuf;
static bool owDataIsValid;

static bool owSyslinkTransfer(uint8_t type, uint8_t length)
{
  syslinkPacket_t slp;

  ASSERT(length <= SYSLINK_MTU);

  slp.type = type;
  slp.length = length;
  memcpy(slp.data, &owCmdBuf, length);

  syslinkSendPacket(&slp);

  // Wait for reply
  if (xSemaphoreTake(waitForReply, M2T(5000)) == pdTRUE) {

    // We have now got a reply and *owCmd has been filled with data
    if (owDataIsValid) {
      owDataIsValid = false;
      return true;
    }
  }
  else {
    consolePrintf("ONEWIRE: Cmd 0x%X timeout.\n", slp.type);
  }

  return false;
}

static bool owGetInfo(uint8_t selectMem, OwSerialNum *serialNum)
{
  bool status = false;

  xSemaphoreTake(lockCmdBuf, portMAX_DELAY);
  owCmdBuf.nmem = selectMem;

  if (owSyslinkTransfer(SYSLINK_OW_GETINFO, 1)) {
    memcpy(serialNum, owCmdBuf.info.memId, sizeof(OwSerialNum));
    if (owCmdBuf.nmem != 0xFF) {
      status = true;
    }
  }
  else {
    status = false;
  }

  xSemaphoreGive(lockCmdBuf);

  return status;
}

static bool handleMemGetSerialNr(const uint8_t selectedMem, uint8_t* serialNr) {
  return owGetInfo(selectedMem, (OwSerialNum*)serialNr);
}

static bool handleMemRead(const uint8_t selectedMem, const uint32_t memAddr,
        const uint8_t readLen, uint8_t* startOfData) 
{
  bool result = false;

  if (memAddr + readLen <= OW_MAX_SIZE) {
    if (owRead(selectedMem, memAddr, readLen, startOfData)) {
      result = true;
    }
  }

  return result;
}

static bool owWrite(uint8_t selectMem, uint16_t address, uint8_t length, 
        const uint8_t *data)
{
  bool status = true;
  uint16_t currAddr = address;
  uint16_t endAddr = address + length;
  uint8_t bytesWritten = 0;

  ASSERT(length <= OW_MAX_SIZE);

  xSemaphoreTake(lockCmdBuf, portMAX_DELAY);
  owCmdBuf.nmem = selectMem;

  while (currAddr < endAddr)
  {
    if (endAddr - currAddr < OW_MAX_WRITE_SIZE) {
      owCmdBuf.write.address = currAddr;
      owCmdBuf.write.length = endAddr - currAddr;
      memcpy(owCmdBuf.write.data, data + bytesWritten, owCmdBuf.write.length);
      if (owSyslinkTransfer(SYSLINK_OW_WRITE, 5 + owCmdBuf.write.length)) {
        currAddr += endAddr - currAddr;
        bytesWritten += endAddr - currAddr;
      }
      else {
        status = false;
        break;
      }
    }
    else { // Size is bigger then OW_MAX_WRITE_SIZE
      owCmdBuf.write.address = currAddr;
      owCmdBuf.write.length = OW_MAX_WRITE_SIZE;
      memcpy(owCmdBuf.write.data, data + bytesWritten, owCmdBuf.write.length);
      if (owSyslinkTransfer(SYSLINK_OW_WRITE, 5 + owCmdBuf.write.length)) {
        currAddr += OW_MAX_WRITE_SIZE;
        bytesWritten += OW_MAX_WRITE_SIZE;
      }
      else {
        status = false;
        break;
      }
    }
  }

  xSemaphoreGive(lockCmdBuf);

  return status;
}

static bool handleMemWrite(const uint8_t selectedMem, const uint32_t memAddr,
        const uint8_t writeLen, const uint8_t* startOfData) 
{
  bool result = false;

  if (memAddr + writeLen <= OW_MAX_SIZE) {
    if (owWrite(selectedMem, memAddr, writeLen, startOfData)) {
      result = true;
    }
  }

  return result;
}

////////////////////////////////////////////////////////////////////////


void owInit()
{
  if (didInit) {
    return;
  }

  syslinkInit();
  vSemaphoreCreateBinary(waitForReply);
  lockCmdBuf = xSemaphoreCreateMutexStatic(&lockCmdBufBuffer);

  // Put reply semaphore in right state.
  xSemaphoreTake(waitForReply, portMAX_DELAY);

  static MemoryOwHandlerDef_t memHandlerDef = {
      .nrOfMems = 0,
      .size = OW_MAX_SIZE,
      .getSerialNr = handleMemGetSerialNr,
      .read = handleMemRead,
      .write = handleMemWrite,
  };


  owScan(&memHandlerDef.nrOfMems);

  memoryRegisterOwHandler(&memHandlerDef);

  didInit = true;
}

void owSyslinkReceive(syslinkPacket_t *slp)
{
    switch (slp->type) {
        case SYSLINK_OW_SCAN:
        case SYSLINK_OW_GETINFO:
        case SYSLINK_OW_READ:
        case SYSLINK_OW_WRITE:
            memcpy(&owCmdBuf, slp->data, sizeof(OwCommand));
            owDataIsValid = true;
            break;
        default:
            // Unknown reply
            owDataIsValid = false;
            break;
    }
    xSemaphoreGive(waitForReply);
}

bool owScan(uint8_t *nMem)
{
    bool status = false;

    xSemaphoreTake(lockCmdBuf, portMAX_DELAY);

    if (owSyslinkTransfer(SYSLINK_OW_SCAN, 0)) {
        *nMem = owCmdBuf.nmem;
        status = true;
    }
    else {
        status = false;
    }

    xSemaphoreGive(lockCmdBuf);

    return status;
}

bool owRead(uint8_t selectMem, uint16_t address, uint8_t length, uint8_t *data)
{
    bool status = true;
    uint16_t currAddr = address;
    uint16_t endAddr = address + length;
    uint8_t bytesRead = 0;

    ASSERT(length <= OW_MAX_SIZE);

    xSemaphoreTake(lockCmdBuf, portMAX_DELAY);
    owCmdBuf.nmem = selectMem;

    while (currAddr < endAddr) {
        if (endAddr - currAddr < OW_READ_SIZE) {
            owCmdBuf.read.address = currAddr;
            if (owSyslinkTransfer(SYSLINK_OW_READ, 3 + endAddr - currAddr)) {
                memcpy(data + bytesRead, owCmdBuf.read.data, endAddr - currAddr);
                currAddr += endAddr - currAddr;
                bytesRead += endAddr - currAddr;
            }
            else {
                status = false;
                break;
            }
        }
        else { // Size is bigger then OW_READ_SIZE
            owCmdBuf.read.address = currAddr;
            if (owSyslinkTransfer(SYSLINK_OW_READ, 3 + OW_READ_SIZE)) {
                memcpy(data + bytesRead, owCmdBuf.read.data, OW_READ_SIZE);
                currAddr += OW_READ_SIZE;
                bytesRead += OW_READ_SIZE;
            }
            else {
                status = false;
                break;
            }
        }
    }

    xSemaphoreGive(lockCmdBuf);

    return status;
}
