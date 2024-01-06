#include <string.h>

#include <free_rtos.h>
#include <task.h>

#include <console.h>
#include <mem.hpp>

#include <hal/i2cdev.h>
#include <hal/eeprom.h>

#define EEPROM_I2C_ADDR     0x50
#define EEPROM_SIZE         0x1FFF

#ifdef EEPROM_RUN_WRITE_READ_TEST
static bool eepromTestWriteRead(void);
#endif

static uint32_t handleMemGetSize(void) { return EEPROM_SIZE; }
static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);
static const MemoryHandlerDef_t memDef = {
  .type = MEM_TYPE_EEPROM,
  .getSize = handleMemGetSize,
  .read = handleMemRead,
  .write = handleMemWrite,
};

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool didInit;

bool eepromInit(void)
{
  if (didInit) {
    return true;
  }

  memoryRegisterHandler(&memDef);

  I2Cx = I2C1_DEV;
  devAddr = EEPROM_I2C_ADDR;

  didInit = true;

  return true;
}

bool eepromTest(void)
{
  bool status;

  status = eepromTestConnection();
  if (status)
  {
    consolePrintf("EEPROM: I2C connection [OK].\n");
  }
  else
  {
    consolePrintf("EEPROM: I2C connection [FAIL].\n");
  }

#ifdef EEPROM_RUN_WRITE_READ_TEST
  status = eepromTestWriteRead();
#endif

  return status;
}

#ifdef EEPROM_RUN_WRITE_READ_TEST
static bool eepromTestWriteRead(void)
{
  bool status = true;
  int i;
  const uint8_t testData[20] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
  uint8_t readData[20];

  for (i = 0; i < sizeof(testData) && status; i++)
  {
    // Write one byte with increasing addresses.
    eepromWriteBuffer(&testData[i], i, 1);
    // Read it all back and check
    eepromReadBuffer(readData, 0, i+1);
    status = (memcmp(testData, readData, i+1) == 0);
  }

  // Write it all.
  eepromWriteBuffer(testData, 0, sizeof(testData));
  for (i = 0; i < sizeof(testData) && status; i++)
  {
    // Read one byte with increasing addresses and check
    eepromReadBuffer(&readData[i], i, 1);
    status = (memcmp(testData, readData, i+1) == 0);
  }

  return status;
}
#endif

bool eepromTestConnection(void)
{
  uint8_t tmp;
  bool status;

  if (!didInit)
    return false;

  status = i2cdevReadReg16(I2Cx, devAddr, 0, 1, &tmp);

  return status;
}

bool eepromReadBuffer(uint8_t* buffer, uint16_t readAddr, uint16_t len)
{
  bool status;

  if ((uint32_t)readAddr + len > EEPROM_SIZE)
  {
     return false;
  }

  status = i2cdevReadReg16(I2Cx, devAddr, readAddr, len, buffer);

  return status;
}

bool eepromWriteBuffer(const uint8_t* buffer, uint16_t writeAddr, uint16_t len)
{
  bool status;

  unsigned char pageBuffer[32];
  int bufferIndex = 0;
  int leftToWrite = len;
  uint16_t currentAddress = writeAddr;
  uint16_t pageAddress = writeAddr;

  if ((uint32_t)writeAddr + len > EEPROM_SIZE)
  {
     return false;
  }

  while (leftToWrite > 0) {
    int pageIndex = 0;
    pageAddress = currentAddress;
    do {
      pageBuffer[pageIndex++] = buffer[bufferIndex++];
      leftToWrite -= 1;
      currentAddress += 1;
    } while ((leftToWrite > 0) && (currentAddress % 32 != 0));

    // Writing page
    for (int retry = 0; retry < 10; retry++) {
      status = i2cdevWriteReg16(I2Cx, devAddr, pageAddress, pageIndex, pageBuffer);
      if (status) {
        break;
      }
      vTaskDelay(M2T(6));
    }
    if (!status) {
      return false;
    }

    // Waiting for page to be written
    for (int retry = 0; retry < 30; retry++) {
      uint8_t dummy;
      status = i2cdevWrite(I2Cx, devAddr, 1, &dummy);

      if (status) {
        break;
      }

      vTaskDelay(M2T(1));
    }
    if (!status) {
      return false;
    }
  }

  return status;
}

bool eepromWritePage(uint8_t* buffer, uint16_t writeAddr)
{
 //TODO: implement
  return false;
}

static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  bool result = false;

  if (memAddr + readLen <= EEPROM_SIZE) {
    if (eepromReadBuffer(buffer, memAddr, readLen)) {
      result = true;
    }
  }

  return result;
}

static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {
  bool result = false;

  if (memAddr + writeLen <= EEPROM_SIZE) {
    if (eepromWriteBuffer(buffer, memAddr, writeLen)) {
      result = true;
    }
  }

  return result;
}
