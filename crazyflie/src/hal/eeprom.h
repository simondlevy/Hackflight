#pragma once

#include <stdint.h>

bool eepromInit();
bool eepromTest(void);
bool eepromTestConnection(void);
bool eepromReadBuffer(uint8_t* buffer, uint16_t readAddr, uint16_t len);
bool eepromWriteBuffer(const uint8_t* buffer, uint16_t writeAddr, uint16_t len);
bool eepromWritePage(uint8_t* buffer, uint16_t writeAddr);
