#pragma once

#include <stdint.h>
#include <stdbool.h>

void     board_checkReboot(bool pendReboot);
uint16_t board_getI2cErrorCounter(void);
void     board_imuInit(uint8_t lpf, uint16_t * acc1G, float * gyroScale);
void     board_imuReadAccel(int16_t * data);
void     board_imuReadGyro(int16_t * data);
uint32_t board_getMicros();
void     board_reboot(void);
void     board_writeMotor(uint8_t index, uint16_t value);
bool     board_serialAvailable(void);
uint8_t  board_serialRead(void);
void     board_serialWrite(uint8_t c);

