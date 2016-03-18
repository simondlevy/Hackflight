#pragma once

#include <stdint.h>
#include <stdbool.h>

bool     board_baroInit(void);
int32_t  board_baroReadPressure(void);
void     board_checkReboot(bool pendReboot);
void     board_delayMilliseconds(uint32_t msec);
uint16_t board_getI2cErrorCounter(void);
uint32_t board_getMicros();
void     board_i2cInit(void);
void     board_imuInit(uint8_t lpf, uint16_t * acc1G, float * gyroScale);
void     board_imuReadAccel(int16_t * data);
void     board_imuReadGyro(int16_t * data);
void     board_led0Off(void);
void     board_led0On(void);
void     board_led0Toggle(void);
void     board_led1Off(void);
void     board_led1On(void);
void     board_led1Toggle(void);
void     board_pwmInit(void);
uint16_t board_pwmRead(uint8_t chan, uint16_t oob_default);
void     board_reboot(void);
void     board_writeMotor(uint8_t index, uint16_t value);
bool     board_serialAvailable(void);
uint8_t  board_serialRead(void);
void     board_serialWrite(uint8_t c);

