#pragma once


void     board_checkReboot(bool pendReboot);
void     board_delayMilliseconds(uint32_t msec);
uint16_t board_getI2cErrorCounter(void);
uint32_t board_getMicros();
void     board_imuInit(uint16_t *acc1G, float * gyroScale);
void     board_imuRead(int16_t accADC[3], int16_t gyroADC[3]);
void     board_init(void);
void     board_led0Off(void);
void     board_led0On(void);
void     board_led0Toggle(void);
void     board_led1Off(void);
void     board_led1On(void);
void     board_led1Toggle(void);
uint16_t board_pwmRead(uint8_t chan);
void     board_reboot(void);
uint8_t  board_serialAvailable(void);
uint8_t  board_serialRead(void);
void     board_serialWrite(uint8_t c);
void     board_writeMotor(uint8_t index, uint16_t value);
