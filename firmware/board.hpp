#pragma once

extern "C" {

class Board {

    public:

        void init(void);

        void delayMilliseconds(uint32_t msec);
        void led0Off(void);
        void led0On(void);
        void led0Toggle(void);
        void led1Off(void);
        void led1On(void);
        void led1Toggle(void);

}; // class Board

void     board_checkReboot(bool pendReboot);
uint32_t board_getMicros();
void     board_imuInit(uint16_t *acc1G, float * gyroScale);
void     board_imuRead(int16_t accADC[3], int16_t gyroADC[3]);
uint16_t board_pwmRead(uint8_t chan);
void     board_reboot(void);
uint8_t  board_serialAvailable(void);
uint8_t  board_serialRead(void);
void     board_serialWrite(uint8_t c);
void     board_writeMotor(uint8_t index, uint16_t value);

} // extern "C"
