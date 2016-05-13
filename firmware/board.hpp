#pragma once

extern "C" {

class Board {

    private:

        // add stuff as needed here

    public:

        // your implementation should support these methods

        void     init(void);

        void     checkReboot(bool pendReboot);
        void     delayMilliseconds(uint32_t msec);
        void     led0Off(void);
        void     led0On(void);
        void     led0Toggle(void);
        void     led1Off(void);
        void     led1On(void);
        void     led1Toggle(void);
        void     reboot(void);
        uint8_t  serialAvailableBytes(void);
        uint8_t  serialReadByte(void);
        void     serialWriteByte(uint8_t c);
        void     writeMotor(uint8_t index, uint16_t value);

}; // class Board

uint32_t board_getMicros();
void     board_imuInit(uint16_t *acc1G, float * gyroScale);
void     board_imuRead(int16_t accADC[3], int16_t gyroADC[3]);
uint16_t board_pwmRead(uint8_t chan);

} // extern "C"
