#include <Arduino.h>
#include <board.h>


void board_delayMilliseconds(uint32_t msec)
{
    delay(msec);
}

uint32_t board_getMicros()
{
    return micros();
}

void board_i2cInit(void)
{
}

void board_imuInit(uint16_t * acc1G, float * gyroScale)
{
}

void board_imuReadAccel(int16_t * data)
{
}

void board_imuReadGyro(int16_t * data)
{
}

void board_ledOff(void)
{
}

void board_ledOn(void)
{
}

void board_ledToggle(void)
{
}

void board_pwmInit(void)
{
}

uint16_t board_pwmRead(uint8_t chan)
{
    return 0;
}

uint8_t board_serialAvailable(void)
{
    return 0;
}

uint8_t board_serialRead(void)
{
    return 0;
}

void board_serialWrite(uint8_t c)
{
}

void board_writeMotor(uint8_t index, uint16_t value)
{
}





void board_checkReboot(bool pendReboot)
{
}

uint16_t board_getI2cErrorCounter(void)
{
    return 0;
}

void board_reboot(void)
{
}

bool board_sonarInit(void)
{
    return false;
}

int32_t board_sonarReadDistance(void)
{
    return 0;
}

bool board_baroInit(void)
{
    return false;
}

int32_t board_baroReadPressure(void)
{
    return 0;
}


