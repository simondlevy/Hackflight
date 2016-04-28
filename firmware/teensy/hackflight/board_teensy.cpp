#include <Arduino.h>
#include <PulsePosition.h>

#include "board.h"

PulsePositionInput ppm;

void board_delayMilliseconds(uint32_t msec)
{
    delay(msec);
}

uint32_t board_getMicros()
{
    return micros();
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

void board_init(void)
{
    Serial.begin(115200);

    pinMode(13, OUTPUT);  // LED

    ppm.begin(23);
}

void board_ledOff(void)
{
    digitalWriteFast(13, LOW);
}

void board_ledOn(void)
{
    digitalWriteFast(13, HIGH);
}

uint16_t board_pwmRead(uint8_t chan)
{
    return ppm.read(chan+1);
}

uint8_t board_serialAvailable(void)
{
    return Serial.available();
}

uint8_t board_serialRead(void)
{
    return Serial.read();
}

void board_serialWrite(uint8_t c)
{
    Serial.write(c);
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


