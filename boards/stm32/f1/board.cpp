/*
   board.cpp : implementation of board-specific routines

   This implemenation is for STM32F103 boards (Naze32, Flip32, etc.)

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

extern "C" {

//#define NEW

#include <Arduino.h>
#include <drv_mpu6050.h>

#include <breezystm32.h>
#include <math.h>

#include "board.hpp"
#include "motorpwm.hpp"

#define BOARD_VERSION           5
#define USE_CPPM                1
#define PWM_FILTER              0     // 0 or 1
#define FAST_PWM                0     // 0 or 1

#define IMU_LOOPTIME_USEC       3500
#define CALIBRATING_GYRO_MSEC   3500

#ifdef NEW
#include <MPU6050.h>
MPU6050 imu;
#endif

void Board::dump(char * msg)
{
    for (char * c = msg; *c; c++)
        Serial.write(*c);
}

void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
#ifdef NEW
    imu.begin(AFS_8G, GFS_2000DPS);
#else
    mpu6050_init(AFS_8G, GFS_2000DPS);
#endif

    acc1G = 4096;
    gyroScale = (1.0f / 16.4f) * (M_PI / 180.0f);
    gyroScale *= 0.000004f;
}

void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
#ifdef NEW
    imu.getMotion6Counts(&accADC[0], &accADC[1], &accADC[2], &gyroADC[0], &gyroADC[1], &gyroADC[2]);
#else
    mpu6050_getMotion6Counts(&accADC[0], &accADC[1], &accADC[2], &gyroADC[0], &gyroADC[1], &gyroADC[2]);
#endif

    for (int k=0; k<3; ++k)
        gyroADC[k] /= 4;
}

void Board::init(uint32_t & looptimeMicroseconds, uint32_t & calibratingGyroMsec)
{
    // Init LEDs
    pinMode(8, OUTPUT);
    pinMode(16, OUTPUT);

    Serial.begin(115200);

    //Wire.begin();
    i2cInit(I2CDEV_2);

    pwmInit(USE_CPPM, PWM_FILTER, FAST_PWM, MOTOR_PWM_RATE, PWM_IDLE_PULSE);

    looptimeMicroseconds = IMU_LOOPTIME_USEC;
    calibratingGyroMsec  = CALIBRATING_GYRO_MSEC;
}

void Board::checkReboot(bool pendReboot)
{
    if (pendReboot)
        systemReset(false); // noreturn
}

void Board::delayMilliseconds(uint32_t msec)
{
    delay(msec);
}

uint32_t Board::getMicros()
{
    return micros();
}

void Board::ledGreenOff(void)
{
    digitalWrite(8, LOW);
}

void Board::ledGreenOn(void)
{
    digitalWrite(8, HIGH);
}

void Board::ledRedOff(void)
{
    digitalWrite(16, LOW);
}

void Board::ledRedOn(void)
{
    digitalWrite(16, HIGH);
}

bool Board::rcSerialReady(void)
{
    return false;
}

bool Board::rcUseSerial(void)
{
    return false;
}

uint16_t Board::rcReadSerial(uint8_t chan)
{
    (void)chan;
    return 0;
}

uint16_t Board::readPWM(uint8_t chan)
{
    return pwmRead(chan);
}

void Board::reboot(void)
{
    systemReset(true);      // reboot to bootloader
}

uint8_t Board::serialAvailableBytes(void)
{
    return Serial.available();
}

uint8_t Board::serialReadByte(void)
{
    return Serial.read();
}

void Board::serialWriteByte(uint8_t c)
{
    Serial.write(c);
}

void Board::writeMotor(uint8_t index, uint16_t value)
{
    pwmWriteMotor(index, value);
}

void Board::showArmedStatus(bool armed)
{
    // XXX this would be a good place to sound a buzzer!

    armed = armed; // avoid compiler warning about unused variable
}
 
void Board::showAuxStatus(uint8_t status)
{
    status = status; // avoid compiler warning about unused variable
}
 
void Board::extrasInit(class MSP * _msp) 
{
    (void)_msp;
}

void Board::extrasCheckSwitch(void)
{
}

uint8_t Board::extrasGetTaskCount(void)
{
    return 0;
}

bool Board::extrasHandleMSP(uint8_t command)
{
    (void)command;
    return true;
}

void Board::extrasPerformTask(uint8_t taskIndex)
{
    (void)taskIndex;
}

} // extern "C"
