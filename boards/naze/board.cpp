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

#include <Arduino.h>
#include <Motor.h>
#include <SpektrumDSM.h>
#include <MPU6050.h>

#include <math.h>

#include "board.hpp"

#define IMU_LOOPTIME_USEC       3500
#define CALIBRATING_GYRO_MSEC   3500

MPU6050 imu;

SpektrumDSM2048 rx;

BrushlessMotor motors[4];

void Board::dump(char * msg)
{
    for (char * c = msg; *c; c++)
        Serial.write(*c);
}

void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
    imu.begin(AFS_8G, GFS_2000DPS);

    acc1G = 4096;
    gyroScale = (1.0f / 16.4f) * (M_PI / 180.0f);
    gyroScale *= 0.000004f;
}

void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
    imu.getMotion6Counts(&accADC[0], &accADC[1], &accADC[2], &gyroADC[0], &gyroADC[1], &gyroADC[2]);

    for (int k=0; k<3; ++k)
        gyroADC[k] /= 4;
}


void Board::init(uint32_t & looptimeMicroseconds, uint32_t & calibratingGyroMsec)
{
    // Init LEDs
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);

    Serial.begin(115200);

    Wire.begin();

    motors[0].attach(8);
    motors[1].attach(11);
    motors[2].attach(6);
    motors[3].attach(7);

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
    digitalWrite(3, LOW);
}

void Board::ledGreenOn(void)
{
    digitalWrite(3, HIGH);
}

void Board::ledRedOff(void)
{
    digitalWrite(4, LOW);
}

void Board::ledRedOn(void)
{
    digitalWrite(4, HIGH);
}

bool Board::rcSerialReady(void)
{
    return rx.frameComplete();
}

bool Board::rcUseSerial(void)
{
    rx.begin();
    return true;
}

uint16_t Board::rcReadSerial(uint8_t chan)
{
    uint8_t chanmap[5] = {1, 2, 3, 0, 5};
    return rx.readRawRC(chanmap[chan]);
}

uint16_t Board::readPWM(uint8_t chan)
{
    (void)chan;
    return 0;
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
    motors[index].setSpeed(value);
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
