/*
   board.cpp : implementation of board-specific routines

   This implemenation is for Teensy 3.1 / 3.2

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

#include <math.h>
#include <stdint.h>
#include <Arduino.h>

#include "board.hpp"

#define USE_CPPM                1
#define PWM_FILTER              0     // 0 or 1
#define FAST_PWM                0     // 0 or 1

#define IMU_LOOPTIME_USEC       3500
#define CALIBRATING_GYRO_MSEC   3500

// Parameters for brushed motors
#define MOTOR_PWM_RATE          32000
#define PWM_IDLE_PULSE          0
    
void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
    //mpu6050_init(false, &acc1G, &gyroScale, BOARD_VERSION);

    gyroScale *= 0.000004f;
}

void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
    //mpu6050_read_accel(accADC);
    //mpu6050_read_gyro(gyroADC);

    for (int k=0; k<3; ++k)
        gyroADC[k] /= 4;
}

void Board::init(uint32_t & looptimeMicroseconds, uint32_t & calibratingGyroMsec)
{
    //i2cInit(I2CDEV_2);
    //pwmInit(USE_CPPM, PWM_FILTER, FAST_PWM, MOTOR_PWM_RATE, PWM_IDLE_PULSE);

    looptimeMicroseconds = IMU_LOOPTIME_USEC;
    calibratingGyroMsec  = CALIBRATING_GYRO_MSEC;
}

bool Board::baroInit(void)
{
  return false;
}

void Board::baroUpdate(void)
{
}

int32_t Board::baroGetPressure(void)
{
    return 0;
}

void Board::checkReboot(bool pendReboot)
{
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
    //digitalHi(LED0_GPIO, LED0_PIN);
}

void Board::ledGreenOn(void)
{
    //digitalLo(LED0_GPIO, LED0_PIN);
}

void Board::ledGreenToggle(void)
{
    //digitalToggle(LED0_GPIO, LED0_PIN);
}

void Board::ledRedOff(void)
{
    //digitalHi(LED1_GPIO, LED1_PIN);
}

void Board::ledRedOn(void)
{
    //digitalLo(LED1_GPIO, LED1_PIN);
}

void Board::ledRedToggle(void)
{
    //digitalToggle(LED1_GPIO, LED1_PIN);
}

uint16_t Board::readPWM(uint8_t chan)
{
    return 0;//pwmRead(chan);
}

void Board::reboot(void)
{
}

uint8_t Board::serialAvailableBytes(void)
{
    return 0;//serialTotalBytesWaiting(Serial1);
}

uint8_t Board::serialReadByte(void)
{
    return 0;//serialRead(Serial1);
}

void Board::serialWriteByte(uint8_t c)
{
    //serialWrite(Serial1, c);
}

void Board::writeMotor(uint8_t index, uint16_t value)
{
    //pwmWriteMotor(index, value);
}

bool Board::sonarInit(uint8_t index) 
{
    index = index; // avoid compiler warning about unused variable
    return false;
}

void Board::sonarUpdate(uint8_t index)
{
    index = index; // avoid compiler warning about unused variable
}

uint16_t Board::sonarGetDistance(uint8_t index)
{
    index = index; // avoid compiler warning about unused variable
    return 0;
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
