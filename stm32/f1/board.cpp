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

#ifdef __arm__
extern "C" {
#endif

#define USE_CPPM                1
#define PWM_FILTER              0     // 0 or 1
#define FAST_PWM                0     // 0 or 1

#include <breezystm32.h>
#include <pwm.h>
#include <drivers/mpu.h>
#include <drivers/ms5611.h>

#include <math.h>

#include "board.hpp"
#include "motorpwm.hpp"

extern serialPort_t * Serial1;

void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
    acc1G = mpu6050_init(INV_FSR_8G, INV_FSR_2000DPS);
    gyroScale = MPU_GYRO_SCALE;
}

void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
    mpu_read_accel(accADC);
    mpu_read_gyro(gyroADC);
}

void Board::init(uint32_t & looptimeMicroseconds, uint32_t & calibratingGyroMsec)
{
    i2cInit(I2CDEV_2);
    pwmInit(USE_CPPM, PWM_FILTER, FAST_PWM, MOTOR_PWM_RATE, PWM_IDLE_PULSE);
    looptimeMicroseconds = Board::DEFAULT_IMU_LOOPTIME_USEC; 
    calibratingGyroMsec  = Board::DEFAULT_GYRO_CALIBRATION_MSEC;
}

void Board::delayMilliseconds(uint32_t msec)
{
    delay(msec);
}

uint32_t Board::getMicros()
{
    return micros();
}

void Board::ledSetState(uint8_t id, bool state)
{
    GPIO_TypeDef * gpio = id ? LED1_GPIO : LED0_GPIO;
    uint8_t pin = id ? LED1_PIN  : LED0_PIN;

    if (state) {
        digitalLo(gpio, pin);
    }
    else {
        digitalHi(gpio, pin);
    }
}

void Board::reboot(void)
{
    systemResetToBootloader();
}

uint8_t Board::serialAvailableBytes(void)
{
    return serialTotalRxBytesWaiting(Serial1);
}

uint8_t Board::serialReadByte(void)
{
    return serialRead(Serial1);
}

void Board::serialWriteByte(uint8_t c)
{
    serialWrite(Serial1, c);
    while (!isSerialTransmitBufferEmpty(Serial1));
}

void Board::serialDebugByte(uint8_t c)
{
    serialWrite(Serial1, c);
    while (!isSerialTransmitBufferEmpty(Serial1));
}

void Board::writeMotor(uint8_t index, float value)
{
    pwmWriteMotor(index, (uint16_t)(1000+value*1000));
}

void Board::showArmedStatus(bool armed)
{
    // XXX this would be a good place to sound a buzzer!

    (void)armed; 
}
 
void Board::showAuxStatus(uint8_t status)
{
    (void)status; 
}

void Board::extrasInit(class MSP * _msp)
{
    // Basic implementation doesn't need MSP
    (void)_msp;
}

void Board::extrasCheckSwitch(void)
{
}

uint8_t  Board::extrasGetTaskCount(void){
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



#ifdef __arm__
} // extern "C"
#endif
