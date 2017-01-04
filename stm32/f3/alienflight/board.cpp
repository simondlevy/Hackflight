/*
   board.cpp : implementation of board-specific routines

   This implemenation is for AlienFlight F3 brushed boad

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

#include <breezystm32.h>
#include <board.hpp>
#include <drivers/mpu.h>
#include <drivers/spektrum.h>

#include "motorpwm.hpp"

extern serialPort_t * Serial1;

void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
    mpu6500_init(INV_FSR_8G, INV_FSR_2000DPS);

    acc1G = 4096;
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
    pwmInitMotors(PWM_IDLE_PULSE, 4);
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
    if (id)
        if (state)
            LED1_ON;
        else
            LED1_OFF;
    else
        if (state)
            LED0_ON;
        else
            LED0_OFF;
}

void Board::reboot(void)
{
    systemResetToBootloader();
}

uint8_t Board::serialAvailableBytes(void)
{
    return serialRxBytesWaiting(Serial1);
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

void Board::writeMotor(uint8_t index, uint16_t value, float fvalue)
{
    (void)fvalue;
    pwmWriteMotor(index, value);
}

uint16_t Board::rcReadSerial(uint8_t chan)
{
    static uint8_t chanmap[5] = {1, 2, 3, 0, 5};
    return chan > 5 ? 0 : spektrumReadRawRC(chanmap[chan]);
}

bool Board::rcUseSerial(void)
{
    spektrumInit(USART2, SERIALRX_SPEKTRUM2048);
    return true;
}

bool Board::rcSerialReady(void)
{
    return spektrumFrameComplete();
}

// Unused =========================================================

bool Board::sonarInit(uint8_t index) 
{
    (void)index;
    return false;
}

void Board::sonarUpdate(uint8_t index)
{
    (void)index; 
}

void Board::showAuxStatus(uint8_t status)
{
    (void)status; 
}

uint16_t Board::sonarGetDistance(uint8_t index)
{
    (void)index; 
    return 0;
}

uint16_t Board::rcReadPWM(uint8_t chan)
{
    (void)chan; // avoid compiler warning about unused variable
    return 0;
}

void Board::showArmedStatus(bool armed)
{
    // XXX this would be a good place to sound a buzzer!

    (void)armed; 
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
#ifdef __arm__
} // extern "C"
#endif
