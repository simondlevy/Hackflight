/*
   board.cpp : implementation of board-specific routines

   This implemenation is for the Erle Robotics PXFMini

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

#include <stdint.h>

#include "board.hpp"
#include "motorpwm.hpp"

// Essentials ---------------------------------------------------------------------------

void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
}

void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
}

void Board::init(uint32_t & looptimeMicroseconds, uint32_t & calibratingGyroMsec)
{
    looptimeMicroseconds = Board::DEFAULT_IMU_LOOPTIME_USEC; 
    calibratingGyroMsec  = Board::DEFAULT_GYRO_CALIBRATION_MSEC;
}

void Board::debug(char c)
{
}

void Board::checkReboot(bool pendReboot)
{
}

void Board::delayMilliseconds(uint32_t msec)
{
}

uint32_t Board::getMicros()
{
    return 0;
}

void Board::ledSetState(uint8_t id, bool state)
{
}

bool Board::rcUseSerial(void)
{
    return false;
}

uint16_t Board::rcReadPWM(uint8_t chan)
{
    return 0;
}

uint8_t Board::serialAvailableBytes(void)
{
    return 0;
}

uint8_t Board::serialReadByte(void)
{
    return 0;
}

void Board::serialWriteByte(uint8_t c)
{
}

void Board::writeMotor(uint8_t index, uint16_t value)
{
}

// unused --------------------------------------------------------------------------

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

void Board::reboot(void)
{
}

bool Board::rcSerialReady(void)
{
    return false;
}

uint16_t Board::rcReadSerial(uint8_t chan)
{
    (void)chan;
    return 0;
}

bool Board::sonarInit(uint8_t index) 
{
    (void)index; 
    return false;
}

void Board::sonarUpdate(uint8_t index)
{
    (void)index; 
}

uint16_t Board::sonarGetDistance(uint8_t index)
{
    (void)index; 
    return 0;
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
