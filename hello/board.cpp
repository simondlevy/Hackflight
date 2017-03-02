/*
   Board implementation for "HelloHackflight" demo

   Copyright (C) Simon D. Levy 2017

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

#include <hackflight.hpp>
#include <board.hpp>

#include <stdio.h>
#include <time.h>

uint32_t Board::getMicros()
{
    return clock(); 
}

void Board::delayMilliseconds(uint32_t msec)
{
    uint32_t start = clock()/1000;

    while ((clock()/1000 - start) < msec)
        ;
}

void Board::dump(char * msg)
{
    printf("%s\n", msg);
}

void Board::init(uint32_t & looptimeMicroseconds, uint32_t & calibratingGyroMsec)
{
    calibratingGyroMsec = 100;  // long enough to see but not to annoy
    looptimeMicroseconds = 10000;

    printf("Calibrating...\n");
}

void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
    // Mimic MPU6050
    acc1G = 4096;
    gyroScale = 16.4f;
}

void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
    for (int k=0; k<3; ++k) {
        accADC[k]  = 0;
        gyroADC[k] = 0;
    }
}

uint16_t Board::readPWM(uint8_t chan)
{

    uint16_t value = (chan == 4) ? 988 : 1500;

    if (chan < 5) {
        printf("chan%d=%d%c", chan+1, value, chan==4?'\n':'\t');
    }

    return value;
}

void Board::writeMotor(uint8_t index, uint16_t value)
{
    printf("motor %d=%d %c", index+1, value, index==3?'\n':'\t');
}

// Unused ======================================================================

void     Board::checkReboot(bool pendReboot) { (void)pendReboot; }
void     Board::extrasCheckSwitch(void) { } 
uint8_t  Board::extrasGetTaskCount(void){ return 0; } 
bool     Board::extrasHandleMSP(uint8_t command) { return true; }
void     Board::extrasInit(class MSP * _msp) { (void)_msp; }
void     Board::extrasPerformTask(uint8_t taskIndex) { (void)taskIndex; }
void     Board::ledGreenOff(void) { } 
void     Board::ledGreenOn(void) { } 
void     Board::ledRedOff(void) { } 
void     Board::ledRedOn(void) { } 
uint16_t Board::rcReadSerial(uint8_t chan) { (void)chan; return 0; }
bool     Board::rcSerialReady(void) { return false; }
bool     Board::rcUseSerial(void) { return false; }
void     Board::reboot(void) { }
uint8_t  Board::serialAvailableBytes(void) { return 0; }
uint8_t  Board::serialReadByte(void) { return 0; }
void     Board::serialWriteByte(uint8_t c) { (void)c; }
void     Board::showArmedStatus(bool armed) { (void)armed; }
void     Board::showAuxStatus(uint8_t status) { (void)status; }
