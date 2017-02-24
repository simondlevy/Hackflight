/*
   board.cpp : implementation of board-specific routines

   This implemenation is for Teensy 3.1 / 3.2 with MPU9250 IMU. 

   MPU9250 code adapted from https://github.com/kriswiner/MPU-9250

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
#include <stdarg.h>

#include <Arduino.h>

// https://github.com/simondlevy/SpektrumDSM
#include <SpektrumDSM.h>
static SpektrumDSM2048 rx;

// https://github.com/simondlevy/ArduinoRXInterrupt
//#include <ArduinoRXInterrupt.h>
//int RX_PINS[5] = {5, 6, 18, 19, 20};

// https://github.com/PaulStoffregen/PulsePosition
//#include <PulsePosition.h>

// https://github.com/bolderflight/MPU9250
// https://www.tindie.com/products/onehorse/mpu9250-teensy-3x-add-on-shields/ (we're using micro shield)
#include <MPU9250.h>

#include "board.hpp"
#include "rc.hpp"
//#include "config.hpp"

// an MPU9250 object with its I2C address 
// of 0x68 (ADDR to GRND) and on Teensy bus 0
// using pins 16 and 17 instead of 18 and 19
// and internal pullups instead of external.
MPU9250 imu(0x68, 0, I2C_PINS_16_17, I2C_PULLUP_INT);

// https://www.tindie.com/products/onehorse/dc-motor-controller-board-for-teensy-31-/

static const uint8_t MOTOR_PINS[4] = {20, 21, 22, 23};

void Board::dump(char * msg)
{
    for (char * c = msg; *c; c++)
        Serial.write(*c);
}



void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
    imu.begin(ACCEL_RANGE_8G,GYRO_RANGE_2000DPS);
 
    // Accel scale 8g (4096 LSB/g)
    acc1G = 4096;

    // 16.4 dps/lsb scalefactor for all Invensense devices
    gyroScale = 16.4f;
}

void Board::init(uint32_t & looptimeMicroseconds, uint32_t & calibratingGyroMsec)
{
    // Stop motors
    for (int k=0; k<4; ++k) {
      analogWrite(MOTOR_PINS[k], 0);
    }
  
    // Set up LED
    pinMode(13, OUTPUT);

    // Set ADO low to guarantee 0x68 address
    digitalWrite(24, LOW);

    // Start receiver
    rx.begin();

    // Set up serial communication over USB
    Serial.begin(115200);

    // Use default hardware loop times
    looptimeMicroseconds = Board::DEFAULT_IMU_LOOPTIME_USEC; 
    calibratingGyroMsec  = Board::DEFAULT_GYRO_CALIBRATION_MSEC;
}


void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
    // For ordering, negation see:
    // https://forum.pjrc.com/threads/37891-MPU-9250-Teensy-Library?p=118198&viewfull=1#post118198

    imu.getMotion6Counts(&accADC[1], &accADC[0], &accADC[2], &gyroADC[1], &gyroADC[0], &gyroADC[2]);

    accADC[2]  = -accADC[2];
    gyroADC[2] = -gyroADC[2];
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
    digitalWrite(13, LOW);
}

void Board::ledGreenOn(void)
{
    digitalWrite(13, HIGH);
}

void Board::ledRedOff(void)
{
    digitalWrite(13, LOW);
}

void Board::ledRedOn(void)
{
    digitalWrite(13, HIGH);
}

bool Board::rcUseSerial(void)
{ 
    rx.begin();
    return true;}

bool  Board::rcSerialReady(void)
{
    return rx.frameComplete();
}

uint16_t Board::rcReadSerial(uint8_t chan)
{  
    uint8_t chanmap[5] = {1, 2, 3, 0, 5};
    return rx.readRawRC(chanmap[chan]);
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
    (void)index;
    (void)value;
}

// Unused -------------------------------------------------------------------------


uint16_t Board::readPWM(uint8_t chan)
{
  (void)chan;
  return 0;
}

void Board::checkReboot(bool pendReboot)
{
    (void)pendReboot;
}



void Board::reboot(void)
{
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


