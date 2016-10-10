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

// https://github.com/simondlevy/ArduinoRXInterrupt
#include <ArduinoRXInterrupt.h>

// https://github.com/bolderflight/MPU9250
#include <MPU9250.h>

#include "board.hpp"
#include "rc.hpp"

#define IMU_LOOPTIME_USEC       3500
#define CALIBRATING_GYRO_MSEC   3500

// an MPU9250 object with its I2C address 
// of 0x68 (ADDR to GRND) and on Teensy bus 0
// using pins 16 and 17 instead of 18 and 19
// and internal pullups instead of external.
MPU9250 imu(0x68, 0, I2C_PINS_16_17, I2C_PULLUP_INT);


// https://www.tindie.com/products/onehorse/dc-motor-controller-board-for-teensy-31-/
// Multiwii M1 = Controller M1 = Pin 23
// Multiwii M2 = Controller M3 = Pin 3
// Multiwii M3 = Controller M2 = Pin 4
// Multiwii M4 = Controller M4 = Pin 22
static const uint8_t MOTOR_PINS[4] = {23, 3, 4, 22};

// for old-school PWM receiver like FrSky VD5M
int RX_PINS[5] = {5, 6, 18, 19, 20};



void Board::debug(char c)
{
    Serial1.write(c);
}

void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
    // wake up device
    imu.begin(ACCEL_RANGE_8G,GYRO_RANGE_2000DPS);
 
    // Accel scale 8g (4096 LSB/g)
    acc1G = 4096;

    // 16.4 dps/lsb scalefactor for all Invensense devices
    gyroScale = (1.0f / 16.4f) * (M_PI / 180.0f);
}

void Board::init(uint32_t & looptimeMicroseconds, uint32_t & calibratingGyroMsec)
{
    // Stop motors
    for (int k=0; k<4; ++k) {
      analogWrite(MOTOR_PINS[k], 0);
    }
  
    // Set up LED
    pinMode(13, OUTPUT);

    // Set up PWM receiver
    initChannels(RX_PINS, 5);

    // Set up serial communication over USB
    Serial.begin(115200);

    // XXX these values should probably be #define'd generally for all physical (non-simulated) boards
    looptimeMicroseconds = IMU_LOOPTIME_USEC;
    calibratingGyroMsec  = CALIBRATING_GYRO_MSEC;
}


void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
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

void Board::ledSetState(uint8_t id, bool state)
{
    digitalWrite(13, state); // we only have one LED
}

uint16_t Board::readPWM(uint8_t chan)
{
    short values[5];
    updateChannels(values, 5);

    return (int16_t)values[chan];
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

void Board::writeMotor(uint8_t index, uint16_t pwmValue)
{ 
  uint8_t analogValue = map(pwmValue, CONFIG_PWM_MIN, CONFIG_PWM_MAX, 0, 255);
  
  analogWrite(MOTOR_PINS[index], analogValue);
}

// Non-essentials ----------------------------------------------------------------

void Board::reboot(void)
{
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


