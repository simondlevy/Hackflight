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
#include <stdarg.h>

#include <Arduino.h>
#include <MPU6050.h>
#include <PulsePosition.h>

#include "board.hpp"

#define IMU_LOOPTIME_USEC       3500
#define CALIBRATING_GYRO_MSEC   3500

// Parameters for brushed motors
#define MOTOR_PWM_RATE          32000
#define PWM_IDLE_PULSE          0

#define PPM_INPUT_PIN           5

static MPU6050 accelgyro;
static bool ledState;
static PulsePositionInput ppmIn;

static void ledSet(uint8_t state)
{
    digitalWrite(13, state);
}

static void ledOff(void)
{
    ledSet(LOW);
}

static void ledOn(void)
{
    ledSet(HIGH);
}

static void ledToggle(void)
{
    ledState = !ledState;
    ledSet(ledState ? HIGH : LOW);
}

void Board::dump(char * msg)
{
    Serial.print(msg);
}

void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
    accelgyro.initialize();

    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);

    acc1G     = 4096;
    gyroScale = 4256./1e12;
}

void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
    accelgyro.getMotion6(&accADC[0], &accADC[1], &accADC[2], &gyroADC[0], &gyroADC[1], &gyroADC[2]);

    for (int k=0; k<3; ++k)
        gyroADC[k] /= 4;
}

void Board::init(uint32_t & looptimeMicroseconds, uint32_t & calibratingGyroMsec)
{
    pinMode(13, OUTPUT);
    ledState = false;

    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);

    // XXX pwmInit(USE_CPPM, PWM_FILTER, FAST_PWM, MOTOR_PWM_RATE, PWM_IDLE_PULSE);

    ppmIn.begin(PPM_INPUT_PIN);

    looptimeMicroseconds = IMU_LOOPTIME_USEC;
    calibratingGyroMsec  = CALIBRATING_GYRO_MSEC;
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
    ledOff();
}

void Board::ledGreenOn(void)
{
    ledOn();
}

void Board::ledGreenToggle(void)
{
    ledToggle();
}

void Board::ledRedOff(void)
{
    ledOff();
}

void Board::ledRedOn(void)
{
    ledOn();
}

void Board::ledRedToggle(void)
{
    ledToggle();
}

uint16_t Board::readPWM(uint8_t chan)
{
    return (uint16_t)ppmIn.read(chan+1);
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
    //pwmWriteMotor(index, value);
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


