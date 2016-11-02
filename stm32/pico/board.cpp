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

#include <breezystm32.h>
#include <math.h>

#include "board.hpp"
#include "motorpwm.hpp"

#define BOARD_VERSION           5
#define USE_CPPM                1
#define PWM_FILTER              0     // 0 or 1
#define FAST_PWM                0     // 0 or 1

extern serialPort_t * Serial1;

void Board::imuInit(uint16_t & acc1G, float & gyroScale)
{
    mpu6050_init(false, &acc1G, &gyroScale, BOARD_VERSION);
}

void Board::imuRead(int16_t accADC[3], int16_t gyroADC[3])
{
    mpu6050_read_accel(accADC);
    mpu6050_read_gyro(gyroADC);
}

void Board::init(uint32_t & looptimeMicroseconds, uint32_t & calibratingGyroMsec)
{
    i2cInit(I2CDEV_2);
    pwmInit(USE_CPPM, PWM_FILTER, FAST_PWM, MOTOR_PWM_RATE, PWM_IDLE_PULSE);

    spektrumInit(USART2, SERIALRX_SPEKTRUM1024);

    looptimeMicroseconds = Board::DEFAULT_IMU_LOOPTIME_USEC; 
    calibratingGyroMsec  = Board::DEFAULT_GYRO_CALIBRATION_MSEC;
}

void Board::debug(char c)
{
    serialWrite(Serial1, c);
}

void Board::checkReboot(bool pendReboot)
{
    if (pendReboot)
        systemResetToBootloader(); // noreturn
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

bool Board::rcUseSerial(void)
{
    return true;
}

bool Board::rcSerialReady(void)
{
    return spektrumFrameComplete();
}

uint16_t Board::rcReadSerial(uint8_t chan)
{
    static uint8_t chanmap[5] = {1, 2, 3, 0, 4};
    return chan > 4 ? 0 : spektrumReadRawRC(chanmap[chan]);
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
}

void Board::writeMotor(uint8_t index, uint16_t value)
{
    pwmWriteMotor(index, value);
}

// unused ---------------------------------------------------------------------

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


uint16_t Board::rcReadPWM(uint8_t chan)
{
    (void)chan; // avoid compiler warning about unused variable
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

#ifdef __arm__
} // extern "C"
#endif
