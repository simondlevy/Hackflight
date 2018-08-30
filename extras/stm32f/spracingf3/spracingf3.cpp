/*
   spracingf3.cpp : Board implementation for Spracing F3 

   Copyright (C) 2018 Simon D. Levy 

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

#include "spracingf3.h"


static const uint16_t BRUSHLESS_PWM_RATE   = 480;
static const uint16_t BRUSHLESS_IDLE_PULSE = 1000; 

static const float    MOTOR_MIN = 1000;
static const float    MOTOR_MAX = 2000;

// Here we put code that interacts with Cleanflight
extern "C" {

    // Cleanflight includes
#include "platform.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/time.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/bus_i2c.h"
#include "pg/bus_i2c.h"
#include "io/serial.h"
#include "target.h"
#include "stm32f30x.h"

    // Hackflight include
#include "../i2c.h"

    static serialPort_t * _serial0;

    SPRacingF3::SPRacingF3(void)
    {
        initMotors();
        initUsb();
        initImu();

        RealBoard::init();
    }

    void SPRacingF3::initImu(void)
    {
        i2c_init(I2CDEV_1);

        delaySeconds(0.1);

        _imu = new MPU6050(AFS_8G, GFS_2000DPS);

        switch (_imu->begin()) {

        case MPU_ERROR_IMU_ID:
                error("Bad device ID");
                break;
            case MPU_ERROR_SELFTEST:
                error("Failed self-test");
                break;
            default:
                break;
        }
    }

    void SPRacingF3::initUsb(void)
    {
        uartPinConfigure(serialPinConfig());

        _serial0 = uartOpen(UARTDEV_1,  NULL, NULL,  115200, MODE_RXTX, SERIAL_NOT_INVERTED);
    }

    void SPRacingF3::initMotors(void)
    {

        motorDevConfig_t dev;

        dev.motorPwmRate = BRUSHLESS_PWM_RATE;
        dev.motorPwmProtocol = PWM_TYPE_STANDARD;
        dev.motorPwmInversion = false;
        dev.useUnsyncedPwm = true;
        dev.useBurstDshot = false;

        dev.ioTags[0] = timerioTagGetByUsage(TIM_USE_MOTOR, 0);
        dev.ioTags[1] = timerioTagGetByUsage(TIM_USE_MOTOR, 1);
        dev.ioTags[2] = timerioTagGetByUsage(TIM_USE_MOTOR, 2);
        dev.ioTags[3] = timerioTagGetByUsage(TIM_USE_MOTOR, 3);

        motorDevInit(&dev, BRUSHLESS_IDLE_PULSE, 4);

        pwmEnableMotors();

        writeMotor(0, 0);
        writeMotor(1, 0);
        writeMotor(2, 0);
        writeMotor(3, 0);
    }

    void SPRacingF3::writeMotor(uint8_t index, float value)
    {
        pwmWriteMotor(index, MOTOR_MIN + value*(MOTOR_MAX-MOTOR_MIN));
    }

    void SPRacingF3::delaySeconds(float sec)
    {
        delay((uint16_t)(sec*1000));
    }

    void SPRacingF3::setLed(bool isOn)
    {
        ledSet(0, isOn);
    }

    uint32_t SPRacingF3::getMicroseconds(void)
    {
        return micros();
    }

    void SPRacingF3::reboot(void)
    {
        systemResetToBootloader();
    }

    uint8_t SPRacingF3::serialAvailableBytes(void)
    {
        return serialRxBytesWaiting(_serial0);
    }

    uint8_t SPRacingF3::serialReadByte(void)
    {
        return serialRead(_serial0);
    }

    void SPRacingF3::serialWriteByte(uint8_t c)
    {
        serialWrite(_serial0, c);
    }

    bool SPRacingF3::imuRead(void)
    {
        if (_imu->checkNewData()) {  

            // Note reversed X/Y order because of IMU rotation
            _imu->readAccelerometer(_ay, _ax, _az);
            _imu->readGyrometer(_gy, _gx, _gz);

            // Negate for same reason
            _ax = -_ax;
            _gx = -_gx;

            return true;

        }  

        return false;
    }

    void hf::Board::outbuf(char * buf)
    {
        for (char *p=buf; *p; p++)
            serialWrite(_serial0, *p);
    }

} // extern "C"
