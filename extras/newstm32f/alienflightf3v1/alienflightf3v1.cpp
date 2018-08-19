/*
   alienflightf3v1.cpp : Board implementation for Alienflight F3 V1

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

#include "alienflightf3v1.h"


static const uint16_t BRUSHED_PWM_RATE     = 16000;
static const uint16_t BRUSHED_IDLE_PULSE   = 0; 

static const float    MOTOR_MIN = 4;
static const float    MOTOR_MAX = 18;

// Here we put code that interacts with Cleanflight
extern "C" {

#include "platform.h"

#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/time.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/bus_i2c.h"

#include "pg/bus_i2c.h"

#include "io/serial.h"

#include "target.h"

#include "stm32f30x.h"

    static serialPort_t * _serial0;

    AlienflightF3V1::AlienflightF3V1(void)
    {
        initMotors();
        initUsb();
        initImu();

        RealBoard::init();
    }

    void AlienflightF3V1::initImu(void)
    {
        i2cHardwareConfigure(i2cConfig(0));
        i2cInit(I2CDEV_2);

        _imu = new MPU6050(AFS_8G, GFS_2000DPS);

        _imu->begin();
    }

    void AlienflightF3V1::initUsb(void)
    {
        uartPinConfigure(serialPinConfig());

        _serial0 = usbVcpOpen();
    }

    void AlienflightF3V1::initMotors(void)
    {

        motorDevConfig_t dev;

        dev.motorPwmRate = BRUSHED_PWM_RATE;
        dev.motorPwmProtocol = PWM_TYPE_STANDARD;
        dev.motorPwmInversion = false;
        dev.useUnsyncedPwm = true;
        dev.useBurstDshot = false;

        dev.ioTags[0] = timerioTagGetByUsage(TIM_USE_MOTOR, 0);
        //dev.ioTags[1] = timerioTagGetByUsage(TIM_USE_MOTOR, 1);
        //dev.ioTags[2] = timerioTagGetByUsage(TIM_USE_MOTOR, 2);
        //dev.ioTags[3] = timerioTagGetByUsage(TIM_USE_MOTOR, 3);

        motorDevInit(&dev, BRUSHED_IDLE_PULSE, 4);

        pwmEnableMotors();

        writeMotor(0, 0);
        //writeMotor(1, 0);
        //writeMotor(2, 0);
        //writeMotor(3, 0);
    }

    void AlienflightF3V1::writeMotor(uint8_t index, float value)
    {
        //pwmWriteMotor(index, MOTOR_MIN + value*(MOTOR_MAX-MOTOR_MIN));
    }

    void AlienflightF3V1::delaySeconds(float sec)
    {
        delay((uint16_t)(sec*1000));
    }

    void AlienflightF3V1::setLed(bool is_on)
    {
        ledSet(0, is_on);
    }

    uint32_t AlienflightF3V1::getMicroseconds(void)
    {
        return micros();
    }

    void AlienflightF3V1::reboot(void)
    {
        systemResetToBootloader();
    }

    uint8_t AlienflightF3V1::serialAvailableBytes(void)
    {
        //return usbVcpAvailable(_serial0);
        return serialRxBytesWaiting(_serial0);
    }

    uint8_t AlienflightF3V1::serialReadByte(void)
    {
        //return usbVcpRead(_serial0);
        return serialRead(_serial0);
    }

    void AlienflightF3V1::serialWriteByte(uint8_t c)
    {
        //usbVcpWrite(_serial0, c);
        serialWrite(_serial0, c);
    }

    bool AlienflightF3V1::imuRead(void)
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
            //usbVcpWrite(_serial0, *p);
            serialWrite(_serial0, *p);
    }

} // extern "C"
