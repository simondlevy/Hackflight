/*
   omnibusf3.cpp : Board implementation for Omnibus F3

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

#include "omnibusf3.h"

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
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_usb_vcp.h"
#include "io/serial.h"
#include "target.h"
#include "stm32f30x.h"

    // Hackflight includes
#include "../../common/spi.h"
#include "../../common/beeperled.h"

    static serialPort_t * _serial0;

    static uint8_t _value;
    static bool _avail;

    static void serial_event(uint16_t value, void * data)
    {
        (void)data;

        _value = (uint8_t)(value & 0xFF);

        _avail = true;
    }

    OmnibusF3::OmnibusF3(void)
    {
        // Set up the LED (uses the beeper for some reason)
        beeperLedInit();

        initMotors();
        initUsb();
        initImu();

        // Set up UART for incoming MSP sensor messages
        uartPinConfigure(serialPinConfig());
        uartOpen(UARTDEV_2,  serial_event, NULL,  115200, MODE_RX, SERIAL_NOT_INVERTED);

        RealBoard::init();
    }

    void OmnibusF3::initImu(void)
    {
        spi_init(MPU6000_SPI_INSTANCE, IOGetByTag(IO_TAG(MPU6000_CS_PIN)));
        _imu = new MPU6000(MPUIMU::AFS_2G, MPUIMU::GFS_250DPS);

        switch (_imu->begin()) {

            case MPUIMU::ERROR_IMU_ID:
                error("Bad device ID");
                break;
            case MPUIMU::ERROR_SELFTEST:
                error("Failed self-test");
                break;
            default:
                break;
        }
    }

    void OmnibusF3::initUsb(void)
    {
        _serial0 = usbVcpOpen();
    }

    void OmnibusF3::initMotors(void)
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

    void OmnibusF3::writeMotor(uint8_t index, float value)
    {
        pwmWriteMotor(index, MOTOR_MIN + value*(MOTOR_MAX-MOTOR_MIN));
    }

    void OmnibusF3::delaySeconds(float sec)
    {
        delay((uint16_t)(sec*1000));
    }

    void OmnibusF3::setLed(bool isOn)
    {
        beeperLedSet(isOn);
    }

    uint32_t OmnibusF3::getMicroseconds(void)
    {
        return micros();
    }

    void OmnibusF3::reboot(void)
    {
        systemResetToBootloader();
    }

    uint8_t OmnibusF3::serialAvailableBytes(void)
    {
        return serialRxBytesWaiting(_serial0);
    }

    uint8_t OmnibusF3::serialReadByte(void)
    {
        return serialRead(_serial0);
    }

    void OmnibusF3::serialWriteByte(uint8_t c)
    {
        serialWrite(_serial0, c);
    }

    bool OmnibusF3::imuRead(void)
    {
        if (_imu->checkNewData()) {  

            _imu->readAccelerometer(_ax, _ay, _az);
            _imu->readGyrometer(_gx, _gy, _gz);

            // Negate to support board orientation
            _ay = -_ay;
            _gx = -_gx;
            _gz = -_gz;

            return true;
        }  

        else {

            if (_avail) {
                hf::Debug::printf("%d\n", _value);
                _avail = false;
            }
        }

        return false;
    }

    void hf::Board::outbuf(char * buf)
    {
        for (char *p=buf; *p; p++)
            serialWrite(_serial0, *p);
    }

} // extern "C"
