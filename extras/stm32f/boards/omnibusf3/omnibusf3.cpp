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
#include "../../common/motors.h"

    // --------------------------------------------------

    // This is static so Board::outbuf() can access it
    static serialPort_t * _serial0;

    // --------------------------------------------------

    // These are static so serial_event can access them
    static uint8_t _value1;
    static bool _avail1;

    static void serial_event1(uint16_t value, void * data)
    {
        (void)data;

        _value1 = (uint8_t)(value & 0xFF);

        _avail1 = true;
    }

    // --------------------------------------------------

    static serialPort_t * _serial2;

    // These are static so serial_event can access them
    static uint8_t _value2;
    static bool _avail2;

    static void serial_event2(uint16_t value, void * data)
    {
        (void)data;

        _value2 = (uint8_t)(value & 0xFF);

        _avail2 = true;
    }

    // --------------------------------------------------

    OmnibusF3::OmnibusF3(void)
    {
        // Set up the LED (uses the beeper for some reason)
        beeperLedInit();

        // Run standard initializations
        initMotors();
        initUsb();
        initImu();

        // Set up UARTs for sensors, telemetry
        uartPinConfigure(serialPinConfig());
        uartOpen(UARTDEV_1,  serial_event1, NULL,  115200, MODE_RX, SERIAL_NOT_INVERTED);
        //uartOpen(UARTDEV_2,  serial_event2, NULL,  115200, MODE_RX, SERIAL_NOT_INVERTED);
        _serial2 = uartOpen(UARTDEV_2,  NULL, NULL,  115200, MODE_RXTX, SERIAL_NOT_INVERTED);

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
        brushless_motors_init(0, 1, 2, 3);
    }

    void OmnibusF3::writeMotor(uint8_t index, float value)
    {
        motor_write(index, value);
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

    uint8_t OmnibusF3::serialNormalAvailable(void)
    {
        return serialRxBytesWaiting(_serial0);
    }

    uint8_t OmnibusF3::serialNormalRead(void)
    {
        return serialRead(_serial0);
    }

    void OmnibusF3::serialNormalWrite(uint8_t c)
    {
        serialWrite(_serial0, c);
    }

    uint8_t OmnibusF3::serialTelemetryAvailable(void)
    {
        return serialRxBytesWaiting(_serial2);
    }

    uint8_t OmnibusF3::serialTelemetryRead(void)
    {
        return serialRead(_serial2);
    }

    void OmnibusF3::serialTelemetryWrite(uint8_t c)
    {
        serialWrite(_serial2, c);
    }

    bool OmnibusF3::getQuaternion(float quat[4])
    {
        return SoftwareQuaternionBoard::getQuaternion(quat, getTime());
    }

    bool OmnibusF3::getGyrometer(float gyroRates[3])
    {
        return SoftwareQuaternionBoard::getGyrometer(gyroRates);
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

            if (_avail2) {
                hf::Debug::printf("%c\n", (char)_value2);
                _avail2 = false;
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
