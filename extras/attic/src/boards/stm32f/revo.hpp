/*
   Board class for Revo

   Reads raw IMU gyro and accel alternately from MPU6000

   Copyright (C) 2019 Simon D. Levy 

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

#pragma once

#include <MPU6000.h>
#include "support/motors.hpp"

#include <boards/realboard.hpp>
#include <boards/softquat.hpp>
#include "support/beeperled.hpp"
#include "support/spi.hpp"

// Cleanflight drivers
extern "C" {
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
#include "drivers/light_led.h"
} // extern "C"

// Required by system_stm32f4xx.c
void * mpuResetFn = NULL;

// We put this outside the class to make it available to static Board::outbuf() below
static serialPort_t * _serial0;

class Revo : public hf::RealBoard, public hf::SoftwareQuaternionBoard {

    private:

        MPU6000 * _imu = NULL;

        bool _accelReady;

        int16_t _accx, _accy, _accz;
        int16_t _gyrox, _gyroy, _gyroz;

        void checkImuError(MPU6000::Error_t errid)
        {
            switch (errid) {

                case MPU6000::ERROR_IMU_ID:
                    error("Bad device ID");
                    break;
                case MPU6000::ERROR_SELFTEST:
                    error("Failed self-test");
                    break;
                default:
                    break;
            }
        }

    protected: 

        // Board class overrides

        virtual void setLed(bool isOn) override
        {
            ledSet(0, isOn);
        }

        virtual void writeMotor(uint8_t index, float value) override
        {
            motor_write(index, value);
        }

        virtual void reboot(void) override
        {
            systemResetToBootloader();
        }

        virtual bool getQuaternion(float & qw, float & qx, float & qy, float & qz) override
        {
            return SoftwareQuaternionBoard::getQuaternion(qw, qx, qy, qz, getTime());
        }

        virtual bool getGyrometer(float & gx, float & gy, float & gz) override
        {
            return SoftwareQuaternionBoard::getGyrometer(gx, gy, gz);
        }

        virtual uint8_t serialNormalAvailable(void) override
        {
            return serialRxBytesWaiting(_serial0);
        }

        virtual uint8_t serialNormalRead(void) override
        {
            return serialRead(_serial0);
        }

        virtual void serialNormalWrite(uint8_t c) override
        {
            serialWrite(_serial0, c);
        }

        // SoftwareQuaternionBoard class overrides

        virtual bool imuReady(void) override
        {
            bool ready = false;

            if (_accelReady) {

                int16_t ax=0, ay=0, az=0;

                if (_imu->readAccelRaw(ax, ay, az)) {
                    _accx = ax;
                    _accy = ay;
                    _accz = az;
                }

                _accelReady = false;

                ready = true;
            }

            else {

                int16_t gx=0, gy=0, gz=0;

                if (_imu->readGyroRaw(gx, gy, gz)) {
                    _gyrox = gx;
                    _gyroy = gy;
                    _gyroz = gz;
                }

                _accelReady  = true;
            }

            return ready;
        }

        virtual void imuReadAccelGyro(void) override
        {
            // Scale the raw accel & gyro values read by imuReady()
            _imu->scaleRawAccel(_accx, _accy,  _accz,  _ax, _ay, _az);
            _imu->scaleRawGyro(_gyrox, _gyroy, _gyroz, _gx, _gy, _gz);

            // Negate for IMU orientation
            _ay = -_ay;
            _gx = -_gx;
            _gz = -_gz;
        }

        // IMU methods

    public:

        Revo(void)
        {
            _serial0 = usbVcpOpen();

            spi_init(MPU6000_SPI_INSTANCE, IOGetByTag(IO_TAG(MPU6000_CS_PIN)));

            // Set up the motors
            brushless_motors_init(0, 1, 2, 3);

            // Start the IMU
            _imu = new MPU6000(MPU6000::AFS_2G, MPU6000::GFS_250DPS);

            // Check IMU ready status
            checkImuError(_imu->begin());

            RealBoard::init();

            // Set up for gyro/accel alternation hack
            _accelReady = false;
            _accx = 0;
            _accy = 0;
            _accz = 0;
            _gyrox = 0;
            _gyroy = 0;
            _gyroz = 0;
        }

}; // class Revo

void hf::Board::outbuf(char * buf)
{
    for (char *p=buf; *p; p++)
        serialWrite(_serial0, *p);
}

