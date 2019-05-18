/*
   Board class for Omnibus F3

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
#include "stm32fboard.hpp"
#include "beeperled.hpp"
#include "motors.hpp"

extern "C" {
#include "spi.h"
}

// These are static so serial_event1 can access them
static uint8_t _value1;
static bool _avail1;

static void serial_event1(uint16_t value, void * data)
{
    (void)data;

    _value1 = (uint8_t)(value & 0xFF);

    _avail1 = true;
}


class OmnibusF3 : public Stm32FBoard {

    private:

        MPU6000 * _imu;

        // For incoming sensor messages
        serialPort_t * _serial2;

    protected:

        // SoftwareQuaternionBoard class overrides
        bool imuReady(void)
        {
            return _imu->checkNewData();
        }

        void imuReadAccelGyro(void)
        {
            _imu->readAccelerometer(_ax, _ay, _az);
            _imu->readGyrometer(_gx, _gy, _gz);

            // Negate for IMU orientation
            _ay = -_ay;
            _gx = -_gx;
            _gz = -_gz;
        }

        // RealBoard class overrides

        virtual uint8_t serialTelemetryAvailable(void) override
        {
            return serialRxBytesWaiting(_serial2);
        }

        virtual uint8_t serialTelemetryRead(void) override
        {
            return serialRead(_serial2);
        }

        virtual void serialTelemetryWrite(uint8_t c) override
        {
            serialWrite(_serial2, c);
        }

    public:

        OmnibusF3(void) : Stm32FBoard(usbVcpOpen())
    {
        // Start SPI bus for MPU6000
        spi_init(MPU6000_SPI_INSTANCE, IOGetByTag(IO_TAG(MPU6000_CS_PIN)));

        // Set up the LED (uses the beeper for some reason)
        beeperLedInit();

        // Run standard initializations
        brushless_motors_init(0, 1, 2, 3);

        // Start the IMU
        _imu = new MPU6000(MPUIMU::AFS_2G, MPUIMU::GFS_250DPS);

        // Check IMU ready status
        checkImuError(_imu->begin());

        // Set up UARTs for sensors, telemetry
        uartPinConfigure(serialPinConfig());
        uartOpen(UARTDEV_1,  serial_event1, NULL,  115200, MODE_RX, SERIAL_NOT_INVERTED);
        _serial2 = uartOpen(UARTDEV_2,  NULL, NULL,  115200, MODE_RXTX, SERIAL_NOT_INVERTED);

        RealBoard::init();
    }

}; // class OmnibusF3

void Stm32FBoard::setLed(bool isOn)
{
    beeperLedSet(isOn);
}


