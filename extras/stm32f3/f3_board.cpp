/*
   f3_board.cpp : Support for STM32F3 boards
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

extern "C" {

#include "f3_board.h"

#include "platform.h"
#include "system.h"
#include "dma.h"
#include "gpio.h"
#include "serial.h"

#include <filters.hpp>

    F3Board::F3Board(void)
    {
        usbInit();

        imuInit();

        RealBoard::init();
    }

    void F3Board::delaySeconds(float sec)
    {
        delay(sec*1000);
    }

    void F3Board::ledSet(bool is_on)
    { 
        uint16_t gpio_pin = LED0_PIN;

        GPIO_TypeDef * gpio = LED0_GPIO;

        if (is_on) {
            digitalLo(gpio, gpio_pin);
        }
        else {
            digitalHi(gpio, gpio_pin);
        }
    }

    uint32_t F3Board::getMicroseconds(void)
    {
        return micros();
    }

    void F3Board::reboot(void)
    {
        systemResetToBootloader();
    }

    bool F3Board::getGyrometer(float gyroRates[3])
    {
        if (_imu.checkNewAccelGyroData()) {

            // Read IMU
            _imu.readAccelerometer(_ax, _ay, _az);
            _imu.readGyrometer(_gx, _gy, _gz);

            // Convert gyrometer values from degrees/sec to radians/sec
            _gx = radians(_gx);
            _gy = radians(_gy);
            _gz = radians(_gz);

            // Copy gyro values back out
            gyro[0] = _gx;
            gyro[1] = _gy;
            gyro[2] = _gz;

            // Increment count for quaternion check
            _gyroCycleCount++;

            return true;

        } // if (_imu.checkNewAccelGyroData())

    } // if gotNewData

    return false;
}

bool F3Board::getQuaternion(float quat[4])
{
    // Update quaternion after some number of IMU readings
    if (_gyroCycleCount == QUATERNION_DIVISOR) {

        _gyroCycleCount = 0;

        // Set integration time by time elapsed since last filter update
        uint32_t timeCurr = micros();
        static uint32_t _timePrev;
        float deltat = ((timeCurr - _timePrev)/1000000.0f); 
        _timePrev = timeCurr;

        // Run the quaternion on the IMU values acquired in getGyrometer()
        _quaternionFilter.update(-_ax, _ay, _az, _gx, -_gy, -_gz, deltat, _q);

        // Copy the quaternion back out
        memcpy(quat, _q, 4*sizeof(float));
        return true;
    }

    return false;
}

void hf::Board::outbuf(char * buf)
{
    for (char *p=buf; *p; p++)
        F3Board::outchar(*p);
}


} // extern "C"
