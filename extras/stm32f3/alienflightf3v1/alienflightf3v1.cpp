/*
   alienflightf3v1.cpp 

   Copyright (c) 2018 Simon D. Levy

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

#include <f3board.h>
#include <hackflight.hpp>
#include <mixers/quadx.hpp>
#include <MPU6050.h>
#include "alienflightf3_dsmx.h"

constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};

extern "C" {

#include "platform.h"
#include "dma.h"
#include "gpio.h"
#include "serial.h"
#include "system.h"
#include "serial_uart.h"
#include "system.h"
#include "serial_usb_vcp.h"

    class AlienflightF3V1 : public F3Board {

        bool getGyrometer(float gyroRates[3])
        {
            (void)gyroRates; // XXX

            static uint32_t _time;
            uint32_t time = micros();
            if (time-_time > 5000) {
                _time = time;
                return true;
            }
            return false;
        }

        bool getQuaternion(float quat[4])
        {
            (void)quat; // XXX

            static uint32_t _time;
            uint32_t time = micros();
            if (time-_time > 10000) {
                _time = time;
                return true;
            }
            return false;
        }

        void writeMotor(uint8_t index, float value)
        {
            (void)index; // XXX
            (void)value;
        }

    }; // class AlienflightF3V1

    static hf::Hackflight h;

    hf::MixerQuadX mixer;

    void setup() {

        hf::Stabilizer * stabilizer = new hf::Stabilizer(
                0.20f,      // Level P
                0.225f,     // Gyro cyclic P
                0.001875f,  // Gyro cyclic I
                0.375f,     // Gyro cyclic D
                1.0625f,    // Gyro yaw P
                0.005625f); // Gyro yaw I

        DSMX_Receiver * rc = new DSMX_Receiver(
                CHANNEL_MAP,
                .005f,  // roll trim
                .01f,  // pitch trim
                0.f);   // yaw trim

        // Initialize Hackflight firmware
        h.init(new AlienflightF3V1(), rc, &mixer, stabilizer);
    }

    void loop() {

        h.update();
    }


    serialPort_t * serial0_open(void)
    {
        return usbVcpOpen();
    }

} // extern "C"
