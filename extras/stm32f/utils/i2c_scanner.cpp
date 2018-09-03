/*
   I^2C scanner sketch for STM32F3 boards

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

#include <debug.hpp>
#include <VL53L1X.h>

extern "C" {

    // Cleanflight includes
#include "platform.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/time.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/bus_i2c.h"
#include "pg/bus_i2c.h"
#include "io/serial.h"
#include "target.h"
#include "stm32f30x.h"

    // Other includes
#include <CrossPlatformI2C.h>
#include "../common/i2c.h"

    static I2CDevice _i2cdev = I2CDEV_1;

    static serialPort_t * _serial0;

    //static VL53L1X * distanceSensor;

    void setup(void)
    {
        i2c_init(_i2cdev);

        delay(100);

        //distanceSensor = new VL53L1X();

        _serial0 = usbVcpOpen();

        /*
        if (!distanceSensor->begin()) {
            while (true) {
                hf::Debug::printf("Sensor offline!\n");
                delay(200);
            }
        }*/
    }

    void loop(void)
    {
        // Scan for and report I^2C devices.
        for (uint8_t addr=0; addr<128; ++addr) {

            // A successful scan is detected as the ability to write to a given address.
            if (i2cWrite(_i2cdev, addr,  0, 0)) { 
                hf::Debug::printf("Found device at address 0X%02X\n", addr);
            }

            // support reboot from host computer
            static uint32_t dbg_start_msec;
            if (millis()-dbg_start_msec > 100) {
                dbg_start_msec = millis();
                while (serialRxBytesWaiting(_serial0)) {
                    uint8_t c = serialRead(_serial0);
                    if (c == 'R') 
                        systemResetToBootloader();

                }
            }
        }

        hf::Debug::printf("--------------------------\n");
    }

    void hf::Board::outbuf(char * buf)
    {
        for (char *p=buf; *p; p++)
            serialWrite(_serial0, *p);
    }

} // extern "C"
