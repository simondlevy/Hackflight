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
#include "drivers/serial_usb_vcp.h"
#include "drivers/bus_i2c.h"
#include "pg/bus_i2c.h"
#include "io/serial.h"
#include "target.h"
#include "stm32f30x.h"

    // Hackflight include
#include "../common/i2c.h"

    static serialPort_t * _serial0;

    void setup(void)
    {
        i2c_init(I2CDEV_2);

        _serial0 = usbVcpOpen();
    }

    void loop(void)
    {

        // Scan for and report I^2C devices
        for (uint8_t addr=0; addr<128; ++addr) {
            //Wire.beginTransmission(addr);
            if (addr==0x33 /*!Wire.endTransmission()*/) {
                hf::Debug::printf("Found device at address 0X%02X\n", addr);
            }
        }
        hf::Debug::printf("--------------------------\n");

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

        // Wait a bit between reports
        delay(500);
    }

   void hf::Board::outbuf(char * buf)
    {
        for (char *p=buf; *p; p++)
            serialWrite(_serial0, *p);
    }

} // extern "C"
