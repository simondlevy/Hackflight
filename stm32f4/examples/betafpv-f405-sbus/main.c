/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include <core_rate.h>
#include <datatypes.h>
#include <hackflight_full.h>
#include <imu.h>
#include <imu_alignment/rotate_270.h>
#include <mixers/fixedpitch/quadxbf.h>
#include <serial.h>

#include "bus_spi.h"
#include "dshot_command.h"
#include "exti.h"
#include "flash.h"
#include "inverter.h"
#include "io.h"
#include "motordev.h"
#include "pinio.h"
#include "serialdev.h"
#include "serial_uart.h"
#include "systemdev.h"
#include "timer.h"
#include "usb_io.h"

int main(void)
{
    systemInit();
    ioInitGlobal();
    extiInit();
    systemClockSetHSEValue(8000000);
    OverclockRebootIfNecessary(0);
    timerInit();
    serialUartPinConfigure();
    serialInit(-1);
    void * motorDevice = motorInitDshot(4);
    inverterInit();
    spiPinConfigure();
    spiPreInit();
    spiInit(0x07); // mask for devices 0,1,2
    dshotSetPidLoopTime(CORE_PERIOD());
    pinioInit();
    usbCableDetectInit();
    flashInit();
    timerStart();
    spiInitBusDMA();
    motorPostInit(motorDevice);
    motorEnable(motorDevice);
    systemInitUnusedPins();

    hackflight_t hf = {0};

    hackflightInitFull(
            &hf,
            mixerQuadXbf,
            motorDevice,
            SERIAL_PORT_USART3, // RX port
            0,                  // dummy value for IMU interrupt pin
            imuRotate270,
            37);                // LED pin

    while (true) {
        hackflightStep(&hf);
    }

    return 0;
}

void HardFault_Handler(void)
{
}
