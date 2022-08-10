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

#include "bus_spi.h"
#include "dshot_command.h"
#include "exti.h"
#include "flash.h"
#include "init.h"
#include "inverter.h"
#include "io.h"
#include "motordev.h"
#include "pinio.h"
#include "serialdev.h"
#include "serial_uart.h"
#include "systemdev.h"
#include "timer.h"
#include "usb_io.h"

void * init(uint32_t core_period)
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
    dshotSetPidLoopTime(core_period);
    pinioInit();
    usbCableDetectInit();
    flashInit();
    timerStart();
    spiInitBusDMA();
    motorPostInit(motorDevice);
    motorEnable(motorDevice);
    systemInitUnusedPins();

    return motorDevice;
}
