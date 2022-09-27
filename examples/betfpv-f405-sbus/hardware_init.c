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

#include "spi.h"
#include "exti.h"
#include "inverter.h"
#include "io.h"
#include "pinio.h"
#include "serialdev.h"
#include "serial_uart.h"
#include "spi.h"
#include "systemdev.h"
#include "timer.h"
#include "usb_io.h"

#include "hardware_init.h"

#if defined (__cplusplus)
extern "C" {
#endif

void hardwareInit(void)
{
    systemInit();
    ioInitGlobal();
    extiInit();
    systemClockSetHSEValue(8000000);
    OverclockRebootIfNecessary(0);
    timerInit();
    serialUartPinConfigure();
    serialInit(-1);
    inverterInit();
    usbCableDetectInit();
    systemInitUnusedPins();
    pinioInit();
    timerStart();
}

#if defined (__cplusplus)
}
#endif
