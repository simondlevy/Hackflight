/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include <accel.h>
#include <datatypes.h>
#include <hackflight.h>
#include <imu.h>
#include <serial.h>

#include "drivers/bus_spi.h"
#include "drivers/dshot_command.h"
#include "drivers/exti.h"
#include "drivers/flash.h"
#include "drivers/inverter.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/motordev.h"
#include "drivers/pinio.h"
#include "drivers/serialdev.h"
#include "drivers/serial_uart.h"
#include "drivers/systemdev.h"
#include "drivers/timer.h"
#include "drivers/usb_io.h"


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
    motorInit(4);
    inverterInit();
    spiPinConfigure();
    spiPreInit();
    spiInit(0x07); // mask for devices 0,1,2
    dshotSetPidLoopTime(GYRO_PERIOD());
    pinioInit();
    usbCableDetectInit();
    flashInit();
    timerStart();
    spiInitBusDMA();
    motorPostInit();
    motorEnable();
    systemInitUnusedPins();

    hackflight_t hf = {0};

    hackflightFullInit(&hf, SERIAL_PORT_USART3);

    // STM32 boards have traditional accel/gyro combo
    hackflightAddSensor(&hf, imuAccelTask, ACCEL_RATE);

    while (true) {
        hackflightStep(&hf);
    }

    return 0;
}

void HardFault_Handler(void)
{
}
