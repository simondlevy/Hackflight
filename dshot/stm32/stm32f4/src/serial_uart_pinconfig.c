/*
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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "rcc.h"
#include "serialdev.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

FAST_DATA_ZERO_INIT uartDevice_t uartDevice[UARTDEV_COUNT];      // Only those configured in target.h
FAST_DATA_ZERO_INIT uartDevice_t *uartDevmap[UARTDEV_COUNT_MAX]; // Full array

void serialUartPinConfigure(void)
{
    uartDevice_t *uartdev = uartDevice;

    for (size_t hindex = 0; hindex < UARTDEV_COUNT; hindex++) {

        const uartHardware_t *hardware = &uartHardware[hindex];
        const UARTDevice_e device = hardware->device;

        for (int pindex = 0 ; pindex < UARTHARDWARE_MAX_PINS ; pindex++) {
            if (IO_TAG_RX[device] == hardware->rxPins[pindex].pin) {
                uartdev->rx = hardware->rxPins[pindex];
            }

            if (IO_TAG_TX[device] == hardware->txPins[pindex].pin) {
                uartdev->tx = hardware->txPins[pindex];
            }
        }

        if (uartdev->rx.pin || uartdev->tx.pin) {
            uartdev->hardware = hardware;
            uartDevmap[device] = uartdev++;
        }
    }
}
