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
#include "io.h"
#include "serialdev.h"
#include "inverter.h"

static const ioTag_t INVERTER[SERIAL_PORT_MAX_INDEX] = {0, 0,  57,  0,  0, 56, 0, 0, 0, 0, 0, 0};

static void inverterSet(int identifier, bool on)
{
    IO_t pin = IOGetByTag(INVERTER[SERIAL_PORT_IDENTIFIER_TO_INDEX(identifier)]);

    if (pin) {
        IOWrite(pin, on);
    }
}

static void initInverter(int identifier)
{
    int uartIndex = SERIAL_PORT_IDENTIFIER_TO_INDEX(identifier);
    IO_t pin = IOGetByTag(INVERTER[uartIndex]);

    IOInit(pin, OWNER_INVERTER, RESOURCE_INDEX(uartIndex));
    IOConfigGPIO(pin, IOCFG_OUT_PP);

    inverterSet(identifier, false);
}

void inverterInit(void)
{
    initInverter(SERIAL_PORT_USART3);
    initInverter(SERIAL_PORT_USART6);
}

void enableInverter(USART_TypeDef *USARTx, bool on)
{
    int identifier = SERIAL_PORT_NONE;

    if (USARTx == USART1) {
        identifier = SERIAL_PORT_USART1;
    }
    if (USARTx == USART2) {
        identifier = SERIAL_PORT_USART2;
    }
    if (USARTx == USART3) {
        identifier = SERIAL_PORT_USART3;
    }
    if (USARTx == UART4) {
        identifier = SERIAL_PORT_UART4;
    }
    if (USARTx == UART5) {
        identifier = SERIAL_PORT_UART5;
    }
    if (USARTx == USART6) {
        identifier = SERIAL_PORT_USART6;
    }

    if (identifier != SERIAL_PORT_NONE) {
        inverterSet(identifier, on);
    }
}
