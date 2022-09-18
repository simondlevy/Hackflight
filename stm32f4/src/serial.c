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
#include <string.h>


#include <serial.h>
#include <time.h>

#include "platform.h"
#include "serialdev.h"
#include "serial_uart.h"
#include "serial_usb_vcp.h"
#include "systemdev.h"

typedef struct serialPortUsage_s {
    serialPort_t *serialPort;
    serialPortFunction_e function;
    serialPortIdentifier_e identifier;
} serialPortUsage_t;

static serialPortUsage_t _serialPortUsageList[SERIAL_PORT_COUNT];

static serialPortUsage_t *findSerialPortUsageByIdentifier(
        serialPortIdentifier_e identifier)
{
    uint8_t index;
    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialPortUsage_t *candidate = &_serialPortUsageList[index];
        if (candidate->identifier == identifier) {
            return candidate;
        }
    }
    return NULL;
}

static const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT] = {
    SERIAL_PORT_USB_VCP,
    SERIAL_PORT_USART1,
    SERIAL_PORT_USART2,
    SERIAL_PORT_USART3,
    SERIAL_PORT_UART4,
    SERIAL_PORT_UART5,
    SERIAL_PORT_USART6,
    SERIAL_PORT_SOFTSERIAL1,
    SERIAL_PORT_SOFTSERIAL2
};

// ----------------------------------------------------------------------------

void serialInit(serialPortIdentifier_e serialPortToDisable)
{
    memset(&_serialPortUsageList, 0, sizeof(_serialPortUsageList));

    for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
        _serialPortUsageList[index].identifier = serialPortIdentifiers[index];

        if (serialPortToDisable != SERIAL_PORT_NONE) {
            if (_serialPortUsageList[index].identifier == serialPortToDisable) {
                _serialPortUsageList[index].identifier = SERIAL_PORT_NONE;
            }
        }

        else if (_serialPortUsageList[index].identifier <= SERIAL_PORT_USART8) {
            int resourceIndex =
                SERIAL_PORT_IDENTIFIER_TO_INDEX(
                        _serialPortUsageList[index].identifier);
            if (!(IO_TAG_TX[resourceIndex] || IO_TAG_RX[resourceIndex])) {
                _serialPortUsageList[index].identifier = SERIAL_PORT_NONE;
            }
        }

    }
}

bool serialIsTransmitBufferEmpty(void * p)
{
    serialPort_t * port = (serialPort_t *)p;

    return port->vTable->isSerialTransmitBufferEmpty(port);
}


void * serialOpenPort(
    serialPortIdentifier_e identifier,
    serialPortFunction_e function,
    serialReceiveCallbackPtr rxCallback,
    void *rxCallbackData,
    uint32_t baudRate,
    portMode_e mode,
    portOptions_e options)
{
    serialPortUsage_t *serialPortUsage = findSerialPortUsageByIdentifier(identifier);

    if (!serialPortUsage || serialPortUsage->function != FUNCTION_NONE) {
        // not available / already in use
        return NULL;
    }

    serialPort_t *serialPort = NULL;

    switch (identifier) {
        case SERIAL_PORT_USB_VCP:
            serialPort = usbVcpOpen();
            break;

        case SERIAL_PORT_USART1:
        case SERIAL_PORT_USART2:
        case SERIAL_PORT_USART3:
        case SERIAL_PORT_UART4:
        case SERIAL_PORT_UART5:
        case SERIAL_PORT_USART6:
            serialPort = uartOpen(SERIAL_PORT_IDENTIFIER_TO_UARTDEV(identifier),
                    rxCallback, rxCallbackData, baudRate, mode, options);
            break;

        default:
            break;
    }

    if (!serialPort) {
        return NULL;
    }

    serialPort->identifier = identifier;

    serialPortUsage->function = function;
    serialPortUsage->serialPort = serialPort;

    return (void *)serialPort;
}

uint8_t serialRead(void * p)
{
    serialPort_t * port = (serialPort_t *)p;

    return port->vTable->serialRead(port);
}

uint32_t serialBytesAvailable(void * p)
{
    serialPort_t * port = (serialPort_t *)p;

    return port->vTable->serialTotalRxWaiting(port);
}

uint32_t serialTxBytesFree(void * p)
{
    serialPort_t * port = (serialPort_t *)p;

    return port->vTable->serialTotalTxFree(port);
}

void serialWrite(void * p, uint8_t ch)
{
    serialPort_t * port = (serialPort_t *)p;

    port->vTable->serialWrite(port, ch);
}

void serialWriteBuf(void * p, const uint8_t *data, uint32_t count)
{
    serialPort_t * port = (serialPort_t *)p;

    if (port->vTable->writeBuf) {
        port->vTable->writeBuf(port, data, count);
    } else {
        for (const uint8_t *p = data; count > 0; count--, p++) {

            while (!serialTxBytesFree(port)) {
            };

            serialWrite(port, *p);
        }
    }
}

void serialOpenPortDsmx(
        serialPortIdentifier_e identifier,
        serialReceiveCallbackPtr callback,
        void * data)
{
    serialOpenPort(
            identifier,
            FUNCTION_RX_SERIAL,
            callback,
            data,
            BAUD_115200,
            MODE_RX,
            SERIAL_NOT_INVERTED);
}

void serialOpenPortSbus(
        serialPortIdentifier_e identifier,
        serialReceiveCallbackPtr callback,
        void * data)
{
    serialOpenPort(
            identifier,
            FUNCTION_RX_SERIAL,
            callback,
            data,
            100000,
            MODE_RX,
            SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN | SERIAL_INVERTED);
}

void * serialOpenPortUsb(void)
{
    return serialOpenPort(
            SERIAL_PORT_USB_VCP,
            FUNCTION_MSP,
            NULL, NULL,
            BAUD_115200,
            MODE_RXTX,
            SERIAL_NOT_INVERTED);
}
