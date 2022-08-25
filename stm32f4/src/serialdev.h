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

#pragma once

#include <serial.h>

#include "io.h"
#include "io_types.h"
#include "resource.h"

typedef void (*serialIdleCallbackPtr)();

typedef struct serialPort_s {

    const struct serialPortVTable *vTable;

    portMode_e mode;
    portOptions_e options;

    uint32_t baudRate;

    uint32_t rxBufferSize;
    uint32_t txBufferSize;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
    uint32_t rxBufferHead;
    uint32_t rxBufferTail;
    uint32_t txBufferHead;
    uint32_t txBufferTail;

    serialReceiveCallbackPtr rxCallback;
    void *rxCallbackData;

    serialIdleCallbackPtr idleCallback;

    uint8_t identifier;

} serialPort_t;

#define SERIAL_PORT_MAX_INDEX (RESOURCE_SOFT_OFFSET + 2)

static const ioTag_t IO_TAG_RX[SERIAL_PORT_MAX_INDEX]  = {26, 0, 43, 17, 66, 55, 0, 0, 0, 0, 0, 0};
static const ioTag_t IO_TAG_TX[SERIAL_PORT_MAX_INDEX]  = {25, 0, 42, 16,  0, 54, 0, 0, 0, 0, 0, 0};

struct serialPortVTable {

    void (*serialWrite)(serialPort_t *instance, uint8_t ch);

    uint32_t (*serialTotalRxWaiting)(const serialPort_t *instance);
    uint32_t (*serialTotalTxFree)(const serialPort_t *instance);

    uint8_t (*serialRead)(serialPort_t *instance);

    // Specified baud rate may not be allowed by an implementation, use
    // serialGetBaudRate to determine actual baud rate in use.
    void (*serialSetBaudRate)(serialPort_t *instance, uint32_t baudRate);

    bool (*isSerialTransmitBufferEmpty)(const serialPort_t *instance);

    void (*setMode)(serialPort_t *instance, portMode_e mode);
    void (*setCtrlLineStateCb)(serialPort_t *instance,
            void (*cb)(void *instance, uint16_t ctrlLineState), void *context);
    void (*setBaudRateCb)(serialPort_t *instance,
            void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context);

    void (*writeBuf)(serialPort_t *instance, const void *data, int count);
    // Optional functions used to buffer large writes.
    void (*beginWrite)(serialPort_t *instance);
    void (*endWrite)(serialPort_t *instance);
};

#define SERIAL_PORT_COUNT 9

#define SERIAL_PORT_IDENTIFIER_TO_INDEX(x) (((x) < RESOURCE_SOFT_OFFSET) ? (x) : \
        (RESOURCE_SOFT_OFFSET + ((x) - SERIAL_PORT_SOFTSERIAL1)))
#define SERIAL_PORT_IDENTIFIER_TO_UARTDEV(x) ((x) - SERIAL_PORT_USART1 + UARTDEV_1)
