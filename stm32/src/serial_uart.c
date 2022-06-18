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

#include "macros.h"

#include "dma.h"
#include "dma_reqmap.h"
#include "rcc.h"
#include "serialdev.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

#define UART_TX_BUFFER_ATTRIBUTE                    // NONE
#define UART_RX_BUFFER_ATTRIBUTE                    // NONE

#define UART_BUFFERS(n) \
    UART_BUFFER(UART_TX_BUFFER_ATTRIBUTE, n, T); \
    UART_BUFFER(UART_RX_BUFFER_ATTRIBUTE, n, R); struct dummy_s

#define LPUART_BUFFERS(n) \
    LPUART_BUFFER(UART_TX_BUFFER_ATTRIBUTE, n, T); \
    LPUART_BUFFER(UART_RX_BUFFER_ATTRIBUTE, n, R); struct dummy_s

UART_BUFFERS(1);
UART_BUFFERS(2);
UART_BUFFERS(3);
UART_BUFFERS(4);
UART_BUFFERS(5);
UART_BUFFERS(6);

#undef UART_BUFFERS

serialPort_t *uartOpen(UARTDevice_e device, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartPort_t *uartPort = serialUART(device, baudRate, mode, options);

    if (!uartPort)
        return (serialPort_t *)uartPort;

    uartPort->txDMAEmpty = true;

    // common serial initialisation code should move to serialPort::init()
    uartPort->port.rxBufferHead = uartPort->port.rxBufferTail = 0;
    uartPort->port.txBufferHead = uartPort->port.txBufferTail = 0;
    // callback works for IRQ-based RX ONLY
    uartPort->port.rxCallback = rxCallback;
    uartPort->port.rxCallbackData = rxCallbackData;
    uartPort->port.mode = mode;
    uartPort->port.baudRate = baudRate;
    uartPort->port.options = options;

    uartReconfigure(uartPort);

    return (serialPort_t *)uartPort;
}

static void uartSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.baudRate = baudRate;
    uartReconfigure(uartPort);
}

static void uartSetMode(serialPort_t *instance, portMode_e mode)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.mode = mode;
    uartReconfigure(uartPort);
}

static uint32_t uartTotalRxBytesWaiting(const serialPort_t *instance)
{
    const uartPort_t *uartPort = (const uartPort_t*)instance;

    if (uartPort->rxDMAResource) {
        // XXX Could be consolidated
        uint32_t rxDMAHead = xDMA_GetCurrDataCounter(uartPort->rxDMAResource);

        // uartPort->rxDMAPos and rxDMAHead represent distances from the end
        // of the buffer.  They count DOWN as they advance.
        if (uartPort->rxDMAPos >= rxDMAHead) {
            return uartPort->rxDMAPos - rxDMAHead;
        } else {
            return uartPort->port.rxBufferSize + uartPort->rxDMAPos - rxDMAHead;
        }
    }

    if (uartPort->port.rxBufferHead >= uartPort->port.rxBufferTail) {
        return uartPort->port.rxBufferHead - uartPort->port.rxBufferTail;
    } else {
        return uartPort->port.rxBufferSize + uartPort->port.rxBufferHead - uartPort->port.rxBufferTail;
    }
}

static uint32_t uartTotalTxBytesFree(const serialPort_t *instance)
{
    const uartPort_t *uartPort = (const uartPort_t*)instance;

    uint32_t bytesUsed;

    if (uartPort->port.txBufferHead >= uartPort->port.txBufferTail) {
        bytesUsed = uartPort->port.txBufferHead - uartPort->port.txBufferTail;
    } else {
        bytesUsed = uartPort->port.txBufferSize + uartPort->port.txBufferHead - uartPort->port.txBufferTail;
    }

    if (uartPort->txDMAResource) {
        /*
         * When we queue up a DMA request, we advance the Tx buffer tail before the transfer finishes, so we must add
         * the remaining size of that in-progress transfer here instead:
         */
        bytesUsed += xDMA_GetCurrDataCounter(uartPort->txDMAResource);

        /*
         * If the Tx buffer is being written to very quickly, we might have advanced the head into the buffer
         * space occupied by the current DMA transfer. In that case the "bytesUsed" total will actually end up larger
         * than the total Tx buffer size, because we'll end up transmitting the same buffer region twice. (So we'll be
         * transmitting a garbage mixture of old and new bytes).
         *
         * Be kind to callers and pretend like our buffer can only ever be 100% full.
         */
        if (bytesUsed >= uartPort->port.txBufferSize - 1) {
            return 0;
        }
    }

    return (uartPort->port.txBufferSize - 1) - bytesUsed;
}

static bool isUartTransmitBufferEmpty(const serialPort_t *instance)
{
    const uartPort_t *uartPort = (const uartPort_t *)instance;
    if (uartPort->txDMAResource) {
        return uartPort->txDMAEmpty;
    } else {
        return uartPort->port.txBufferTail == uartPort->port.txBufferHead;
    }
}

static uint8_t uartRead(serialPort_t *instance)
{
    uint8_t ch;
    uartPort_t *uartPort = (uartPort_t *)instance;

    if (uartPort->rxDMAResource) {
        ch = uartPort->port.rxBuffer[uartPort->port.rxBufferSize - uartPort->rxDMAPos];
        if (--uartPort->rxDMAPos == 0)
            uartPort->rxDMAPos = uartPort->port.rxBufferSize;
    } else {
        ch = uartPort->port.rxBuffer[uartPort->port.rxBufferTail];
        if (uartPort->port.rxBufferTail + 1 >= uartPort->port.rxBufferSize) {
            uartPort->port.rxBufferTail = 0;
        } else {
            uartPort->port.rxBufferTail++;
        }
    }

    return ch;
}

static void uartWrite(serialPort_t *instance, uint8_t ch)
{
    uartPort_t *uartPort = (uartPort_t *)instance;

    uartPort->port.txBuffer[uartPort->port.txBufferHead] = ch;

    if (uartPort->port.txBufferHead + 1 >= uartPort->port.txBufferSize) {
        uartPort->port.txBufferHead = 0;
    } else {
        uartPort->port.txBufferHead++;
    }

    if (uartPort->txDMAResource) {
        uartTryStartTxDMA(uartPort);
    } else {
        USART_ITConfig(uartPort->USARTx, USART_IT_TXE, ENABLE);
    }
}

const struct serialPortVTable uartVTable[] = {
    {
        .serialWrite = uartWrite,
        .serialTotalRxWaiting = uartTotalRxBytesWaiting,
        .serialTotalTxFree = uartTotalTxBytesFree,
        .serialRead = uartRead,
        .serialSetBaudRate = uartSetBaudRate,
        .isSerialTransmitBufferEmpty = isUartTransmitBufferEmpty,
        .setMode = uartSetMode,
        .setCtrlLineStateCb = NULL,
        .setBaudRateCb = NULL,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL,
    }
};

void uartConfigureDma(uartDevice_t *uartdev)
{
    uartPort_t *uartPort = &(uartdev->port);
    const uartHardware_t *hardware = uartdev->hardware;

    if (uartPort->txDMAResource) {
        dmaIdentifier_e identifier = dmaGetIdentifier(uartPort->txDMAResource);
        if (dmaAllocate(identifier, OWNER_SERIAL_TX, RESOURCE_INDEX(hardware->device))) {
            dmaEnable(identifier);
            dmaSetHandler(identifier, uartDmaIrqHandler, hardware->txPriority, (uint32_t)uartdev);
            uartPort->txDMAPeripheralBaseAddr = (uint32_t)&UART_REG_TXD(hardware->reg);
        }
    }

    if (uartPort->rxDMAResource) {
        dmaIdentifier_e identifier = dmaGetIdentifier(uartPort->rxDMAResource);
        if (dmaAllocate(identifier, OWNER_SERIAL_RX, RESOURCE_INDEX(hardware->device))) {
            dmaEnable(identifier);
            uartPort->rxDMAPeripheralBaseAddr = (uint32_t)&UART_REG_RXD(hardware->reg);
        }
    }
}

#define UART_IRQHandler(type, number, dev)                    \
    void type ## number ## _IRQHandler(void)                  \
    {                                                         \
        uartPort_t *uartPort = &(uartDevmap[UARTDEV_ ## dev]->port); \
        uartIrqHandler(uartPort);                                    \
    }

UART_IRQHandler(USART, 1, 1) // USART1 Rx/Tx IRQ Handler
UART_IRQHandler(USART, 2, 2) // USART2 Rx/Tx IRQ Handler
UART_IRQHandler(USART, 3, 3) // USART3 Rx/Tx IRQ Handler
UART_IRQHandler(UART, 4, 4)  // UART4 Rx/Tx IRQ Handler
UART_IRQHandler(UART, 5, 5)  // UART5 Rx/Tx IRQ Handler
UART_IRQHandler(USART, 6, 6) // USART6 Rx/Tx IRQ Handler
