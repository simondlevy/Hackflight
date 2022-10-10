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

#include "systemdev.h"
#include "io.h"
#include "dma.h"
#include "nvic.h"
#include "rcc.h"

#include "serialdev.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
    {
        .device = UARTDEV_1,
        .reg = USART1,
        .rxDMAChannel = DMA_Channel_4,
        .txDMAChannel = DMA_Channel_4,
        .rxPins = { { DEFIO_TAG_E(PA10) }, { DEFIO_TAG_E(PB7) },
            },
        .txPins = { { DEFIO_TAG_E(PA9) }, { DEFIO_TAG_E(PB6) },
            },
        .af = GPIO_AF_USART1,
        .rcc = RCC_APB2(USART1),
        .irqn = USART1_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART1,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },

    {
        .device = UARTDEV_2,
        .reg = USART2,
        .rxDMAChannel = DMA_Channel_4,
        .txDMAChannel = DMA_Channel_4,
        .rxPins = { { DEFIO_TAG_E(PA3) }, { DEFIO_TAG_E(PD6) } },
        .txPins = { { DEFIO_TAG_E(PA2) }, { DEFIO_TAG_E(PD5) } },
        .af = GPIO_AF_USART2,
        .rcc = RCC_APB1(USART2),
        .irqn = USART2_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART2,
        .txBuffer = uart2TxBuffer,
        .rxBuffer = uart2RxBuffer,
        .txBufferSize = sizeof(uart2TxBuffer),
        .rxBufferSize = sizeof(uart2RxBuffer),
    },

    {
        .device = UARTDEV_3,
        .reg = USART3,
        .rxDMAChannel = DMA_Channel_4,
        .txDMAChannel = DMA_Channel_4,
        .rxPins = { { DEFIO_TAG_E(PB11) }, { DEFIO_TAG_E(PC11) }, { DEFIO_TAG_E(PD9) } },
        .txPins = { { DEFIO_TAG_E(PB10) }, { DEFIO_TAG_E(PC10) }, { DEFIO_TAG_E(PD8) } },
        .af = GPIO_AF_USART3,
        .rcc = RCC_APB1(USART3),
        .irqn = USART3_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART3,
        .txBuffer = uart3TxBuffer,
        .rxBuffer = uart3RxBuffer,
        .txBufferSize = sizeof(uart3TxBuffer),
        .rxBufferSize = sizeof(uart3RxBuffer),
    },

    {
        .device = UARTDEV_4,
        .reg = UART4,
        .rxDMAChannel = DMA_Channel_4,
        .txDMAChannel = DMA_Channel_4,
        .rxPins = { { DEFIO_TAG_E(PA1) }, { DEFIO_TAG_E(PC11) } },
        .txPins = { { DEFIO_TAG_E(PA0) }, { DEFIO_TAG_E(PC10) } },
        .af = GPIO_AF_UART4,
        .rcc = RCC_APB1(UART4),
        .irqn = UART4_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART4_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART4,
        .txBuffer = uart4TxBuffer,
        .rxBuffer = uart4RxBuffer,
        .txBufferSize = sizeof(uart4TxBuffer),
        .rxBufferSize = sizeof(uart4RxBuffer),
    },

    {
        .device = UARTDEV_5,
        .reg = UART5,
        .rxDMAChannel = DMA_Channel_4,
        .txDMAChannel = DMA_Channel_4,
        .rxPins = { { DEFIO_TAG_E(PD2) } },
        .txPins = { { DEFIO_TAG_E(PC12) } },
        .af = GPIO_AF_UART5,
        .rcc = RCC_APB1(UART5),
        .irqn = UART5_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART5_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART5,
        .txBuffer = uart5TxBuffer,
        .rxBuffer = uart5RxBuffer,
        .txBufferSize = sizeof(uart5TxBuffer),
        .rxBufferSize = sizeof(uart5RxBuffer),
    },

    {
        .device = UARTDEV_6,
        .reg = USART6,
        .rxDMAChannel = DMA_Channel_5,
        .txDMAChannel = DMA_Channel_5,
        .rxPins = { { DEFIO_TAG_E(PC7) },
            { DEFIO_TAG_E(PG9) },
            },
        .txPins = { { DEFIO_TAG_E(PC6) },
            { DEFIO_TAG_E(PG14) },
            },
        .af = GPIO_AF_USART6,
        .rcc = RCC_APB2(USART6),
        .irqn = USART6_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART6_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART6,
        .txBuffer = uart6TxBuffer,
        .rxBuffer = uart6RxBuffer,
        .txBufferSize = sizeof(uart6TxBuffer),
        .rxBufferSize = sizeof(uart6RxBuffer),
    },
};

static void handleUsartTxDma(uartPort_t *s)
{
    uartTryStartTxDMA(s);
}

void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = &(((uartDevice_t*)(descriptor->userParam))->port);
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_HTIF);
        if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_FEIF))
        {
            DMA_CLEAR_FLAG(descriptor, DMA_IT_FEIF);
        }
        handleUsartTxDma(s);
    }
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TEIF);
    }
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_DMEIF))
    {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_DMEIF);
    }
}

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartDevice_t *uart = uartDevmap[device];
    if (!uart) return NULL;

    const uartHardware_t *hardware = uart->hardware;

    if (!hardware) return NULL; // XXX Can't happen !?

    uartPort_t *s = &(uart->port);
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;

    s->USARTx = hardware->reg;

    uartConfigureDma(uart);

    IO_t txIO = IOGetByTag(uart->tx.pin);
    IO_t rxIO = IOGetByTag(uart->rx.pin);

    if (hardware->rcc) {
        RCC_ClockCmd(hardware->rcc, ENABLE);
    }

    if (options & SERIAL_BIDIR) {
        IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        IOConfigGPIOAF(txIO, ((options & SERIAL_BIDIR_PP) || (options & SERIAL_BIDIR_PP_PD)) ? IOCFG_AF_PP : IOCFG_AF_OD, hardware->af);
    } else {
        if ((mode & MODE_TX) && txIO) {
            IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(txIO, IOCFG_AF_PP_UP, hardware->af);
        }

        if ((mode & MODE_RX) && rxIO) {
            IOInit(rxIO, OWNER_SERIAL_RX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(rxIO, IOCFG_AF_PP_UP, hardware->af);
        }
    }

    if (!(s->rxDMAResource)) {
        NVIC_InitTypeDef NVIC_InitStructure;

        NVIC_InitStructure.NVIC_IRQChannel = hardware->irqn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(hardware->rxPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(hardware->rxPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    return s;
}

void uartIrqHandler(uartPort_t *s)
{
    if (!s->rxDMAResource && (USART_GetITStatus(s->USARTx, USART_IT_RXNE) == SET)) {
        if (s->port.rxCallback) {
            s->port.rxCallback(s->USARTx->DR, s->port.rxCallbackData, microsISR());
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = s->USARTx->DR;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }

    if (!s->txDMAResource && (USART_GetITStatus(s->USARTx, USART_IT_TXE) == SET)) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            USART_SendData(s->USARTx, s->port.txBuffer[s->port.txBufferTail]);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }

    if (USART_GetITStatus(s->USARTx, USART_IT_ORE) == SET) {
        USART_ClearITPendingBit(s->USARTx, USART_IT_ORE);
    }

    if (USART_GetITStatus(s->USARTx, USART_IT_IDLE) == SET) {
        if (s->port.idleCallback) {
            s->port.idleCallback();
        }

        // clear
        (void) s->USARTx->SR;
        (void) s->USARTx->DR;
    }
}
