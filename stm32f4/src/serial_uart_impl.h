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

#define UARTDEV_COUNT_MAX 6
#define UARTHARDWARE_MAX_PINS 4
#define UART_RX_BUFFER_SIZE     128
#define UART_TX_BUFFER_SIZE     256

// Count number of configured UARTs

#define UARTDEV_COUNT_1 1
#define UARTDEV_COUNT_2 1
#define UARTDEV_COUNT_3 1
#define UARTDEV_COUNT_4 1
#define UARTDEV_COUNT_5 1
#define UARTDEV_COUNT_6 1
#define UARTDEV_COUNT_7 0
#define UARTDEV_COUNT_8 0
#define UARTDEV_COUNT_9 0
#define UARTDEV_COUNT_10 0
#define LPUARTDEV_COUNT_1 1

#define UARTDEV_COUNT (UARTDEV_COUNT_1 + UARTDEV_COUNT_2 + UARTDEV_COUNT_3 + UARTDEV_COUNT_4 + UARTDEV_COUNT_5 + UARTDEV_COUNT_6 + UARTDEV_COUNT_7 + UARTDEV_COUNT_8 + UARTDEV_COUNT_9 + UARTDEV_COUNT_10 + LPUARTDEV_COUNT_1)

typedef struct uartPinDef_s {
    ioTag_t pin;
} uartPinDef_t;

typedef struct uartHardware_s {
    UARTDevice_e device;    // XXX Not required for full allocation
    USART_TypeDef* reg;

    dmaResource_t *txDMAResource;
    dmaResource_t *rxDMAResource;
    // For H7 and G4, {tx|rx}DMAChannel are DMAMUX input index for  peripherals (DMA_REQUEST_xxx); H7:RM0433 Table 110, G4:RM0440 Table 80.
    // For F4 and F7, these are 32-bit channel identifiers (DMA_CHANNEL_x).
    uint32_t txDMAChannel;
    uint32_t rxDMAChannel;

    uartPinDef_t rxPins[UARTHARDWARE_MAX_PINS];
    uartPinDef_t txPins[UARTHARDWARE_MAX_PINS];

    uint8_t rcc;

    uint8_t af;
    uint8_t irqn;
    uint8_t txPriority;
    uint8_t rxPriority;

    volatile uint8_t *txBuffer;
    volatile uint8_t *rxBuffer;
    uint16_t txBufferSize;
    uint16_t rxBufferSize;
} uartHardware_t;

extern const uartHardware_t uartHardware[];

// uartDevice_t is an actual device instance.
// XXX Instances are allocated for uarts defined by USE_UARTx atm.

typedef struct uartDevice_s {
    uartPort_t port;
    const uartHardware_t *hardware;
    uartPinDef_t rx;
    uartPinDef_t tx;
    volatile uint8_t *rxBuffer;
    volatile uint8_t *txBuffer;
} uartDevice_t;

extern uartDevice_t *uartDevmap[];

extern const struct serialPortVTable uartVTable[];

void uartTryStartTxDMA(uartPort_t *s);

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options);

void uartIrqHandler(uartPort_t *s);

void uartReconfigure(uartPort_t *uartPort);

void uartConfigureDma(uartDevice_t *uartdev);

void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor);

#define UART_REG_RXD(base) ((base)->DR)
#define UART_REG_TXD(base) ((base)->DR)

#define UART_BUFFER(type, n, rxtx) type volatile uint8_t uart ## n ## rxtx ## xBuffer[UART_ ## rxtx ## X_BUFFER_SIZE]

#define UART_BUFFERS_EXTERN(n) \
    UART_BUFFER(extern, n, R); \
    UART_BUFFER(extern, n, T); struct dummy_s

#define LPUART_BUFFER(type, n, rxtx) type volatile uint8_t lpuart ## n ## rxtx ## xBuffer[UART_ ## rxtx ## X_BUFFER_SIZE]

#define LPUART_BUFFERS_EXTERN(n) \
    LPUART_BUFFER(extern, n, R); \
    LPUART_BUFFER(extern, n, T); struct dummy_s

UART_BUFFERS_EXTERN(1);
UART_BUFFERS_EXTERN(2);
UART_BUFFERS_EXTERN(3);
UART_BUFFERS_EXTERN(4);
UART_BUFFERS_EXTERN(5);
UART_BUFFERS_EXTERN(6);
LPUART_BUFFERS_EXTERN(1);

#undef UART_BUFFERS_EXTERN
