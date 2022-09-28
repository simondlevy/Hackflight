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

#include <core/constrain.h>

#include "platform.h"

#include "atomic.h"
#include "exti.h"
#include "io.h"
#include "rcc.h"
#include "nvic.h"
#include "resource.h"
#include "spi.h"
#include "systemdev.h"

// Platform-dependent
uint8_t spiInstanceDenom(const SPI_TypeDef *instance);

// Bus interface, independent of connected device
typedef struct {
    SPI_TypeDef *instance;
    uint16_t speed;
    bool leadingEdge;
    uint8_t deviceCount;
    busSegment_t * volatile curSegment;
    bool initSegment;
} busDevice_t;

// External device has an associated bus and bus dependent address
typedef struct {
    busDevice_t *bus;
    uint16_t speed;
    IO_t csnPin;
    bool leadingEdge;
    uint8_t *txBuf, *rxBuf;
    uint32_t callbackArg;
} spiDevice_t;

static const uint32_t BUS_SPI_FREE   = 0x00000000;
static const uint32_t BUS_SPI_LOCKED = 0x00000004;

typedef struct {
    SPI_TypeDef *dev;
    ioTag_t sck;
    ioTag_t miso;
    ioTag_t mosi;
    uint8_t af;
    uint8_t rcc;
    volatile uint16_t errorCount;
    bool leadingEdge;
    uint8_t dmaIrqHandler;
} spiInfo_t;

static spiInfo_t  m_spiInfo;
static busDevice_t  m_spiBusDevice;

// STM32F405 can't DMA to/from FASTRAM (CCM SRAM)
static bool isCcm(const uint8_t * p)
{
    return (((uint32_t)p & 0xffff0000) == 0x10000000);
}

static SPI_InitTypeDef defaultInit = {
    .SPI_Mode = SPI_Mode_Master,
    .SPI_Direction = SPI_Direction_2Lines_FullDuplex,
    .SPI_DataSize = SPI_DataSize_8b,
    .SPI_NSS = SPI_NSS_Soft,
    .SPI_FirstBit = SPI_FirstBit_MSB,
    .SPI_CRCPolynomial = 7,
    .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8,
    .SPI_CPOL = SPI_CPOL_High,
    .SPI_CPHA = SPI_CPHA_2Edge
};

// Return true if DMA engine is busy
static bool spiIsBusy(const spiDevice_t *dev)
{
    return (dev->bus->curSegment != (busSegment_t *)BUS_SPI_FREE);
}

static uint16_t spiDivisorToBRbits(const SPI_TypeDef *instance, uint16_t divisor)
{
    // SPI2 and SPI3 are on APB1/AHB1 which PCLK is half that of APB2/AHB2.
    divisor /= spiInstanceDenom(instance);

    divisor = constrain_u16(divisor, 2, 256);

    return (ffs(divisor) - 2) << 3; // SPI_CR1_BR_Pos
}

static void spiSetDivisorBRreg(SPI_TypeDef *instance, uint16_t divisor)
{
    const uint16_t BR_BITS = 1<<5 | 1<<4 | 1<<3;
    const uint16_t tempRegister = (instance->CR1 & ~BR_BITS);
    instance->CR1 = tempRegister | spiDivisorToBRbits(instance, divisor);
}

static bool spiInternalReadWriteBufPolled(
        SPI_TypeDef *instance,
        const uint8_t *txData,
        uint8_t *rxData,
        uint32_t len)
{
    uint8_t b;

    while (len--) {
        b = txData ? *(txData++) : 0xFF;
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(instance, b);

        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET);
        b = SPI_I2S_ReceiveData(instance);
        if (rxData) {
            *(rxData++) = b;
        }
    }

    return true;
}

static void spiSequenceStart(const spiDevice_t *dev, busSegment_t *segments)
{
    busDevice_t *bus = dev->bus;
    SPI_TypeDef *instance = bus->instance;
    uint32_t xferLen = 0;
    uint32_t segmentCount = 0;

    dev->bus->initSegment = true;
    dev->bus->curSegment = segments;

    SPI_Cmd(instance, DISABLE);

    // Switch bus speed
    if (dev->speed != bus->speed) {
        spiSetDivisorBRreg(bus->instance, dev->speed);
        bus->speed = dev->speed;
    }

    if (dev->leadingEdge != bus->leadingEdge) {
        // Switch SPI clock polarity/phase
        instance->CR1 &= ~(SPI_CPOL_High | SPI_CPHA_2Edge);

        // Apply setting
        if (dev->leadingEdge) {
            instance->CR1 |= SPI_CPOL_Low | SPI_CPHA_1Edge;
        } else
        {
            instance->CR1 |= SPI_CPOL_High | SPI_CPHA_2Edge;
        }
        bus->leadingEdge = dev->leadingEdge;
    }

    SPI_Cmd(instance, ENABLE);

    for (busSegment_t *checkSegment = bus->curSegment;
            checkSegment->len; checkSegment++) {
        // Check there is no receive data as only transmit DMA is available
        if (((checkSegment->rxData) && (isCcm(checkSegment->rxData))) ||
            ((checkSegment->txData) && isCcm(checkSegment->txData))) {
            break;
        }

        segmentCount++;
        xferLen += checkSegment->len;
    }

    // Manually work through the segment list performing a transfer for each
    while (bus->curSegment->len) {
        // Assert Chip Select
        IOLo(dev->csnPin);

        spiInternalReadWriteBufPolled(
                bus->instance,
                bus->curSegment->txData,
                bus->curSegment->rxData,
                bus->curSegment->len);

        if (bus->curSegment->negateCS) {
            // Negate Chip Select
            IOHi(dev->csnPin);
        }

        bus->curSegment++;
    }

    // If a following transaction has been linked, start it
    if (bus->curSegment->txData) {
        const spiDevice_t *nextDev = (const spiDevice_t *)bus->curSegment->txData;
        busSegment_t *nextSegments = (busSegment_t *)bus->curSegment->rxData;
        bus->curSegment->txData = NULL;
        spiSequenceStart(nextDev, nextSegments);
    } else {
        // The end of the segment list has been reached, so mark
        // transactions as complete
        bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
    }
}

// ----------------------------------------------------------------------------

void spiInit(const uint8_t sckPin, const uint8_t misoPin, const uint8_t mosiPin)
{
    spiInfo_t *pDev = &m_spiInfo;

    pDev->sck = sckPin;
    pDev->miso = misoPin;
    pDev->mosi = mosiPin;
    pDev->dev = SPI1;
    pDev->af = GPIO_AF_SPI1;
    pDev->rcc = RCC_APB2(SPI1);
    pDev->leadingEdge = false; 

    const uint8_t device = 0;

    spiInfo_t *spi = &m_spiInfo;

    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);

    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK,  RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_MISO, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_MOSI, RESOURCE_INDEX(device));

    IOConfigGPIOAF(
            IOGetByTag(spi->sck), 
            IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_DOWN), 
            spi->af);

    IOConfigGPIOAF(
            IOGetByTag(spi->miso),
            IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP), 
            spi->af);

    IOConfigGPIOAF(
            IOGetByTag(spi->mosi),
            IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL),
            spi->af);

    // Init SPI hardware
    SPI_I2S_DeInit(spi->dev);

    SPI_I2S_DMACmd(spi->dev, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
    SPI_Init(spi->dev, &defaultInit);
    SPI_Cmd(spi->dev, ENABLE);
}

// Wait for completion
static void _spiWait(const spiDevice_t *dev)
{
    // Wait for completion
    while (dev->bus->curSegment != (busSegment_t *)BUS_SPI_FREE);
}

static void _spiSequence(const spiDevice_t *dev, busSegment_t *segments)
{
    busDevice_t *bus = dev->bus;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if ((bus->curSegment != (busSegment_t *)BUS_SPI_LOCKED) && spiIsBusy(dev)) {
            // Defer this transfer to be triggered upon completion of the
            // current transfer. Blocking calls and those from non-interrupt
            // context will have already called spiWait() so this will
            // only happen for non-blocking calls called from an ISR.
            busSegment_t *endSegment = bus->curSegment;

            if (endSegment) {
                // Find the last segment of the current transfer
                for (; endSegment->len; endSegment++);

                // Record the dev and segments parameters in the terminating
                // segment entry
                endSegment->txData = (uint8_t *)dev;
                endSegment->rxData = (uint8_t *)segments;

                return;
            }
        }
    }

    spiSequenceStart(dev, segments);
}


// Write data to a register
static void _spiWriteReg(const spiDevice_t *dev, const uint8_t reg, uint8_t data)
{
    uint8_t regg = reg;

    // This routine blocks so no need to use static data
    busSegment_t segments[] = {
            {&regg, NULL, 1, false},
            {&data, NULL, 1, true},
            {NULL, NULL, 0, true},
    };

    _spiWait(dev);

    _spiSequence(dev, segments);

    _spiWait(dev);
}

// Mark this bus as being SPI and record the first owner to use it
static void _spiSetBusInstance(spiDevice_t *dev, const uint8_t csPin)
{
    static busDevice_t busDevice;
    dev->bus = &busDevice;

    // MPU datasheet specifies 30ms.
    delay(35);

    dev->bus = &m_spiBusDevice;

    busDevice_t *bus = dev->bus;

    bus->instance = m_spiInfo.dev;

    bus->deviceCount = 1;

    dev->csnPin = IOGetByTag(csPin);

    IOInit(dev->csnPin, OWNER_GYRO_CS, RESOURCE_INDEX(0));

    IOConfigGPIO(dev->csnPin, 
            IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL));

    // Ensure device is disabled, important when two devices are on the same bus.
    IOHi(dev->csnPin); 

    IOInit(dev->csnPin, OWNER_GYRO_CS, RESOURCE_INDEX(0));

    IOConfigGPIO(dev->csnPin, 
            IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL));

    // Ensure device is disabled, important when two devices are on the same bus.
    IOHi(dev->csnPin); 
}

static void _spiSetClkDivisor(const spiDevice_t *dev, const uint16_t divisor)
{
    ((spiDevice_t *)dev)->speed = divisor;
}

// ------------------------------------------------------------------------------------

void spiWait(const void * dev)
{
    _spiWait((spiDevice_t *)dev);
}

void spiSequence(const void * dev, busSegment_t *segments)
{
    _spiSequence((spiDevice_t *)dev, segments);
}

void spiSetBusInstance(void * dev, const uint8_t csPin)
{
    _spiSetBusInstance((spiDevice_t *)dev, csPin);
}

void spiSetClkDivisor(const void * dev, const uint16_t divider)
{
    _spiSetClkDivisor((spiDevice_t *)dev, divider);
}

void spiWriteReg(const void * dev, const uint8_t reg, uint8_t data)
{
    _spiWriteReg((spiDevice_t *)dev, reg, data);
}

uint8_t * spiGetRxBuf(const void * dev)
{
    return ((spiDevice_t *)dev)->rxBuf;
}

uint8_t * spiGetTxBuf(const void * dev)
{
    return ((spiDevice_t *)dev)->txBuf;
}

void spiSetRxBuf(void * dev, uint8_t * buf)
{
    ((spiDevice_t *)dev)->rxBuf = buf;
}

void spiSetTxBuf(void * dev, uint8_t * buf)
{
    ((spiDevice_t *)dev)->txBuf = buf;
}

static spiDevice_t _spi1;

void * spiGetInstance(const uint8_t csPin)
{
    _spiSetBusInstance(&_spi1, csPin);

    return &_spi1;
}
