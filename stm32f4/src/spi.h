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

#include "bus.h"
#include "io_types.h"
#include "bus.h"
#include "rcc_types.h"

#define SPI_TIMEOUT_US  10000

#define MAX_SPI_PIN_SEL 2

#define BUS_SPI_FREE   0x0
#define BUS_SPI_LOCKED 0x4

#define SPI_IO_AF_CFG      IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, \
        GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define SPI_IO_AF_SCK_CFG  IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, \
        GPIO_OType_PP, GPIO_PuPd_DOWN)
#define SPI_IO_AF_MISO_CFG IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, \
        GPIO_OType_PP, GPIO_PuPd_UP)
#define SPI_IO_CS_CFG      IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, \
        GPIO_OType_PP, GPIO_PuPd_NOPULL)

#define SPIDEV_COUNT 3

// Macros to convert between CLI bus number and SPIDevice.
#define SPI_CFG_TO_DEV(x)   ((x) - 1)
#define SPI_DEV_TO_CFG(x)   ((x) + 1)

// Work around different check routines in the libraries for different MCU types
#define CHECK_SPI_RX_DATA_AVAILABLE(instance) LL_SPI_IsActiveFlag_RXNE(instance)
#define SPI_RX_DATA_REGISTER(base) ((base)->DR)

typedef struct spiPinConfig_s {
    ioTag_t ioTagSck;
    ioTag_t ioTagMiso;
    ioTag_t ioTagMosi;
    int8_t txDmaopt;
    int8_t rxDmaopt;
} spiPinConfig_t;

// De facto standard mode
// See https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
//
// Mode CPOL CPHA
//  0    0    0
//  1    0    1
//  2    1    0
//  3    1    1
typedef enum {
    SPI_MODE0_POL_LOW_EDGE_1ST = 0,
    SPI_MODE1_POL_LOW_EDGE_2ND,
    SPI_MODE2_POL_HIGH_EDGE_1ST,
    SPI_MODE3_POL_HIGH_EDGE_2ND
} SPIMode_e;

typedef enum SPIDevice {
    SPIINVALID = -1,
    SPIDEV_1   = 0,
    SPIDEV_2,
    SPIDEV_3,
    SPIDEV_4,
    SPIDEV_5,
    SPIDEV_6
} SPIDevice;

typedef struct spiPinDef_s {
    ioTag_t pin;
} spiPinDef_t;

typedef struct spiHardware_s {
    SPIDevice device;
    SPI_TypeDef *reg;
    spiPinDef_t sckPins[MAX_SPI_PIN_SEL];
    spiPinDef_t misoPins[MAX_SPI_PIN_SEL];
    spiPinDef_t mosiPins[MAX_SPI_PIN_SEL];
    uint8_t af;
    rccPeriphTag_t rcc;
    uint8_t dmaIrqHandler;
} spiHardware_t;

extern const spiHardware_t spiHardware[];

typedef struct SPIDevice_s {
    SPI_TypeDef *dev;
    ioTag_t sck;
    ioTag_t miso;
    ioTag_t mosi;
    uint8_t af;
    rccPeriphTag_t rcc;
    volatile uint16_t errorCount;
    bool leadingEdge;
    uint8_t dmaIrqHandler;
} spiDevice_t;

extern spiDevice_t spiDevice[SPIDEV_COUNT];

void spiInitDevice(SPIDevice device);
void spiInternalInitStream(const extDevice_t *dev, bool preInit);
void spiInternalStartDMA(const extDevice_t *dev);
void spiInternalStopDMA (const extDevice_t *dev);
void spiInternalResetStream(dmaChannelDescriptor_t *descriptor);
void spiInternalResetDescriptors(busDevice_t *bus);
void spiSequenceStart(const extDevice_t *dev, busSegment_t *segments);

#if defined(__cplusplus)
extern "C" {
#endif

void spiPreinitRegister(ioTag_t iotag, uint8_t iocfg, uint8_t init);
void spiPreinitByIO(IO_t io);
void spiPreinitByTag(ioTag_t tag);

SPIDevice spiDeviceByInstance(SPI_TypeDef *instance);
SPI_TypeDef *spiInstanceByDevice(SPIDevice device);

// BusDevice API

// Mark a device's associated bus as being SPI
bool spiSetBusInstance(extDevice_t *dev, uint32_t device);
// Determine the divisor to use for a given bus frequency
uint16_t spiCalculateDivider(uint32_t freq);
// Set the clock divisor to be used for accesses by the given device
void spiSetClkDivisor(const extDevice_t *dev, uint16_t divider);
// Set the clock phase/polarity to be used for accesses by the given device
void spiSetClkPhasePolarity(const extDevice_t *dev, bool leadingEdge);
// Enable/disable DMA on a specific device. Enabled by default.
void spiDmaEnable(const extDevice_t *dev, bool enable);

// DMA transfer setup and start
void spiSequence(const extDevice_t *dev, busSegment_t *segments);
// Wait for DMA completion
void spiWait(const extDevice_t *dev);
// Indicate that the bus on which this device resides may initiate DMA transfers from interrupt context
void spiSetAtomicWait(const extDevice_t *dev);
// Wait for DMA completion and claim the bus driver - use this when waiting for a prior access to complete before starting a new one
void spiWaitClaim(const extDevice_t *dev);
// Return true if DMA engine is busy
bool spiIsBusy(const extDevice_t *dev);

/*
 * Routine naming convention is:
 *  spi[Read][Write][Reg][Msk][Buf][RB]
 *
 *      Read: Perform a read, returning the value read unless 'Buf' is specified
 *      Write Perform a write
 *      ReadWrite: Perform both a read and write, returning the value read unless 'Buf' is specified
 *      Reg: Register number 'reg' is written prior to the read being performed
 *      Msk: Register number is logically ORed with 0x80 as some devices indicate a read by accessing a register with bit 7 set
 *      Buf: Pass data of given length by reference
 *      RB:  Return false immediately if the bus is busy, otherwise complete the access and return true
 */
uint8_t spiReadReg(const extDevice_t *dev, uint8_t reg);
uint8_t spiReadRegMsk(const extDevice_t *dev, uint8_t reg);
void spiReadRegBuf(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
bool spiReadRegBufRB(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
bool spiReadRegMskBufRB(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);

void spiWrite(const extDevice_t *dev, uint8_t data);
void spiWriteReg(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool spiWriteRegRB(const extDevice_t *dev, uint8_t reg, uint8_t data);

uint8_t spiReadWrite(const extDevice_t *dev, uint8_t data);

void spiWriteRegBuf(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint32_t length);
uint8_t spiReadWriteReg(const extDevice_t *dev, uint8_t reg, uint8_t data);
void spiReadWriteBuf(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int len);
bool spiReadWriteBufRB(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int length);

//
// Config
//

void    spiBusDeviceRegister(const extDevice_t *dev);
uint8_t spiGetExtDeviceCount(const extDevice_t *dev);
uint8_t spiGetRegisteredDeviceCount(void);
void    spiInit(uint8_t mask);
void    spiInitBusDMA(void);
void    spiPinConfigure(void);
void    spiPreInit(void);
bool    spiUseDMA(const extDevice_t *dev);
bool    spiUseMOSI_DMA(const extDevice_t *dev);

#if defined(__cplusplus)
}
#endif


