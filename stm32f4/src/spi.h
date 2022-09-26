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

typedef enum SPIDevice {
    SPIINVALID = -1,
    SPIDEV_1   = 0,
    SPIDEV_2,
    SPIDEV_3,
    SPIDEV_4,
    SPIDEV_5,
    SPIDEV_6
} SPIDevice;

#if defined(__cplusplus)
extern "C" {
#endif

void spiBusDeviceRegister(const extDevice_t *dev);

uint16_t spiCalculateDivider(const uint32_t freq);

void spiInit(const uint8_t mask);

void spiInitBusDMA(void);

uint8_t spiInstanceDenom(const SPI_TypeDef *instance);

void spi1PinConfigure(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin);

uint8_t spiReadReg(const extDevice_t *dev, const uint8_t reg);

void spiReadRegBuf(const extDevice_t *dev, const uint8_t reg, uint8_t *data,
        const uint8_t length);

bool spiReadRegBufRB(const extDevice_t *dev, const uint8_t reg, uint8_t *data,
        uint8_t length);

uint8_t spiReadRegMsk(const extDevice_t *dev, const uint8_t reg);

bool spiReadRegMskBufRB(const extDevice_t *dev, const uint8_t reg, uint8_t *data,
        const uint8_t length);

void spiWait(const extDevice_t *dev);

uint8_t spiReadWrite(const extDevice_t *dev, uint8_t data);

void spiReadWriteBuf(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, 
        const uint8_t len);

bool spiReadWriteBufRB(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, 
        const uint8_t length);

uint8_t spiReadWriteReg(const extDevice_t *dev, const uint8_t reg, const uint8_t data);

void spiSequence(const extDevice_t *dev, busSegment_t *segments);

bool spiSetBusInstance(extDevice_t *dev, const uint32_t device);

void spiSetClkDivisor(const extDevice_t *dev, const uint16_t divider);

void spiWrite(const extDevice_t *dev, uint8_t data);

void spiWriteReg(const extDevice_t *dev, const uint8_t reg, uint8_t data);

bool spiWriteRegRB(const extDevice_t *dev, const uint8_t reg, uint8_t data);

void spiWriteRegBuf(const extDevice_t *dev, const uint8_t reg, uint8_t *data,
        uint32_t length);

#if defined(__cplusplus)
}
#endif


