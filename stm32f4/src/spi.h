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

void spiInit(const uint8_t device);

void spiInitBusDMA(void);

void spi1PinConfigure(uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin);

void spiWait(const extDevice_t *dev);

void spiSequence(const extDevice_t *dev, busSegment_t *segments);

bool spiSetBusInstance(extDevice_t *dev, const uint32_t device);

void spiSetClkDivisor(const extDevice_t *dev, const uint16_t divider);

void spiWriteReg(const extDevice_t *dev, const uint8_t reg, uint8_t data);

// Platform-dependent
uint8_t spiInstanceDenom(const SPI_TypeDef *instance);

#if defined(__cplusplus)
}
#endif


