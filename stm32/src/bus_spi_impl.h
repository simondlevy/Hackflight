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

#define SPI_TIMEOUT_US  10000

#define MAX_SPI_PIN_SEL 2

#define BUS_SPI_FREE   0x0
#define BUS_SPI_LOCKED 0x4

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
