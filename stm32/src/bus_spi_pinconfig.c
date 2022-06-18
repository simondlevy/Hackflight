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

#include "platform.h"
#include "bus_spi.h"
#include "bus_spi_impl.h"
#include "dma.h"
#include "exti.h"
#include "io.h"
#include "rcc.h"

const spiHardware_t spiHardware[] = {
    {
        .device = SPIDEV_1,
        .reg = SPI1,
        .sckPins = {
            { DEFIO_TAG_E(PA5) },
            { DEFIO_TAG_E(PB3) },
        },
        .misoPins = {
            { DEFIO_TAG_E(PA6) },
            { DEFIO_TAG_E(PB4) },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PA7) },
            { DEFIO_TAG_E(PB5) },
        },
        .af = GPIO_AF_SPI1,
        .rcc = RCC_APB2(SPI1),
    },
    {
        .device = SPIDEV_2,
        .reg = SPI2,
        .sckPins = {
            { DEFIO_TAG_E(PB10) },
            { DEFIO_TAG_E(PB13) },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB14) },
            { DEFIO_TAG_E(PC2) },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB15) },
            { DEFIO_TAG_E(PC3) },
        },
        .af = GPIO_AF_SPI2,
        .rcc = RCC_APB1(SPI2),
    },
    {
        .device = SPIDEV_3,
        .reg = SPI3,
        .sckPins = {
            { DEFIO_TAG_E(PB3) },
            { DEFIO_TAG_E(PC10) },
        },
        .misoPins = {
            { DEFIO_TAG_E(PB4) },
            { DEFIO_TAG_E(PC11) },
        },
        .mosiPins = {
            { DEFIO_TAG_E(PB5) },
            { DEFIO_TAG_E(PC12) },
        },
        .af = GPIO_AF_SPI3,
        .rcc = RCC_APB1(SPI3),
    },
};

void spiPinConfigure(void)
{
    spiPinConfig_t spiPinConfig;

    spiPinConfig.ioTagSck = 21;
    spiPinConfig.ioTagMiso = 22;
    spiPinConfig.ioTagMosi = 23;
    spiPinConfig.txDmaopt = -1;
    spiPinConfig.rxDmaopt = -1;

    spiPinConfig_t * pConfig = &spiPinConfig;

    for (size_t hwindex = 0 ; hwindex < ARRAYLEN(spiHardware) ; hwindex++) {
        const spiHardware_t *hw = &spiHardware[hwindex];

        if (!hw->reg) {
            continue;
        }

        SPIDevice device = hw->device;
        spiDevice_t *pDev = &spiDevice[device];

        for (int pindex = 0 ; pindex < MAX_SPI_PIN_SEL ; pindex++) {
            if (pConfig[device].ioTagSck == hw->sckPins[pindex].pin) {
                pDev->sck = hw->sckPins[pindex].pin;
            }
            if (pConfig[device].ioTagMiso == hw->misoPins[pindex].pin) {
                pDev->miso = hw->misoPins[pindex].pin;
            }
            if (pConfig[device].ioTagMosi == hw->mosiPins[pindex].pin) {
                pDev->mosi = hw->mosiPins[pindex].pin;
            }
        }

        if (pDev->sck && pDev->miso && pDev->mosi) {
            pDev->dev = hw->reg;
            pDev->af = hw->af;
            pDev->rcc = hw->rcc;
            pDev->leadingEdge = false; // XXX Should be part of transfer context
            pDev->dmaIrqHandler = hw->dmaIrqHandler;
        }
    }
}
