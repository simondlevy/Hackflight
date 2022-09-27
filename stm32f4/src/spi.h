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

#include <stdbool.h>

#include "platform.h"
#include "io_types.h"
#include "dma.h"

typedef enum {
    BUS_READY,
    BUS_BUSY,
    BUS_ABORT
} busStatus_e;

/* Each SPI access may comprise multiple parts, for example, wait/write
 * enable/write/data each of which is defined by a segment, with optional
 * callback after each is completed
 */
typedef struct {
    /* Note that txData may point to the transmit buffer, or in the case of the
     * final segment to a const spiDevice_t * structure to link to the next
     * transfer.
     */
    uint8_t *txData;
    /* Note that rxData may point to the receive buffer, or in the case of the
     * final segment to a busSegment_t * structure to link to the next
     * transfer.
     */
    uint8_t *rxData;
    int32_t len;
    bool negateCS; // Should CS be negated at the end of this segment
    busStatus_e (*callback)(uint32_t arg);
} busSegment_t;

// Bus interface, independent of connected device
typedef struct {
    SPI_TypeDef *instance;
    uint16_t speed;
    bool leadingEdge;
    bool useDMA;
    uint8_t deviceCount;
    dmaChannelDescriptor_t *dmaTx;
    dmaChannelDescriptor_t *dmaRx;
    // Use a reference here as this saves RAM for unused descriptors
    DMA_InitTypeDef  *initTx;
    DMA_InitTypeDef  *initRx;

    busSegment_t * volatile curSegment;
    bool initSegment;
} busDevice_t;

// External device has an associated bus and bus dependent address
typedef struct {
    busDevice_t *bus;
    uint16_t speed;
    IO_t csnPin;
    bool leadingEdge;
    // Cache the init structure for the next DMA transfer to reduce inter-segment delay
    DMA_InitTypeDef initTx;
    DMA_InitTypeDef initRx;
    // Support disabling DMA on a per device basis
    bool useDMA;
    // Per device buffer reference if needed
    uint8_t *txBuf, *rxBuf;
    // Connected devices on the same bus may support different speeds
    uint32_t callbackArg;
} spiDevice_t;

#if defined(__cplusplus)
extern "C" {
#endif

    void spiInit(const uint8_t sckPin, const uint8_t misoPin, const uint8_t mosiPin);

    void spiInitBusDMA(void);

    void spiWait(const spiDevice_t *dev);

    void spiSequence(const spiDevice_t *dev, busSegment_t *segments);

    void spiSetBusInstance(spiDevice_t *dev, const uint8_t csPin);

    void spiSetClkDivisor(const spiDevice_t *dev, const uint16_t divider);

    void spiWriteReg(const spiDevice_t *dev, const uint8_t reg, uint8_t data);

#if defined(__cplusplus)
}
#endif

// Platform-dependent
uint8_t spiInstanceDenom(const SPI_TypeDef *instance);




