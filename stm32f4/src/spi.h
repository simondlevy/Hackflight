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

#include <stdint.h>
#include <stdbool.h>

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
} busSegment_t;


#if defined(__cplusplus)
extern "C" {
#endif

    void spiInit(const uint8_t sckPin, const uint8_t misoPin, const uint8_t mosiPin);

    void spiWait(const void * dev);

    void spiSequence(const void * dev, busSegment_t *segments);

    void * spiGetInstance(const uint8_t csPin);

    void spiSetClkDivisor(const void * dev, const uint16_t divider);

    void spiWriteReg(const void * dev, const uint8_t reg, uint8_t data);

    uint8_t * spiGetRxBuf(const void * dev);
    uint8_t * spiGetTxBuf(const void * dev);

    void spiSetRxBuf(void * dev, uint8_t * buf);
    void spiSetTxBuf(void * dev, uint8_t * buf);

#if defined(__cplusplus)
}
#endif
