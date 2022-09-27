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

#if defined(__cplusplus)
extern "C" {
#endif

    void attachInterrupt(const uint8_t pint, void (*isr)(void));

    void delay(const uint16_t msec);

    void spiInit(const uint8_t sckPin, const uint8_t misoPin, const uint8_t mosiPin);

    void spiInitBusDMA(void);

    void spiWait(void *dev);

    void spiSequence(void *dev, busSegment_t *segments);

    void spiSetBusInstance(spiDevice_t *dev, const uint8_t csPin);

    void spiSetClkDivisor(void *dev, const uint16_t divider);

    void spiWriteReg(void *dev, const uint8_t reg, uint8_t data);

#if defined(__cplusplus)
}
#endif




