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

#include "platform.h"

#include "bus.h"
#include "spi.h"


// Access routines where the register is accessed directly
bool busRawWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    return spiWriteRegRB(dev, reg, data);
}

bool busRawWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    return spiWriteRegRB(dev, reg, data);
}

bool busRawReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    return spiReadRegBufRB(dev, reg, data, length);
}

bool busRawReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    return spiReadRegBufRB(dev, reg, data, length);
}

// Write routines where the register is masked with 0x7f
bool busWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    return spiWriteRegRB(dev, reg & 0x7f, data);
}

bool busWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    return spiWriteRegRB(dev, reg & 0x7f, data);
}

// Read routines where the register is ORed with 0x80
bool busReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    return spiReadRegMskBufRB(dev, reg | 0x80, data, length);
}

// Start the I2C read, but do not wait for completion
bool busReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    return spiReadRegMskBufRB(dev, reg | 0x80, data, length);
}

uint8_t busReadRegister(const extDevice_t *dev, uint8_t reg)
{
    uint8_t data;
    busReadRegisterBuffer(dev, reg, &data, 1);
    return data;
}
