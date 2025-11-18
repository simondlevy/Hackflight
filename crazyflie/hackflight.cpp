/**
 * Copyright (C) 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <hackflight.hpp>

static const uint8_t CS_PIN = PB4;

SPIClass * Hackflight::spi_device()
{
    static SPIClass _spi;

    _spi.setSCLK(PA5);
    _spi.setMISO(PA6);
    _spi.setMOSI(PA7);

    _spi.begin();

    return &_spi;
}

const uint8_t Hackflight::spi_cs_pin()
{
    return PB4;
}

const uint8_t Hackflight::led_pin()
{
    return PC0;
}

const bool Hackflight::led_inverted()
{
    return true;
}
        
TwoWire * Hackflight::wire_device()
{
    return &Wire;
}
