/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy
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

#include <pmw3901.hpp>

#include <tasks/opticalflow.hpp>

static const uint8_t MISO_PIN = PA6;
static const uint8_t MOSI_PIN = PA7;
static const uint8_t SCLK_PIN = PA5;
static const uint8_t CS_PIN = PC5;

static SPIClass spi;

static PMW3901 pmw3901;

bool OpticalFlowTask::device_init()
{
    return true;

    spi.setSCLK(PA5);
    spi.setMISO(PA6);
    spi.setMOSI(PA7);

    spi.begin();

    return pmw3901.begin(CS_PIN, spi);
}

void OpticalFlowTask::device_read(
        int16_t & dx, int16_t & dy, bool &gotMotion)
{
    //pmw3901.readMotion(dx, dy, gotMotion);

}
