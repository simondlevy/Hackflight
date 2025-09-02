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

#pragma once

#include <SPI.h>
#include <pmw3901.hpp>

#include <task_opticalflow.hpp>

static PMW3901 pmw3901;

static SPIClass spi;

bool OpticalFlowTask::device_init()
{
    spi.setSCLK(PA5);
    spi.setMISO(PA6);
    spi.setMOSI(PA7);

    spi.begin();

    return pmw3901.begin(PB4, spi);
}

void OpticalFlowTask::device_read(
        int16_t & deltaX, int16_t & deltaY, bool & gotMotion)
{
    pmw3901.readMotion(deltaX, deltaY, gotMotion);
}
