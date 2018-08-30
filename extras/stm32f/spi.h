/*
   SPI support for STM32F3 boards

   Copyright (C) 2018 Simon D. Levy 

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// Here we put code that interacts with Cleanflight
extern "C" {

#include "platform.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/time.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "pg/bus_spi.h"
#include "io/serial.h"
#include "target.h"
#include "stm32f30x.h"

    void spi_init(SPI_TypeDef * instance, IO_t pin);
} 
