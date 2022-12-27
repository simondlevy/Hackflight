/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include <core_rate.h>
#include <datatypes.h>
#include <hackflight_full.h>
#include <serial.h>

#include "hardware_init.h"

int main(void)
{
    void * motorDevice = hardwareInit(CORE_PERIOD());

    Hackflight::data_t hf = {};

    Hackflight::init(&hf, motorDevice, 37);

    while (true) {

        Hackflight::step(&hf);
    }

    return 0;
}

void HardFault_Handler(void)
{
}
