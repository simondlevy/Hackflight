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

#include <time.h>

#include "io_types.h"

#define ADC_INSTANCE                ADC1

#define ADC1_DMA_STREAM DMA2_Stream4 // ST0 or ST4
#define ADC2_DMA_STREAM DMA2_Stream3 // ST2 or ST3
#define ADC3_DMA_STREAM DMA2_Stream0 // ST0 or ST1

typedef enum ADCDevice {
    ADCINVALID = -1,
    ADCDEV_1   = 0,
    ADCDEV_2,
    ADCDEV_3,
    ADCDEV_COUNT
} ADCDevice;
