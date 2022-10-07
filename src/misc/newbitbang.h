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
#include <math.h>
#include <string.h>

#include <time.h>

#include "timer.h"

#include "platform.h"
#include "atomic.h"
#include "io.h"
#include "io_impl.h"
#include "dma.h"
#include "dma_reqmap.h"
#include "nvic.h"
#include "timer.h"

class Bitbang {

    typedef enum {
        BITBANG_DIRECTION_OUTPUT, 
        BITBANG_DIRECTION_INPUT
    } bitbangDirection_e;

    typedef struct dmaRegCache_s {
        uint32_t CR;
        uint32_t FCR;
        uint32_t NDTR;
        uint32_t PAR;
        uint32_t M0AR;
    } dmaRegCache_t;

    // Per pacer timer
    typedef struct bbPacer_s {
        TIM_TypeDef *tim;
        uint16_t dmaSources;
    } bbPacer_t;

    // Per GPIO port and timer channel
    typedef struct bbPort_s {
        int portIndex;
        GPIO_TypeDef *gpio;
        const timerHardware_t *timhw;

        uint16_t dmaSource;

        dmaResource_t *dmaResource; // DMA resource for this port & timer channel
        uint32_t dmaChannel;        // DMA channel or peripheral request

        uint8_t direction;

        // DMA resource register cache
        dmaRegCache_t dmaRegOutput;
        dmaRegCache_t dmaRegInput;

        // For direct manipulation of GPIO_MODER register
        uint32_t gpioModeMask;
        uint32_t gpioModeInput;
        uint32_t gpioModeOutput;

        // Idle value
        uint32_t gpioIdleBSRR;

        TIM_TimeBaseInitTypeDef timeBaseInit;

        // Output
        uint16_t outputARR;
        DMA_InitTypeDef outputDmaInit;
        uint32_t *portOutputBuffer;
        uint32_t portOutputCount;

        // Input
        uint16_t inputARR;
        DMA_InitTypeDef inputDmaInit;
        uint16_t *portInputBuffer;
        uint32_t portInputCount;
        bool inputActive;

        // Misc
        uint32_t outputIrq;
        uint32_t inputIrq;
        resourceOwner_t owner;
    } bbPort_t;

}; // class Bitbang
