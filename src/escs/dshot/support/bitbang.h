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

#include <atomic.h>
#include <dma.h>
#include <io.h>
#include <io_impl.h>
#include <nvic.h>

class Bitbang {

    private:

    typedef struct dmaRegCache_s {
        uint32_t CR;
        uint32_t FCR;
        uint32_t NDTR;
        uint32_t PAR;
        uint32_t M0AR;
    } dmaRegCache_t;

    static void loadDmaRegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
    {
        ((DMA_Stream_TypeDef *)dmaResource)->CR = dmaRegCache->CR;
        ((DMA_Stream_TypeDef *)dmaResource)->FCR = dmaRegCache->FCR;
        ((DMA_Stream_TypeDef *)dmaResource)->NDTR = dmaRegCache->NDTR;
        ((DMA_Stream_TypeDef *)dmaResource)->PAR = dmaRegCache->PAR;
        ((DMA_Stream_TypeDef *)dmaResource)->M0AR = dmaRegCache->M0AR;
    }

    static void saveDmaRegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
    {
        dmaRegCache->CR = ((DMA_Stream_TypeDef *)dmaResource)->CR;
        dmaRegCache->FCR = ((DMA_Stream_TypeDef *)dmaResource)->FCR;
        dmaRegCache->NDTR = ((DMA_Stream_TypeDef *)dmaResource)->NDTR;
        dmaRegCache->PAR = ((DMA_Stream_TypeDef *)dmaResource)->PAR;
        dmaRegCache->M0AR = ((DMA_Stream_TypeDef *)dmaResource)->M0AR;
    }

    public:

    // Per pacer timer
    typedef struct bbPacer_s {
        TIM_TypeDef *tim;
        uint16_t dmaSources;
    } bbPacer_t;

    // Per GPIO port and timer channel
    typedef struct port_s {
        int portIndex;
        GPIO_TypeDef *gpio;
        const timerHardware_t *timhw;

        uint16_t dmaSource;

        dmaResource_t *dmaResource; // DMA resource for this port & timer channel
        uint32_t dmaChannel;        // DMA channel or peripheral request

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
    } port_t;

    static void dmaCmd(port_t *bbPort, FunctionalState NewState)
    {
        xDMA_Cmd(bbPort->dmaResource, NewState);
    }

    static void dmaIrqHandler(dmaChannelDescriptor_t *descriptor)
    {
        port_t *bbPort = (port_t *)descriptor->userParam;

        dmaCmd(bbPort, DISABLE);

        timDmaCmd(bbPort->timhw->tim, bbPort->dmaSource, DISABLE);

        if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF)) {
            while (1) {};
        }

        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }

    static void dmaItConfig(port_t *bbPort)
    {
        xDMA_ITConfig(bbPort->dmaResource, DMA_IT_TC, ENABLE);
    }

    static void dmaPreconfigure(port_t *bbPort)
    {
        DMA_InitTypeDef * dmainit = &bbPort->outputDmaInit;

        DMA_StructInit(dmainit);

        dmainit->DMA_Mode = DMA_Mode_Normal;
        dmainit->DMA_Channel = bbPort->dmaChannel;
        dmainit->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dmainit->DMA_MemoryInc = DMA_MemoryInc_Enable;
        dmainit->DMA_FIFOMode = DMA_FIFOMode_Enable ;
        dmainit->DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
        dmainit->DMA_MemoryBurst = DMA_MemoryBurst_Single ;
        dmainit->DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        dmainit->DMA_Priority = DMA_Priority_High;
        dmainit->DMA_DIR = DMA_DIR_MemoryToPeripheral;

        dmainit->DMA_BufferSize = bbPort->portOutputCount;
        dmainit->DMA_PeripheralBaseAddr = (uint32_t)&bbPort->gpio->BSRRL;
        dmainit->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        dmainit->DMA_Memory0BaseAddr = (uint32_t)bbPort->portOutputBuffer;
        dmainit->DMA_MemoryDataSize = DMA_MemoryDataSize_Word;

        xDMA_Init(bbPort->dmaResource, dmainit);
        saveDmaRegs(bbPort->dmaResource, &bbPort->dmaRegOutput);
    }

    static void gpioSetup(port_t * bbPort, int pinIndex, IO_t io, uint8_t puPdMode)
    {
        bbPort->gpioModeMask |= (GPIO_MODER_MODER0 << (pinIndex * 2));
        bbPort->gpioModeInput |= (GPIO_Mode_IN << (pinIndex * 2));
        bbPort->gpioModeOutput |= (GPIO_Mode_OUT << (pinIndex * 2));

        bbPort->gpioIdleBSRR |= (1 << (pinIndex + 16));  // BR (higher half)

        IOWrite(io, 0);

        IOConfigGPIO(io,
                IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, puPdMode));
    }

    static void switchToOutput(port_t * bbPort)
    {
        // Output idle level before switching to output
        // Use BSRR register for this
        // Normal: Use BR (higher half)
        // Inverted: Use BS (lower half)

        WRITE_REG(bbPort->gpio->BSRRL, bbPort->gpioIdleBSRR);

        // Set GPIO to output
        ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
            MODIFY_REG(bbPort->gpio->MODER, bbPort->gpioModeMask, bbPort->gpioModeOutput);
        }

        // Reinitialize port group DMA for output

        dmaResource_t *dmaResource = bbPort->dmaResource;
        loadDmaRegs(dmaResource, &bbPort->dmaRegOutput);

        // Reinitialize pacer timer for output

        bbPort->timhw->tim->ARR = bbPort->outputARR;
    }

    static void timDmaCmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState)
    {
        TIM_DMACmd(TIMx, TIM_DMASource, NewState);
    }

    static void bbTIM_TimeBaseInit(port_t *bbPort, uint16_t period)
    {
        TIM_TimeBaseInitTypeDef *init = &bbPort->timeBaseInit;

        init->TIM_Prescaler = 0; // Feed raw timerClock
        init->TIM_ClockDivision = TIM_CKD_DIV1;
        init->TIM_CounterMode = TIM_CounterMode_Up;
        init->TIM_Period = period;
        TIM_TimeBaseInit(bbPort->timhw->tim, init);
        TIM_ARRPreloadConfig(bbPort->timhw->tim, ENABLE);
    }

    static void timerChannelInit(port_t *bbPort, resourceOwner_e owner)
    {
        const timerHardware_t *timhw = bbPort->timhw;

        TIM_OCInitTypeDef TIM_OCStruct;

        TIM_OCStructInit(&TIM_OCStruct);
        TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

        // Duty doesn't matter, but too value small would make monitor output invalid
        TIM_OCStruct.TIM_Pulse = 10; 

        TIM_Cmd(bbPort->timhw->tim, DISABLE);

        timerOCInit(timhw->tim, timhw->channel, &TIM_OCStruct);
        // timerOCPreloadConfig(timhw->tim, timhw->channel, TIM_OCPreload_Enable);

        if (timhw->tag) {
            IO_t io = IOGetByTag(timhw->tag);
            IOConfigGPIOAF(io, IOCFG_AF_PP, timhw->alternateFunction);
            IOInit(io, owner, 0);
            TIM_CtrlPWMOutputs(timhw->tim, ENABLE);
        }

        // Enable and keep it running

        TIM_Cmd(bbPort->timhw->tim, ENABLE);
    }

}; // class Bitbang
