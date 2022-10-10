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

#include <stdint.h>

#include "platform.h"
#include "adc.h"
#include "dma.h"
#include "dma_impl.h"
#include "dma_reqmap.h"
#include "nvic.h"
#include "resource.h"
#include "serialdev.h"
#include "serial_uart.h"
#include "spi.h"
#include "timer_def.h"


typedef struct dmaPeripheralMapping_s {
    dmaPeripheral_e device;
    uint8_t index;
    dmaChannelSpec_t channelSpec[MAX_PERIPHERAL_DMA_OPTIONS];
} dmaPeripheralMapping_t;

typedef struct dmaTimerMapping_s {
    TIM_TypeDef *tim;
    uint8_t channel;
    dmaChannelSpec_t channelSpec[MAX_TIMER_DMA_OPTIONS];
} dmaTimerMapping_t;

#define DMA(d, s, c) { DMA_CODE(d, s, c), (dmaResource_t *)DMA ## d ## _Stream ## s, DMA_Channel_ ## c }

#define TC(chan) DEF_TIM_CHANNEL(CH_ ## chan)

static const dmaTimerMapping_t dmaTimerMapping[] = {
    // Generated from 'timer_def.h'
    { TIM1, TC(CH1), { DMA(2, 6, 0), DMA(2, 1, 6), DMA(2, 3, 6) } },
    { TIM1, TC(CH2), { DMA(2, 6, 0), DMA(2, 2, 6) } },
    { TIM1, TC(CH3), { DMA(2, 6, 0), DMA(2, 6, 6) } },
    { TIM1, TC(CH4), { DMA(2, 4, 6) } },

    { TIM2, TC(CH1), { DMA(1, 5, 3) } },
    { TIM2, TC(CH2), { DMA(1, 6, 3) } },
    { TIM2, TC(CH3), { DMA(1, 1, 3) } },
    { TIM2, TC(CH4), { DMA(1, 7, 3), DMA(1, 6, 3) } },

    { TIM3, TC(CH1), { DMA(1, 4, 5) } },
    { TIM3, TC(CH2), { DMA(1, 5, 5) } },
    { TIM3, TC(CH3), { DMA(1, 7, 5) } },
    { TIM3, TC(CH4), { DMA(1, 2, 5) } },

    { TIM4, TC(CH1), { DMA(1, 0, 2) } },
    { TIM4, TC(CH2), { DMA(1, 3, 2) } },
    { TIM4, TC(CH3), { DMA(1, 7, 2) } },

    { TIM5, TC(CH1), { DMA(1, 2, 6) } },
    { TIM5, TC(CH2), { DMA(1, 4, 6) } },
    { TIM5, TC(CH3), { DMA(1, 0, 6) } },
    { TIM5, TC(CH4), { DMA(1, 1, 6), DMA(1, 3, 6) } },

    { TIM8, TC(CH1), { DMA(2, 2, 0), DMA(2, 2, 7) } },
    { TIM8, TC(CH2), { DMA(2, 2, 0), DMA(2, 3, 7) } },
    { TIM8, TC(CH3), { DMA(2, 2, 0), DMA(2, 4, 7) } },
    { TIM8, TC(CH4), { DMA(2, 7, 7) } },
};
#undef TC
#undef DMA

const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(TIM_TypeDef *tim, uint8_t channel, dmaoptValue_t dmaopt)
{
    if (dmaopt < 0 || dmaopt >= MAX_TIMER_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0 ; i < ARRAYLEN(dmaTimerMapping) ; i++) {
        const dmaTimerMapping_t *timerMapping = &dmaTimerMapping[i];
        if (timerMapping->tim == tim && timerMapping->channel == channel && timerMapping->channelSpec[dmaopt].ref) {
            return &timerMapping->channelSpec[dmaopt];
        }
    }

    return NULL;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimer(const timerHardware_t *timer)
{
    if (!timer) {
        return NULL;
    }

    dmaoptValue_t dmaopt = dmaoptByTag(timer->tag);
    return dmaGetChannelSpecByTimerValue(timer->tim, timer->channel, dmaopt);
}

// dmaGetOptionByTimer is called by pgResetFn_timerIOConfig to find out dmaopt for pre-configured timer.

dmaoptValue_t dmaGetOptionByTimer(const timerHardware_t *timer)
{
    for (unsigned i = 0 ; i < ARRAYLEN(dmaTimerMapping); i++) {
        const dmaTimerMapping_t *timerMapping = &dmaTimerMapping[i];
        if (timerMapping->tim == timer->tim && timerMapping->channel == timer->channel) {
            for (unsigned j = 0; j < MAX_TIMER_DMA_OPTIONS; j++) {
                const dmaChannelSpec_t *dma = &timerMapping->channelSpec[j];
                if (dma->ref == timer->dmaRefConfigured && dma->channel == timer->dmaChannelConfigured) {
                    return j;
                }
            }
        }
    }

    return DMA_OPT_UNUSED;
}

/*
 * DMA descriptors.
 */
dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER] = {
    DEFINE_DMA_CHANNEL(DMA1, 0,  0),
    DEFINE_DMA_CHANNEL(DMA1, 1,  6),
    DEFINE_DMA_CHANNEL(DMA1, 2, 16),
    DEFINE_DMA_CHANNEL(DMA1, 3, 22),
    DEFINE_DMA_CHANNEL(DMA1, 4, 32),
    DEFINE_DMA_CHANNEL(DMA1, 5, 38),
    DEFINE_DMA_CHANNEL(DMA1, 6, 48),
    DEFINE_DMA_CHANNEL(DMA1, 7, 54),

    DEFINE_DMA_CHANNEL(DMA2, 0,  0),
    DEFINE_DMA_CHANNEL(DMA2, 1,  6),
    DEFINE_DMA_CHANNEL(DMA2, 2, 16),
    DEFINE_DMA_CHANNEL(DMA2, 3, 22),
    DEFINE_DMA_CHANNEL(DMA2, 4, 32),
    DEFINE_DMA_CHANNEL(DMA2, 5, 38),
    DEFINE_DMA_CHANNEL(DMA2, 6, 48),
    DEFINE_DMA_CHANNEL(DMA2, 7, 54),
};

/*
 * DMA IRQ Handlers
 */
DEFINE_DMA_IRQ_HANDLER(1, 0, DMA1_ST0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 1, DMA1_ST1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 2, DMA1_ST2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 3, DMA1_ST3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 4, DMA1_ST4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 5, DMA1_ST5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 6, DMA1_ST6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 7, DMA1_ST7_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 0, DMA2_ST0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 1, DMA2_ST1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 2, DMA2_ST2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 3, DMA2_ST3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 4, DMA2_ST4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 5, DMA2_ST5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 6, DMA2_ST6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(2, 7, DMA2_ST7_HANDLER)

#define DMA_RCC(x) ((x) == DMA1 ? RCC_AHB1Periph_DMA1 : RCC_AHB1Periph_DMA2)
void dmaEnable(dmaIdentifier_e identifier)
{
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    RCC_AHB1PeriphClockCmd(DMA_RCC(dmaDescriptors[index].dma), ENABLE);
}

#define RETURN_TCIF_FLAG(s, n) if (s == DMA1_Stream ## n || s == DMA2_Stream ## n) return DMA_IT_TCIF ## n

uint32_t dmaFlag_IT_TCIF(const dmaResource_t *stream)
{
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 0);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 1);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 2);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 3);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 4);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 5);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 6);
    RETURN_TCIF_FLAG((DMA_ARCH_TYPE *)stream, 7);
    return 0;
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);

    RCC_AHB1PeriphClockCmd(DMA_RCC(dmaDescriptors[index].dma), ENABLE);
    dmaDescriptors[index].irqHandlerCallback = callback;
    dmaDescriptors[index].userParam = userParam;
    dmaDescriptors[index].completeFlag = dmaFlag_IT_TCIF(dmaDescriptors[index].ref);

    NVIC_InitStructure.NVIC_IRQChannel = dmaDescriptors[index].irqN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(priority);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(priority);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

dmaIdentifier_e dmaAllocate(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex)
{
    if (dmaGetOwner(identifier)->owner != OWNER_FREE) {
        return DMA_NONE;
    }

    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    dmaDescriptors[index].owner.owner = owner;
    dmaDescriptors[index].owner.resourceIndex = resourceIndex;

    return identifier;
}

const resourceOwner_t *dmaGetOwner(dmaIdentifier_e identifier)
{
    return &dmaDescriptors[DMA_IDENTIFIER_TO_INDEX(identifier)].owner;
}

dmaIdentifier_e dmaGetIdentifier(const dmaResource_t* channel)
{
    for (int i = 0; i < DMA_LAST_HANDLER; i++) {
        if (dmaDescriptors[i].ref == channel) {
            return i + 1;
        }
    }

    return 0;
}

dmaChannelDescriptor_t* dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier)
{
    return &dmaDescriptors[DMA_IDENTIFIER_TO_INDEX(identifier)];
}

uint32_t dmaGetChannel(const uint8_t channel)
{
    return ((uint32_t)channel*2)<<24;
}

