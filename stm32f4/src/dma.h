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

#include "io_types.h"
#include "platform.h"
#include "resource.h"

#define xDMA_Init(dmaResource, initStruct) DMA_Init((DMA_ARCH_TYPE *)(dmaResource), initStruct)
#define xDMA_DeInit(dmaResource) DMA_DeInit((DMA_ARCH_TYPE *)(dmaResource))
#define xDMA_Cmd(dmaResource, newState) DMA_Cmd((DMA_ARCH_TYPE *)(dmaResource), newState)
#define xDMA_ITConfig(dmaResource, flags, newState) DMA_ITConfig((DMA_ARCH_TYPE *)(dmaResource), flags, newState)
#define xDMA_GetCurrDataCounter(dmaResource) DMA_GetCurrDataCounter((DMA_ARCH_TYPE *)(dmaResource))
#define xDMA_SetCurrDataCounter(dmaResource, count) DMA_SetCurrDataCounter((DMA_ARCH_TYPE *)(dmaResource), count)
#define xDMA_GetFlagStatus(dmaResource, flags) DMA_GetFlagStatus((DMA_ARCH_TYPE *)(dmaResource), flags)
#define xDMA_ClearFlag(dmaResource, flags) DMA_ClearFlag((DMA_ARCH_TYPE *)(dmaResource), flags)
#define xDMA_MemoryTargetConfig(dmaResource, address, target) DMA_MemoryTargetConfig((DMA_ARCH_TYPE *)(dmaResource), address, target)


#define DMA_CLEAR_FLAG(d, flag) if (d->flagsShift > 31) d->dma->HIFCR = (flag << (d->flagsShift - 32)); else d->dma->LIFCR = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->flagsShift > 31 ? d->dma->HISR & (flag << (d->flagsShift - 32)): d->dma->LISR & (flag << d->flagsShift))


#define DMA_IT_TCIF         ((uint32_t)0x00000020)
#define DMA_IT_HTIF         ((uint32_t)0x00000010)
#define DMA_IT_TEIF         ((uint32_t)0x00000008)
#define DMA_IT_DMEIF        ((uint32_t)0x00000004)
#define DMA_IT_FEIF         ((uint32_t)0x00000001)

#define IS_DMA_ENABLED(reg) (((DMA_ARCH_TYPE *)(reg))->CR & DMA_SxCR_EN)
#define REG_NDTR NDTR

#define DMA_DEVICE_NO(x)    ((((x)-1) / 8) + 1)
#define DMA_DEVICE_INDEX(x) ((((x)-1) % 8))
#define DMA_OUTPUT_INDEX    0
#define DMA_OUTPUT_STRING   "DMA%d Stream %d:"
#define DMA_INPUT_STRING    "DMA%d_ST%d"

typedef enum {
    DMA_NONE = 0,
    DMA1_ST0_HANDLER = 1,
    DMA1_ST1_HANDLER,
    DMA1_ST2_HANDLER,
    DMA1_ST3_HANDLER,
    DMA1_ST4_HANDLER,
    DMA1_ST5_HANDLER,
    DMA1_ST6_HANDLER,
    DMA1_ST7_HANDLER,
    DMA2_ST0_HANDLER,
    DMA2_ST1_HANDLER,
    DMA2_ST2_HANDLER,
    DMA2_ST3_HANDLER,
    DMA2_ST4_HANDLER,
    DMA2_ST5_HANDLER,
    DMA2_ST6_HANDLER,
    DMA2_ST7_HANDLER,
    DMA_LAST_HANDLER = DMA2_ST7_HANDLER
} dmaIdentifier_e;
#define CACHE_LINE_SIZE 32
#define CACHE_LINE_MASK (CACHE_LINE_SIZE - 1)

// dmaResource_t is a opaque data type which represents a single DMA engine,
// called and implemented differently in different families of STM32s.
// The opaque data type provides uniform handling of the engine in source code.
// The engines are referenced by dmaResource_t through out the Betaflight code,
// and then converted back to DMA_ARCH_TYPE which is a native type for
// the particular MCU type when calling library functions.

typedef struct dmaResource_s dmaResource_t;

#define DMA_ARCH_TYPE DMA_Stream_TypeDef

struct dmaChannelDescriptor_s;
typedef void (*dmaCallbackHandlerFuncPtr)(struct dmaChannelDescriptor_s *channelDescriptor);

typedef struct dmaChannelDescriptor_s {
    DMA_TypeDef*                dma;
    dmaResource_t               *ref;
    uint8_t                     stream;
    uint32_t                    channel;
    dmaCallbackHandlerFuncPtr   irqHandlerCallback;
    uint8_t                     flagsShift;
    IRQn_Type                   irqN;
    uint32_t                    userParam;
    resourceOwner_t             owner;
    uint8_t                     resourceIndex;
    uint32_t                    completeFlag;
} dmaChannelDescriptor_t;

typedef uint16_t dmaCode_t;

typedef struct dmaChannelSpec_s {
    dmaCode_t             code;
    dmaResource_t         *ref;
    uint32_t              channel;
} dmaChannelSpec_t;

#define DMA_CODE(dma, stream, chanreq) ((dma << 12)|(stream << 8)|(chanreq << 0))
#define DMA_CODE_CONTROLLER(code) ((code >> 12) & 0xf)
#define DMA_CODE_STREAM(code) ((code >> 8) & 0xf)
#define DMA_CODE_CHANNEL(code) ((code >> 0) & 0xff)
#define DMA_CODE_REQUEST(code) DMA_CODE_CHANNEL(code)

typedef enum {
    DMA_PERIPH_SPI_MOSI,
    DMA_PERIPH_SPI_MISO,
    DMA_PERIPH_ADC,
    DMA_PERIPH_SDIO,
    DMA_PERIPH_UART_TX,
    DMA_PERIPH_UART_RX,
    DMA_PERIPH_TIMUP,
} dmaPeripheral_e;

typedef int8_t dmaoptValue_t;

#define DMA_OPT_UNUSED (-1)

#define MAX_PERIPHERAL_DMA_OPTIONS 2
#define MAX_TIMER_DMA_OPTIONS 3

struct timerHardware_s;

#if defined (__cplusplus)
extern "C" {
#endif

    void dmaInit(void);

    dmaoptValue_t dmaoptByTag(ioTag_t ioTag);

    const dmaChannelSpec_t * dmaGetChannelSpecByPeripheral(
            dmaPeripheral_e device, uint8_t index, int8_t opt);

    const dmaChannelSpec_t * dmaGetChannelSpecByTimerValue(
            TIM_TypeDef *tim, uint8_t channel, dmaoptValue_t dmaopt);

    dmaoptValue_t dmaGetOptionByTimer(const struct timerHardware_s *timer);

    dmaIdentifier_e dmaAllocate(
            dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex);

    void dmaEnable(dmaIdentifier_e identifier);

    dmaIdentifier_e dmaGetIdentifier(const dmaResource_t* channel);

    const resourceOwner_t *dmaGetOwner(dmaIdentifier_e identifier);

    void dmaSetHandler(
            dmaIdentifier_e identifier,
            dmaCallbackHandlerFuncPtr callback,
            uint32_t priority,
            uint32_t userParam);

#if defined (__cplusplus)
}
#endif


