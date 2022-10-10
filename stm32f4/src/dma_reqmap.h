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

#include "platform.h"
#include "resource.h"
#include "timer.h"

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

    dmaoptValue_t dmaoptByTag(ioTag_t ioTag);

    const dmaChannelSpec_t * dmaGetChannelSpecByPeripheral(
            dmaPeripheral_e device, uint8_t index, int8_t opt);

    const dmaChannelSpec_t * dmaGetChannelSpecByTimerValue(
            TIM_TypeDef *tim, uint8_t channel, dmaoptValue_t dmaopt);

    const dmaChannelSpec_t * dmaGetChannelSpecByTimer(const struct timerHardware_s *timer);

    dmaoptValue_t dmaGetOptionByTimer(const struct timerHardware_s *timer);

    dmaoptValue_t dmaGetUpOptionByTimer(const struct timerHardware_s *timer);

    dmaIdentifier_e dmaAllocate(
            dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex);

    void dmaEnable(dmaIdentifier_e identifier);

    uint32_t dmaGetChannel(const uint8_t channel);

    dmaChannelDescriptor_t* dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier);

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


