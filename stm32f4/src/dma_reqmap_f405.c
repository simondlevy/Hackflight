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
#include "spi.h"
#include "serialdev.h"
#include "serial_uart.h"
#include "timer_def.h"

#include "dma_reqmap.h"

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

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {

    // Everything including F405 and F446
    { DMA_PERIPH_SPI_MOSI,  SPIDEV_1,  { DMA(2, 3, 3), DMA(2, 5, 3) } },
    { DMA_PERIPH_SPI_MISO,  SPIDEV_1,  { DMA(2, 0, 3), DMA(2, 2, 3) } },
    { DMA_PERIPH_SPI_MOSI,  SPIDEV_2,  { DMA(1, 4, 0) } },
    { DMA_PERIPH_SPI_MISO,  SPIDEV_2,  { DMA(1, 3, 0) } },
    { DMA_PERIPH_SPI_MOSI,  SPIDEV_3,  { DMA(1, 5, 0), DMA(1, 7, 0) } },
    { DMA_PERIPH_SPI_MISO,  SPIDEV_3,  { DMA(1, 0, 0), DMA(1, 2, 0) } },

#if defined(STM32F411xE)
    { DMA_PERIPH_SPI_MOSI,  SPIDEV_4,  { DMA(2, 1, 4), DMA(2, 4, 5) } },
    { DMA_PERIPH_SPI_MISO,  SPIDEV_4,  { DMA(2, 0, 4), DMA(2, 3, 5) } },

#endif

    { DMA_PERIPH_ADC,     ADCDEV_1,  { DMA(2, 0, 0), DMA(2, 4, 0) } },
    { DMA_PERIPH_ADC,     ADCDEV_2,  { DMA(2, 2, 1), DMA(2, 3, 1) } },
    { DMA_PERIPH_ADC,     ADCDEV_3,  { DMA(2, 0, 2), DMA(2, 1, 2) } },

    { DMA_PERIPH_UART_TX, UARTDEV_1, { DMA(2, 7, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_1, { DMA(2, 5, 4), DMA(2, 2, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_2, { DMA(1, 6, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_2, { DMA(1, 5, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_3, { DMA(1, 3, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_3, { DMA(1, 1, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_4, { DMA(1, 4, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_4, { DMA(1, 2, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_5, { DMA(1, 7, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_5, { DMA(1, 0, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_6, { DMA(2, 6, 5), DMA(2, 7, 5) } },
    { DMA_PERIPH_UART_RX, UARTDEV_6, { DMA(2, 1, 5), DMA(2, 2, 5) } },
};

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

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    if (opt < 0 || opt >= MAX_PERIPHERAL_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0 ; i < ARRAYLEN(dmaPeripheralMapping) ; i++) {
        const dmaPeripheralMapping_t *periph = &dmaPeripheralMapping[i];
        if (periph->device == device && periph->index == index && periph->channelSpec[opt].ref) {
            return &periph->channelSpec[opt];
        }
    }

    return NULL;
}

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
