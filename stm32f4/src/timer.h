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
#include <stdint.h>

#include "dma.h"
#include "io_types.h"
#include "resource.h"
#include "timer_def.h"

#define CC_CHANNELS_PER_TIMER         4 // TIM_Channel_1..4
#define CC_INDEX_FROM_CHANNEL(x)      ((uint8_t)((x) >> 2))
#define CC_CHANNEL_FROM_INDEX(x)      ((uint16_t)(x) << 2)

typedef uint16_t captureCompare_t;        // 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)

typedef uint32_t timCCR_t;
typedef uint32_t timCCER_t;
typedef uint32_t timSR_t;
typedef uint32_t timCNT_t;

typedef enum {
    TIM_USE_ANY            = 0x0,
    TIM_USE_NONE           = 0x0,
    TIM_USE_PPM            = 0x1,
    TIM_USE_PWM            = 0x2,
    TIM_USE_MOTOR          = 0x4,
    TIM_USE_SERVO          = 0x8,
    TIM_USE_LED            = 0x10,
    TIM_USE_TRANSPONDER    = 0x20,
    TIM_USE_BEEPER         = 0x40,
    TIM_USE_CAMERA_CONTROL = 0x80,
} timerUsageFlag_e;

// use different types from capture and overflow - multiple overflow handlers are implemented as linked list
struct timerCCHandlerRec_s;
struct timerOvrHandlerRec_s;
typedef void timerCCHandlerCallback(struct timerCCHandlerRec_s* self, uint16_t capture);
typedef void timerOvrHandlerCallback(struct timerOvrHandlerRec_s* self, uint16_t capture);

typedef struct timerCCHandlerRec_s {
    timerCCHandlerCallback* fn;
} timerCCHandlerRec_t;

typedef struct timerOvrHandlerRec_s {
    timerOvrHandlerCallback* fn;
    struct timerOvrHandlerRec_s* next;
} timerOvrHandlerRec_t;

typedef struct timerDef_s {
    TIM_TypeDef *TIMx;
    uint8_t rcc;
    uint8_t inputIrq;
} timerDef_t;

typedef struct timerHardware_s {
    TIM_TypeDef *tim;
    ioTag_t tag;
    uint8_t channel;
    timerUsageFlag_e usageFlags;
    uint8_t output;
    uint8_t alternateFunction;
    dmaResource_t *dmaRefConfigured;
    uint32_t dmaChannelConfigured;
    dmaResource_t *dmaTimUPRef;
    uint32_t dmaTimUPChannel;
    uint8_t dmaTimUPIrqHandler;
} timerHardware_t;

typedef enum {
    TIMER_OUTPUT_NONE      = 0,
    TIMER_OUTPUT_INVERTED  = (1 << 0),
    TIMER_OUTPUT_N_CHANNEL = (1 << 1),
} timerFlag_e;

#define MHZ_TO_HZ(x) ((x) * 1000000)

#define TIMER_CHANNEL_COUNT 78

extern const timerHardware_t fullTimerHardware[];

#define TIMER_HARDWARE fullTimerHardware


typedef enum {
    TYPE_FREE,
    TYPE_PWMINPUT,
    TYPE_PPMINPUT,
    TYPE_PWMOUTPUT_MOTOR,
    TYPE_PWMOUTPUT_FAST,
    TYPE_PWMOUTPUT_SERVO,
    TYPE_SOFTSERIAL_RX,
    TYPE_SOFTSERIAL_TX,
    TYPE_SOFTSERIAL_RXTX,        // bidirectional pin for softserial
    TYPE_SOFTSERIAL_AUXTIMER,    // timer channel is used for softserial. No IO function on pin
    TYPE_ADC,
    TYPE_SERIAL_RX,
    TYPE_SERIAL_TX,
    TYPE_SERIAL_RXTX,
    TYPE_TIMER
} channelType_t;

//
// Legacy API
//
void timerConfigure(const timerHardware_t *timHw, uint16_t period, uint32_t hz);

//
// Initialisation
//
void timerInit(void);
void timerStart(void);

//
// per-channel
//

void timerChConfigIC(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterSamples);
void timerChConfigICDual(const timerHardware_t* timHw, bool polarityRising, unsigned inputFilterSamples);
void timerChICPolarity(const timerHardware_t *timHw, bool polarityRising);
volatile timCCR_t* timerChCCR(const timerHardware_t* timHw);
volatile timCCR_t* timerChCCRLo(const timerHardware_t* timHw);
volatile timCCR_t* timerChCCRHi(const timerHardware_t* timHw);
void timerChConfigOC(const timerHardware_t* timHw, bool outEnable, bool stateHigh);
void timerChConfigGPIO(const timerHardware_t* timHw, ioConfig_t mode);

void timerChCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn);
void timerChOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn);
void timerChConfigCallbacks(const timerHardware_t *channel, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback);
void timerChConfigCallbacksDual(const timerHardware_t *channel, timerCCHandlerRec_t *edgeCallbackLo, timerCCHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback);
void timerChITConfigDualLo(const timerHardware_t* timHw, FunctionalState newState);
void timerChITConfig(const timerHardware_t* timHw, FunctionalState newState);
void timerChClearCCFlag(const timerHardware_t* timHw);

void timerChInit(const timerHardware_t *timHw, channelType_t type, int irqPriority, uint8_t irq);

//
// per-timer
//

#if defined (__cplusplus)
extern "C" {
#endif

    void timerForceOverflow(TIM_TypeDef *tim);

    void timerConfigUpdateCallback(
            const TIM_TypeDef *tim, timerOvrHandlerRec_t *updateCallback);

    uint32_t timerClock(TIM_TypeDef *tim);

    void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint32_t hz);  

    void timerReconfigureTimeBase(TIM_TypeDef *tim, uint16_t period, uint32_t hz);

    uint8_t timerRCC(TIM_TypeDef *tim);
    uint8_t timerInputIrq(TIM_TypeDef *tim);

    extern const resourceOwner_t freeOwner;

    struct timerIOConfig_s;

    struct timerIOConfig_s *timerIoConfigByTag(ioTag_t ioTag);

    const timerHardware_t *timerGetConfiguredByTag(ioTag_t ioTag);

    const timerHardware_t *timerAllocate(
            ioTag_t ioTag, resourceOwner_e owner, uint8_t resourceIndex);

    const timerHardware_t *timerGetByTagAndIndex(ioTag_t ioTag, unsigned timerIndex);

    ioTag_t timerioTagGetByUsage(timerUsageFlag_e usageFlag, uint8_t index);

    void timerOCInit(TIM_TypeDef *tim, uint8_t channel, TIM_OCInitTypeDef *init);

    void timerOCPreloadConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t preload);

    volatile timCCR_t *timerCCR(TIM_TypeDef *tim, uint8_t channel);

    uint16_t timerDmaSource(uint8_t channel);

    uint16_t timerGetPrescalerByDesiredHertz(TIM_TypeDef *tim, uint32_t hz);

    uint16_t timerGetPrescalerByDesiredMhz(TIM_TypeDef *tim, uint16_t mhz);

    uint16_t timerGetPeriodByPrescaler(TIM_TypeDef *tim, uint16_t prescaler, uint32_t hz);

    int8_t timerGetNumberByIndex(uint8_t index);

    int8_t timerGetTIMNumber(const TIM_TypeDef *tim);

    uint8_t timerLookupChannelIndex(const uint16_t channel);

#if defined (__cplusplus)
}
#endif
