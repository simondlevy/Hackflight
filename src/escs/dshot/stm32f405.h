/*
   Copyright (c) 2022 Simon D. Levy

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

#include <string.h>

#include "escs/dshot.h"

#include "stm32f4xx.h"
#include "dma.h"

__attribute__( ( always_inline ) ) static inline void __set_BASEPRI_nb(uint32_t basePri)
{
    __ASM volatile ("\tMSR basepri, %0\n" : : "r" (basePri) );
}

static inline void __basepriRestoreMem(uint8_t *val)
{
    __set_BASEPRI(*val);
}

static inline uint8_t __basepriSetMemRetVal(uint8_t prio)
{
    __set_BASEPRI_MAX(prio);
    return 1;
}

#define ATOMIC_BLOCK(prio) \
    for ( uint8_t __basepri_save __attribute__ \
            ((__cleanup__ (__basepriRestoreMem), __unused__)) = __get_BASEPRI(), \
            __ToDo = __basepriSetMemRetVal(prio); __ToDo ; __ToDo = 0 )


class Stm32F405DshotEsc : public DshotEsc {

    private:

        // Constants ====================================================================

        static const uint32_t RCC_AHB1ENR_GPIOAEN_MSK = 0x00000001;
        static const uint32_t RCC_AHB1ENR_GPIOBEN_MSK = 0x00000002;
        static const uint32_t RCC_AHB1ENR_GPIOCEN_MSK = 0x00000004;
        static const uint32_t RCC_AHB1ENR_GPIODEN_MSK = 0x00000008;
        static const uint32_t RCC_AHB1ENR_GPIOEEN_MSK = 0x00000010;
        static const uint32_t RCC_AHB1ENR_GPIOFEN_MSK = 0x00000020;

        static const uint8_t GPIO_FAST_SPEED = 0x02;
        static const uint8_t GPIO_MODE_OUT   = 0x01;
        static const uint8_t GPIO_PUPD_UP    = 0x01;
        static const uint8_t GPIO_OTYPE_PP   = 0x00;

        static const uint32_t RCC_AHB1PERIPH_DMA2 = 0x00400000;

        static const uint32_t _MY_DMA_IT_TCIF0 = 0x10008020;
        static const uint32_t _MY_DMA_IT_TCIF1 = 0x10008800;
        static const uint32_t _MY_DMA_IT_TCIF2 = 0x10208000;
        static const uint32_t _MY_DMA_IT_TCIF3 = 0x18008000;
        static const uint32_t _MY_DMA_IT_TCIF4 = 0x20008020;
        static const uint32_t _MY_DMA_IT_TCIF5 = 0x20008800;
        static const uint32_t _MY_DMA_IT_TCIF6 = 0x20208000;
        static const uint32_t _MY_DMA_IT_TCIF7 = 0x28008000;

        static const uint8_t DMA_TIMER_MAPPING_COUNT = 6;

        static const uint32_t NVIC_PRIORITY_GROUPING = 0x500;

        static const uint8_t DEFIO_PORT_USED_COUNT = 6;

        static const uint8_t HARDWARE_TIMER_DEFINITION_COUNT = 14;

        static const uint8_t MAX_TIMER_DMA_OPTIONS = 3;

        static const uint32_t TRANSFER_IT_ENABLE_MASK = 
            (uint32_t)(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);

        static const uint32_t DMA_IT_TCIF  = 0x00000020;
        static const uint32_t DMA_IT_HTIF  = 0x00000010;
        static const uint32_t DMA_IT_TEIF  = 0x00000008;
        static const uint32_t DMA_IT_DMEIF = 0x00000004;
        static const uint32_t DMA_IT_FEIF  = 0x00000001;

        static const uint8_t MAX_MOTORS = 4;
        static const uint8_t STATE_PER_SYMBOL = 3;
        static const uint8_t FRAME_BITS = 16;
        static const uint8_t BUF_LENGTH = FRAME_BITS * STATE_PER_SYMBOL;

        static const uint32_t USED_TIMER_COUNT = 14;

        // amount we can safely shift timer address to the right. gcc will
        // throw error if some timers overlap
        static const uint8_t CASE_SHF = 10;           

        // Typedefs =====================================================================

        typedef enum { 
            GPIO_Mode_IN, 
            GPIO_Mode_OUT, 
            GPIO_Mode_AF, 
            GPIO_Mode_AN
        } GPIOMode_TypeDef;

        typedef enum {
            OWNER_FREE,
            OWNER_MOTOR,
            OWNER_DSHOT_BITBANG,
            OWNER_TOTAL_COUNT
        } resourceOwner_e;

        typedef struct resourceOwner_s {
            resourceOwner_e owner;
            uint8_t resourceIndex;
        } resourceOwner_t;

        enum rcc_reg {
            RCC_EMPTY = 0,   // make sure that default value (0) does not enable anything
            RCC_AHB,
            RCC_APB2,
            RCC_APB1,
            RCC_AHB1,
        };

        typedef enum {
            DMA_NONE,
            DMA1_ST0_HANDLER,
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

        typedef void * IO_t; 

        typedef struct {
            TIM_TypeDef *TIMx;
            uint8_t rcc;
        } timerDef_t;

        typedef struct {
            uint8_t channel;
        } timerHardware_t;

        struct dmaChannelDescriptor_s;

        typedef void (*dmaCallbackHandlerFuncPtr)(
                struct dmaChannelDescriptor_s *channelDescriptor);

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

        // Per pacer timer
        typedef struct bbPacer_s {
            TIM_TypeDef *tim;
            uint16_t dmaSources;
        } bbPacer_t;

        typedef struct dmaRegCache_s {
            uint32_t CR;
            uint32_t FCR;
            uint32_t NDTR;
            uint32_t PAR;
            uint32_t M0AR;
        } dmaRegCache_t;

        // Per GPIO port and timer channel
        typedef struct {

            int32_t portIndex;

            GPIO_TypeDef *gpio;

            const timerHardware_t * timhw;

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

            // Output
            uint16_t outputARR;
            uint32_t *portOutputBuffer;
            uint32_t portOutputCount;

            // Misc
            resourceOwner_t owner;

            // TIM initialization
            uint16_t TIM_Prescaler;       
            uint16_t TIM_CounterMode;       
            uint32_t TIM_Period;            
            uint16_t TIM_ClockDivision;     
            uint8_t  TIM_RepetitionCounter;  

        } port_t;

        typedef struct {
            uint16_t        code;
            dmaResource_t * ref;
            uint32_t        channel;
        } dmaChannelSpec_t;

        typedef struct {
            TIM_TypeDef *tim;
            uint8_t channel;
            dmaChannelSpec_t channelSpec[MAX_TIMER_DMA_OPTIONS];
        } dmaTimerMapping_t;

        typedef struct bbMotor_s {
            uint16_t value;
            int32_t pinIndex;    
            int32_t portIndex;
            IO_t io;        
            uint8_t output;
            uint32_t iocfg;
            port_t *bbPort;
            bool configured;
            bool enabled;
        } motor_t;

        typedef struct {
            GPIO_TypeDef *gpio;
            uint16_t pin;
            resourceOwner_e owner;
            uint8_t index;
        } ioRec_t;

        // Static local funs ============================================================

        static uint8_t io_config(
                const uint8_t mode,
                const uint8_t speed,
                const uint8_t otype,
                const uint8_t pupd) 
        {
            return mode | (speed << 2) | (otype << 4) | (pupd << 5);
        }


        static uint32_t log2_8bit(uint32_t v)  
        {
            return 8 - 90/((v/4+14)|1) - 2/(v/2+1);
        }

        static uint32_t log2_16bit(uint32_t v) 
        {
            return 8*(v>255) + log2_8bit(v >>8*(v>255));
        }

        static uint32_t log2_32bit(uint32_t v) 
        {
            return 16*(v>65535L) + log2_16bit(v*1L >>16*(v>65535L));
        }


        static uint32_t rcc_encode(uint32_t reg, uint32_t mask) 
        {
            return (reg << 5) | log2_32bit(mask);
        }

        static uint32_t nvic_build_priority(uint32_t base, uint32_t sub) 
        {
            return (((((base)<<(4-(7-(NVIC_PRIORITY_GROUPING>>8))))|
                            ((sub)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8)))))<<4)&0xf0);
        }

        static uint32_t nvic_priority_base(uint32_t prio) 
        {
            return (((prio)>>(4-(7-(NVIC_PRIORITY_GROUPING>>8))))>>4);
        }

        static uint32_t nvic_priority_sub(uint32_t prio) 
        {
            return (((prio)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8))))>>4);
        }

        static void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState newState)
        {
            if (newState != DISABLE) {
                RCC->APB2ENR |= RCC_APB2Periph;
            }
            else {
                RCC->APB2ENR &= ~RCC_APB2Periph;
            }
        }

        static void RCC_AHB1PeriphResetCmd(uint32_t RCC_AHB1Periph, FunctionalState newState)
        {
            if (newState != DISABLE) {
                RCC->AHB1RSTR |= RCC_AHB1Periph;
            }
            else {
                RCC->AHB1RSTR &= ~RCC_AHB1Periph;
            }
        }

        static void RCC_AHB1PeriphClockCmd(uint32_t RCC_AHB1Periph, FunctionalState newState)
        {
            if (newState != DISABLE) {
                RCC->AHB1ENR |= RCC_AHB1Periph;
            }
            else {
                RCC->AHB1ENR &= ~RCC_AHB1Periph;
            }
        }

        static void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState newState)
        {
            if (newState != DISABLE) {
                RCC->APB2RSTR |= RCC_APB2Periph;
            }
            else {
                RCC->APB2RSTR &= ~RCC_APB2Periph;
            }
        }

        static void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState newState)
        {
            if (newState != DISABLE) {
                RCC->APB1RSTR |= RCC_APB1Periph;
            }
            else {
                RCC->APB1RSTR &= ~RCC_APB1Periph;
            }
        }

        static void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState newState)
        {
            if (newState != DISABLE) {
                RCC->APB1ENR |= RCC_APB1Periph;
            }
            else {
                RCC->APB1ENR &= ~RCC_APB1Periph;
            }
        }

        static void RCC_ClockCmd(uint8_t periphTag, FunctionalState newState)
        {
            int tag = periphTag >> 5;
            uint32_t mask = 1 << (periphTag & 0x1f);

            switch (tag) {
                case RCC_APB2:
                    RCC_APB2PeriphClockCmd(mask, newState);
                    break;
                case RCC_APB1:
                    RCC_APB1PeriphClockCmd(mask, newState);
                    break;
                case RCC_AHB1:
                    RCC_AHB1PeriphClockCmd(mask, newState);
                    break;
            }
        }

        static uint8_t lookupTimerIndex(const TIM_TypeDef *tim)
        {
            switch ((uint32_t)tim >> CASE_SHF) {

                case ((uint32_t)TIM1_BASE >> CASE_SHF): return 1;
                case ((uint32_t)TIM2_BASE >> CASE_SHF): return 2;
                case ((uint32_t)TIM5_BASE >> CASE_SHF): return 5;
            }

            return ~1;  // make sure final index is out of range
        }

        static void loadDmaRegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
        {
            ((DMA_Stream_TypeDef *)dmaResource)->CR = dmaRegCache->CR;
        }

        static void saveDmaRegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
        {
            dmaRegCache->CR = ((DMA_Stream_TypeDef *)dmaResource)->CR;
        }

        static void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState newState)
        {
            if (newState != DISABLE) {
                TIMx->CR1 |= TIM_CR1_ARPE;
            }
            else {
                TIMx->CR1 &= (uint16_t)~TIM_CR1_ARPE;
            }
        }

        static void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState newState)
        {
            if (newState != DISABLE) {
                // Enable the TIM Counter
                TIMx->CR1 |= TIM_CR1_CEN;
            }
            else {
                // Disable the TIM Counter
                TIMx->CR1 &= (uint16_t)~TIM_CR1_CEN;
            }
        }

        static void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState newState)
        {
            if (newState != DISABLE) {
                // Enable the TIM Main Output
                TIMx->BDTR |= TIM_BDTR_MOE;
            }
            else {
                // Disable the TIM Main Output
                TIMx->BDTR &= (uint16_t)~TIM_BDTR_MOE;
            }  
        }

        static uint32_t getDmaFlagStatus(
                dmaChannelDescriptor_t * descriptor, uint32_t flag) 
        {
            return descriptor->flagsShift > 31 ?
                descriptor->dma->HISR & (flag << (descriptor->flagsShift - 32)) :
                descriptor->dma->LISR & (flag << descriptor->flagsShift);
        }

        static void clearDmaFlag(
                dmaChannelDescriptor_t * descriptor, uint32_t flag) 
        {
            if (descriptor->flagsShift > 31) {
                descriptor->dma->HIFCR = (flag << (descriptor->flagsShift - 32)); 
            }
            else {
                descriptor->dma->LIFCR = (flag << descriptor->flagsShift);
            }
        }

        static void DMA_ITConfig(
                DMA_Stream_TypeDef* DMAy_Streamx, uint32_t DMA_IT, FunctionalState newState)
        {
            // Check if the DMA_IT parameter contains a FIFO interrupt 
            if ((DMA_IT & DMA_IT_FE) != 0)
            {
                if (newState != DISABLE) {
                    // Enable the selected DMA FIFO interrupts 
                    DMAy_Streamx->FCR |= (uint32_t)DMA_IT_FE;
                }    
                else {
                    // Disable the selected DMA FIFO interrupts 
                    DMAy_Streamx->FCR &= ~(uint32_t)DMA_IT_FE;  
                }
            }

            // Check if the DMA_IT parameter contains a Transfer interrupt 
            if (DMA_IT != DMA_IT_FE) {
                if (newState != DISABLE) {
                    // Enable the selected DMA transfer interrupts 
                    DMAy_Streamx->CR |= (uint32_t)(DMA_IT  & TRANSFER_IT_ENABLE_MASK);
                }
                else {
                    // Disable the selected DMA transfer interrupts 
                    DMAy_Streamx->CR &= ~(uint32_t)(DMA_IT & TRANSFER_IT_ENABLE_MASK);
                }    
            }
        }

        static void dmaCmd(port_t *bbPort, FunctionalState newState)
        {
            DMA_Stream_TypeDef * DMAy_Streamx = (DMA_Stream_TypeDef *)bbPort->dmaResource;
    
            if (newState != DISABLE) {
                DMAy_Streamx->CR |= (uint32_t)DMA_SxCR_EN;
            }
            else {
                DMAy_Streamx->CR &= ~(uint32_t)DMA_SxCR_EN;
            }
         }

        static void dmaIrqHandler(dmaChannelDescriptor_t *descriptor)
        {
            port_t *bbPort = (port_t *)descriptor->userParam;

            dmaCmd(bbPort, DISABLE);

            timDmaCmd(TIM1, bbPort->dmaSource, DISABLE);

            if (getDmaFlagStatus(descriptor, DMA_IT_TEIF)) {
                while (1) {};
            }

            clearDmaFlag(descriptor, DMA_IT_TCIF);
        }

        static void dmaItConfig(port_t *bbPort)
        {
            DMA_ITConfig((DMA_Stream_TypeDef *)bbPort->dmaResource, DMA_IT_TC, ENABLE);
        }

        static void dmaPreconfigure(port_t *bbPort)
        {
            DMA_Stream_TypeDef * DMAy_Streamx = (DMA_Stream_TypeDef *)bbPort->dmaResource;

            DMAy_Streamx->CR = 0x0c025450;

            DMAy_Streamx->FCR =
                ((DMAy_Streamx->FCR & (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH)) |
                 (DMA_FIFOMODE_ENABLE | DMA_FIFO_THRESHOLD_1QUARTERFULL));
            DMAy_Streamx->NDTR = bbPort->portOutputCount;
            DMAy_Streamx->PAR = (uint32_t)&bbPort->gpio->BSRR;
            DMAy_Streamx->M0AR = (uint32_t)bbPort->portOutputBuffer;

            saveDmaRegs(bbPort->dmaResource, &bbPort->dmaRegOutput);
        }

        static void timDmaCmd(
                TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState newState)
        {
            TIM_DMACmd(TIMx, TIM_DMASource, newState);
        }

        static void bbTIM_TimeBaseInit(port_t *bbPort, uint16_t period)
        {
            bbPort->TIM_Prescaler = 0; // Feed raw timerClock
            bbPort->TIM_ClockDivision = TIM_CLOCKDIVISION_DIV1;
            bbPort->TIM_CounterMode = TIM_COUNTERMODE_UP;
            bbPort->TIM_Period = period;
            TIM_TimeBaseInit(TIM1, bbPort);
            TIM_ARRPreloadConfig(TIM1, ENABLE);
        }

        static void outputDataInit(uint32_t *buffer, uint16_t portMask, bool inverted)
        {
            uint32_t resetMask;
            uint32_t setMask;

            if (inverted) {
                resetMask = portMask;
                setMask = (portMask << 16);
            } else {
                resetMask = (portMask << 16);
                setMask = portMask;
            }

            for (auto bitpos=0; bitpos<16; bitpos++) {
                buffer[bitpos * 3 + 0] |= setMask ; // Always set all ports
                buffer[bitpos * 3 + 1] = 0;          // Reset bits are port dependent
                buffer[bitpos * 3 + 2] |= resetMask; // Always reset all ports
            }
        }

        static void outputDataSet(
                uint32_t *buffer, int32_t pinNumber, uint16_t value, bool inverted)
        {
            uint32_t middleBit;

            if (inverted) {
                middleBit = (1 << (pinNumber + 0));
            } else {
                middleBit = (1 << (pinNumber + 16));
            }

            for (auto pos=0; pos<16; pos++) {
                if (!(value & 0x8000)) {
                    buffer[pos * 3 + 1] |= middleBit;
                }
                value <<= 1;
            }
        }

        static void outputDataClear(uint32_t *buffer)
        {
            // Middle position to no change
            for (auto bitpos=0; bitpos<16; bitpos++) {
                buffer[bitpos * 3 + 1] = 0;
            }
        }

        static uint32_t dmaFlag_IT_TCIF(const dmaResource_t *stream)
        {

            if ((DMA_Stream_TypeDef *)stream == DMA2_Stream0) return _MY_DMA_IT_TCIF0;
            if ((DMA_Stream_TypeDef *)stream == DMA2_Stream1) return _MY_DMA_IT_TCIF1;
            if ((DMA_Stream_TypeDef *)stream == DMA2_Stream2) return _MY_DMA_IT_TCIF2;
            if ((DMA_Stream_TypeDef *)stream == DMA2_Stream3) return _MY_DMA_IT_TCIF3;
            if ((DMA_Stream_TypeDef *)stream == DMA2_Stream4) return _MY_DMA_IT_TCIF4;
            if ((DMA_Stream_TypeDef *)stream == DMA2_Stream5) return _MY_DMA_IT_TCIF5;
            if ((DMA_Stream_TypeDef *)stream == DMA2_Stream6) return _MY_DMA_IT_TCIF6;
            if ((DMA_Stream_TypeDef *)stream == DMA2_Stream7) return _MY_DMA_IT_TCIF7;

            return 0;
        }

        static uint32_t timerClock(TIM_TypeDef *tim)
        {
            if (tim == TIM8 || tim == TIM1 || tim == TIM9 || tim == TIM10 || tim == TIM11) {
                return SystemCoreClock;
            } else {
                return SystemCoreClock / 2;
            }
        }

        static void timebaseSetup(port_t *bbPort, protocol_t dshotProtocolType)
        {
            uint32_t timerclock = timerClock(TIM1);

            uint32_t outputFreq = 1000 * getDshotBaseFrequency(dshotProtocolType);
            bbPort->outputARR = timerclock / outputFreq - 1;
        }

        static uint16_t timerDmaSource(uint8_t channel)
        {
            switch (channel) {
                case TIM_CHANNEL_1:
                    return TIM_DMA_CC1;
                case TIM_CHANNEL_2:
                    return TIM_DMA_CC2;
                case TIM_CHANNEL_3:
                    return TIM_DMA_CC3;
                case TIM_CHANNEL_4:
                    return TIM_DMA_CC4;
            }
            return 0;
        }

        static void TIM_DMACmd(
                TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState newState)
        { 
            if (newState != DISABLE) {
                // Enable the DMA sources
                TIMx->DIER |= TIM_DMASource; 
            }
            else {
                // Disable the DMA sources
                TIMx->DIER &= (uint16_t)~TIM_DMASource;
            }
        }

        static void TIM_TimeBaseInit(TIM_TypeDef * TIMx, port_t * bbPort)
        {
            uint16_t tmpcr1 = TIMx->CR1;  

            if((TIMx == TIM1) || (TIMx == TIM8)||
                    (TIMx == TIM2) || (TIMx == TIM3)||
                    (TIMx == TIM4) || (TIMx == TIM5)) 
            {
                // Select the Counter Mode
                tmpcr1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS));
                tmpcr1 |= (uint32_t)bbPort->TIM_CounterMode;
            }

            if((TIMx != TIM6) && (TIMx != TIM7)) {
                // Set the clock division 
                tmpcr1 &=  (uint16_t)(~TIM_CR1_CKD);
                tmpcr1 |= (uint32_t)bbPort->TIM_ClockDivision;
            }

            TIMx->CR1 = tmpcr1;

            // Set the Autoreload value 
            TIMx->ARR = bbPort->TIM_Period ;

            // Set the Prescaler value
            TIMx->PSC = bbPort->TIM_Prescaler;

            if ((TIMx == TIM1) || (TIMx == TIM8))  {
                // Set the Repetition Counter value
                TIMx->RCR = bbPort->TIM_RepetitionCounter;
            }

            // Generate an update event to reload the Prescaler 
            // and the repetition counter(only for TIM1 and TIM8) value immediately
            TIMx->EGR = 0x0001;          
        }

        static ioRec_t* _IORec(IO_t io)
        {
            return (ioRec_t *)io;
        }

        static GPIO_TypeDef* _IO_GPIO(IO_t io)
        {
            const ioRec_t *ioRec =_IORec(io);
            return ioRec->gpio;
        }

        static uint16_t _IO_Pin(IO_t io)
        {
            const ioRec_t *ioRec =_IORec(io);
            return ioRec->pin;
        }

        static void _IOInit(IO_t io, resourceOwner_e owner, uint8_t index)
        {
            ioRec_t *ioRec =_IORec(io);
            ioRec->owner = owner;
            ioRec->index = index;
        }

        static resourceOwner_e _IOGetOwner(IO_t io)
        {
            if (!io) {
                return OWNER_FREE;
            }
            const ioRec_t *ioRec = _IORec(io);
            return ioRec->owner;
        }

        static int32_t _IO_GPIOPortIdx(IO_t io)
        {
            if (!io) {
                return -1;
            }

            return (((size_t)_IO_GPIO(io) - GPIOA_BASE) >> 10);
        }

        static uint8_t rcc_ahb1(uint32_t gpio)
        {
            return (uint8_t)rcc_encode(RCC_AHB1, gpio); 
        }

        static void GPIO_Init(
                GPIO_TypeDef* GPIOx,
                uint32_t pin,
                uint32_t mode,
                uint32_t speed,
                uint32_t pull
                )
        {
            uint32_t pinpos = 0x00, pos = 0x00 , currentpin = 0x00;

            for (pinpos = 0x00; pinpos < 0x10; pinpos++)
            {
                pos = ((uint32_t)0x01) << pinpos;

                currentpin = pin & pos;

                if (currentpin == pos)
                {
                    GPIOx->MODER  &= ~(GPIO_MODER_MODER0 << (pinpos * 2));
                    GPIOx->MODER |= (mode << (pinpos * 2));

                    if ((mode == GPIO_Mode_OUT) || (mode == GPIO_Mode_AF)) {

                        GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinpos * 2));
                        GPIOx->OSPEEDR |= (speed << (pinpos * 2));

                        GPIOx->OTYPER  &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)pinpos)) ;
                    }

                    GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)pinpos * 2));
                    GPIOx->PUPDR |= (pull << (pinpos * 2)); 
                }
            }
        }

        static void _configGPIO(IO_t io, uint8_t cfg)
        {
            if (!io) {
                return;
            }

            const uint8_t ioPortDefs[6] = {
                { rcc_ahb1(RCC_AHB1ENR_GPIOAEN_MSK) },
                { rcc_ahb1(RCC_AHB1ENR_GPIOBEN_MSK) },
                { rcc_ahb1(RCC_AHB1ENR_GPIOCEN_MSK) },
                { rcc_ahb1(RCC_AHB1ENR_GPIODEN_MSK) },
                { rcc_ahb1(RCC_AHB1ENR_GPIOEEN_MSK) },
                { rcc_ahb1(RCC_AHB1ENR_GPIOFEN_MSK) },
            };

            const uint8_t rcc = ioPortDefs[_IO_GPIOPortIdx(io)];

            RCC_ClockCmd(rcc, ENABLE);

            uint32_t pin =_IO_Pin(io);
            uint32_t mode  = (cfg >> 0) & 0x03;
            uint32_t speed = (cfg >> 2) & 0x03;
            uint32_t pull  = (cfg >> 5) & 0x03;

            GPIO_Init(_IO_GPIO(io), pin, mode, speed, pull);
        }

        static void _IOConfigGPIO(IO_t io, uint8_t cfg)
        {
            if (!io) {
                return;
            }

            _configGPIO(io, cfg);
        }

        static int32_t _IO_GPIOPinIdx(IO_t io)
        {
            if (!io) {
                return -1;
            }
            return 31 - __builtin_clz(_IO_Pin(io));  // CLZ is a bit faster than FFS
        }

        static int32_t _IO_GPIO_PinSource(IO_t io)
        {
            return _IO_GPIOPinIdx(io);
        }

        static void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF)
        {

            uint32_t temp = ((uint32_t)(GPIO_AF) << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4)) ;
            GPIOx->AFR[GPIO_PinSource >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4)) ;
            uint32_t temp_2 = GPIOx->AFR[GPIO_PinSource >> 0x03] | temp;
            GPIOx->AFR[GPIO_PinSource >> 0x03] = temp_2;
        }

        static void _IOConfigGPIOAF(IO_t io, uint8_t cfg, uint8_t af)
        {
            if (!io) {
                return;
            }

            GPIO_PinAFConfig(_IO_GPIO(io), _IO_GPIO_PinSource(io), af);

            _configGPIO(io, cfg);
        }

        // Instance variables ===========================================================

        uint8_t m_ioDefUsedOffset[DEFIO_PORT_USED_COUNT] = { 0, 16, 32, 48, 64, 80 };

        bbPacer_t m_pacers[MAX_MOTORS];  // TIM1 or TIM8
        int32_t m_usedMotorPacers = 0;

        port_t m_ports[MAX_MOTORS];
        int32_t m_usedMotorPorts;

        motor_t m_motors[MAX_SUPPORTED_MOTORS];

        uint32_t m_outputBuffer[BUF_LENGTH * MAX_MOTORS];

        uint8_t m_puPdMode;

        dmaChannelDescriptor_t m_dmaDescriptors[DMA_LAST_HANDLER];

        dmaTimerMapping_t m_dmaTimerMapping[DMA_TIMER_MAPPING_COUNT] = {};

        timerDef_t m_timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {};

        timerHardware_t m_timer1Hardware[4] = {};

        static constexpr DMA_Stream_TypeDef * m_streams[8] = {
            DMA2_Stream0,
            DMA2_Stream1,
            DMA2_Stream2,
            DMA2_Stream3,
            DMA2_Stream4,
            DMA2_Stream5,
            DMA2_Stream6,
            DMA2_Stream7
        };

        ioRec_t m_ioRecs[96];

        // Private instance methods =====================================================

        IO_t _IOGetByTag(uint8_t tag)
        {
            const int32_t portIdx = (tag >> 4) - 1;
            const int32_t pinIdx = tag & 0x0F;

            if (portIdx < 0 || portIdx >= DEFIO_PORT_USED_COUNT) {
                return NULL;
            }

            // check if pin exists
            if (!(0xffff & (1 << pinIdx))) {
                return NULL;
            }

            // count bits before this pin on single port
            int32_t offset = __builtin_popcount(((1 << pinIdx) - 1) & 0xffff);

            // and add port offset
            offset += m_ioDefUsedOffset[portIdx];
            return m_ioRecs + offset;
        }

        static void TIM_OC1Init(TIM_TypeDef* TIMx)
        {
            uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
            TIMx->CCER &= (uint16_t)~TIM_CCER_CC1E;
            tmpccer = TIMx->CCER;
            tmpcr2 =  TIMx->CR2;
            tmpccmrx = TIMx->CCMR1;
            tmpccmrx &= (uint16_t)~TIM_CCMR1_OC1M;
            tmpccmrx &= (uint16_t)~TIM_CCMR1_CC1S;
            tmpccmrx |= TIM_OCMODE_TIMING;
            tmpccer &= (uint16_t)~TIM_CCER_CC1P;
            tmpccer |= TIM_OCPOLARITY_HIGH;
            tmpccer |= TIM_OUTPUTSTATE_ENABLE;

            if((TIMx == TIM1) || (TIMx == TIM8)) {

                tmpccer &= (uint16_t)~TIM_CCER_CC1NP;
                tmpccer |= TIM_OCPOLARITY_HIGH;
                tmpccer &= (uint16_t)~TIM_CCER_CC1NE;
                tmpcr2 &= (uint16_t)~TIM_CR2_OIS1;
                tmpcr2 &= (uint16_t)~TIM_CR2_OIS1N;
            }

            TIMx->CR2 = tmpcr2;
            TIMx->CCMR1 = tmpccmrx;
            TIMx->CCR1 = 0x00000000;
            TIMx->CCER = tmpccer;
        }

        static void TIM_OC2Init(TIM_TypeDef* TIMx)
        {
            uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
            TIMx->CCER &= (uint16_t)~TIM_CCER_CC2E;
            tmpccer = TIMx->CCER;
            tmpcr2 =  TIMx->CR2;
            tmpccmrx = TIMx->CCMR1;
            tmpccmrx &= (uint16_t)~TIM_CCMR1_OC2M;
            tmpccmrx &= (uint16_t)~TIM_CCMR1_CC2S;
            tmpccmrx |= (uint16_t)(TIM_OCMODE_TIMING << 8);
            tmpccer &= (uint16_t)~TIM_CCER_CC2P;
            tmpccer |= (uint16_t)(TIM_OCPOLARITY_HIGH << 4);
            tmpccer |= (uint16_t)(TIM_OUTPUTSTATE_ENABLE << 4);

            if((TIMx == TIM1) || (TIMx == TIM8)) {

                tmpccer &= (uint16_t)~TIM_CCER_CC2NP;
                tmpccer |= (uint16_t)(TIM_OCPOLARITY_HIGH << 4);
                tmpccer &= (uint16_t)~TIM_CCER_CC2NE;
                tmpcr2 &= (uint16_t)~TIM_CR2_OIS2;
                tmpcr2 &= (uint16_t)~TIM_CR2_OIS2N;
            }

            TIMx->CR2 = tmpcr2;
            TIMx->CCMR1 = tmpccmrx;
            TIMx->CCR2 = 0x00000000;
            TIMx->CCER = tmpccer;
        }

        static void TIM_OC3Init(TIM_TypeDef* TIMx)
        {
            uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
            TIMx->CCER &= (uint16_t)~TIM_CCER_CC3E;
            tmpccer = TIMx->CCER;
            tmpcr2 =  TIMx->CR2;
            tmpccmrx = TIMx->CCMR2;
            tmpccmrx &= (uint16_t)~TIM_CCMR2_OC3M;
            tmpccmrx &= (uint16_t)~TIM_CCMR2_CC3S;  
            tmpccmrx |= TIM_OCMODE_TIMING;
            tmpccer &= (uint16_t)~TIM_CCER_CC3P;
            tmpccer |= (uint16_t)(TIM_OCPOLARITY_HIGH << 8);
            tmpccer |= (uint16_t)(TIM_OUTPUTSTATE_ENABLE << 8);

            if((TIMx == TIM1) || (TIMx == TIM8)) {

                tmpccer &= (uint16_t)~TIM_CCER_CC3NP;
                tmpccer |= (uint16_t)(TIM_OCPOLARITY_HIGH << 8);
                tmpccer &= (uint16_t)~TIM_CCER_CC3NE;
                tmpcr2 &= (uint16_t)~TIM_CR2_OIS3;
                tmpcr2 &= (uint16_t)~TIM_CR2_OIS3N;
            }

            TIMx->CR2 = tmpcr2;
            TIMx->CCMR2 = tmpccmrx;
            TIMx->CCR3 = 0x00000000;
            TIMx->CCER = tmpccer;
        }

        static void TIM_OC4Init(TIM_TypeDef* TIMx)
        {
            uint16_t tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
            TIMx->CCER &= (uint16_t)~TIM_CCER_CC4E;
            tmpccer = TIMx->CCER;
            tmpcr2 =  TIMx->CR2;
            tmpccmrx = TIMx->CCMR2;
            tmpccmrx &= (uint16_t)~TIM_CCMR2_OC4M;
            tmpccmrx &= (uint16_t)~TIM_CCMR2_CC4S;
            tmpccmrx |= (uint16_t)(TIM_OCMODE_TIMING << 8);
            tmpccer &= (uint16_t)~TIM_CCER_CC4P;
            tmpccer |= (uint16_t)(TIM_OCPOLARITY_HIGH << 12);
            tmpccer |= (uint16_t)(TIM_OUTPUTSTATE_ENABLE << 12);

            if((TIMx == TIM1) || (TIMx == TIM8)) {
                tmpcr2 &=(uint16_t) ~TIM_CR2_OIS4;
            }

            TIMx->CR2 = tmpcr2;
            TIMx->CCMR2 = tmpccmrx;
            TIMx->CCR4 = 0x00000000;
            TIMx->CCER = tmpccer;
        }

        static void rccInit(void)
        {
            RCC_APB2PeriphClockCmd(
                    RCC_APB2LPENR_TIM1LPEN_Msk   |
                    RCC_APB2LPENR_TIM8LPEN_Msk   |
                    RCC_APB2LPENR_USART1LPEN_Msk |
                    RCC_APB2LPENR_USART6LPEN_Msk |
                    RCC_APB2LPENR_ADC1LPEN_Msk   |
                    RCC_APB2LPENR_ADC2LPEN_Msk   |
                    RCC_APB2LPENR_ADC3LPEN_Msk   |
                    RCC_APB2LPENR_SDIOLPEN_Msk   |
                    RCC_APB2LPENR_SPI1LPEN_Msk   |
                    RCC_APB2LPENR_SYSCFGLPEN_Msk |
                    RCC_APB2LPENR_TIM9LPEN_Msk   |
                    RCC_APB2LPENR_TIM10LPEN_Msk  |
                    RCC_APB2LPENR_TIM11LPEN_Msk  |
                    0, ENABLE);
        }



        void timerOCInit(TIM_TypeDef *tim, uint8_t channel)
        {
            switch (channel) {
                case TIM_CHANNEL_1:
                    TIM_OC1Init(tim);
                    break;
                case TIM_CHANNEL_2:
                    TIM_OC2Init(tim);
                    break;
                case TIM_CHANNEL_3:
                    TIM_OC3Init(tim);
                    break;
                case TIM_CHANNEL_4:
                    TIM_OC4Init(tim);
                    break;
            }
        }

        void timerChannelInit(port_t *bbPort)
        {
            const timerHardware_t *timhw = bbPort->timhw;

            TIM_Cmd(TIM1, DISABLE);

            timerOCInit(TIM1, timhw->channel);

            TIM_Cmd(TIM1, ENABLE);
        }

        bbPacer_t *findMotorPacer(TIM_TypeDef *tim)
        {
            for (auto i=0; i<MAX_MOTORS; i++) {

                bbPacer_t *bbPacer = &m_pacers[i];

                if (bbPacer->tim == NULL) {
                    bbPacer->tim = tim;
                    ++m_usedMotorPacers;
                    return bbPacer;
                }

                if (bbPacer->tim == tim) {
                    return bbPacer;
                }
            }

            return NULL;
        }

        port_t *findMotorPort(int32_t portIndex)
        {
            for (auto i=0; i<m_usedMotorPorts; i++) {
                if (m_ports[i].portIndex == portIndex) {
                    return &m_ports[i];
                }
            }
            return NULL;
        }

        port_t *allocateMotorPort(int32_t portIndex)
        {
            if (m_usedMotorPorts >= MAX_MOTORS) {
                return NULL;
            }

            port_t *bbPort = &m_ports[m_usedMotorPorts];

            if (!bbPort->timhw) {
                // No more pacer channel available
                return NULL;
            }

            bbPort->portIndex = portIndex;
            bbPort->owner.owner = OWNER_DSHOT_BITBANG;
            bbPort->owner.resourceIndex = portIndex + 1;

            ++m_usedMotorPorts;

            return bbPort;
        }

        dmaIdentifier_e dmaGetIdentifier(const dmaResource_t* channel)
        {
            for (uint8_t i=0; i<DMA_LAST_HANDLER; i++) {
                if (m_dmaDescriptors[i].ref == channel) {
                    return (dmaIdentifier_e)(i + 1);
                }
            }

            return DMA_NONE;
        }

        void dmaSetHandler(
                dmaIdentifier_e identifier,
                dmaCallbackHandlerFuncPtr callback,
                uint32_t priority,
                uint32_t userParam)
        {
            const int8_t index = identifier - 1;

            RCC_AHB1PeriphClockCmd(RCC_AHB1PERIPH_DMA2, ENABLE);
            m_dmaDescriptors[index].irqHandlerCallback = callback;
            m_dmaDescriptors[index].userParam = userParam;
            m_dmaDescriptors[index].completeFlag =
                dmaFlag_IT_TCIF(m_dmaDescriptors[index].ref);

            uint8_t irqChannel = m_dmaDescriptors[index].irqN;

            uint8_t tmppriority = 0x00, tmppre = 0x00, tmpsub = 0x0F;

            tmppriority = (0x700 - ((SCB->AIRCR) & (uint32_t)0x700))>> 0x08;
            tmppre = (0x4 - tmppriority);
            tmpsub = tmpsub >> tmppriority;

            tmppriority = nvic_priority_base(priority) << tmppre;
            tmppriority |= (uint8_t)(nvic_priority_sub(priority) & tmpsub);

            tmppriority = tmppriority << 0x04;

            NVIC->IP[irqChannel] = tmppriority;

            NVIC->ISER[irqChannel >> 0x05] =
                (uint32_t)0x01 << (irqChannel & (uint8_t)0x1F);
        }

        void setupDma(port_t *bbPort)
        {
            RCC_AHB1PeriphClockCmd(RCC_AHB1PERIPH_DMA2, ENABLE);

            bbPort->dmaSource = timerDmaSource(bbPort->timhw->channel);

            bbPacer_t *bbPacer = findMotorPacer(TIM1);

            bbPacer->dmaSources |= bbPort->dmaSource;

            dmaSetHandler(
                    dmaGetIdentifier(bbPort->dmaResource),
                    dmaIrqHandler,
                    nvic_build_priority(2, 1),
                    (uint32_t)bbPort);

            dmaItConfig(bbPort);
        }

        const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(
                TIM_TypeDef *tim, uint8_t channel, int8_t dmaOpt)
        {
            if (dmaOpt < 0 || dmaOpt >= MAX_TIMER_DMA_OPTIONS) {
                return NULL;
            }

            for (uint8_t i=0 ; i<DMA_TIMER_MAPPING_COUNT; i++) {
                const dmaTimerMapping_t *timerMapping = &m_dmaTimerMapping[i];

                if (timerMapping->tim == tim && timerMapping->channel == channel &&
                        timerMapping->channelSpec[dmaOpt].ref) {

                    return &timerMapping->channelSpec[dmaOpt];
                }
            }

            return NULL;
        }

        bool motorConfig(uint8_t motorIndex)
        {
            IO_t io = m_motors[motorIndex].io;
            uint8_t output = m_motors[motorIndex].output;

            int32_t pinIndex = _IO_GPIOPinIdx(io);
            int32_t portIndex = _IO_GPIOPortIdx(io);

            port_t *bbPort = findMotorPort(portIndex);

            if (!bbPort) {

                // New port group

                bbPort = allocateMotorPort(portIndex);

                if (bbPort) {
                    static uint8_t options[4] = {1, 0, 1, 0};
                    const timerHardware_t *timhw = bbPort->timhw;
                    const int8_t option = options[motorIndex];
                    const dmaChannelSpec_t *dmaChannelSpec =
                        dmaGetChannelSpecByTimerValue(TIM1, timhw->channel, option); 
                    bbPort->dmaResource = dmaChannelSpec->ref;
                    bbPort->dmaChannel = dmaChannelSpec->channel;
                }

                bbPort->gpio =_IO_GPIO(io);

                bbPort->portOutputCount = BUF_LENGTH;
                bbPort->portOutputBuffer =
                    &m_outputBuffer[(bbPort - m_ports) * BUF_LENGTH];

                timebaseSetup(bbPort, m_protocol);
                bbTIM_TimeBaseInit(bbPort, bbPort->outputARR);
                timerChannelInit(bbPort);

                setupDma(bbPort);
                dmaPreconfigure(bbPort);
                dmaItConfig(bbPort);
            }

            motor_t * bbMotor = &m_motors[motorIndex];

            bbMotor->pinIndex = pinIndex;
            bbMotor->io = io;
            bbMotor->output = output;
            bbMotor->bbPort = bbPort;

            _IOInit(io, OWNER_MOTOR, motorIndex+1);

            // Setup GPIO_MODER and GPIO_ODR register manipulation values

            bbPort->gpioModeMask |= (GPIO_MODER_MODER0 << (pinIndex * 2));
            bbPort->gpioModeInput |= (GPIO_Mode_IN << (pinIndex * 2));
            bbPort->gpioModeOutput |= (GPIO_Mode_OUT << (pinIndex * 2));

            bbPort->gpioIdleBSRR |= (1 << (pinIndex + 16));  // BR (higher half)

            _IO_GPIO(io)->BSRR |= (((uint32_t)(_IO_Pin(io))) << 16);

            _IOConfigGPIO(io,
                    io_config(GPIO_Mode_OUT, GPIO_FAST_SPEED, GPIO_OTYPE_PP, m_puPdMode));

            // not inverted
            outputDataInit(bbPort->portOutputBuffer, (1 << pinIndex), false); 

            // Output idle level before switching to output
            // Use BSRR register for this
            // Normal: Use BR (higher half)
            // Inverted: Use BS (lower half)

            bbPort->gpio->BSRR = bbPort->gpioIdleBSRR;

            // Set GPIO to output
            ATOMIC_BLOCK(nvic_build_priority(1, 1)) {
                MODIFY_REG(bbPort->gpio->MODER,
                        bbPort->gpioModeMask,
                        bbPort->gpioModeOutput);
            }

            // Reinitialize port group DMA for output

            dmaResource_t *dmaResource = bbPort->dmaResource;
            loadDmaRegs(dmaResource, &bbPort->dmaRegOutput);

            // Reinitialize pacer timer for output

            TIM1->ARR = bbPort->outputARR;

            bbMotor->configured = true;

            return true;
        }

        void initChannel(
                const uint8_t timerId,
                TIM_TypeDef * tim,
                const uint8_t channel,
                const uint8_t specId,
                const uint32_t d,
                const uint32_t s,
                const uint32_t c)
        {
            m_dmaTimerMapping[timerId].tim = tim;
            m_dmaTimerMapping[timerId].channel = channel;

            dmaChannelSpec_t * spec = &m_dmaTimerMapping[timerId].channelSpec[specId];
            spec->code = (d << 12) |( s << 8) | (c << 0);
            spec->ref = (dmaResource_t *)m_streams[s];
            spec->channel = c << 25;
        }

        void initTimerMapping(void)
        {
            initChannel(0, TIM1, TIM_CHANNEL_1, 1, 2, 1, 6); 
            initChannel(1, TIM1, TIM_CHANNEL_2, 1, 2, 2, 6); 
            initChannel(2, TIM2, TIM_CHANNEL_1, 0, 1, 5, 3); 
            initChannel(3, TIM2, TIM_CHANNEL_3, 0, 1, 1, 3); 
            initChannel(4, TIM5, TIM_CHANNEL_2, 0, 1, 4, 6); 
            initChannel(5, TIM5, TIM_CHANNEL_4, 0, 1, 1, 6); 
        }

        void initTimerDefinition(uint8_t id, TIM_TypeDef * tim, uint32_t apb, uint32_t en)
        {
            m_timerDefinitions[id].TIMx = tim;
            m_timerDefinitions[id].rcc = rcc_encode(apb, en);
        }

        void initTimerDefinitions(void)
        {
            initTimerDefinition(0, TIM1, RCC_APB2, RCC_APB2ENR_TIM1EN);
            initTimerDefinition(1, TIM2, RCC_APB1, RCC_APB1ENR_TIM2EN);
            initTimerDefinition(2, TIM5, RCC_APB1, RCC_APB1ENR_TIM5EN);
        }

        void initTimer1Hardware(void)
        {
            m_timer1Hardware[0].channel = TIM_CHANNEL_1;
            m_timer1Hardware[1].channel = TIM_CHANNEL_2;
            m_timer1Hardware[2].channel = TIM_CHANNEL_3;
            m_timer1Hardware[3].channel = TIM_CHANNEL_4;
        }

        void defineDma2Channel(
                uint8_t stream,
                DMA_Stream_TypeDef * ref,
                uint8_t flagsShift,
                IRQn_Type irqN)
        {
            dmaChannelDescriptor_t * desc = &m_dmaDescriptors[stream+8];

            desc->dma = DMA2;
            desc->ref = (dmaResource_t *)ref;
            desc->stream = stream;
            desc->flagsShift = flagsShift;
            desc->irqN = irqN;
        }

        void dmaInit(void)
        {
            defineDma2Channel(0, DMA2_Stream0, 0,  DMA2_Stream0_IRQn); 
            defineDma2Channel(1, DMA2_Stream1, 6,  DMA2_Stream1_IRQn); 
            defineDma2Channel(2, DMA2_Stream2, 16, DMA2_Stream2_IRQn); 
            defineDma2Channel(3, DMA2_Stream3, 22, DMA2_Stream3_IRQn); 
            defineDma2Channel(4, DMA2_Stream4, 32, DMA2_Stream4_IRQn); 
            defineDma2Channel(5, DMA2_Stream5, 38, DMA2_Stream5_IRQn); 
            defineDma2Channel(6, DMA2_Stream6, 48, DMA2_Stream6_IRQn); 
            defineDma2Channel(7, DMA2_Stream7, 54, DMA2_Stream7_IRQn); 
        }

        uint8_t timerRCC(TIM_TypeDef *tim)
        {
            for (int32_t i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
                if (m_timerDefinitions[i].TIMx == tim) {
                    return m_timerDefinitions[i].rcc;
                }
            }
            return 0;
        }

        uint16_t timerGetPrescalerByDesiredHertz(TIM_TypeDef *tim, uint32_t hz)
        {
            // protection here for desired hertz > SystemCoreClock???
            if (hz > timerClock(tim)) {
                return 0;
            }
            return (uint16_t)((timerClock(tim) + hz / 2 ) / hz) - 1;
        }

        int8_t timerGetTIMNumber(const TIM_TypeDef *tim)
        {
            const uint8_t index = lookupTimerIndex(tim);

            if (index < USED_TIMER_COUNT) {
                return index + 1;
            } else {
                return 0;
            }
        }

        void timerInit(void)
        {
            RCC_ClockCmd(timerRCC(TIM5), ENABLE);
            RCC_ClockCmd(timerRCC(TIM2), ENABLE);
            RCC_ClockCmd(timerRCC(TIM2), ENABLE);
            RCC_ClockCmd(timerRCC(TIM5), ENABLE);
        }

        const resourceOwner_t freeOwner = { .owner = OWNER_FREE, .resourceIndex = 0 };

        void ioInit(void)
        {
            ioRec_t *ioRec = m_ioRecs;

            for (uint8_t port=0; port<4; port++) {
                for (uint8_t pin=0; pin < 16; pin++) {
                    ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE + (port << 10));
                    ioRec->pin = 1 << pin;
                    ioRec++;
                }
            }
        }

    protected: // DshotEsc method overrides =============================================

        virtual void deviceInit(void) override
        {
            rccInit();

            ioInit();

            timerInit();

            initTimerMapping();

            initTimerDefinitions();

            initTimer1Hardware();

            dmaInit();

            memset(m_outputBuffer, 0, sizeof(m_outputBuffer));

            uint8_t motorIndex = 0;

            for (auto pin : *m_pins) {

                const IO_t io = _IOGetByTag(pin);

                m_puPdMode = GPIO_PUPD_UP;

                int32_t pinIndex = _IO_GPIOPinIdx(io);

                m_motors[motorIndex].pinIndex = pinIndex;
                m_motors[motorIndex].io = io;
                m_motors[motorIndex].output = 0;
                m_motors[motorIndex].iocfg =
                    io_config(GPIO_MODE_OUT, GPIO_FAST_SPEED, GPIO_OTYPE_PP, m_puPdMode);

                _IOInit(io, OWNER_MOTOR, motorIndex+1);
                _IOConfigGPIO(io, m_motors[motorIndex].iocfg);

                _IO_GPIO(io)->BSRR |= (uint32_t)_IO_Pin(io);

                motorIndex++;
            }
        }        

        virtual bool enable(void) override
        {
            for (auto i=0; i<m_motorCount; i++) {
                if (m_motors[i].configured) {
                    _IOConfigGPIO(m_motors[i].io, m_motors[i].iocfg);
                }
            }
            return true;
        }

        virtual void postInit(void) override
        {
            m_ports[0].timhw = &m_timer1Hardware[0];
            m_ports[1].timhw = &m_timer1Hardware[1];
            m_ports[2].timhw = &m_timer1Hardware[2];
            m_ports[3].timhw = &m_timer1Hardware[3];

            for (auto motorIndex=0; motorIndex<MAX_SUPPORTED_MOTORS && motorIndex <
                    m_motorCount; motorIndex++) {

                if (!motorConfig(motorIndex)) {
                    return;
                }

                m_motors[motorIndex].enabled = true;
            }
        }

        virtual void updateComplete(void) override
        {
            // If there is a dshot command loaded up, time it correctly with motor update

            if (!commandQueueEmpty()) {
                if (!commandOutputIsEnabled()) {
                    return;
                }
            }

            for (auto i=0; i<m_usedMotorPorts; i++) {
                port_t *bbPort = &m_ports[i];

                dmaCmd(bbPort, ENABLE);
            }

            for (auto i=0; i<m_usedMotorPacers; i++) {
                bbPacer_t *bbPacer = &m_pacers[i];
                timDmaCmd(bbPacer->tim, bbPacer->dmaSources, ENABLE);
            }
        }

        virtual bool updateStart(void) override
        {
            for (auto i=0; i<m_usedMotorPorts; i++) {
                dmaCmd(&m_ports[i], DISABLE);
                outputDataClear(m_ports[i].portOutputBuffer);
            }

            return true;
        }

        virtual void write(uint8_t index, float value) override
        {
            uint16_t ivalue = (uint16_t)value;

            motor_t *const bbmotor = &m_motors[index];

            if (!bbmotor->configured) {
                return;
            }

            // If there is a command ready to go overwrite the value and send that instead
            if (commandIsProcessing()) {
                ivalue = commandGetCurrent(index);
            }

            bbmotor->value = ivalue;

            uint16_t packet = prepareDshotPacket(bbmotor->value);

            port_t *bbPort = bbmotor->bbPort;

            outputDataSet(bbPort->portOutputBuffer, bbmotor->pinIndex, packet, false); 
        }

    public:

        Stm32F405DshotEsc(vector<uint8_t> & pins, protocol_t protocol=DSHOT600)
            : DshotEsc(pins, protocol)
        {
        }

        void handleDmaIrq(const uint8_t id)
        {
            const uint8_t index = id - 1;

            dmaCallbackHandlerFuncPtr handler =
                m_dmaDescriptors[index].irqHandlerCallback;

            if (handler) {
                handler(&m_dmaDescriptors[index]);
            }
        }

}; // class Stm32F4DshotEsc
