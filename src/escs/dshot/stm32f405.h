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

        static const uint8_t DMA_TIMER_MAPPING_COUNT = 6;

        static const uint32_t NVIC_PRIORITY_GROUPING = 0x500;

        static const uint8_t DEFIO_PORT_USED_COUNT = 6;

        static const uint8_t HARDWARE_TIMER_DEFINITION_COUNT = 14;

        static const uint8_t MAX_TIMER_DMA_OPTIONS = 3;

        static const uint32_t TRANSFER_IT_ENABLE_MASK = 
            (uint32_t)(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);

        static const uint32_t DMA_IT_TCIF  = 0x00000020;
        static const uint32_t DMA_IT_TEIF  = 0x00000008;

        static const uint8_t MAX_MOTORS = 4;
        static const uint8_t STATE_PER_SYMBOL = 3;
        static const uint8_t FRAME_BITS = 16;
        static const uint8_t BUF_LENGTH = FRAME_BITS * STATE_PER_SYMBOL;

        const uint32_t MOTOR_PINS[4]  = {0, 1, 3, 2};
        const uint32_t MOTOR_PORTS[4] = {1, 1, 0, 0};

        // Typedefs =====================================================================

        typedef enum { 
            GPIO_Mode_IN, 
            GPIO_Mode_OUT, 
            GPIO_Mode_AF, 
            GPIO_Mode_AN
        } GPIOMode_TypeDef;

        enum rcc_reg {
            RCC_EMPTY,
            RCC_AHB,
            RCC_APB2,
            RCC_APB1,
            RCC_AHB1,
        };

        typedef void * IO_t; 

        typedef struct {
            TIM_TypeDef *TIMx;
            uint8_t rcc;
        } timerDef_t;

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
            uint8_t                     resourceIndex;
        } dmaChannelDescriptor_t;

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

            uint8_t channel;

            uint16_t dmaSource;

            dmaResource_t *dmaResource; // DMA resource for this port & timer channel
            uint32_t dmaChannel;        // DMA channel or peripheral request

            // DMA resource register cache
            dmaRegCache_t dmaRegOutput;
            dmaRegCache_t dmaRegInput;

            // Output
            uint32_t *portOutputBuffer;

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
            int32_t pinIndex;    
            port_t *bbPort;
        } motor_t;

        typedef struct {
            GPIO_TypeDef *gpio;
            uint16_t pin;
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

        static void RCC_APB2PeriphClockEnable(uint32_t RCC_APB2Periph)
        {
            RCC->APB2ENR |= RCC_APB2Periph;
        }

        static void RCC_AHB1PeriphClockEnable(uint32_t RCC_AHB1Periph)
        {
            RCC->AHB1ENR |= RCC_AHB1Periph;
        }

        static void RCC_APB1PeriphClockEnable(uint32_t RCC_APB1Periph)
        {
            RCC->APB1ENR |= RCC_APB1Periph;
        }

        static void RCC_ClockEnable(uint8_t periphTag)
        {
            int tag = periphTag >> 5;
            uint32_t mask = 1 << (periphTag & 0x1f);

            switch (tag) {
                case RCC_APB2:
                    RCC_APB2PeriphClockEnable(mask);
                    break;
                case RCC_APB1:
                    RCC_APB1PeriphClockEnable(mask);
                    break;
                case RCC_AHB1:
                    RCC_AHB1PeriphClockEnable(mask);
                    break;
            }
        }

        static void loadDmaRegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
        {
            ((DMA_Stream_TypeDef *)dmaResource)->CR = dmaRegCache->CR;
        }

        static void saveDmaRegs(dmaResource_t *dmaResource, dmaRegCache_t *dmaRegCache)
        {
            dmaRegCache->CR = ((DMA_Stream_TypeDef *)dmaResource)->CR;
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

            timDmaCmd(bbPort->dmaSource, DISABLE);

            if (getDmaFlagStatus(descriptor, DMA_IT_TEIF)) {
                while (1) {};
            }

            clearDmaFlag(descriptor, DMA_IT_TCIF);
        }

        static void dmaItConfig(port_t *bbPort)
        {
            DMA_Stream_TypeDef * DMAy_Streamx =
                (DMA_Stream_TypeDef *)bbPort->dmaResource;

            // Check if the DMA_IT parameter contains a FIFO interrupt 
            if ((DMA_IT_TC & DMA_IT_FE) != 0)
            {
                // Enable the selected DMA FIFO interrupts 
                DMAy_Streamx->FCR |= (uint32_t)DMA_IT_FE;
            }

            // Check if the DMA_IT parameter contains a Transfer interrupt 
            if (DMA_IT_TC != DMA_IT_FE) {
                // Enable the selected DMA transfer interrupts 
                DMAy_Streamx->CR |= (uint32_t)(DMA_IT_TC  & TRANSFER_IT_ENABLE_MASK);
            }
        }

        static void dmaPreconfigure(port_t *bbPort)
        {
            DMA_Stream_TypeDef * DMAy_Streamx = (DMA_Stream_TypeDef *)bbPort->dmaResource;

            DMAy_Streamx->CR = 0x0c025450;

            DMAy_Streamx->FCR =
                ((DMAy_Streamx->FCR & (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH)) |
                 (DMA_FIFOMODE_ENABLE | DMA_FIFO_THRESHOLD_1QUARTERFULL));
            DMAy_Streamx->NDTR = BUF_LENGTH;
            DMAy_Streamx->PAR = (uint32_t)&bbPort->gpio->BSRR;
            DMAy_Streamx->M0AR = (uint32_t)bbPort->portOutputBuffer;

            saveDmaRegs(bbPort->dmaResource, &bbPort->dmaRegOutput);
        }

        static void timDmaCmd(uint16_t TIM_DMASource, FunctionalState newState)
        {
            if (newState != DISABLE) {
                // Enable the DMA sources
                TIM1->DIER |= TIM_DMASource; 
            }
            else {
                // Disable the DMA sources
                TIM1->DIER &= (uint16_t)~TIM_DMASource;
            }
        }

        static void outputDataInit(uint32_t *buffer, uint16_t portMask)
        {
            uint32_t resetMask = (portMask << 16);
            uint32_t setMask = portMask;

            for (auto bitpos=0; bitpos<16; bitpos++) {
                buffer[bitpos * 3 + 0] |= setMask ; // Always set all ports
                buffer[bitpos * 3 + 1] = 0;          // Reset bits are port dependent
                buffer[bitpos * 3 + 2] |= resetMask; // Always reset all ports
            }
        }

        static void outputDataSet(uint32_t *buffer, int32_t pinNumber, uint16_t value)
        {
            uint32_t middleBit = (1 << (pinNumber + 16));

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

        static void _IOInit(IO_t io, uint8_t index)
        {
            ioRec_t *ioRec =_IORec(io);
            ioRec->index = index;
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

        void _IOConfigGPIO(uint8_t motorIndex, IO_t io, uint8_t cfg)
        {
            const uint8_t ioPortDefs[6] = {
                { rcc_ahb1(RCC_AHB1ENR_GPIOAEN_MSK) },
                { rcc_ahb1(RCC_AHB1ENR_GPIOBEN_MSK) },
                { rcc_ahb1(RCC_AHB1ENR_GPIOCEN_MSK) },
                { rcc_ahb1(RCC_AHB1ENR_GPIODEN_MSK) },
                { rcc_ahb1(RCC_AHB1ENR_GPIOEEN_MSK) },
                { rcc_ahb1(RCC_AHB1ENR_GPIOFEN_MSK) },
            };

            uint8_t portIndex = MOTOR_PORTS[motorIndex];

            const uint8_t rcc = ioPortDefs[portIndex];

            RCC_ClockEnable(rcc);

            uint32_t pin =_IO_Pin(io);
            uint32_t mode  = (cfg >> 0) & 0x03;
            uint32_t speed = (cfg >> 2) & 0x03;
            uint32_t pull  = (cfg >> 5) & 0x03;

            GPIO_Init(_IO_GPIO(io), pin, mode, speed, pull);
        }

        // Instance variables ===========================================================

        uint8_t m_ioDefUsedOffset[DEFIO_PORT_USED_COUNT] = { 0, 16, 32, 48, 64, 80 };

        port_t m_ports[MAX_MOTORS];

        int32_t m_usedMotorPorts;

        motor_t m_motors[MAX_SUPPORTED_MOTORS];

        uint32_t m_outputBuffer[BUF_LENGTH * MAX_MOTORS];

        uint8_t m_puPdMode;

        dmaChannelDescriptor_t m_dmaDescriptors[DMA_LAST_HANDLER];

        dmaTimerMapping_t m_dmaTimerMapping[DMA_TIMER_MAPPING_COUNT] = {};

        timerDef_t m_timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {};

        uint8_t m_timer1channels[4] = {};

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

        uint16_t m_pacerDmaSources = 0;

        uint16_t m_outputARR;

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

        static void timOcInit(
                volatile uint32_t * ccmr,
                volatile uint32_t * ccr,
                const uint32_t ccer_cc_e,
                const uint32_t ccmr_oc,
                const uint32_t ccmr_cc,
                const uint32_t ccer_ccp,
                const uint32_t ccer_ccnp,
                const uint32_t cr2_ois,
                const uint8_t mode_shift,
                const uint8_t polarity_shift1,
                const uint8_t state_shift,
                const uint8_t polarity_shift2
                )
        {
            TIM1->CCER &= (uint16_t)~ccer_cc_e;
            uint16_t tmpccer = TIM1->CCER;
            uint16_t tmpcr2 =  TIM1->CR2;
            uint16_t tmpccmrx = *ccmr;
            tmpccmrx &= (uint16_t)~ccmr_oc;
            tmpccmrx &= (uint16_t)~ccmr_cc;
            tmpccmrx |= (TIM_OCMODE_TIMING << mode_shift);
            tmpccer &= (uint16_t)~ccer_ccp;
            tmpccer |= (TIM_OCPOLARITY_HIGH << polarity_shift1);
            tmpccer |= (TIM_OUTPUTSTATE_ENABLE < state_shift);

            tmpccer &= (uint16_t)~ccer_ccnp;
            tmpccer |= (TIM_OCPOLARITY_HIGH << polarity_shift2);
            tmpcr2 &= (uint16_t)~cr2_ois;

            TIM1->CR2 = tmpcr2;
            *ccmr = tmpccmrx;
            *ccr = 0x00000000;
            TIM1->CCER = tmpccer;
        }

        static void rccInit(void)
        {
            RCC_APB2PeriphClockEnable(
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
                    RCC_APB2LPENR_TIM11LPEN_Msk);
        }

        void timerChannelInit(port_t *bbPort)
        {
            const uint8_t channel = bbPort->channel;

            // Disable the TIM Counter
            TIM1->CR1 &= (uint16_t)~TIM_CR1_CEN;

            switch (channel) {
                case TIM_CHANNEL_1:
                    timOcInit( &TIM1->CCMR1, &TIM1->CCR1, TIM_CCER_CC1E,
                            TIM_CCMR1_OC1M, TIM_CCMR1_CC1S, TIM_CCER_CC1P,
                            TIM_CCER_CC1NP, TIM_CR2_OIS1, 0, 0, 0, 0);
                    break;
                case TIM_CHANNEL_2:
                    timOcInit( &TIM1->CCMR1, &TIM1->CCR2, TIM_CCER_CC2E,
                            TIM_CCMR1_OC2M, TIM_CCMR1_CC2S, TIM_CCER_CC2P,
                            TIM_CCER_CC2NP, TIM_CR2_OIS2, 8, 4, 4, 4);
                    break;
                case TIM_CHANNEL_3:
                    timOcInit( &TIM1->CCMR2, &TIM1->CCR3, TIM_CCER_CC3E,
                            TIM_CCMR2_OC3M, TIM_CCMR2_CC3S, TIM_CCER_CC3P,
                            TIM_CCER_CC3NP, TIM_CR2_OIS3, 0, 8, 8, 8);
                    break;
                case TIM_CHANNEL_4:
                    timOcInit( &TIM1->CCMR2, &TIM1->CCR4, TIM_CCER_CC4E,
                            TIM_CCMR2_OC4M, TIM_CCMR2_CC4S, TIM_CCER_CC4P,
                            TIM_CCER_CC4NP, TIM_CR2_OIS4, 8, 12, 12, 4);
                    break;
            }

            // Enable the TIM Counter
            TIM1->CR1 |= TIM_CR1_CEN;
        }

        void updateStartMotorPort(uint8_t index)
        {
            dmaCmd(&m_ports[index], DISABLE);
            outputDataClear(m_ports[index].portOutputBuffer);
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

        dmaIdentifier_e findDmaIdentifier(const dmaResource_t* channel)
        {
            for (uint8_t i=0; i<DMA_LAST_HANDLER; i++) {
                if (m_dmaDescriptors[i].ref == channel) {
                    return (dmaIdentifier_e)(i + 1);
                }
            }

            return DMA_NONE;
        }

        port_t *allocateMotorPort(IO_t io, uint8_t motorIndex, int32_t portIndex)
        {
            port_t *bbPort = &m_ports[m_usedMotorPorts];
            bbPort->portIndex = portIndex;
            ++m_usedMotorPorts;

            static uint8_t options[4] = {1, 0, 1, 0};
            const int8_t option = options[motorIndex];
            const dmaChannelSpec_t *dmaChannelSpec =
                dmaGetChannelSpecByTimerValue(TIM1, bbPort->channel, option); 
            bbPort->dmaResource = dmaChannelSpec->ref;
            bbPort->dmaChannel = dmaChannelSpec->channel;

            bbPort->gpio =_IO_GPIO(io);

            bbPort->portOutputBuffer = &m_outputBuffer[(bbPort - m_ports) * BUF_LENGTH];

            uint16_t tmpcr1 = TIM1->CR1;  

            // Select the Counter Mode
            tmpcr1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS));
            tmpcr1 |= (uint32_t)TIM_COUNTERMODE_UP;

            // Set the clock division 
            tmpcr1 &=  (uint16_t)(~TIM_CR1_CKD);
            tmpcr1 |= (uint32_t)TIM_CLOCKDIVISION_DIV1;

            TIM1->CR1 = tmpcr1;

            // Set the Autoreload value 
            TIM1->ARR = m_outputARR;
            TIM1->PSC = 0;
            TIM1->RCR = 0;
            TIM1->EGR = 0x0001;          

            TIM1->CR1 |= TIM_CR1_ARPE;

            timerChannelInit(bbPort);

            setupDma(motorIndex, bbPort);

            dmaPreconfigure(bbPort);

            dmaItConfig(bbPort);

            return bbPort;
        }

        void dmaSetHandler(
                dmaIdentifier_e identifier,
                dmaCallbackHandlerFuncPtr callback,
                uint32_t priority,
                uint32_t userParam)
        {
            const int8_t index = identifier - 1;

            RCC_AHB1PeriphClockEnable(RCC_AHB1PERIPH_DMA2);
            m_dmaDescriptors[index].irqHandlerCallback = callback;
            m_dmaDescriptors[index].userParam = userParam;

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

    public:

        void setupDma(uint8_t motorIndex, port_t *bbPort)
        {
            RCC_AHB1PeriphClockEnable(RCC_AHB1PERIPH_DMA2);

            bbPort->dmaSource = timerDmaSource(bbPort->channel);

            m_pacerDmaSources |= bbPort->dmaSource;

            dmaSetHandler(
                    findDmaIdentifier(bbPort->dmaResource),
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
            m_timer1channels[0] = TIM_CHANNEL_1;
            m_timer1channels[1] = TIM_CHANNEL_2;
            m_timer1channels[2] = TIM_CHANNEL_3;
            m_timer1channels[3] = TIM_CHANNEL_4;
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

        void timerInit(void)
        {
            RCC_ClockEnable(timerRCC(TIM5));
            RCC_ClockEnable(timerRCC(TIM2));
            RCC_ClockEnable(timerRCC(TIM2));
            RCC_ClockEnable(timerRCC(TIM5));
        }

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

            uint32_t outputFreq = 1000 * getDshotBaseFrequency(m_protocol);
            m_outputARR = SystemCoreClock / outputFreq - 1;

            memset(m_outputBuffer, 0, sizeof(m_outputBuffer));

            uint8_t motorIndex = 0;

            for (auto pin : *m_pins) {

                const IO_t io = _IOGetByTag(pin);

                m_puPdMode = GPIO_PUPD_UP;

                int32_t pinIndex = MOTOR_PINS[motorIndex];

                m_motors[motorIndex].pinIndex = pinIndex;

                uint32_t iocfg =
                    io_config(GPIO_MODE_OUT, GPIO_FAST_SPEED, GPIO_OTYPE_PP, m_puPdMode);

                _IOInit(io, motorIndex+1);
                _IOConfigGPIO(motorIndex, io, iocfg);

                _IO_GPIO(io)->BSRR |= (uint32_t)_IO_Pin(io);

                m_ports[motorIndex].channel = m_timer1channels[motorIndex];

                int32_t portIndex = MOTOR_PORTS[motorIndex];

                port_t *bbPort = findMotorPort(portIndex);

                if (!bbPort) {
                    bbPort = allocateMotorPort(io, motorIndex, portIndex);
                }

                motor_t * bbMotor = &m_motors[motorIndex];

                bbMotor->pinIndex = pinIndex;
                bbMotor->bbPort = bbPort;

                _IOInit(io, motorIndex+1);

                _IO_GPIO(io)->BSRR |= (((uint32_t)(_IO_Pin(io))) << 16);

                _IOConfigGPIO(motorIndex, io, io_config(GPIO_Mode_OUT,
                            GPIO_FAST_SPEED, GPIO_OTYPE_PP, m_puPdMode));

                outputDataInit(bbPort->portOutputBuffer, (1 << pinIndex)); 

                bbPort->gpio->BSRR = (1 << (pinIndex + 16));  // BR (higher half)

                // Set GPIO to output
                ATOMIC_BLOCK(nvic_build_priority(1, 1)) {
                    uint32_t gpioModeMask = (GPIO_MODER_MODER0 << (pinIndex * 2));
                    uint32_t gpioModeOutput = (GPIO_Mode_OUT << (pinIndex * 2));
                    MODIFY_REG(bbPort->gpio->MODER, gpioModeMask, gpioModeOutput);
                }

                // Reinitialize port group DMA for output

                dmaResource_t *dmaResource = bbPort->dmaResource;
                loadDmaRegs(dmaResource, &bbPort->dmaRegOutput);

                // Reinitialize pacer timer for output

                TIM1->ARR = m_outputARR;

                _IOConfigGPIO(motorIndex, io, iocfg);

                motorIndex++;
            }
        }        

        virtual void updateComplete(void) override
        {

            dmaCmd(&m_ports[0], ENABLE);
            dmaCmd(&m_ports[1], ENABLE);

            timDmaCmd(m_pacerDmaSources, ENABLE);
        }

        virtual void updateStart(void) override
        {
            updateStartMotorPort(0);
            updateStartMotorPort(1);
        }

        virtual void write(uint8_t index, float value) override
        {
            uint16_t ivalue = (uint16_t)value;

            motor_t * const bbmotor = &m_motors[index];

            if (commandIsProcessing()) {
                ivalue = commandGetCurrent(index);
            }

            uint16_t packet = prepareDshotPacket(ivalue);

            port_t *bbPort = bbmotor->bbPort;

            outputDataSet(bbPort->portOutputBuffer, bbmotor->pinIndex, packet); 
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
