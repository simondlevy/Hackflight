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

#include "board/stm32/stm32f.h"
#include "imu/softquat/invensense.h"

#include <stm32f4xx.h>

    __attribute__( ( always_inline ) )
static inline void __set_BASEPRI_nb(const uint32_t basePri)
{
    __ASM volatile ("\tMSR basepri, %0\n" : : "r" (basePri) );
}

static inline void __basepriRestoreMem(const uint8_t *val)
{
    __set_BASEPRI(*val);
}

static inline uint8_t __basepriSetMemRetVal(const uint8_t prio)
{
    __set_BASEPRI_MAX(prio);
    return 1;
}

#define ATOMIC_BLOCK(prio) \
    for ( uint8_t __basepri_save __attribute__ \
            ((__cleanup__ (__basepriRestoreMem), __unused__)) = __get_BASEPRI(), \
            __ToDo = __basepriSetMemRetVal(prio); __ToDo ; __ToDo = 0 )

class Stm32F4Board : public Stm32FBoard {

    private:

        // Constants ===================================================================

        static const uint8_t GPIO_FAST_SPEED = 0x02;
        static const uint8_t GPIO_PUPD_UP    = 0x01;
        static const uint8_t GPIO_OTYPE_PP   = 0x00;

        static const uint32_t RCC_AHB1PERIPH_DMA2 = 0x00400000;

        static const uint32_t NVIC_PRIORITY_GROUPING = 0x500;

        static const uint32_t TRANSFER_IT_ENABLE_MASK = 
            (uint32_t)(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);

        static const uint32_t DMA_IT_TCIF  = 0x00000020;

        static const uint8_t STATE_PER_SYMBOL = 3;
        static const uint8_t FRAME_BITS = 16;
        static const uint8_t BUF_LENGTH = FRAME_BITS * STATE_PER_SYMBOL;

        // Enums =======================================================================

        enum { 
            GPIO_MODE_IN, 
            GPIO_MODE_OUT, 
            GPIO_MODE_AF, 
            GPIO_MODE_AN
        } ;

        enum rcc_reg {
            RCC_EMPTY,
            RCC_AHB,
            RCC_APB2,
            RCC_APB1,
            RCC_AHB1,
        };

        // Typedefs ====================================================================

        typedef struct {
            DMA_Stream_TypeDef * dmaStream;
            uint16_t             dmaSource;
            uint32_t             outputBuffer[BUF_LENGTH];
            uint8_t              flagsShift;
        } port_t;

        typedef struct {
            uint32_t middleBit;    
            port_t * port;
        } motor_t;

        typedef struct {
            GPIO_TypeDef *gpio;
        } ioRec_t;

        // Static local funs ===========================================================

        static uint32_t log2_8bit(const uint32_t v)  
        {
            return 8 - 90/((v/4+14)|1) - 2/(v/2+1);
        }

        static uint32_t log2_16bit(const uint32_t v) 
        {
            return 8*(v>255) + log2_8bit(v >>8*(v>255));
        }

        static uint32_t log2_32bit(const uint32_t v) 
        {
            return 16*(v>65535L) + log2_16bit(v*1L >>16*(v>65535L));
        }


        static uint32_t rcc_encode(const uint32_t reg, const uint32_t mask) 
        {
            return (reg << 5) | log2_32bit(mask);
        }

        static uint32_t nvic_build_priority(const uint32_t base, const uint32_t sub) 
        {
            return (((((base)<<(4-(7-(NVIC_PRIORITY_GROUPING>>8))))|
                            ((sub)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8)))))<<4)&0xf0);
        }

        static uint32_t nvic_priority_base(const uint32_t prio) 
        {
            return (((prio)>>(4-(7-(NVIC_PRIORITY_GROUPING>>8))))>>4);
        }

        static uint32_t nvic_priority_sub(const uint32_t prio) 
        {
            return (((prio)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING>>8))))>>4);
        }

        static void RCC_APB2PeriphClockEnable(const uint32_t mask)
        {
            RCC->APB2ENR |= mask;
        }

        static void RCC_AHB1PeriphClockEnable(const uint32_t mask)
        {
            RCC->AHB1ENR |= mask;
        }

        static void dmaCmd(const port_t * port, const FunctionalState newState)
        {
            DMA_Stream_TypeDef * DMAy_Streamx = port->dmaStream;

            DMAy_Streamx->CR  = 
                newState == DISABLE ?
                DMAy_Streamx->CR & ~(uint32_t)DMA_SxCR_EN :
                DMAy_Streamx->CR | (uint32_t)DMA_SxCR_EN;
        }

        static void timDmaCmd(const uint16_t TIM_DMASource, const FunctionalState newState)
        {
            TIM1->DIER = 
                newState == DISABLE ? 
                TIM1->DIER & ~(uint16_t)TIM_DMASource :
                TIM1->DIER | TIM_DMASource;
        }

        static uint8_t rcc_ahb1(const uint32_t gpio)
        {
            return (uint8_t)rcc_encode(RCC_AHB1, gpio); 
        }

        // Instance variables ==========================================================

        port_t m_ports[2];

        uint8_t m_portCount;

        motor_t m_motors[4];

        GPIO_TypeDef * m_gpios[96];

        uint16_t m_pacerDmaMask = 0x0000;

        // Private instance methods ====================================================

        void dmaUpdateStartMotorPort(port_t * port)
        {
            dmaCmd(port, DISABLE);

            for (auto bitpos=0; bitpos<16; bitpos++) {
                port->outputBuffer[bitpos * 3 + 1] = 0;
            }
        }

        virtual void dmaWriteMotor(const uint8_t motorIndex, uint16_t packet) override
        {
            motor_t * const motor = &m_motors[motorIndex];
            port_t *port = motor->port;

            for (auto pos=0; pos<16; pos++) {
                if (!(packet & 0x8000)) {
                    port->outputBuffer[pos * 3 + 1] |= motor->middleBit;
                }
                packet <<= 1;
            }        
        }

        void initPort(
                const uint8_t portIndex,
                const uint16_t dmaSource,
                DMA_Stream_TypeDef * stream,
                const uint8_t flagsShift,
                const IRQn_Type irqChannel,
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
                const uint8_t polarity_shift2)
        {
            port_t * port = &m_ports[portIndex];
            port->dmaStream = stream;
            port->flagsShift = flagsShift;

            memset(port->outputBuffer, 0, sizeof(port->outputBuffer));

            TIM1->CR1 = (TIM1->CR1 & (uint16_t)~TIM_CR1_CEN) | TIM_CR1_CEN;

            RCC_AHB1PeriphClockEnable(RCC_AHB1PERIPH_DMA2);

            port->dmaSource = dmaSource;

            m_pacerDmaMask |= port->dmaSource;

            const uint32_t priority = nvic_build_priority(2, 1);

            RCC_AHB1PeriphClockEnable(RCC_AHB1PERIPH_DMA2);

            const uint8_t tmppriority = (0x700 - ((SCB->AIRCR) & (uint32_t)0x700))>> 0x08;
            const uint8_t tmppre = (0x4 - tmppriority);
            const uint8_t tmppriority2 = nvic_priority_base(priority) << tmppre;

            NVIC->IP[irqChannel] = tmppriority2 << 0x04;

            NVIC->ISER[irqChannel >> 0x05] =
                (uint32_t)0x01 << (irqChannel & (uint8_t)0x1F);

            DMA_Stream_TypeDef * DMAy_Streamx = port->dmaStream;

            DMAy_Streamx->CR = 0x0c025450;

            DMAy_Streamx->FCR =
                ((DMAy_Streamx->FCR & (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH)) |
                 (DMA_FIFOMODE_ENABLE | DMA_FIFO_THRESHOLD_1QUARTERFULL));
            DMAy_Streamx->NDTR = BUF_LENGTH;
            DMAy_Streamx->M0AR = (uint32_t)port->outputBuffer;

            DMAy_Streamx->CR |= (uint32_t)(DMA_IT_TC  & TRANSFER_IT_ENABLE_MASK);

            TIM1->CR2 = TIM1->CR2 & (uint16_t)~cr2_ois;

            TIM1->CCMR1 = (TIM1->CCMR1 & (uint16_t)~ccmr_oc & (uint16_t)~ccmr_cc) |
                (TIM_OCMODE_TIMING << mode_shift);

            TIM1->CCER = (TIM1->CCER & (uint16_t)~ccer_cc_e & (uint16_t)~ccer_ccp) |
                (TIM_OCPOLARITY_HIGH << polarity_shift1) | 
                ((TIM_OUTPUTSTATE_ENABLE < state_shift) & (uint16_t)~ccer_ccnp) |
                (TIM_OCPOLARITY_HIGH << polarity_shift2);

            *ccr = 0x00000000;

        } // initPort

        virtual void dmaInit(
                const std::vector<uint8_t> * motorPins, const uint32_t outputFreq) override
        {
            RCC_APB2PeriphClockEnable(
                    RCC_APB2LPENR_TIM1LPEN_Msk   |
                    RCC_APB2LPENR_USART1LPEN_Msk |
                    RCC_APB2LPENR_USART6LPEN_Msk |
                    RCC_APB2LPENR_ADC1LPEN_Msk   |
                    RCC_APB2LPENR_SDIOLPEN_Msk   |
                    RCC_APB2LPENR_SPI1LPEN_Msk   |
                    RCC_APB2LPENR_SYSCFGLPEN_Msk |
                    RCC_APB2LPENR_TIM9LPEN_Msk   |
                    RCC_APB2LPENR_TIM10LPEN_Msk  |
                    RCC_APB2LPENR_TIM11LPEN_Msk);

            uint8_t k = 0;
            for (uint8_t port=0; port<4; port++) {
                for (uint8_t pin=0; pin < 16; pin++) {
                    m_gpios[k] = (GPIO_TypeDef *)(GPIOA_BASE + (port << 10));
                    k++;
                }
            }

            TIM1->CR1 = (TIM1->CR1 & ((uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS)))) |
                (((uint32_t)TIM_COUNTERMODE_UP) & ((uint16_t)(~TIM_CR1_CKD))) | 
                ((uint32_t)TIM_CLOCKDIVISION_DIV1) |
                TIM_CR1_ARPE;

            const uint16_t outputARR = SystemCoreClock / outputFreq - 1;

            // Set the Autoreload value 
            TIM1->ARR = outputARR;
            TIM1->PSC = 0;
            TIM1->RCR = 0;
            TIM1->EGR = 0x0001;          

            // Reinitialize pacer timer for output
            TIM1->ARR = outputARR;

            initPortsAndMotors(motorPins);
        }

        virtual void dmaUpdateComplete(void) override
        {
            for (auto k=0; k<m_portCount; ++k) {
                dmaCmd(&m_ports[k], ENABLE);
            }

            timDmaCmd(m_pacerDmaMask, ENABLE);
        }

        virtual void dmaUpdateStart(void) override
        {
            for (auto k=0; k<m_portCount; ++k) {
                dmaUpdateStartMotorPort(&m_ports[k]);
            }
        }

    protected:

        void initMotor(
                const std::vector<uint8_t> * motorPins,
                const uint8_t motorIndex,
                const uint8_t portIndex)
        {
            const uint8_t motorPin = (*motorPins)[motorIndex];

            // 0, 1, 2, 3, ...
            const uint8_t pinIndex = motorPin & 0x0f;

            m_motors[motorIndex].middleBit = (1 << (pinIndex + 16));

            const uint8_t rcc = rcc_ahb1(portIndex+1);

            const uint32_t mask = 1 << (rcc & 0x1f);

            RCC_AHB1PeriphClockEnable(mask);

            const uint8_t config =
                GPIO_MODE_OUT |
                (GPIO_FAST_SPEED << 2) |
                (GPIO_OTYPE_PP << 4) |
                (GPIO_PUPD_UP << 5);

            const uint32_t mode  = (config >> 0) & 0x03;
            const uint32_t speed = (config >> 2) & 0x03;
            const uint32_t pull  = (config >> 5) & 0x03;

            GPIO_TypeDef * gpio = m_gpios[motorPin]; // XXX

            gpio->MODER  &= ~(GPIO_MODER_MODER0 << (pinIndex * 2));
            gpio->MODER |= (mode << (pinIndex * 2));

            gpio->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinIndex * 2));
            gpio->OSPEEDR |= (speed << (pinIndex * 2));

            gpio->OTYPER  &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)pinIndex)) ;

            gpio->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)pinIndex * 2));
            gpio->PUPDR |= (pull << (pinIndex * 2)); 

            const uint8_t pinmask = 1 << pinIndex;

            gpio->BSRR |= pinmask;

            port_t * port = &m_ports[portIndex];

            m_motors[motorIndex].port = port;

            if (motorIndex == 0 || motorIndex == 2) {
                port->dmaStream->PAR = (uint32_t)&gpio->BSRR;
            }

            gpio->BSRR |= (pinmask << 16);

            uint32_t * buffer = port->outputBuffer;
            const uint16_t portMask = 1 << pinIndex;
            const uint32_t resetMask = (portMask << 16);
            const uint32_t setMask = portMask;

            for (auto bitpos=0; bitpos<16; bitpos++) {
                buffer[bitpos * 3 + 0] |= setMask ; // Always set all ports
                buffer[bitpos * 3 + 1] = 0;          // Reset bits are port dependent
                buffer[bitpos * 3 + 2] |= resetMask; // Always reset all ports
            }

            gpio->BSRR = (1 << (pinIndex + 16));  // BR (higher half)

            // Set GPIO to output
            ATOMIC_BLOCK(nvic_build_priority(1, 1)) {
                uint32_t gpioModeMask = (GPIO_MODER_MODER0 << (pinIndex * 2));
                uint32_t gpioModeOutput = (GPIO_MODE_OUT << (pinIndex * 2));
                MODIFY_REG(gpio->MODER, gpioModeMask, gpioModeOutput);
            }

        } // initMotor

        void initStream1(uint8_t portIndex)
        {
            initPort(portIndex, TIM_DMA_CC1, DMA2_Stream1, 6,  DMA2_Stream1_IRQn,
                    &TIM1->CCR1, TIM_CCER_CC1E,
                    TIM_CCMR1_OC1M, TIM_CCMR1_CC1S, TIM_CCER_CC1P,
                    TIM_CCER_CC1NP, TIM_CR2_OIS1, 0, 0, 0, 0);
        }

        void initStream2(uint8_t portIndex)
        {
            initPort(portIndex, TIM_DMA_CC2, DMA2_Stream2, 16, DMA2_Stream2_IRQn,
                    &TIM1->CCR2, TIM_CCER_CC2E,
                    TIM_CCMR1_OC2M, TIM_CCMR1_CC2S, TIM_CCER_CC2P,
                    TIM_CCER_CC2NP, TIM_CR2_OIS2, 8, 4, 4, 4);
        }

        virtual void initPortsAndMotors(const std::vector<uint8_t> * motorPins) = 0;

    public:

        Stm32F4Board(
                const uint8_t portCount,
                Receiver & receiver,
                InvenSenseImu & imu,
                std::vector<PidController *> & pids,
                Mixer & mixer,
                Esc & esc,
                const uint8_t ledPin)
            : Stm32FBoard(receiver, imu, pids, mixer, esc, ledPin)
        {
            m_portCount = portCount;
        }

        void handleDmaIrq(const uint8_t portIndex)
        {
            port_t *port = &m_ports[portIndex];

            dmaCmd(port, DISABLE);

            timDmaCmd(port->dmaSource, DISABLE);

            DMA2->LIFCR = (DMA_IT_TCIF << port->flagsShift);
        }

        virtual void reboot(void) override
        {
            __enable_irq();
            HAL_RCC_DeInit();
            HAL_DeInit();
            SysTick->CTRL = SysTick->LOAD = SysTick->VAL = 0;
            __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

            const uint32_t p = (*((uint32_t *) 0x1FFF0000));
            __set_MSP( p );

            void (*SysMemBootJump)(void);
            SysMemBootJump = (void (*)(void)) (*((uint32_t *) 0x1FFF0004));
            SysMemBootJump();

            NVIC_SystemReset();
        }

}; // class Stm32F4Board
