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

#include "boards/stm32.h"

#include <stm32f4xx.h>

__attribute__( ( always_inline ) ) static inline void __set_BASEPRI_nb(const uint32_t basePri)
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

class Stm32F4Board : public Stm32Board {

    protected:

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

    public:

        Stm32F4Board(
                Receiver & receiver,
                Imu & imu,
                Imu::align_fun align,
                vector<PidController *> & pids,
                Mixer & mixer,
                Esc & esc,
                const uint8_t ledPin) 
            : Stm32Board(receiver, imu, align, pids, mixer, esc, ledPin)
        {
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
