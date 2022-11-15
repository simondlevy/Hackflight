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

#include "boards/stm32/stm32f4.h"

#include <stm32f4xx.h>

class Stm32F411Board : public Stm32F4Board {

    protected: // DshotEsc method overrides ============================================

        virtual void dmaInit(
                vector<uint8_t> * motorPins, uint32_t outputFreq) override
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

            initPort(0, TIM_DMA_CC1, DMA2_Stream1, 6,  DMA2_Stream1_IRQn,
                    &TIM1->CCR1, TIM_CCER_CC1E,
                    TIM_CCMR1_OC1M, TIM_CCMR1_CC1S, TIM_CCER_CC1P,
                    TIM_CCER_CC1NP, TIM_CR2_OIS1, 0, 0, 0, 0);

            initMotor(motorPins, 0, 0); 
            initMotor(motorPins, 1, 0);
            initMotor(motorPins, 2, 0);
            initMotor(motorPins, 3, 0);
        }        

    public:

        Stm32F411Board(
                Receiver & receiver,
                Imu & imu,
                Imu::align_fun align,
                vector<PidController *> & pids,
                Mixer & mixer,
                Esc & esc,
                const uint8_t ledPin) 
            : Stm32F4Board(1, receiver, imu, align, pids, mixer, esc, ledPin)
        {
        }
};
