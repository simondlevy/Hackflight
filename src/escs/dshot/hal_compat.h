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

#ifndef DMA_FIFOMODE_ENABLE

#define DMA_FIFOMODE_ENABLE             DMA_FIFOMode_Enable 
#define DMA_FIFO_THRESHOLD_1QUARTERFULL DMA_FIFOThreshold_1QuarterFull

#define TIM_CLOCKDIVISION_DIV1          TIM_CKD_DIV1
#define TIM_COUNTERMODE_UP              TIM_CounterMode_Up
#define TIM_CHANNEL_1                   TIM_Channel_1
#define TIM_CHANNEL_2                   TIM_Channel_2
#define TIM_CHANNEL_3                   TIM_Channel_3
#define TIM_CHANNEL_4                   TIM_Channel_4
#define TIM_OCPOLARITY_HIGH             TIM_OCPolarity_High
#define TIM_OUTPUTSTATE_ENABLE          TIM_OutputState_Enable
#define TIM_CLOCKDIVISION_DIV1          TIM_CKD_DIV1
#define TIM_COUNTERMODE_UP              TIM_CounterMode_Up
#define TIM_OCMODE_TIMING               TIM_OCMode_Timing

#define RCC_APB2LPENR_TIM1LPEN_Msk      RCC_APB2Periph_TIM1 
#define RCC_APB2LPENR_TIM8LPEN_Msk      RCC_APB2Periph_TIM8 
#define RCC_APB2LPENR_USART1LPEN_Msk    RCC_APB2Periph_USART1 
#define RCC_APB2LPENR_USART6LPEN_Msk    RCC_APB2Periph_USART6 
#define RCC_APB2LPENR_ADC1LPEN_Msk      RCC_APB2Periph_ADC1 
#define RCC_APB2LPENR_ADC2LPEN_Msk      RCC_APB2Periph_ADC2 
#define RCC_APB2LPENR_ADC3LPEN_Msk      RCC_APB2Periph_ADC3 
#define RCC_APB2LPENR_SDIOLPEN_Msk      RCC_APB2Periph_SDIO 
#define RCC_APB2LPENR_SPI1LPEN_Msk      RCC_APB2Periph_SPI1 
#define RCC_APB2LPENR_SYSCFGLPEN_Msk    RCC_APB2Periph_SYSCFG 
#define RCC_APB2LPENR_TIM9LPEN_Msk      RCC_APB2Periph_TIM9 
#define RCC_APB2LPENR_TIM10LPEN_Msk     RCC_APB2Periph_TIM10 
#define RCC_APB2LPENR_TIM11LPEN_Msk     RCC_APB2Periph_TIM11 
 
#endif
