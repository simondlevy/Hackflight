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

class Stm32F405Board : public Stm32F4Board {

    virtual void initPortsAndMotors(vector<uint8_t> * motorPins) override
    {
        initPort(1, TIM_DMA_CC1, DMA2_Stream1, 6,  DMA2_Stream1_IRQn,
                &TIM1->CCR1, TIM_CCER_CC1E,
                TIM_CCMR1_OC1M, TIM_CCMR1_CC1S, TIM_CCER_CC1P,
                TIM_CCER_CC1NP, TIM_CR2_OIS1, 0, 0, 0, 0);

        initPort(0, TIM_DMA_CC2, DMA2_Stream2, 16, DMA2_Stream2_IRQn,
                &TIM1->CCR2, TIM_CCER_CC2E,
                TIM_CCMR1_OC2M, TIM_CCMR1_CC2S, TIM_CCER_CC2P,
                TIM_CCER_CC2NP, TIM_CR2_OIS2, 8, 4, 4, 4);

        initMotor(motorPins, 0, 0); 
        initMotor(motorPins, 1, 0);
        initMotor(motorPins, 2, 1);
        initMotor(motorPins, 3, 1);
    }        

    public:

    Stm32F405Board(
            Receiver & receiver,
            Imu & imu,
            Imu::align_fun align,
            vector<PidController *> & pids,
            Mixer & mixer,
            Esc & esc,
            const uint8_t ledPin) 
        : Stm32F4Board(2, receiver, imu, align, pids, mixer, esc, ledPin)
    {
    }
};
