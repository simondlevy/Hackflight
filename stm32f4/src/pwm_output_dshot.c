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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <system.h>
#include <time.h>

#include "platform.h"
#include "dma.h"
#include "dma_reqmap.h"
#include "io.h"
#include "nvic.h"
#include "rcc.h"
#include "timer.h"
#include "stm32f4xx.h"
#include "pwm_output.h"
#include "dshot.h"
#include "dshot_dpwm.h"
#include "dshot_command.h"
#include "systemdev.h"

#include "pwm_output_dshot_shared.h"

void dshotEnableChannels(uint8_t motorCount)
{
    for (int i = 0; i < motorCount; i++) {
        if (dmaMotors[i].output & TIMER_OUTPUT_N_CHANNEL) {
            TIM_CCxNCmd(dmaMotors[i].timerHardware->tim, dmaMotors[i].timerHardware->channel, TIM_CCxN_Enable);
        } else {
            TIM_CCxCmd(dmaMotors[i].timerHardware->tim, dmaMotors[i].timerHardware->channel, TIM_CCx_Enable);
        }
    }
}

 void pwmDshotSetDirectionOutput( motorDmaOutput_t * const motor)
{
    TIM_OCInitTypeDef* pOcInit = &motor->ocInitStruct;
    DMA_InitTypeDef* pDmaInit = &motor->dmaInitStruct;

    const timerHardware_t * const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = timerHardware->tim;

    dmaResource_t *dmaRef = motor->dmaRef;

    xDMA_DeInit(dmaRef);

    motor->isInput = false;
    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Disable);
    timerOCInit(timer, timerHardware->channel, pOcInit);
    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Enable);

    if (useBurstDshot) {
        pDmaInit->DMA_DIR = DMA_DIR_MemoryToPeripheral;
    } else {
        pDmaInit->DMA_DIR = DMA_DIR_MemoryToPeripheral;
    }

    xDMA_Init(dmaRef, pDmaInit);
    xDMA_ITConfig(dmaRef, DMA_IT_TC, ENABLE);
}


 static void pwmDshotSetDirectionInput( motorDmaOutput_t * const motor)
{
    DMA_InitTypeDef* pDmaInit = &motor->dmaInitStruct;

    const timerHardware_t * const timerHardware = motor->timerHardware;
    TIM_TypeDef *timer = timerHardware->tim;

    dmaResource_t *dmaRef = motor->dmaRef;

    xDMA_DeInit(dmaRef);

    motor->isInput = true;
    if (!inputStampUs) {
        inputStampUs = micros();
    }
    TIM_ARRPreloadConfig(timer, ENABLE);
    timer->ARR = 0xffffffff;

    TIM_ICInit(timer, &motor->icInitStruct);

    motor->dmaInitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;

    xDMA_Init(dmaRef, pDmaInit);
}


void pwmCompleteDshotMotorUpdate(void)
{
    /* If there is a dshot command loaded up, time it correctly with motor update*/
    if (!dshotCommandQueueEmpty()) {
        if (!dshotCommandOutputIsEnabled(dshotPwmDevice.count)) {
            return;
        }
    }

    for (int i = 0; i < dmaMotorTimerCount; i++) {
        if (useBurstDshot) {
            xDMA_SetCurrDataCounter(dmaMotorTimers[i].dmaBurstRef, dmaMotorTimers[i].dmaBurstLength);
            xDMA_Cmd(dmaMotorTimers[i].dmaBurstRef, ENABLE);
            TIM_DMAConfig(dmaMotorTimers[i].timer, TIM_DMABase_CCR1, TIM_DMABurstLength_4Transfers);
            TIM_DMACmd(dmaMotorTimers[i].timer, TIM_DMA_Update, ENABLE);
        } else {
            TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, DISABLE);
            dmaMotorTimers[i].timer->ARR = dmaMotorTimers[i].outputPeriod;
            TIM_ARRPreloadConfig(dmaMotorTimers[i].timer, ENABLE);
            TIM_SetCounter(dmaMotorTimers[i].timer, 0);
            TIM_DMACmd(dmaMotorTimers[i].timer, dmaMotorTimers[i].timerDmaSources, ENABLE);
            dmaMotorTimers[i].timerDmaSources = 0;
        }
    }
}
