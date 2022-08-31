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

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"
#include <time.h>

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
#include "escdev.h"
#include "systemdev.h"

#include "pwm_output_dshot_shared.h"

FAST_DATA_ZERO_INIT uint8_t dmaMotorTimerCount = 0;
motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];

FAST_DATA_ZERO_INIT uint32_t inputStampUs;

FAST_DATA_ZERO_INIT dshotDMAHandlerCycleCounters_t dshotDMAHandlerCycleCounters;

motorDmaOutput_t *getMotorDmaOutput(uint8_t index)
{
    return &dmaMotors[index];
}

uint8_t getTimerIndex(TIM_TypeDef *timer)
{
    for (int i = 0; i < dmaMotorTimerCount; i++) {
        if (dmaMotorTimers[i].timer == timer) {
            return i;
        }
    }
    dmaMotorTimers[dmaMotorTimerCount++].timer = timer;
    return dmaMotorTimerCount - 1;
}


 void pwmWriteDshotInt(uint8_t index, uint16_t value)
{
    motorDmaOutput_t *const motor = &dmaMotors[index];

    if (!motor->configured) {
        return;
    }

    /*If there is a command ready to go overwrite the value and send that instead*/
    if (dshotCommandIsProcessing()) {
        value = dshotCommandGetCurrent(index);
        if (value) {
            motor->protocolControl.requestTelemetry = true;
        }
    }

    motor->protocolControl.value = value;

    uint16_t packet = prepareDshotPacket(&motor->protocolControl);
    uint8_t bufferSize;

    if (useBurstDshot) {
        bufferSize = loadDmaBuffer(&motor->timer->dmaBurstBuffer[timerLookupChannelIndex(motor->timerHardware->channel)], 4, packet);
        motor->timer->dmaBurstLength = bufferSize * 4;
    } else {
        bufferSize = loadDmaBuffer(motor->dmaBuffer, 1, packet);
        motor->timer->timerDmaSources |= motor->timerDmaSource;
        xDMA_SetCurrDataCounter(motor->dmaRef, bufferSize);
        xDMA_Cmd(motor->dmaRef, ENABLE);
    }
}


void dshotEnableChannels(uint8_t motorCount);
