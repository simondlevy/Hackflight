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

extern FAST_DATA_ZERO_INIT uint8_t dmaMotorTimerCount;
extern motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
extern motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];

extern uint32_t readDoneCount;

FAST_DATA_ZERO_INIT extern uint32_t inputStampUs;

typedef struct dshotDMAHandlerCycleCounters_s {
    uint32_t irqAt;
    uint32_t changeDirectionCompletedAt;
} dshotDMAHandlerCycleCounters_t;

FAST_DATA_ZERO_INIT extern dshotDMAHandlerCycleCounters_t dshotDMAHandlerCycleCounters;


uint8_t getTimerIndex(TIM_TypeDef *timer);
motorDmaOutput_t *getMotorDmaOutput(uint8_t index);
void dshotEnableChannels(uint8_t motorCount);

void pwmDshotSetDirectionOutput(
    motorDmaOutput_t * const motor
);

bool pwmStartDshotMotorUpdate(void);
