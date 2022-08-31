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

#include "dshot_dev.h"

#include "timer.h"
#include "io.h"

#define DSHOT_DMA_BUFFER_UNIT uint32_t

typedef struct {
    TIM_TypeDef *timer;
    uint16_t outputPeriod;
    dmaResource_t *dmaBurstRef;
    uint16_t dmaBurstLength;
    uint32_t *dmaBurstBuffer;
    uint16_t timerDmaSources;
} motorDmaTimer_t;

typedef struct motorDmaOutput_s {
    dshotProtocolControl_t protocolControl;
    ioTag_t ioTag;
    const timerHardware_t *timerHardware;
    uint16_t timerDmaSource;
    uint8_t timerDmaIndex;
    bool configured;
    uint8_t output;
    uint8_t index;
    uint32_t iocfg;
    DMA_InitTypeDef   dmaInitStruct;
    volatile bool isInput;
    int32_t dshotTelemetryDeadtimeUs;
    uint8_t dmaInputLen;
    TIM_OCInitTypeDef ocInitStruct;
    TIM_ICInitTypeDef icInitStruct;
    dmaResource_t *dmaRef;
    motorDmaTimer_t *timer;
    DSHOT_DMA_BUFFER_UNIT *dmaBuffer;
} motorDmaOutput_t;

#if defined(__cplusplus)
extern "C" {
#endif
    motorDmaOutput_t *getMotorDmaOutput(uint8_t index);
#if defined(__cplusplus)
}
#endif
