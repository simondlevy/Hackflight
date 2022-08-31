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

#include "dshot.h"
#include "escdev.h"

#define MOTOR_DSHOT600_HZ     MHZ_TO_HZ(12)
#define MOTOR_DSHOT300_HZ     MHZ_TO_HZ(6)
#define MOTOR_DSHOT150_HZ     MHZ_TO_HZ(3)

#define MOTOR_BIT_0           7
#define MOTOR_BIT_1           14
#define MOTOR_BITLENGTH       20

#define MOTOR_PROSHOT1000_HZ         MHZ_TO_HZ(24)
#define PROSHOT_BASE_SYMBOL          24 // 1uS
#define PROSHOT_BIT_WIDTH            3
#define MOTOR_NIBBLE_LENGTH_PROSHOT  (PROSHOT_BASE_SYMBOL * 4) // 4uS

#define DSHOT_TELEMETRY_DEADTIME_US   (30 + 5) // 30 to switch lines and 5 to switch lines back


typedef uint8_t loadDmaBufferFn(uint32_t *dmaBuffer, int stride, uint16_t packet);  // function pointer used to encode a digital motor value into the DMA buffer representation
extern FAST_DATA_ZERO_INIT loadDmaBufferFn *loadDmaBuffer;
uint8_t loadDmaBufferDshot(uint32_t *dmaBuffer, int stride, uint16_t packet);
uint8_t loadDmaBufferProshot(uint32_t *dmaBuffer, int stride, uint16_t packet);

uint32_t getDshotHz(escProtocol_t pwmProtocolType);

struct escDevConfig_s;

/* Motor DMA related, moved from pwm_output.h */

#define MAX_DMA_TIMERS        8

#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */
#define PROSHOT_DMA_BUFFER_SIZE 6  /* resolution + frame reset (2us) */

#define GCR_TELEMETRY_INPUT_LEN MAX_GCR_EDGES

#define DSHOT_DMA_BUFFER_ATTRIBUTE // None

#define DSHOT_DMA_BUFFER_UNIT uint32_t

#define DSHOT_DMA_BUFFER_ALLOC_SIZE GCR_TELEMETRY_INPUT_LEN

extern DSHOT_DMA_BUFFER_UNIT dshotDmaBuffer[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];
extern DSHOT_DMA_BUFFER_UNIT dshotDmaInputBuffer[MAX_SUPPORTED_MOTORS][DSHOT_DMA_BUFFER_ALLOC_SIZE];

extern DSHOT_DMA_BUFFER_UNIT dshotBurstDmaBuffer[MAX_DMA_TIMERS][DSHOT_DMA_BUFFER_SIZE * 4];

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

void pwmWriteDshotInt(uint8_t index, uint16_t value);
bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, escProtocol_t pwmProtocolType, uint8_t output);
void pwmCompleteDshotMotorUpdate(void);
extern bool useBurstDshot;
extern escDevice_t dshotPwmDevice;
