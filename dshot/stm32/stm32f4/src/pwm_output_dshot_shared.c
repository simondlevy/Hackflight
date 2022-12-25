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
#include "motordev.h"
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


static uint32_t decodeTelemetryPacket(uint32_t buffer[], uint32_t count)
{
    uint32_t value = 0;
    uint32_t oldValue = buffer[0];
    int bits = 0;
    int len;
    for (uint32_t i = 1; i <= count; i++) {
        if (i < count) {
            int diff = buffer[i] - oldValue;
            if (bits >= 21) {
                break;
            }
            len = (diff + 8) / 16;
        } else {
            len = 21 - bits;
        }

        value <<= len;
        value |= 1 << (len - 1);
        oldValue = buffer[i];
        bits += len;
    }
    if (bits != 21) {
        return 0xffff;
    }

    static const uint32_t decode[32] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
        0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0 };

    uint32_t decodedValue = decode[value & 0x1f];
    decodedValue |= decode[(value >> 5) & 0x1f] << 4;
    decodedValue |= decode[(value >> 10) & 0x1f] << 8;
    decodedValue |= decode[(value >> 15) & 0x1f] << 12;

    uint32_t csum = decodedValue;
    csum = csum ^ (csum >> 8); // xor bytes
    csum = csum ^ (csum >> 4); // xor nibbles

    if ((csum & 0xf) != 0xf) {
        return 0xffff;
    }
    decodedValue >>= 4;

    if (decodedValue == 0x0fff) {
        return 0;
    }
    decodedValue = (decodedValue & 0x000001ff) << ((decodedValue & 0xfffffe00) >> 9);
    if (!decodedValue) {
        return 0xffff;
    }
    uint32_t ret = (1000000 * 60 / 100 + decodedValue / 2) / decodedValue;
    return ret;
}

 bool pwmStartDshotMotorUpdate(void)
{
    if (!useDshotTelemetry) {
        return true;
    }
    const uint32_t currentTimeMs = millis();
    const uint32_t currentUs = micros();
    for (int i = 0; i < dshotPwmDevice.count; i++) {
        int32_t usSinceInput = cmpTimeUs(currentUs, inputStampUs);
        if (usSinceInput >= 0 && usSinceInput < dmaMotors[i].dshotTelemetryDeadtimeUs) {
            return false;
        }
        if (dmaMotors[i].isInput) {
            uint32_t edges = GCR_TELEMETRY_INPUT_LEN - xDMA_GetCurrDataCounter(dmaMotors[i].dmaRef);

            TIM_DMACmd(dmaMotors[i].timerHardware->tim, dmaMotors[i].timerDmaSource, DISABLE);

            uint16_t value = 0xffff;

            if (edges > MIN_GCR_EDGES) {
                dshotTelemetryState.readCount++;
                value = decodeTelemetryPacket(dmaMotors[i].dmaBuffer, edges);

                bool validTelemetryPacket = false;
                if (value != 0xffff) {
                    dshotTelemetryState.motorState[i].telemetryValue = value;
                    dshotTelemetryState.motorState[i].telemetryActive = true;
                    if (i < 4) {
                    }
                    validTelemetryPacket = true;
                } else {
                    dshotTelemetryState.invalidPacketCount++;
                    if (i == 0) {
                        memcpy(dshotTelemetryState.inputBuffer, dmaMotors[i].dmaBuffer, sizeof(dshotTelemetryState.inputBuffer));
                    }
                }
                updateDshotTelemetryQuality(&dshotTelemetryQuality[i], validTelemetryPacket, currentTimeMs);
            }
        }
        pwmDshotSetDirectionOutput(&dmaMotors[i]);
    }
    inputStampUs = 0;
    dshotEnableChannels(dshotPwmDevice.count);
    return true;
}

bool isDshotMotorTelemetryActive(uint8_t motorIndex)
{
    return dshotTelemetryState.motorState[motorIndex].telemetryActive;
}

int16_t motorGetDshotTelemetryMotorInvalidPercent(uint8_t motorIndex)
{
    int16_t invalidPercent = 0;

    if (dshotTelemetryState.motorState[motorIndex].telemetryActive) {
        const uint32_t totalCount = dshotTelemetryQuality[motorIndex].packetCountSum;
        const uint32_t invalidCount = dshotTelemetryQuality[motorIndex].invalidCountSum;
        if (totalCount > 0) {
            invalidPercent = lrintf(invalidCount * 10000.0f / totalCount);
        }
    } else {
        invalidPercent = 10000;  // 100.00%
    }
    return invalidPercent;
}
