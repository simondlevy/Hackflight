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

#include <core/constrain.h>
#include <esc.h>
#include <pwm.h>
#include <time.h>

#include "platform.h"
#include "atomic.h"
#include "dshot.h"
#include "dshot_command.h"
#include "escdev.h"
#include "nvic.h"
#include "pwm_output.h" // for PWM_TYPE_* and others
#include "timer.h"
#include "dshot_dpwm.h" // for motorDmaOutput_t, should be gone

// Time to separate dshot beacon and armining/disarming events
static const uint32_t DSHOT_BEACON_GUARD_DELAY_US = 1200000;  

static float scaleRangef(float x, float srcFrom, float srcTo, float destFrom, float destTo)
{
    float a = (destTo - destFrom) * (x - srcFrom);
    float b = srcTo - srcFrom;
    return (a / b) + destFrom;
}


void dshotInitEndpoints(float outputLimit, float *outputLow, float *outputHigh, float *disarm) {
    float outputLimitOffset = DSHOT_RANGE * (1 - outputLimit);
    *disarm = DSHOT_CMD_MOTOR_STOP;
    *outputLow = DSHOT_MIN_THROTTLE + motorGetDigitalIdOffset() * DSHOT_RANGE;
    *outputHigh = DSHOT_MAX_THROTTLE - outputLimitOffset;
}

float dshotConvertFromExternal(uint16_t externalValue)
{
    float motorValue;

    externalValue = constrain_u16(externalValue, PWM_MIN, PWM_MAX);

    motorValue = (externalValue == PWM_MIN) ?
        DSHOT_CMD_MOTOR_STOP :
        scaleRangef(externalValue, PWM_MIN + 1, PWM_MAX,
                DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);

    return motorValue;
}

uint16_t dshotConvertToExternal(float motorValue)
{
    uint16_t externalValue;

    externalValue = (motorValue < DSHOT_MIN_THROTTLE) ? PWM_MIN : scaleRangef(motorValue, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE, PWM_MIN + 1, PWM_MAX);

    return externalValue;
}

uint16_t prepareDshotPacket(dshotProtocolControl_t *pcb)
{
    uint16_t packet;

    ATOMIC_BLOCK(NVIC_PRIO_DSHOT_DMA) {
        packet = (pcb->value << 1) | (pcb->requestTelemetry ? 1 : 0);
        pcb->requestTelemetry = false;    // reset telemetry request to make sure it's triggered only once in a row
    }

    // compute checksum
    unsigned csum = 0;
    unsigned csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    // append checksum

    if (useDshotTelemetry) {
        csum = ~csum;
    }

    csum &= 0xf;
    packet = (packet << 4) | csum;

    return packet;
}


FAST_DATA_ZERO_INIT dshotTelemetryState_t dshotTelemetryState;

uint16_t motorGetDshotTelemetry(uint8_t index)
{
    return dshotTelemetryState.motorState[index].telemetryValue;
}

FAST_DATA_ZERO_INIT dshotTelemetryQuality_t dshotTelemetryQuality[MAX_SUPPORTED_MOTORS];

void updateDshotTelemetryQuality(dshotTelemetryQuality_t *qualityStats, bool packetValid, uint32_t currentTimeMs)
{
    uint8_t statsBucketIndex = (currentTimeMs / DSHOT_TELEMETRY_QUALITY_BUCKET_MS) % DSHOT_TELEMETRY_QUALITY_BUCKET_COUNT;
    if (statsBucketIndex != qualityStats->lastBucketIndex) {
        qualityStats->packetCountSum -= qualityStats->packetCountArray[statsBucketIndex];
        qualityStats->invalidCountSum -= qualityStats->invalidCountArray[statsBucketIndex];
        qualityStats->packetCountArray[statsBucketIndex] = 0;
        qualityStats->invalidCountArray[statsBucketIndex] = 0;
        qualityStats->lastBucketIndex = statsBucketIndex;
    }
    qualityStats->packetCountSum++;
    qualityStats->packetCountArray[statsBucketIndex]++;
    if (!packetValid) {
        qualityStats->invalidCountSum++;
        qualityStats->invalidCountArray[statsBucketIndex]++;
    }
}

// temporarily here, needs to be moved during refactoring
void validateAndfixMotorOutputReordering(uint8_t *array, const unsigned size)
{
    bool invalid = false;

    for (unsigned i = 0; i < size; i++) {
        if (array[i] >= size) {
            invalid = true;
            break;
        }
    }

    int valuesAsIndexes[size];

    for (unsigned i = 0; i < size; i++) {
        valuesAsIndexes[i] = -1;
    }

    if (!invalid) {
        for (unsigned i = 0; i < size; i++) {
            if (-1 != valuesAsIndexes[array[i]]) {
                invalid = true;
                break;
            }

            valuesAsIndexes[array[i]] = array[i];
        }
    }

    if (invalid) {
        for (unsigned i = 0; i < size; i++) {
            array[i] = i;
        }
    }
}

float escDevValueDisarmed(void)
{
    return DSHOT_CMD_MOTOR_STOP;
}

float escDevValueHigh(void) 
{
    return DSHOT_MAX_THROTTLE;
}

float escDevValueLow(void)
{
    return DSHOT_MIN_THROTTLE + 0.045 * DSHOT_RANGE;
}

bool escDevIsReady(uint32_t currentTimeUs)
{
    return currentTimeUs >= DSHOT_BEACON_GUARD_DELAY_US;
}
