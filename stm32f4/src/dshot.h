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

#include <stdbool.h>

#include <time.h>
#include "escdev.h"

#define DSHOT_MIN_THROTTLE       48
#define DSHOT_MAX_THROTTLE     2047
#define DSHOT_RANGE (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

#define MIN_GCR_EDGES         7
#define MAX_GCR_EDGES         22

#define DSHOT_TELEMETRY_QUALITY_WINDOW 1       // capture a rolling 1 second of packet stats
#define DSHOT_TELEMETRY_QUALITY_BUCKET_MS 100  // determines the granularity of the stats and the overall number of rolling buckets
#define DSHOT_TELEMETRY_QUALITY_BUCKET_COUNT (DSHOT_TELEMETRY_QUALITY_WINDOW * 1000 / DSHOT_TELEMETRY_QUALITY_BUCKET_MS)

typedef struct dshotTelemetryQuality_s {
    uint32_t packetCountSum;
    uint32_t invalidCountSum;
    uint32_t packetCountArray[DSHOT_TELEMETRY_QUALITY_BUCKET_COUNT];
    uint32_t invalidCountArray[DSHOT_TELEMETRY_QUALITY_BUCKET_COUNT];
    uint8_t lastBucketIndex;
}  dshotTelemetryQuality_t;

extern dshotTelemetryQuality_t dshotTelemetryQuality[MAX_SUPPORTED_MOTORS];

typedef struct dshotProtocolControl_s {
    uint16_t value;
    bool requestTelemetry;
} dshotProtocolControl_t;

void dshotInitEndpoints(float outputLimit, float *outputLow, float *outputHigh, float *disarm);
float dshotConvertFromExternal(uint16_t externalValue);
uint16_t dshotConvertToExternal(float motorValue);

uint16_t prepareDshotPacket(dshotProtocolControl_t *pcb);

extern bool useDshotTelemetry;

typedef struct dshotTelemetryMotorState_s {
    uint16_t telemetryValue;
    bool telemetryActive;
} dshotTelemetryMotorState_t;


typedef struct dshotTelemetryState_s {
    bool useDshotTelemetry;
    uint32_t invalidPacketCount;
    uint32_t readCount;
    dshotTelemetryMotorState_t motorState[MAX_SUPPORTED_MOTORS];
    uint32_t inputBuffer[MAX_GCR_EDGES];
} dshotTelemetryState_t;

extern dshotTelemetryState_t dshotTelemetryState;

void updateDshotTelemetryQuality(dshotTelemetryQuality_t *qualityStats, bool packetValid, uint32_t currentTimeMs);

bool isDshotTelemetryActive(void);

void validateAndfixMotorOutputReordering(uint8_t *array, const unsigned size);
