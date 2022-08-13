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

#include <math.h>

#include "datatypes.h"
#include "serial.h"
#include "time.h"

#define CHANNEL_COUNT 18
#define THROTTLE_LOOKUP_LENGTH 12

class Receiver {

    public:

        typedef struct {
            demands_t demands;
            float aux1;
            float aux2;
        } rx_axes_t;

        typedef enum rc_alias {
            THROTTLE,
            ROLL,
            PITCH,
            YAW,
            AUX1,
            AUX2
        } rc_alias_e;

        typedef enum {
            RX_FRAME_PENDING = 0,
            RX_FRAME_COMPLETE = (1 << 0),
            RX_FRAME_FAILSAFE = (1 << 1),
            RX_FRAME_PROCESSING_REQUIRED = (1 << 2),
            RX_FRAME_DROPPED = (1 << 3)
        } rxFrameState_e;

        typedef enum {
            RX_FAILSAFE_MODE_AUTO = 0,
            RX_FAILSAFE_MODE_HOLD,
            RX_FAILSAFE_MODE_SET,
            RX_FAILSAFE_MODE_INVALID
        } rxFailsafeChannelMode_e;

        typedef struct rxFailsafeChannelConfig_s {
            uint8_t mode; 
            uint8_t step;
        } rxFailsafeChannelConfig_t;

        typedef struct rxChannelRangeConfig_s {
            uint16_t min;
            uint16_t max;
        } rxChannelRangeConfig_t;


        typedef enum {
            RX_STATE_CHECK,
            RX_STATE_PROCESS,
            RX_STATE_MODES,
            RX_STATE_UPDATE,
            RX_STATE_COUNT
        } rxState_e;

        typedef struct rxSmoothingFilter_s {

            uint8_t     autoSmoothnessFactorSetpoint;
            uint32_t    averageFrameTimeUs;
            uint8_t     autoSmoothnessFactorThrottle;
            uint16_t    feedforwardCutoffFrequency;
            uint8_t     ffCutoffSetting;

            pt3Filter_t filterThrottle;
            pt3Filter_t filterRoll;
            pt3Filter_t filterPitch;
            pt3Filter_t filterYaw;

            pt3Filter_t filterDeflectionRoll;
            pt3Filter_t filterDeflectionPitch;

            bool        filterInitialized;
            uint16_t    setpointCutoffFrequency;
            uint8_t     setpointCutoffSetting;
            uint16_t    throttleCutoffFrequency;
            uint8_t     throttleCutoffSetting;
            float       trainingSum;
            uint32_t    trainingCount;
            uint16_t    trainingMax;
            uint16_t    trainingMin;

        } rxSmoothingFilter_t;

        typedef void    (*rx_dev_init_fun)(serialPortIdentifier_e port);
        typedef uint8_t (*rx_dev_check_fun)(uint16_t * channelData, uint32_t * frameTimeUs);
        typedef float   (*rx_dev_convert_fun)(uint16_t * channelData, uint8_t chan);

        typedef struct {

            rx_dev_init_fun init;
            rx_dev_check_fun check;
            rx_dev_convert_fun convert;

        } rx_dev_funs_t;

        typedef struct {

            rxSmoothingFilter_t smoothingFilter;

            bool               auxiliaryProcessingRequired;
            bool               calculatedCutoffs;
            uint16_t           channelData[CHANNEL_COUNT];
            float              command[4];
            demands_t          commands;
            bool               dataProcessingRequired;
            demands_t          dataToSmooth;
            rx_dev_check_fun   devCheck;
            rx_dev_convert_fun devConvert;
            int32_t            frameTimeDeltaUs;
            bool               gotNewData;
            bool               inFailsafeMode;
            bool               initializedFilter;
            bool               initializedThrottleTable;
            uint32_t           invalidPulsePeriod[CHANNEL_COUNT];
            bool               isRateValid;
            uint32_t           lastFrameTimeUs;
            uint32_t           lastRxTimeUs;
            int16_t            lookupThrottleRc[THROTTLE_LOOKUP_LENGTH];
            uint32_t           needSignalBefore;
            uint32_t           nextUpdateAtUs;
            uint32_t           previousFrameTimeUs;
            float              raw[CHANNEL_COUNT];
            uint32_t           refreshPeriod;
            bool               signalReceived;
            rxState_e          state;
            uint32_t           validFrameTimeMs;

        } rx_t;


}; // class Receiver

#if defined(__cplusplus)
extern "C" {
#endif

    // For both hardware and sim implementations -------------------------------

    void rxDevInit(serialPortIdentifier_e port);

    void rxGetDemands(
            rx_t * rx,
            uint32_t currentTimeUs,
            anglePid_t * ratepid,
            demands_t * demands);

    void rxPoll(
            rx_t * rx,
            uint32_t currentTimeUs,
            bool imuIsLevel,
            bool calibrating,
            rx_axes_t * rxax,
            void * motorDevice,
            arming_t * arming,
            bool * pidItermResetReady,
            bool * pidItermResetValue,
            bool * gotNewData);

    bool rxCheck(rx_t * rx, uint32_t currentTimeUs);

#if defined(__cplusplus)
}
#endif

// For hardware impelmentations ------------------------------------------------

uint8_t rxDevCheck(uint16_t * channelData, uint32_t * frameTimeUs);

float rxDevConvert(uint16_t * channelData, uint8_t chan);
