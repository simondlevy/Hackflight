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

class Receiver {

    private:

        static const uint8_t CHAN_COUNT = 18;
        static const uint8_t THROTTLE_LOOKUP_SIZE = 12;

        static const uint32_t FAILSAFE_POWER_ON_DELAY_US = (1000 * 1000 * 5);

        // Minimum rc smoothing cutoff frequency
        static const uint16_t RC_SMOOTHING_CUTOFF_MIN_HZ = 15;    

        // The value to use for "auto" when interpolated feedforward is enabled
        static const uint16_t RC_SMOOTHING_FEEDFORWARD_INITIAL_HZ = 100;   

        // Guard time to wait after retraining to prevent retraining again too
        // quickly
        static const uint16_t RC_SMOOTHING_FILTER_RETRAINING_DELAY_MS = 2000;  

        // Number of rx frame rate samples to average during frame rate changes
        static const uint8_t  RC_SMOOTHING_FILTER_RETRAINING_SAMPLES = 20;    

        // Time to wait after power to let the PID loop stabilize before starting
        // average frame rate calculation
        static const uint16_t RC_SMOOTHING_FILTER_STARTUP_DELAY_MS = 5000;  

        // Additional time to wait after receiving first valid rx frame before
        // initial training starts
        static const uint16_t RC_SMOOTHING_FILTER_TRAINING_DELAY_MS = 1000;  

        // Number of rx frame rate samples to average during initial training
        static const uint8_t  RC_SMOOTHING_FILTER_TRAINING_SAMPLES = 50;    

        // Look for samples varying this much from the current detected frame
        // rate to initiate retraining
        static const uint8_t  RC_SMOOTHING_RX_RATE_CHANGE_PERCENT = 20;    

        // 65.5ms or 15.26hz
        static const uint32_t RC_SMOOTHING_RX_RATE_MAX_US = 65500; 

        // 0.950ms to fit 1kHz without an issue
        static const uint32_t RC_SMOOTHING_RX_RATE_MIN_US = 950;   

        static const uint32_t DELAY_15_HZ       = 1000000 / 15;

        static const uint32_t NEED_SIGNAL_MAX_DELAY_US = 1000000 / 10;

        static const uint16_t  MAX_INVALID__PULSE_TIME     = 300;
        static const uint16_t  RATE_LIMIT                  = 1998;
        static constexpr float THR_EXPO8                   = 0;
        static constexpr float THR_MID8                    = 50;
        static constexpr float COMMAND_DIVIDER             = 500;
        static constexpr float YAW_COMMAND_DIVIDER         = 500;

        // minimum PWM pulse width which is considered valid
        static const uint16_t PWM_PULSE_MIN   = 750;   

        // maximum PWM pulse width which is considered valid
        static const uint16_t PWM_PULSE_MAX   = 2250;  


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

        typedef void    (*rx_dev_init_fun
                )(serialPortIdentifier_e port);
        typedef uint8_t (*rx_dev_check_fun)
            (uint16_t * channelData, uint32_t * frameTimeUs);
        typedef float   (*rx_dev_convert_fun)
            (uint16_t * channelData, uint8_t chan);

        typedef struct {

            rx_dev_init_fun init;
            rx_dev_check_fun check;
            rx_dev_convert_fun convert;

        } rx_dev_funs_t;

        typedef struct {

            rxSmoothingFilter_t smoothingFilter;

            bool               auxiliaryProcessingRequired;
            bool               calculatedCutoffs;
            uint16_t           channelData[CHAN_COUNT];
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
            uint32_t           invalidPulsePeriod[CHAN_COUNT];
            bool               isRateValid;
            uint32_t           lastFrameTimeUs;
            uint32_t           lastRxTimeUs;
            int16_t            lookupThrottleRc[THROTTLE_LOOKUP_SIZE];
            uint32_t           needSignalBefore;
            uint32_t           nextUpdateAtUs;
            uint32_t           previousFrameTimeUs;
            float              raw[CHAN_COUNT];
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
