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

#include "arming.h"
#include "core/clock.h"
#include "core/constrain.h"
#include "core/demands.h"
#include "core/filters/pt3.h"
#include "core/pids/angle.h"
#include "esc.h"
#include "pwm.h"
#include "serial.h"
#include "sticks.h"
#include "time.h"

class Receiver {

    friend class Hackflight;

    public:

    typedef struct {
        Demands demands;
        float aux1;
        float aux2;
    } sticks_t;

    typedef enum {
        FRAME_PENDING = 0,
        FRAME_COMPLETE = (1 << 0),
        FRAME_FAILSAFE = (1 << 1),
        FRAME_PROCESSING_REQUIRED = (1 << 2),
        FRAME_DROPPED = (1 << 3)
    } frameVehicleState_e;

    private:

    static const uint8_t CHANNEL_COUNT = 6;
    static const uint8_t THROTTLE_LOOKUP_TABLE_SIZE = 12;

    static const uint8_t RATE = 67;

    static const uint32_t FAILSAFE_POWER_ON_DELAY_US = (1000 * 1000 * 5);

    // Minimum rc smoothing cutoff frequency
    static const uint16_t SMOOTHING_CUTOFF_MIN_HZ = 15;    

    // The value to use for "auto" when interpolated feedforward is enabled
    static const uint16_t SMOOTHING_FEEDFORWARD_INITIAL_HZ = 100;   

    // Guard time to wait after retraining to prevent retraining again too
    // quickly
    static const uint16_t SMOOTHING_FILTER_RETRAINING_DELAY_MS = 2000;  

    // Number of rx frame rate samples to average during frame rate changes
    static const uint8_t  SMOOTHING_FILTER_RETRAINING_SAMPLES = 20;    

    // Time to wait after power to let the PID loop stabilize before starting
    // average frame rate calculation
    static const uint16_t SMOOTHING_FILTER_STARTUP_DELAY_MS = 5000;  

    // Additional time to wait after receiving first valid rx frame before
    // initial training starts
    static const uint16_t SMOOTHING_FILTER_TRAINING_DELAY_MS = 1000;  

    // Number of rx frame rate samples to average during initial training
    static const uint8_t  SMOOTHING_FILTER_TRAINING_SAMPLES = 50;    

    // Look for samples varying this much from the current detected frame
    // rate to initiate retraining
    static const uint8_t  SMOOTHING_RATE_CHANGE_PERCENT = 20;    

    // 65.5ms or 15.26hz
    static const uint32_t SMOOTHING_RATE_MAX_US = 65500; 

    // 0.950ms to fit 1kHz without an issue
    static const uint32_t SMOOTHING_RATE_MIN_US = 950;   

    static const uint32_t DELAY_15_HZ       = 1000000 / 15;

    static const uint32_t NEED_SIGNAL_MAX_DELAY_US    = 1000000 / 10;

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

    static inline int32_t cmp32(const uint32_t a, const uint32_t b)
    {
        return (int32_t)(a-b); 
    }

    static uint8_t rxfail_step_to_channel_value(const uint8_t step)
    {
        return (PWM_PULSE_MIN + 25 * step);
    }

    static bool isPulseValid(const uint16_t pulseDuration)
    {
        return  pulseDuration >= 885 && pulseDuration <= 2115;
    }

    typedef enum {
        FAILSAFE_MODE_AUTO = 0,
        FAILSAFE_MODE_HOLD,
        FAILSAFE_MODE_SET,
        FAILSAFE_MODE_INVALID
    } failsafeChannelMode_e;

    typedef struct failsafeChannelConfig_s {
        uint8_t mode; 
        uint8_t step;
    } failsafeChannelConfig_t;

    typedef enum {
        STATE_CHECK,
        STATE_PROCESS,
        STATE_MODES,
        STATE_UPDATE,
        STATE_COUNT
    } state_e;

    uint8_t     m_autoSmoothnessFactorSetpoint;
    uint32_t    m_averageFrameTimeUs;
    uint8_t     m_autoSmoothnessFactorThrottle;
    uint16_t    m_feedforwardCutoffFrequency;
    uint8_t     m_ffCutoffSetting;

    Pt3Filter m_filterThrottle;
    Pt3Filter m_filterRoll;
    Pt3Filter m_filterPitch;
    Pt3Filter m_filterYaw;

    Pt3Filter m_filterDeflectionRoll;
    Pt3Filter m_filterDeflectionPitch;

    Pt3Filter  m_feedforwardPt3[3];
    uint16_t   m_channelData[CHANNEL_COUNT];
    uint32_t   m_invalidPulsePeriod[CHANNEL_COUNT];
    float      m_raw[CHANNEL_COUNT];

    float      m_command[4];

    int16_t    m_lookupThrottleRc[THROTTLE_LOOKUP_TABLE_SIZE];

    bool        m_filterInitialized;
    uint16_t    m_setpointCutoffFrequency;
    uint8_t     m_setpointCutoffSetting;
    uint16_t    m_throttleCutoffFrequency;
    uint8_t     m_throttleCutoffSetting;
    float       m_trainingSum;
    uint32_t    m_trainingCount;
    uint16_t    m_trainingMax;
    uint16_t    m_trainingMin;

    bool         m_auxiliaryProcessingRequired;
    bool         m_calculatedCutoffs;
    Demands      m_commands;
    bool         m_dataProcessingRequired;
    Demands      m_dataToSmooth;
    bool         m_feedforwardLpfInitialized;
    int32_t      m_frameTimeDeltaUs;
    bool         m_gotNewData;
    bool         m_inFailsafeMode;
    bool         m_initializedFilter;
    bool         m_initializedThrottleTable;
    bool         m_isRateValid;
    uint32_t     m_lastFrameTimeUs;
    uint32_t     m_lastRxTimeUs;
    uint32_t     m_needSignalBefore;
    uint32_t     m_nextUpdateAtUs;
    uint32_t     m_previousFrameTimeUs;
    uint32_t     m_refreshPeriod;
    bool         m_signalReceived;
    state_e      m_state;
    uint32_t     m_validFrameTimeMs;

    private:

    uint16_t getFailValue(const uint8_t channel)
    {
        failsafeChannelConfig_t failsafeChannelConfigs[CHANNEL_COUNT];

        failsafeChannelConfigs[THROTTLE].step = 30;
        failsafeChannelConfigs[ROLL].step     = 30;
        failsafeChannelConfigs[PITCH].step    = 30;
        failsafeChannelConfigs[YAW].step      = 5;
        failsafeChannelConfigs[AUX1].step     = 30;
        failsafeChannelConfigs[AUX2].step     = 30;

        for (auto i = 0; i < 4; i++) {
            failsafeChannelConfigs[i].mode = 0;
        }

        for (auto i = 4; i < CHANNEL_COUNT; i++) {
            failsafeChannelConfigs[i].mode = 1;
        }

        const failsafeChannelConfig_t *channelFailsafeConfig =
            &failsafeChannelConfigs[channel];

        switch (channelFailsafeConfig->mode) {
            case FAILSAFE_MODE_AUTO:
                return channel == ROLL || channel == PITCH || channel == YAW ?
                    1500 :
                    885;
            case FAILSAFE_MODE_INVALID:
            case FAILSAFE_MODE_HOLD:
                return m_raw[channel];
            case FAILSAFE_MODE_SET:
                return
                    rxfail_step_to_channel_value(channelFailsafeConfig->step);
        }

        return 0;
    }

    // Determine a cutoff frequency based on smoothness factor and calculated
    // average rx frame time
    static int calcAutoSmoothingCutoff(
            const uint32_t avgRxFrameTimeUs,
            const uint8_t autoSmoothnessFactor)
    {
        if (avgRxFrameTimeUs > 0) {
            const auto cutoffFactor =
                1.5f / (1.0f + (autoSmoothnessFactor / 10.0f));
            auto cutoff =
                (1 / (avgRxFrameTimeUs * 1e-6f));  // link frequency
            cutoff = cutoff * cutoffFactor;
            return lrintf(cutoff);
        } else {
            return 0;
        }
    }

    void smoothingResetAccumulation(void)
    {
        m_trainingSum = 0;
        m_trainingCount = 0;
        m_trainingMin = UINT16_MAX;
        m_trainingMax = 0;
    }

    void readChannelsAndApplyRanges(void)
    {
        for (auto channel=0; channel<CHANNEL_COUNT; ++channel) {

            float sample = convert(m_channelData, channel);

            m_raw[channel] = channel < 4 && sample != 0 ?
                constrain_f(sample, PWM_PULSE_MIN, PWM_PULSE_MAX) :
                sample;
        }
    }

    void detectAndApplySignalLossBehaviour(
            Arming * arming,
            Failsafe * failsafe,
            const uint32_t currentTimeUs)
    {
        auto currentTimeMs = currentTimeUs/ 1000;

        auto useValueFromRx = m_signalReceived && !m_inFailsafeMode;

        auto flightChannelsValid = true;

        for (auto channel = 0; channel < CHANNEL_COUNT; channel++) {

            auto sample = m_raw[channel];

            auto validPulse = useValueFromRx && isPulseValid(sample);

            if (validPulse) {
                m_invalidPulsePeriod[channel] =
                    currentTimeMs + MAX_INVALID__PULSE_TIME;
            } else {
                if (cmp32(currentTimeMs,
                            m_invalidPulsePeriod[channel]) < 0) {
                    // skip to next channel to hold channel value
                    // MAX_INVALID__PULSE_TIME
                    continue;           
                } else {

                    // after that apply rxfail value
                    sample = getFailValue(channel); 
                    if (channel < 4) {
                        flightChannelsValid = false;
                    }
                }
            }

            m_raw[channel] = sample;
        }

        if (flightChannelsValid) {
            failsafe->onValidDataReceived(arming);
        } else {
            m_inFailsafeMode = true;
            failsafe->onValidDataFailed(arming);
            for (auto channel = 0; channel < CHANNEL_COUNT; channel++) {
                m_raw[channel] = getFailValue(channel);
            }
        }
    }

    int16_t lookupThrottle(const int32_t tmp)
    {
        if (!m_initializedThrottleTable) {
            for (auto i = 0; i < THROTTLE_LOOKUP_TABLE_SIZE; i++) {
                const int16_t tmp2 = 10 * i - THR_MID8;
                uint8_t y = tmp2 > 0 ?
                    100 - THR_MID8 :
                    tmp2 < 0 ?
                    THR_MID8 :
                    1;
                m_lookupThrottleRc[i] =
                    10 * THR_MID8 + tmp2 * (100 - THR_EXPO8 + (int32_t)
                            THR_EXPO8 * (tmp2 * tmp2) / (y * y)) / 10;
                m_lookupThrottleRc[i] = PWM_MIN + (PWM_MAX - PWM_MIN) *
                    m_lookupThrottleRc[i] / 1000; 
            }
        }

        m_initializedThrottleTable = true;

        const auto tmp3 = tmp / 100;

        // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
        return m_lookupThrottleRc[tmp3] + (tmp - tmp3 * 100) *
            (m_lookupThrottleRc[tmp3 + 1] - m_lookupThrottleRc[tmp3]) / 100;
    }

    static float updateCommand(const float raw, const float sgn)
    {
        float tmp = fminf(fabs(raw - 1500), 500);

        float cmd = tmp * sgn;

        return raw < 1500 ? -cmd : cmd;
    }

    void updateCommands(void)
    {
        for (uint8_t axis=ROLL; axis<=YAW; axis++) {

            // non coupled PID reduction scaler used in PID controller 1
            // and PID controller 2.
            m_command[axis] = updateCommand(m_raw[axis], axis == YAW ? -1 : +1);

        }

        auto tmp = constrain_f_i32(m_raw[THROTTLE], 1050, PWM_MAX);
        auto tmp2 = (uint32_t)(tmp - 1050) * PWM_MIN / (PWM_MAX - 1050);

        m_commands.throttle = lookupThrottle(tmp2);
    }

    bool calculateChannelsAndUpdateFailsafe(
            Arming * arming,
            Failsafe * failsafe,
            const uint32_t currentTimeUs)
    {
        if (m_auxiliaryProcessingRequired) {
            m_auxiliaryProcessingRequired = false;
        }

        if (!m_dataProcessingRequired) {
            return false;
        }

        m_dataProcessingRequired = false;
        m_nextUpdateAtUs = currentTimeUs + DELAY_15_HZ;

        readChannelsAndApplyRanges();

        detectAndApplySignalLossBehaviour(arming, failsafe, currentTimeUs);

        return true;
    }

    int32_t getFrameDelta(const uint32_t currentTimeUs, int32_t *frameAgeUs)
    {
        auto frameTimeUs = m_lastFrameTimeUs;

        *frameAgeUs = cmpTimeUs(currentTimeUs, frameTimeUs);

        const auto deltaUs = cmpTimeUs(frameTimeUs, m_previousFrameTimeUs);

        if (deltaUs) {
            m_frameTimeDeltaUs = deltaUs;
            m_previousFrameTimeUs = frameTimeUs;
        }

        return m_frameTimeDeltaUs;
    }

    bool processData(
            Esc * esc,
            const uint32_t currentTimeUs,
            Arming * arming,
            Failsafe * failsafe)
    {
        int32_t frameAgeUs;

        auto refreshPeriodUs =
            getFrameDelta(currentTimeUs, &frameAgeUs);

        if (!refreshPeriodUs ||
                cmpTimeUs(currentTimeUs, m_lastRxTimeUs) <= frameAgeUs) {

            // calculate a delta here if not supplied by the protocol
            refreshPeriodUs = cmpTimeUs(currentTimeUs, m_lastRxTimeUs); 
        }

        m_lastRxTimeUs = currentTimeUs;

        m_isRateValid =
            ((uint32_t)refreshPeriodUs >= SMOOTHING_RATE_MIN_US &&
             (uint32_t)refreshPeriodUs <= SMOOTHING_RATE_MAX_US);

        m_refreshPeriod =
            constrain_i32_u32(refreshPeriodUs, SMOOTHING_RATE_MIN_US,
                    SMOOTHING_RATE_MAX_US);

        if (currentTimeUs >
                FAILSAFE_POWER_ON_DELAY_US && !failsafe->isMonitoring()) {
            failsafe->startMonitoring();
        }

        failsafe->update(m_raw, esc, arming);

        return throttleIsDown(m_raw);
    }

    void ratePidFeedforwardLpfInit(const uint16_t filterCutoff)
    {
        if (filterCutoff > 0) {
            m_feedforwardLpfInitialized = true;
            m_feedforwardPt3[0].init(filterCutoff);
            m_feedforwardPt3[1].init(filterCutoff);
            m_feedforwardPt3[2].init(filterCutoff);
        }
    }

    void ratePidFeedforwardLpfUpdate(const uint16_t filterCutoff)
    {
        if (filterCutoff > 0) {
            for (uint8_t axis=ROLL; axis<=YAW; axis++) {
                m_feedforwardPt3[axis].computeGain(filterCutoff);
            }
        }
    }

    void smoothingFilterInit(Pt3Filter * filter, const float setpointCutoffFrequency)
    {
        if (!m_filterInitialized) {
            filter->init(setpointCutoffFrequency);
        } else {
            filter->computeGain(setpointCutoffFrequency); 
        }
    }

    void smoothingFilterInitRollPitchYaw(Pt3Filter * filter)
    {
        filter->init(m_setpointCutoffFrequency);
    }

    void levelFilterInit(Pt3Filter * filter)
    {
        if (!m_filterInitialized) {
            filter->init(m_setpointCutoffFrequency);
        } else {
            filter->computeGain(m_setpointCutoffFrequency);
        }
    }

    float smoothingFilterApply(Pt3Filter * filter, const float dataToSmooth)
    {
        return m_filterInitialized ?
            filter->apply(dataToSmooth) :
            dataToSmooth;
    }

    void setSmoothingFilterCutoffs(void)
    {
        uint16_t oldCutoff = m_setpointCutoffFrequency;

        if (m_setpointCutoffSetting == 0) {
            m_setpointCutoffFrequency =
                fmaxf(SMOOTHING_CUTOFF_MIN_HZ,
                        calcAutoSmoothingCutoff(
                            m_averageFrameTimeUs,
                            m_autoSmoothnessFactorSetpoint)); 
        }
        if (m_throttleCutoffSetting == 0) {
            m_throttleCutoffFrequency =
                fmaxf(SMOOTHING_CUTOFF_MIN_HZ,
                        calcAutoSmoothingCutoff(
                            m_averageFrameTimeUs,
                            m_autoSmoothnessFactorThrottle));
        }

        // initialize or update the Setpoint filter
        if ((m_setpointCutoffFrequency != oldCutoff) ||
                !m_filterInitialized) {

            m_filterThrottle.init(m_throttleCutoffFrequency);

            smoothingFilterInitRollPitchYaw(&m_filterRoll);
            smoothingFilterInitRollPitchYaw(&m_filterPitch);
            smoothingFilterInitRollPitchYaw(&m_filterYaw);

            levelFilterInit(&m_filterDeflectionRoll);
            levelFilterInit(&m_filterDeflectionPitch);
        }

        // update or initialize the FF filter
        oldCutoff = m_feedforwardCutoffFrequency;
        if (m_ffCutoffSetting == 0) {
            m_feedforwardCutoffFrequency =
                fmaxf(SMOOTHING_CUTOFF_MIN_HZ,
                        calcAutoSmoothingCutoff(
                            m_averageFrameTimeUs,
                            m_autoSmoothnessFactorSetpoint)); 
        }
        if (!m_filterInitialized) {
            ratePidFeedforwardLpfInit(
                    m_feedforwardCutoffFrequency);
        } else if (m_feedforwardCutoffFrequency != oldCutoff) {
            ratePidFeedforwardLpfUpdate(
                    m_feedforwardCutoffFrequency);
        }
    }

    bool smoothingAccumulateSample(void)
    {
        m_trainingSum += m_refreshPeriod;
        m_trainingCount++;
        m_trainingMax =
            fmaxf(m_trainingMax, m_refreshPeriod);
        m_trainingMin =
            fminf(m_trainingMin, m_refreshPeriod);

        // if we've collected enough samples then calculate the average and
        // reset the accumulation
        uint32_t sampleLimit = (m_filterInitialized) ?
            SMOOTHING_FILTER_RETRAINING_SAMPLES :
            SMOOTHING_FILTER_TRAINING_SAMPLES;

        if (m_trainingCount >= sampleLimit) {
            // Throw out high and low samples
            m_trainingSum = m_trainingSum -
                m_trainingMin - m_trainingMax; 

            m_averageFrameTimeUs =
                lrintf(m_trainingSum /
                        (m_trainingCount - 2));
            smoothingResetAccumulation();
            return true;
        }
        return false;
    }


    bool smoothingAutoCalculate(void)
    {
        // if any rc smoothing cutoff is 0 (auto) then we need to calculate
        // cutoffs
        if ((m_setpointCutoffSetting == 0) ||
                (m_ffCutoffSetting == 0) ||
                (m_throttleCutoffSetting == 0)) {
            return true;
        }
        return false;
    }

    void processSmoothingFilter(
            const uint32_t currentTimeUs,
            float * setpointRate,
            float * rawSetpoint)
    {
        // first call initialization
        if (!m_initializedFilter) {

            m_filterInitialized = false;
            m_averageFrameTimeUs = 0;
            m_autoSmoothnessFactorSetpoint = 30;
            m_autoSmoothnessFactorThrottle = 30;
            m_setpointCutoffSetting = 0;
            m_throttleCutoffSetting = 0;
            m_ffCutoffSetting = 0;
            smoothingResetAccumulation();
            m_setpointCutoffFrequency =
                m_setpointCutoffSetting;
            m_throttleCutoffFrequency =
                m_throttleCutoffSetting;
            if (m_ffCutoffSetting == 0) {
                // calculate and use an initial derivative cutoff until the RC
                // interval is known
                const auto cutoffFactor = 1.5f /
                    (1.0f +
                     (m_autoSmoothnessFactorSetpoint /
                      10.0f));
                auto ffCutoff = SMOOTHING_FEEDFORWARD_INITIAL_HZ * cutoffFactor;
                m_feedforwardCutoffFrequency = 
                    lrintf(ffCutoff);
            } else {
                m_feedforwardCutoffFrequency =
                    m_ffCutoffSetting;
            }

            m_calculatedCutoffs = smoothingAutoCalculate();

            // if we don't need to calculate cutoffs dynamically then the
            // filters can be initialized now
            if (!m_calculatedCutoffs) {
                setSmoothingFilterCutoffs();
                m_filterInitialized = true;
            }
        }

        m_initializedFilter = true;

        if (m_gotNewData) {
            // for auto calculated filters we need to examine each rx frame
            // interval
            if (m_calculatedCutoffs) {
                const auto currentTimeMs = currentTimeUs / 1000;

                // If the filter cutoffs in auto mode, and we have good rx
                // data, then determine the average rx frame rate and use
                // that to calculate the filter cutoff frequencies

                // skip during FC initialization
                if ((currentTimeMs > SMOOTHING_FILTER_STARTUP_DELAY_MS))
                {
                    if (m_signalReceived && m_isRateValid) {

                        // set the guard time expiration if it's not set
                        if (m_validFrameTimeMs == 0) {
                            m_validFrameTimeMs =
                                currentTimeMs +
                                (m_filterInitialized ?
                                 SMOOTHING_FILTER_RETRAINING_DELAY_MS :
                                 SMOOTHING_FILTER_TRAINING_DELAY_MS);
                        } else {
                        }

                        // if the guard time has expired then process the
                        // rx frame time
                        if (currentTimeMs > m_validFrameTimeMs) {
                            auto accumulateSample = true;

                            // During initial training process all samples.
                            // During retraining check samples to determine
                            // if they vary by more than the limit
                            // percentage.
                            if (m_filterInitialized) {
                                const auto percentChange =
                                    fabs((m_refreshPeriod -
                                                m_averageFrameTimeUs) /
                                            (float)m_averageFrameTimeUs) *
                                    100;

                                if (percentChange <
                                        SMOOTHING_RATE_CHANGE_PERCENT) {
                                    // We received a sample that wasn't
                                    // more than the limit percent so reset
                                    // the accumulation During retraining
                                    // we need a contiguous block of
                                    // samples that are all significantly
                                    // different than the current average
                                    smoothingResetAccumulation();
                                    accumulateSample = false;
                                }
                            }

                            // accumlate the sample into the average
                            if (accumulateSample) { 
                                if (smoothingAccumulateSample()) {
                                    // the required number of samples were
                                    // collected so set the filter cutoffs, but
                                    // only if smoothing is active
                                    setSmoothingFilterCutoffs();
                                    m_filterInitialized = true;
                                    m_validFrameTimeMs = 0;
                                }
                            }

                        }
                    } else {
                        smoothingResetAccumulation();
                    }
                }
            }

            m_dataToSmooth.throttle = m_commands.throttle;
            m_dataToSmooth.roll  = rawSetpoint[0];
            m_dataToSmooth.pitch = rawSetpoint[1];
            m_dataToSmooth.yaw   = rawSetpoint[2];
        }

        // Each pid loop, apply the last received channel value to the
        // filter, if initialised - thanks @klutvott
        m_commands.throttle = smoothingFilterApply(
                &m_filterThrottle,
                m_dataToSmooth.throttle);
        setpointRate[0] = smoothingFilterApply(
                &m_filterRoll, m_dataToSmooth.roll);
        setpointRate[1] = smoothingFilterApply(
                &m_filterPitch, m_dataToSmooth.pitch);
        setpointRate[2] = smoothingFilterApply(
                &m_filterYaw, m_dataToSmooth.yaw);
    }

    static float getRawSetpoint(const float command, const float divider)
    {
        auto commandf = command / divider;

        auto commandfAbs = fabsf(commandf);

        auto angleRate = AnglePidController::applyRates(commandf, commandfAbs);

        return constrain_f(angleRate, -(float)RATE_LIMIT, +(float)RATE_LIMIT);
    }


    protected:

    virtual void begin(/*serialPortIdentifier_e port*/) = 0;

    virtual uint8_t devCheck(uint16_t * chanData, uint32_t * frameTimeUs) = 0;

    virtual float convert(uint16_t * chanData, uint8_t chanId) = 0;

    public:

    // Called from tasks/receiver.h::adjustRxDynamicPriority()
    bool check(const uint32_t currentTimeUs)
    {
        auto signalReceived = false;
        auto useDataDrivenProcessing = true;

        if (m_state != STATE_CHECK) {
            return true;
        }

        const auto frameStatus =
            devCheck(m_channelData, &m_lastFrameTimeUs);

        if (frameStatus & FRAME_COMPLETE) {
            m_inFailsafeMode = (frameStatus & FRAME_FAILSAFE) != 0;
            auto frameDropped = (frameStatus & FRAME_DROPPED) != 0;
            signalReceived = !(m_inFailsafeMode || frameDropped);
            if (signalReceived) {
                m_needSignalBefore =
                    currentTimeUs + NEED_SIGNAL_MAX_DELAY_US;
            }
        }

        if (frameStatus & FRAME_PROCESSING_REQUIRED) {
            m_auxiliaryProcessingRequired = true;
        }

        if (signalReceived) {
            m_signalReceived = true;
        } else if (currentTimeUs >= m_needSignalBefore) {
            m_signalReceived = false;
        }

        if ((signalReceived && useDataDrivenProcessing) ||
                cmpTimeUs(currentTimeUs, m_nextUpdateAtUs) > 0) {
            m_dataProcessingRequired = true;
        }

        // data driven or 50Hz
        return m_dataProcessingRequired || m_auxiliaryProcessingRequired; 

    } // check

    void poll(
            const uint32_t currentTimeUs,
            const bool imuIsLevel,
            const bool calibrating,
            sticks_t * sticks,
            Esc * esc,
            Arming * arming,
            Failsafe * failsafe,
            bool * pidItermResetReady,
            bool * pidItermResetValue,
            bool * gotNewData)
    {
        *pidItermResetReady = false;

        m_gotNewData = false;

        switch (m_state) {
            default:
            case STATE_CHECK:
                m_state = STATE_PROCESS;
                break;

            case STATE_PROCESS:
                if (!calculateChannelsAndUpdateFailsafe(
                            arming, 
                            failsafe,
                            currentTimeUs)) {
                    m_state = STATE_CHECK;
                    break;
                }
                *pidItermResetReady = true;
                *pidItermResetValue = processData(
                        esc,
                        currentTimeUs,
                        arming, 
                        failsafe);
                m_state = STATE_MODES;
                break;

            case STATE_MODES:
                arming->check(esc, currentTimeUs, m_raw,
                        imuIsLevel,
                        calibrating);
                m_state = STATE_UPDATE;
                break;

            case STATE_UPDATE:
                m_gotNewData = true;
                updateCommands();
                arming->updateStatus(m_raw, imuIsLevel, calibrating);
                m_state = STATE_CHECK;
                break;
        }

        sticks->demands.throttle = m_raw[THROTTLE];
        sticks->demands.roll     = m_raw[ROLL];
        sticks->demands.pitch    = m_raw[PITCH];
        sticks->demands.yaw      = m_raw[YAW];
        sticks->aux1             = m_raw[AUX1];
        sticks->aux2             = m_raw[AUX2];

        *gotNewData = m_gotNewData;

    } // poll

    // Runs in fast (inner, core) loop
    void getDemands(
            const uint32_t currentTimeUs,
            float rawSetpoints[3],
            Demands * demands)
    {
        if (m_gotNewData) {

            m_previousFrameTimeUs = 0;

            rawSetpoints[0] =
                getRawSetpoint(m_command[ROLL], COMMAND_DIVIDER);
            rawSetpoints[1] =
                getRawSetpoint(m_command[PITCH], COMMAND_DIVIDER);
            rawSetpoints[2] =
                getRawSetpoint(m_command[YAW], YAW_COMMAND_DIVIDER);
        }

        float setpointRate[3] = {};
        processSmoothingFilter(currentTimeUs, setpointRate, rawSetpoints);

        // Find min and max throttle based on conditions. Throttle has to
        // be known before mixing
        demands->throttle =
            constrain_f((m_commands.throttle - PWM_MIN) /
                    (PWM_MAX - PWM_MIN), 0.0f, 1.0f);

        demands->roll  = setpointRate[0];
        demands->pitch = setpointRate[1];
        demands->yaw   = setpointRate[2];

        m_gotNewData = false;

    } // getDemands

}; // class Receiver
