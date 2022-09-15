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

#include "arming.h"
#include "core/clock.h"
#include "core/constrain.h"
#include "core/demands.h"
#include "core/filters/pt3.h"
#include "core/pids/angle.h"
#include "pwm.h"
#include "serial.h"
#include "task.h"
#include "time.h"

class Receiver : public Task {

    friend class Hackflight;
    friend class Msp;

    static const uint8_t CHANNEL_COUNT = 6;
    static const uint8_t THROTTLE_LOOKUP_TABLE_SIZE = 12;

    static const uint8_t RATE = 67;

    static const uint32_t TIMEOUT_MS = 500;

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

    static const uint16_t  RATE_LIMIT                  = 1998;
    static constexpr float THR_EXPO8                   = 0;
    static constexpr float THR_MID8                    = 50;
    static constexpr float COMMAND_DIVIDER             = 500;
    static constexpr float YAW_COMMAND_DIVIDER         = 500;

    // minimum PWM pulse width which is considered valid
    static const uint16_t PWM_PULSE_MIN   = 750;   

    // maximum PWM pulse width which is considered valid
    static const uint16_t PWM_PULSE_MAX   = 2250;  

    Arming *   m_arming;

    bool m_gotPidReset;

    typedef enum {
        STATE_CHECK,
        STATE_PROCESS,
        STATE_MODES,
        STATE_UPDATE,
        STATE_COUNT
    } state_e;

    Pt3Filter m_filterThrottle;
    Pt3Filter m_filterRoll;
    Pt3Filter m_filterPitch;
    Pt3Filter m_filterYaw;

    Pt3Filter m_filterDeflectionRoll;
    Pt3Filter m_filterDeflectionPitch;

    Pt3Filter  m_feedforwardPt3Roll;
    Pt3Filter  m_feedforwardPt3Pitch;
    Pt3Filter  m_feedforwardPt3Yaw;

    float m_commandThrottle;
    float m_commandRoll;
    float m_commandPitch;
    float m_commandYaw;

    float m_rawThrottle;
    float m_rawRoll;
    float m_rawPitch;
    float m_rawYaw;
    float m_rawAux1;
    float m_rawAux2;

    int16_t  m_lookupThrottleRc[THROTTLE_LOOKUP_TABLE_SIZE];

    uint8_t  m_autoSmoothnessFactorSetpoint;
    uint8_t  m_autoSmoothnessFactorThrottle;
    bool     m_auxiliaryProcessingRequired;
    uint32_t m_averageFrameTimeUs;
    bool     m_calculatedCutoffs;
    bool     m_dataProcessingRequired;
    Demands  m_dataToSmooth;
    uint16_t m_feedforwardCutoffFrequency;
    uint8_t  m_feedforwardCutoffSetting;
    bool     m_feedforwardLpfInitialized;
    bool     m_filterInitialized;
    int32_t  m_frameTimeDeltaUs;
    bool     m_gotNewData;
    bool     m_inFailsafeMode;
    bool     m_initializedFilter;
    bool     m_initializedThrottleTable;
    bool     m_isRateValid;
    uint32_t m_lastFrameTimeUs;
    uint32_t m_lastRxTimeUs;
    uint32_t m_needSignalBefore;
    uint32_t m_nextUpdateAtUs;
    uint32_t m_previousFrameTimeUs;
    uint32_t m_refreshPeriod;
    uint16_t m_setpointCutoffFrequency;
    uint8_t  m_setpointCutoffSetting;
    bool     m_signalReceived;
    state_e  m_state;
    uint16_t m_throttleCutoffFrequency;
    uint8_t  m_throttleCutoffSetting;
    uint32_t m_trainingCount;
    uint16_t m_trainingMax;
    uint16_t m_trainingMin;
    float    m_trainingSum;
    uint32_t m_validFrameTimeMs;

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

    float constrainRaw(const float value)
    {
        return value == 0 ?  value : constrain_f(value, PWM_PULSE_MIN, PWM_PULSE_MAX);
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
        auto tmp = constrain_f_i32(m_rawThrottle, 1050, PWM_MAX);
        auto tmp2 = (uint32_t)(tmp - 1050) * PWM_MIN / (PWM_MAX - 1050);

        m_commandThrottle = lookupThrottle(tmp2);

        m_commandRoll  = updateCommand(m_rawRoll,  +1);
        m_commandPitch = updateCommand(m_rawPitch, +1);
        m_commandYaw   = updateCommand(m_rawYaw,   -1);
    }

    bool calculateChannels(const uint32_t usec)
    {
        if (m_auxiliaryProcessingRequired) {
            m_auxiliaryProcessingRequired = false;
        }

        if (!m_dataProcessingRequired) {
            return false;
        }

        m_dataProcessingRequired = false;
        m_nextUpdateAtUs = usec + DELAY_15_HZ;

        m_rawThrottle = constrainRaw(m_rawThrottle);
        m_rawRoll     = constrainRaw(m_rawRoll);
        m_rawPitch    = constrainRaw(m_rawPitch);
        m_rawYaw      = constrainRaw(m_rawYaw);
 
        return true;
    }

    int32_t getFrameDelta(const uint32_t usec, int32_t *frameAgeUs)
    {
        auto frameTimeUs = m_lastFrameTimeUs;

        *frameAgeUs = cmpTimeUs(usec, frameTimeUs);

        const auto deltaUs = cmpTimeUs(frameTimeUs, m_previousFrameTimeUs);

        if (deltaUs) {
            m_frameTimeDeltaUs = deltaUs;
            m_previousFrameTimeUs = frameTimeUs;
        }

        return m_frameTimeDeltaUs;
    }

    bool processData(const uint32_t usec)
    {
        int32_t frameAgeUs;

        auto refreshPeriodUs = getFrameDelta(usec, &frameAgeUs);

        if (!refreshPeriodUs || cmpTimeUs(usec, m_lastRxTimeUs) <= frameAgeUs) {

            // calculate a delta here if not supplied by the protocol
            refreshPeriodUs = cmpTimeUs(usec, m_lastRxTimeUs); 
        }

        m_lastRxTimeUs = usec;

        m_isRateValid =
            ((uint32_t)refreshPeriodUs >= SMOOTHING_RATE_MIN_US &&
             (uint32_t)refreshPeriodUs <= SMOOTHING_RATE_MAX_US);

        m_refreshPeriod =
            constrain_i32_u32(refreshPeriodUs, SMOOTHING_RATE_MIN_US,
                    SMOOTHING_RATE_MAX_US);

        return throttleIsDown();
    }

    void ratePidFeedforwardLpfInit(const uint16_t filterCutoff)
    {
        if (filterCutoff > 0) {

            m_feedforwardLpfInitialized = true;

            m_feedforwardPt3Roll.init(filterCutoff);
            m_feedforwardPt3Pitch.init(filterCutoff);
            m_feedforwardPt3Yaw.init(filterCutoff);
        }
    }

    void ratePidFeedforwardLpfUpdate(const uint16_t filterCutoff)
    {
        if (filterCutoff > 0) {

            m_feedforwardPt3Roll.computeGain(filterCutoff);
            m_feedforwardPt3Pitch.computeGain(filterCutoff);
            m_feedforwardPt3Yaw.computeGain(filterCutoff);
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
        if (m_feedforwardCutoffSetting == 0) {
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
        return ((m_setpointCutoffSetting == 0) ||
                (m_feedforwardCutoffSetting == 0) ||
                (m_throttleCutoffSetting == 0)); 
    }

    void initializeFilter(void)
    {
        m_filterInitialized = false;
        m_averageFrameTimeUs = 0;
        m_autoSmoothnessFactorSetpoint = 30;
        m_autoSmoothnessFactorThrottle = 30;
        m_setpointCutoffSetting = 0;
        m_throttleCutoffSetting = 0;
        m_feedforwardCutoffSetting = 0;
        smoothingResetAccumulation();
        m_setpointCutoffFrequency =
            m_setpointCutoffSetting;
        m_throttleCutoffFrequency =
            m_throttleCutoffSetting;
        if (m_feedforwardCutoffSetting == 0) {
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
                m_feedforwardCutoffSetting;
        }

        m_calculatedCutoffs = smoothingAutoCalculate();

        // if we don't need to calculate cutoffs dynamically then the
        // filters can be initialized now
        if (!m_calculatedCutoffs) {
            setSmoothingFilterCutoffs();
            m_filterInitialized = true;
        }
    }

    auto processSmoothingFilter(const uint32_t usec, const Axes & rawSetpoints) -> Axes
    {
        if (!m_initializedFilter) {
            initializeFilter();
        }

        m_initializedFilter = true;

        if (m_gotNewData) {
            // for auto calculated filters we need to examine each rx frame
            // interval
            if (m_calculatedCutoffs) {

                const auto currentTimeMs = usec / 1000;

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

            m_dataToSmooth.throttle = m_commandThrottle;
            m_dataToSmooth.roll  = rawSetpoints.x;
            m_dataToSmooth.pitch = rawSetpoints.y;
            m_dataToSmooth.yaw   = rawSetpoints.z;
        }

        m_commandThrottle = smoothingFilterApply(
                &m_filterThrottle,
                m_dataToSmooth.throttle);

        return Axes(
                smoothingFilterApply(&m_filterRoll, m_dataToSmooth.roll),
                smoothingFilterApply(&m_filterPitch, m_dataToSmooth.pitch),
                smoothingFilterApply(&m_filterYaw, m_dataToSmooth.yaw));
    }

    static float getRawSetpoint(const float command, const float divider)
    {
        auto commandf = command / divider;

        auto commandfAbs = fabsf(commandf);

        auto angleRate = AnglePidController::applyRates(commandf, commandfAbs);

        return constrain_f(angleRate, -(float)RATE_LIMIT, +(float)RATE_LIMIT);
    }

    bool aux1IsSet(void)
    {
        return m_rawAux1 > 0.2;
    }

    bool throttleIsDown(void)
    {
        return m_rawThrottle < 1050;
    }

    bool check(const uint32_t usec)
    {
        auto signalReceived = false;
        auto useDataDrivenProcessing = true;

        if (m_state != STATE_CHECK) {
            return true;
        }

        const auto frameStatus = 
            devRead(m_rawThrottle,
                    m_rawRoll,
                    m_rawPitch,
                    m_rawYaw,
                    m_rawAux1,
                    m_rawAux2,
                    m_lastFrameTimeUs);

        if (frameStatus) {
            m_inFailsafeMode = false;
            signalReceived = true;
            if (signalReceived) {
                m_needSignalBefore = usec + NEED_SIGNAL_MAX_DELAY_US;
            }
        }

        if (signalReceived) {
            m_signalReceived = true;
        } else if (usec >= m_needSignalBefore) {
            m_signalReceived = false;
        }

        if ((signalReceived && useDataDrivenProcessing) ||
                cmpTimeUs(usec, m_nextUpdateAtUs) > 0) {
            m_dataProcessingRequired = true;
        }

        // data driven or 50Hz
        return m_dataProcessingRequired || m_auxiliaryProcessingRequired; 

    } // check

    // Runs in fast (inner, core) loop
    auto getDemands(const uint32_t usec) -> Demands
    {
        m_previousFrameTimeUs = m_gotNewData ? 0 : m_previousFrameTimeUs;

        Axes rawSetpoints = m_gotNewData ?

            Axes(
                    rawSetpoints.x = getRawSetpoint(m_commandRoll, COMMAND_DIVIDER),
                    rawSetpoints.y = getRawSetpoint(m_commandPitch, COMMAND_DIVIDER),
                    rawSetpoints.z = getRawSetpoint(m_commandYaw, YAW_COMMAND_DIVIDER)) :

                Axes(0,0,0);

        Axes setpointRates = processSmoothingFilter(usec, rawSetpoints);

        m_gotNewData = false;

        return Demands(
                constrain_f((m_commandThrottle - PWM_MIN) / (PWM_MAX - PWM_MIN), 0, 1),
                setpointRates.x,
                setpointRates.y,
                setpointRates.z);
    }

    void begin(Arming * arming)
    {
        m_arming = arming;

        devStart();
    }

    bool gotPidReset(void)
    {
        return m_gotPidReset;
    }

    // Increase priority for RX task
    void adjustDynamicPriority(uint32_t usec) 
    {
        if (m_dynamicPriority > 0) {
            m_ageCycles = 1 + (cmpTimeUs(usec, m_lastSignaledAtUs) / m_desiredPeriodUs);
            m_dynamicPriority = 1 + m_ageCycles;
        } else  {
            if (check(usec)) {
                m_lastSignaledAtUs = usec;
                m_ageCycles = 1;
                m_dynamicPriority = 2;
            } else {
                m_ageCycles = 0;
            }
        }
    }    

    protected:

    static float convert(
            const uint16_t value,
            const uint16_t srcmin,
            const uint16_t srcmax,
            const float dstmin=1000,
            const float dstmax=2000)
    {
        return dstmin + (dstmax-dstmin) * ((float)value - srcmin) / (srcmax - srcmin);
    }

    virtual void devStart(void) = 0;

    virtual bool devRead(
            float & rawThrottle,
            float & rawRoll,
            float & rawPitch,
            float & rawYaw,
            float & rawAux1,
            float & rawAux2,
            uint32_t & frameTimeUs) = 0;

    Receiver()
        : Task(33) // Hz
    {
    }

    void fun(uint32_t usec)
    {
        const auto haveSignal =
            (usec - m_lastFrameTimeUs) < (int32_t)(1000*TIMEOUT_MS);

        auto pidItermResetReady = false;
        auto pidItermResetValue = false;

        m_gotNewData = false;

        switch (m_state) {
            default:
            case STATE_CHECK:
                m_state = STATE_PROCESS;
                break;

            case STATE_PROCESS:
                pidItermResetReady = true;
                pidItermResetValue = processData(usec);
                if (!calculateChannels(usec)) {
                    m_state = STATE_CHECK;
                    break;
                }
                m_state = STATE_MODES;
                break;

            case STATE_MODES:
                m_arming->attempt(usec, aux1IsSet());
                m_state = STATE_UPDATE;
                break;

            case STATE_UPDATE:
                m_gotNewData = true;
                updateCommands();
                m_arming->updateFromReceiver(throttleIsDown(), aux1IsSet(), haveSignal);
                m_state = STATE_CHECK;
                break;
        }

        if (pidItermResetReady) {
            m_gotPidReset = pidItermResetValue;
        }
    }

    float getRawThrottle(void)
    {
        return m_rawThrottle;
    }

    float getRawRoll(void)
    {
        return m_rawRoll;
    }

    float getRawPitch(void)
    {
        return m_rawPitch;
    }

    float getRawYaw(void)
    {
        return m_rawYaw;
    }

    float getRawAux1(void)
    {
        return m_rawAux1;
    }

    float getRawAux2(void)
    {
        return m_rawAux2;
    }
};
