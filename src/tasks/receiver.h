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

    static const uint8_t THROTTLE_LOOKUP_TABLE_SIZE = 12;

    static const uint32_t TIMEOUT_MS = 500;

    static const uint32_t DELAY_15_HZ = 1000000 / 15;

    static const uint32_t NEED_SIGNAL_MAX_DELAY_US = 1000000 / 10;

    static constexpr float THR_EXPO8 = 0;
    static constexpr float THR_MID8  = 50;

    // minimum PWM pulse width which is considered valid
    static const uint16_t PWM_PULSE_MIN   = 750;   

    // maximum PWM pulse width which is considered valid
    static const uint16_t PWM_PULSE_MAX   = 2250;  

    Arming * m_arming;

    typedef enum {
        STATE_CHECK,
        STATE_PROCESS,
        STATE_MODES,
        STATE_UPDATE,
        STATE_COUNT
    } state_e;

    bool     m_auxiliaryProcessingRequired;
    bool     m_dataProcessingRequired;
    int32_t  m_frameTimeDeltaUs;
    bool     m_gotNewData;
    bool     m_gotPidReset;
    uint32_t m_lastFrameTimeUs;
    int16_t  m_lookupThrottleRc[THROTTLE_LOOKUP_TABLE_SIZE];
    uint32_t m_nextUpdateAtUs;
    uint32_t m_previousFrameTimeUs;
    float    m_rawThrottle;
    float    m_rawRoll;
    float    m_rawPitch;
    float    m_rawYaw;
    float    m_rawAux1;
    float    m_rawAux2;
    state_e  m_state;

    float constrainRaw(const float value)
    {
        return value == 0 ?  value : constrain_f(value, PWM_PULSE_MIN, PWM_PULSE_MAX);
    }

    float lookupThrottle(const int32_t tmp)
    {
        static bool _initializedThrottleTable;

        if (!_initializedThrottleTable) {
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

        _initializedThrottleTable = true;

        const auto tmp3 = tmp / 100;

        // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
        return (float)(m_lookupThrottleRc[tmp3] + (tmp - tmp3 * 100) *
            (m_lookupThrottleRc[tmp3 + 1] - m_lookupThrottleRc[tmp3]) / 100);
    }

     // [1000,2000] => [-1,+1]
    static float rescaleCommand(const float raw, const float sgn)
    {
        const auto tmp = fminf(fabs(raw - 1500), 500);
        const auto cmd = tmp * sgn;
        const auto command = raw < 1500 ? -cmd : cmd;
        return command / 500;
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
        static uint32_t _lastRxTimeUs;

        int32_t frameAgeUs;

        auto refreshPeriodUs = getFrameDelta(usec, &frameAgeUs);

        if (!refreshPeriodUs || cmpTimeUs(usec, _lastRxTimeUs) <= frameAgeUs) {

            // calculate a delta here if not supplied by the protocol
            refreshPeriodUs = cmpTimeUs(usec, _lastRxTimeUs); 
        }

        _lastRxTimeUs = usec;

        return throttleIsDown();
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
            signalReceived = true;
        }

        if ((signalReceived && useDataDrivenProcessing) ||
                cmpTimeUs(usec, m_nextUpdateAtUs) > 0) {
            m_dataProcessingRequired = true;
        }

        // data driven or 50Hz
        return m_dataProcessingRequired || m_auxiliaryProcessingRequired; 

    } // check

    // Runs in fast (inner, core) loop
    auto getDemands(void) -> Demands
    {
        m_previousFrameTimeUs = m_gotNewData ? 0 : m_previousFrameTimeUs;

        // Throttle [1000,2000] => [1000,2000]
        auto tmp = constrain_f_i32(m_rawThrottle, 1050, PWM_MAX);
        auto tmp2 = (uint32_t)(tmp - 1050) * PWM_MIN / (PWM_MAX - 1050);
        auto commandThrottle = lookupThrottle(tmp2);

        Axes rawSetpoints = m_gotNewData ?

            Axes(
                    rescaleCommand(m_rawRoll, +1),
                    rescaleCommand(m_rawPitch, +1),
                    rescaleCommand(m_rawYaw, -1)
                    ) :

                Axes(0,0,0);

        static Axes _axes;

        if (m_gotNewData) {

            _axes.x = rawSetpoints.x;
            _axes.y = rawSetpoints.y;
            _axes.z = rawSetpoints.z;
        }

        m_gotNewData = false;

        return Demands(
                constrain_f((commandThrottle - PWM_MIN) / (PWM_MAX - PWM_MIN), 0, 1),
                _axes.x,
                _axes.y,
                _axes.z);
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

    // Task function, called periodically
    void fun(uint32_t usec)
    {
        const auto haveSignal = (usec - m_lastFrameTimeUs) < (int32_t)(1000*TIMEOUT_MS);

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
