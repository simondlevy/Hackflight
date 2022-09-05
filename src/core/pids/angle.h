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

#include <string.h>
#include <math.h>

#include "core/clock.h"
#include "core/constrain.h"
#include "core/filters/pt1.h"
#include "core/filters/pt2.h"
#include "core/pid.h"
#include "time.h"

class AnglePidController : public PidController {

    private:

        // minimum of 5ms between updates
        static const uint16_t DYN_LPF_THROTTLE_UPDATE_DELAY_US = 5000; 

        static const uint16_t DYN_LPF_THROTTLE_STEPS = 100;

        // The constant scale factor to replace the Kd component of the
        // feedforward calculation.  This value gives the same "feel" as the
        // previous Kd default of 26 (26 * DTERM_SCALE)
        static constexpr float FEEDFORWARD_SCALE = 0.013754;

        // Full iterm suppression in setpoint mode at high-passed setpoint rate
        // > 40deg/sec
        static constexpr float   ITERM_RELAX_SETPOINT_THRESHOLD = 40;
        static const uint8_t ITERM_RELAX_CUTOFF     = 15;

        static const uint16_t DTERM_LPF1_DYN_MIN_HZ = 75;
        static const uint16_t DTERM_LPF1_DYN_MAX_HZ = 150;
        static const uint16_t DTERM_LPF2_HZ         = 150;
        static const uint16_t YAW_LOWPASS_HZ     = 100;
        static const bool     USE_INTEGRATED_YAW = false;  // XXX try true?
        static const uint8_t  ITERM_WINDUP_POINT_PERCENT = 85;        

        // How much integrated yaw should be reduced to offset the drag based
        // yaw component
        static const uint8_t INTEGRATED_YAW_RELAX = 200;  

        static const uint8_t D_MIN = 30;
        static const uint8_t D_MIN_GAIN = 37;
        static const uint8_t D_MIN_ADVANCE = 20;

        // Amount of lowpass type smoothing for feedforward steps
        static constexpr float FEEDFORWARD_SMOOTH_FACTOR = 0.75;      

        static const uint8_t FEEDFORWARD_JITTER_FACTOR = 7;
        static const uint8_t FEEDFORWARD_BOOST_FACTOR  = 15;
        static const uint8_t FEEDFORWARD_MAX_RATE_LIMIT = 90;
        static const uint8_t DYN_LPF_CURVE_EXPO = 5;

        // Scale factors to make best use of range with D_LPF debugging, aiming
        // for max +/-16K as debug values are 16 bit
        static constexpr float D_LPF_FILT_SCALE = 22;

        // PT2 lowpass input cutoff to peak D around propwash frequencies
        static constexpr float D_MIN_RANGE_HZ   = 85;  

        // PT2 lowpass cutoff to smooth the boost effect
        static constexpr float D_MIN_LOWPASS_HZ = 35;  

        static constexpr float D_MIN_GAIN_FACTOR          = 0.00008;
        static constexpr float D_MIN_SETPOINT_GAIN_FACTOR = 0.00008f;

        static const uint16_t RATE_ACCEL_LIMIT = 0;
        static const uint16_t YAW_RATE_ACCEL_LIMIT = 0;
        static const uint16_t ITERM_LIMIT = 400;

        static constexpr float LEVEL_ANGLE_LIMIT = 45;

        static const uint8_t RC_EXPO = 0;
        static const uint8_t RC_RATE = 7;
        static const uint8_t RATE    = 67;

        static float MAX_VELOCITY_CYCLIC() 
        {
            return RATE_ACCEL_LIMIT * 100 * Clock::DT();
        }

        static float MAX_VELOCITY_YAW() 
        {
            return YAW_RATE_ACCEL_LIMIT * 100 * Clock::DT(); 
        }

        static float FREQUENCY() 
        {
            return 1.0f / Clock::DT(); 
        }

        typedef struct pidAxisData_s {
            float P;
            float I;
            float D;
            float F;
            float Sum;
        } pidAxisData_t;

        Pt1Filter m_dtermLpf1[3] = {
            Pt1Filter(DTERM_LPF1_DYN_MIN_HZ),
            Pt1Filter(DTERM_LPF1_DYN_MIN_HZ),
            Pt1Filter(DTERM_LPF1_DYN_MIN_HZ)
        };

        Pt1Filter m_dtermLpf2[3] = {
            Pt1Filter(DTERM_LPF2_HZ),
            Pt1Filter(DTERM_LPF2_HZ),
            Pt1Filter(DTERM_LPF2_HZ)
        };

        Pt2Filter m_dMinLpf[3] = {
            Pt2Filter(D_MIN_LOWPASS_HZ),
            Pt2Filter(D_MIN_LOWPASS_HZ),
            Pt2Filter(D_MIN_LOWPASS_HZ)
        };

        Pt2Filter m_dMinRange[3] = {
            Pt2Filter(D_MIN_RANGE_HZ),
            Pt2Filter(D_MIN_RANGE_HZ),
            Pt2Filter(D_MIN_RANGE_HZ)
        };

        pidAxisData_t m_data[3];
        int32_t       m_dynLpfPreviousQuantizedThrottle;  
        bool          m_feedforwardLpfInitialized;
        float         m_k_rate_p;
        float         m_k_rate_i;
        float         m_k_rate_d;
        float         m_k_rate_f;
        float         m_k_level_p;
        float         m_sum;
        uint32_t      m_lastDynLpfUpdateUs;
        float         m_previousSetpointCorrection[3];
        float         m_previousGyroRateDterm[3];
        float         m_previousSetpoint[3];

        Pt1Filter m_ptermYawLpf = Pt1Filter(YAW_LOWPASS_HZ);

        Pt1Filter  m_windupLpf[3] = {
            Pt1Filter(ITERM_RELAX_CUTOFF),
            Pt1Filter(ITERM_RELAX_CUTOFF),
            Pt1Filter(ITERM_RELAX_CUTOFF)
        };

        float applyFeedforwardLimit(
                float value,
                float currentPidSetpoint,
                float maxRateLimit) {

            if (value * currentPidSetpoint > 0.0f) {
                if (fabsf(currentPidSetpoint) <= maxRateLimit) {
                    value = constrain_f(value, (-maxRateLimit -
                                currentPidSetpoint) * m_k_rate_p,
                            (maxRateLimit - currentPidSetpoint) *
                            m_k_rate_p);
                } else {
                    value = 0;
                }
            }

            return value;
        }

        float accelerationLimit(const uint8_t axis, float currentPidSetpoint)
        {
            const float currentVelocity =
                currentPidSetpoint - m_previousSetpoint[axis];

            float maxVelocity =
                axis == 2 ? MAX_VELOCITY_YAW() : MAX_VELOCITY_CYCLIC();

            if (fabsf(currentVelocity) > maxVelocity) {
                currentPidSetpoint = (currentVelocity > 0) ?
                    m_previousSetpoint[axis] + maxVelocity :
                    m_previousSetpoint[axis] - maxVelocity;
            }

            m_previousSetpoint[axis] = currentPidSetpoint;

            return currentPidSetpoint;
        }

        void applyItermRelax(
                const int axis,
                const float iterm,
                float *itermErrorRate,
                float *currentPidSetpoint)
        {
            const float setpointLpf = m_windupLpf[axis].apply(*currentPidSetpoint);

            const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);

            if (axis < 2) {

                const auto itermRelaxFactor =
                    fmaxf(0, 1 - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD);

                const auto isDecreasingI =
                    ((iterm > 0) && (*itermErrorRate < 0)) ||
                    ((iterm < 0) && (*itermErrorRate > 0));

                if (isDecreasingI) {
                    // Do Nothing, use the precalculed itermErrorRate
                } else {
                    *itermErrorRate *= itermRelaxFactor;
                } 
            }
        }

        static float dynLpfCutoffFreq(
                const float throttle,
                const uint16_t dynLpfMin,
                const uint16_t dynLpfMax,
                const uint8_t expo) {
            const float expof = expo / 10.0f;
            const auto curve = throttle * (1 - throttle) * expof + throttle;
            return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
        }

        void pidDynLpfDTermUpdate(const float throttle)
        {
            const auto dyn_lpf_min = DTERM_LPF1_DYN_MIN_HZ;
            const auto dyn_lpf_max = DTERM_LPF1_DYN_MAX_HZ;
            auto cutoffFreq =
                dynLpfCutoffFreq(throttle, dyn_lpf_min, dyn_lpf_max,
                        DYN_LPF_CURVE_EXPO);

            m_dtermLpf1[0].computeGain(cutoffFreq);
            m_dtermLpf1[1].computeGain(cutoffFreq);
            m_dtermLpf1[2].computeGain(cutoffFreq);
        }

        void updateDynLpfCutoffs(const uint32_t currentTimeUs, const float throttle)
        {
            if (cmpTimeUs(currentTimeUs, m_lastDynLpfUpdateUs) >=
                    DYN_LPF_THROTTLE_UPDATE_DELAY_US) {

                // quantize the throttle reduce the number of filter updates
                const int32_t quantizedThrottle =
                    lrintf(throttle * DYN_LPF_THROTTLE_STEPS); 

                if (quantizedThrottle != m_dynLpfPreviousQuantizedThrottle) {

                    // scale the quantized value back to the throttle range so the
                    // filter cutoff steps are repeatable
                    auto dynLpfThrottle =
                        (float)quantizedThrottle / DYN_LPF_THROTTLE_STEPS;
                    pidDynLpfDTermUpdate(dynLpfThrottle);
                    m_dynLpfPreviousQuantizedThrottle = quantizedThrottle;
                    m_lastDynLpfUpdateUs = currentTimeUs;
                }
            }
        }

        float levelPid(const float currentSetpoint, const float currentAngle)
        {
            // calculate error angle and limit the angle to the max inclination
            // rcDeflection in [-1.0, 1.0]
            float angle = LEVEL_ANGLE_LIMIT * currentSetpoint;
            angle = constrain_f(angle, -LEVEL_ANGLE_LIMIT, LEVEL_ANGLE_LIMIT);
            const float errorAngle = angle - (currentAngle / 10);
            return m_k_level_p > 0 ?
                errorAngle * m_k_level_p :
                currentSetpoint;
        }

        void initGyroRateDterm(float gyroRate, float gyroRateDterm[], uint8_t axis)
        {
            gyroRateDterm[axis] = gyroRate;
            gyroRateDterm[axis] = m_dtermLpf1[axis].apply(gyroRateDterm[axis]);
            gyroRateDterm[axis] = m_dtermLpf2[axis].apply(gyroRateDterm[axis]);
        }

        void runAxis(
                float angularVelocity,
                float pidSetpoint,
                float currentAngle,
                float gyroRateDterm[],
                float dynCi,
                uint8_t axis)
        {
            auto currentPidSetpoint = pidSetpoint;

            auto maxVelocity =
                axis == 2 ? MAX_VELOCITY_YAW() : MAX_VELOCITY_CYCLIC();
            if (maxVelocity) {
                currentPidSetpoint =
                    accelerationLimit(axis, currentPidSetpoint);
            }

            if (axis != 2) {
                currentPidSetpoint = levelPid(currentPidSetpoint, currentAngle);
            }

            // Handle yaw spin recovery - zero the setpoint on yaw to aid in
            // recovery It's not necessary to zero the set points for R/P
            // because the PIDs will be zeroed below

            // -----calculate error rate
            auto errorRate = currentPidSetpoint - angularVelocity; // r - y
            const auto previousIterm = m_data[axis].I;
            auto itermErrorRate = errorRate;
            auto uncorrectedSetpoint = currentPidSetpoint;

            applyItermRelax(axis, previousIterm, &itermErrorRate,
                    &currentPidSetpoint);
            errorRate = currentPidSetpoint - angularVelocity;
            float setpointCorrection =
                currentPidSetpoint - uncorrectedSetpoint;

            // --------low-level gyro-based PID based on 2DOF PID controller.
            // ---------- 2-DOF PID controller with optional filter on
            // derivative term.  b = 1 and only c (feedforward weight) can be
            // tuned (amount derivative on measurement or error).

            // -----calculate P component
            m_data[axis].P = m_k_rate_p * errorRate;
            if (axis == 2) {
                m_data[axis].P = m_ptermYawLpf.apply(m_data[axis].P);
            }

            // -----calculate I component
            // if launch control is active override the iterm gains and apply
            // iterm windup protection to all axes
            auto Ki =
                m_k_rate_i * ((axis == 2 && !USE_INTEGRATED_YAW) ?
                        2.5 :
                        1);

            auto axisDynCi = // check windup for yaw only
                (axis == 2) ? dynCi : Clock::DT(); 

            m_data[axis].I =
                constrain_f(previousIterm + (Ki * axisDynCi) * itermErrorRate,
                        -ITERM_LIMIT, ITERM_LIMIT);

            // -----calculate pidSetpointDelta
            auto pidSetpointDelta = 0.0f;
            auto feedforwardMaxRate = applyRates(1, 1);

            // -----calculate D component
            if ((axis < 2 && m_k_rate_d > 0)) {

                // Divide rate change by dT to get differential (ie dr/dt).
                // dT is fixed and calculated from the target PID loop time
                // This is done to avoid DTerm spikes that occur with
                // dynamically calculated deltaT whenever another task causes
                // the PID loop execution to be delayed.
                const float delta = -(gyroRateDterm[axis] -
                        m_previousGyroRateDterm[axis]) * FREQUENCY();

                float preTpaD = m_k_rate_d * delta;

                auto dMinFactor = 1.0f;

                auto dMinPercent = axis == 2 ?
                    0.0f :
                    D_MIN > 0 && D_MIN < m_k_rate_d ?
                    D_MIN / m_k_rate_d :
                    0.0f;

                if (dMinPercent > 0) {
                    const auto d_min_gyro_gain =
                        D_MIN_GAIN * D_MIN_GAIN_FACTOR / D_MIN_LOWPASS_HZ;
                    auto dMinGyroFactor = m_dMinRange[axis].apply(delta);
                    dMinGyroFactor = fabsf(dMinGyroFactor) * d_min_gyro_gain;
                    const auto d_min_setpoint_gain =
                        D_MIN_GAIN * D_MIN_SETPOINT_GAIN_FACTOR *
                        D_MIN_ADVANCE * FREQUENCY() /
                        (100 * D_MIN_LOWPASS_HZ);
                    const auto dMinSetpointFactor =
                        (fabsf(pidSetpointDelta)) * d_min_setpoint_gain;
                    dMinFactor = fmaxf(dMinGyroFactor, dMinSetpointFactor);
                    dMinFactor =
                        dMinPercent + (1.0f - dMinPercent) * dMinFactor;
                    dMinFactor = m_dMinLpf[axis].apply(dMinFactor);
                    dMinFactor = fminf(dMinFactor, 1.0f);
                }

                // Apply the dMinFactor
                preTpaD *= dMinFactor;
                m_data[axis].D = preTpaD;

                // Log the value of D pre application of TPA
                preTpaD *= D_LPF_FILT_SCALE;

            } else {
                m_data[axis].D = 0;
            }

            m_previousGyroRateDterm[axis] = gyroRateDterm[axis];

            // -----calculate feedforward component
            // include abs control correction in feedforward
            pidSetpointDelta += setpointCorrection -
                m_previousSetpointCorrection[axis];
            m_previousSetpointCorrection[axis] = setpointCorrection;

            // no feedforward in launch control
            auto feedforwardGain = m_k_rate_f;
            if (feedforwardGain > 0) {
                // halve feedforward in Level mode since stick sensitivity is
                // weaker by about half transition now calculated in
                // feedforward.c when new RC data arrives 
                auto feedForward =
                    feedforwardGain * pidSetpointDelta * FREQUENCY();

                auto feedforwardMaxRateLimit =
                    feedforwardMaxRate * FEEDFORWARD_MAX_RATE_LIMIT * 0.01f;

                bool shouldApplyFeedforwardLimits =
                    feedforwardMaxRateLimit != 0.0f && axis < 2;

                m_data[axis].F = shouldApplyFeedforwardLimits ?
                    applyFeedforwardLimit(
                            feedForward,
                            currentPidSetpoint,
                            feedforwardMaxRateLimit) :
                    feedForward;
            } else {
                m_data[axis].F = 0;
            }

            // calculating the PID sum
            const auto pidSum =
                m_data[axis].P +
                m_data[axis].I +
                m_data[axis].D +
                m_data[axis].F;
            if (axis == 2 && USE_INTEGRATED_YAW) {
                m_data[axis].Sum += pidSum * Clock::DT() * 100.0f;
                m_data[axis].Sum -= m_data[axis].Sum *
                    INTEGRATED_YAW_RELAX / 100000.0f * Clock::DT() /
                    0.000125f;
            } else {
                m_data[axis].Sum = pidSum;
            }
        }

    public:

        AnglePidController(
                const float k_rate_p,
                const float k_rate_i,
                const float k_rate_d,
                const float k_rate_f,
                const float k_level_p)
        {
            m_k_rate_p = k_rate_p;
            m_k_rate_i = k_rate_i;
            m_k_rate_d = k_rate_d;
            m_k_rate_f = k_rate_f;
            m_k_level_p = k_level_p;

            // to allow an initial zero throttle to set the filter cutoff
            m_dynLpfPreviousQuantizedThrottle = -1;  
        }

        static float applyRates(const float commandf, const float commandfAbs)
        {
            float expof = RC_EXPO / 100.0f;
            expof =
                commandfAbs * (powf(commandf, 5) * expof + commandf * (1 - expof));

            const auto centerSensitivity = RC_RATE * 10.0f;
            const auto stickMovement = fmaxf(0, RATE * 10.0f - centerSensitivity);
            const auto angleRate = commandf * centerSensitivity + 
                stickMovement * expof;

            return angleRate;
        }

        virtual auto update(
                const uint32_t currentTimeUs,
                const Demands & demands,
                const VehicleState & vstate,
                const bool reset) -> Demands override
        {
            // gradually scale back integration when above windup point
            auto dynCi = Clock::DT();
            const auto itermWindupPointInv =
                1 / (1 - (ITERM_WINDUP_POINT_PERCENT / 100));
            if (itermWindupPointInv > 1.0f) {
                dynCi *= constrain_f(itermWindupPointInv, 0.0f, 1.0f);
            }

            float gyroRateDterm[3] = {};

            // Precalculate gyro deta for D-term here, this allows loop unrolling
            initGyroRateDterm(vstate.dphi,   gyroRateDterm, 0);
            initGyroRateDterm(vstate.dtheta, gyroRateDterm, 1);
            initGyroRateDterm(vstate.dpsi,   gyroRateDterm, 2);

            // ----------PID controller----------
            runAxis(vstate.dphi, demands.roll, vstate.phi, gyroRateDterm, dynCi, 0);
            runAxis(vstate.dtheta, demands.pitch, vstate.theta, gyroRateDterm, dynCi, 1);
            runAxis(vstate.dpsi, demands.yaw, vstate.psi, gyroRateDterm, dynCi, 2);

            // Disable PID control if at zero throttle or if gyro overflow
            // detected This may look very innefficient, but it is done on
            // purpose to always show real CPU usage as in flight
            if (reset) {
                m_data[0].I = 0.0f;
                m_data[1].I = 0.0f;
                m_data[2].I = 0.0f;
            }

            updateDynLpfCutoffs(currentTimeUs, demands.throttle);

            return Demands(
                    demands.throttle,
                    m_data[0].Sum,
                    m_data[1].Sum,
                    m_data[2].Sum);

        } // update

}; // class AnglePidController

