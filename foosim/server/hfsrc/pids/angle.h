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

#include "constrain.h"
#include "filters/pt1.h"
#include "filters/pt2.h"
#include "pid.h"
#include "utils.h"

class AnglePidController : public PidController {

    private:

        // minimum of 5ms between updates
        static const uint16_t DYN_LPF_THROTTLE_UPDATE_DELAY_US = 5000; 

        static const uint16_t DYN_LPF_THROTTLE_STEPS = 100;

        // Full iterm suppression in setpoint mode at high-passed setpoint rate
        // > 40deg/sec
        static constexpr float ITERM_RELAX_SETPOINT_THRESHOLD = 40;
        static const uint8_t   ITERM_RELAX_CUTOFF     = 15;

        static const uint16_t DTERM_LPF1_DYN_MIN_HZ = 75;
        static const uint16_t DTERM_LPF1_DYN_MAX_HZ = 150;
        static const uint16_t DTERM_LPF2_HZ         = 150;

        static const uint16_t YAW_LOWPASS_HZ        = 100;

        static const uint8_t  ITERM_WINDUP_POINT_PERCENT = 85;        

        static const uint8_t D_MIN = 30;
        static const uint8_t D_MIN_GAIN = 37;
        static const uint8_t D_MIN_ADVANCE = 20;

        static const uint8_t FEEDFORWARD_MAX_RATE_LIMIT = 90;

        static const uint8_t DYN_LPF_CURVE_EXPO = 5;

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

        static constexpr float OUTPUT_SCALING = 1000;
        static const uint16_t  LIMIT_YAW  = 400;
        static const uint16_t  LIMIT      = 500;

        static float MAX_VELOCITY_CYCLIC() 
        {
            return RATE_ACCEL_LIMIT * 100 * DT;
        }

        static float MAX_VELOCITY_YAW() 
        {
            return YAW_RATE_ACCEL_LIMIT * 100 * DT; 
        }

        static float FREQUENCY() 
        {
            return 1.0f / DT; 
        }

        // Common values for all three axes
        typedef struct {

            float previousSetpoint;
            float I;

        } axis_t;

        // Values for roll and pitch
        typedef struct {

            axis_t    axis;
            Pt1Filter dtermLpf1 = Pt1Filter(DTERM_LPF1_DYN_MIN_HZ);
            Pt1Filter dtermLpf2 = Pt1Filter(DTERM_LPF2_HZ);
            Pt2Filter dMinLpf = Pt2Filter(D_MIN_LOWPASS_HZ);
            Pt2Filter dMinRange = Pt2Filter(D_MIN_RANGE_HZ);
            Pt1Filter windupLpf = Pt1Filter(ITERM_RELAX_CUTOFF); 
            float     previousDterm;

        } cyclicAxis_t;

        // Value for yaw
        Pt1Filter m_ptermYawLpf = Pt1Filter(YAW_LOWPASS_HZ);

        cyclicAxis_t m_roll;
        cyclicAxis_t m_pitch;
        axis_t       m_yaw;

        int32_t  m_dynLpfPreviousQuantizedThrottle;  
        float    m_k_rate_p;
        float    m_k_rate_i;
        float    m_k_rate_d;
        float    m_k_rate_f;
        float    m_k_level_p;

        float applyFeedforwardLimit(
                const float value,
                const float currentSetpoint,
                const float maxRateLimit) {

            return value * currentSetpoint > 0.0f ?
                fabsf(currentSetpoint) <= maxRateLimit ?
                constrain_f(value, (-maxRateLimit -
                            currentSetpoint) * m_k_rate_p,
                        (maxRateLimit - currentSetpoint) *
                        m_k_rate_p) :
                0 :
                0;
        }

        float accelerationLimit(
                axis_t * axis,
                const float currentSetpoint,
                const float maxVelocity)
        {
            const float currentVelocity = currentSetpoint - axis->previousSetpoint;

            const float newSetpoint = 
                fabsf(currentVelocity) > maxVelocity ?
                currentVelocity > 0 ?
                axis->previousSetpoint + maxVelocity :
                axis->previousSetpoint - maxVelocity :
                currentSetpoint;

            axis->previousSetpoint = newSetpoint;

            return newSetpoint;
        }

        float applyItermRelax(
                cyclicAxis_t & cyclicAxis,
                const float iterm,
                const float currentSetpoint,
                const float itermErrorRate)
        {
            const float setpointLpf = cyclicAxis.windupLpf.apply(currentSetpoint);

            const float setpointHpf = fabsf(currentSetpoint - setpointLpf);

            const auto itermRelaxFactor =
                fmaxf(0, 1 - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD);

            const auto isDecreasingI =
                ((iterm > 0) && (itermErrorRate < 0)) ||
                ((iterm < 0) && (itermErrorRate > 0));

            return itermErrorRate * (!isDecreasingI ? itermRelaxFactor : 1);
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

        static void initLpf1(cyclicAxis_t & cyclicAxis, const float cutoffFreq)
        {
            cyclicAxis.dtermLpf1.computeGain(cutoffFreq);
        }

        void pidDynLpfDTermUpdate(const float throttle)
        {
            const auto dyn_lpf_min = DTERM_LPF1_DYN_MIN_HZ;
            const auto dyn_lpf_max = DTERM_LPF1_DYN_MAX_HZ;
            const auto cutoffFreq =
                dynLpfCutoffFreq(throttle, dyn_lpf_min, dyn_lpf_max,
                        DYN_LPF_CURVE_EXPO);

            initLpf1(m_roll, cutoffFreq);
            initLpf1(m_pitch, cutoffFreq);
        }

        float levelPid(const float currentSetpoint, const float currentAngle)
        {
            // calculate error angle and limit the angle to the max inclination
            // rcDeflection in [-1.0, 1.0]

            const auto angle = constrain_f(LEVEL_ANGLE_LIMIT * currentSetpoint,
                    -LEVEL_ANGLE_LIMIT, +LEVEL_ANGLE_LIMIT);

            const auto angleError = angle - (currentAngle / 10);

            return m_k_level_p > 0 ?
                angleError * m_k_level_p :
                currentSetpoint;
        }

        float computeFeedforward(
                const float currentSetpoint,
                const float feedforwardMaxRate,
                const float demandDelta)
        {
            // halve feedforward in Level mode since stick sensitivity is
            // weaker by about half transition now calculated in
            // feedforward.c when new RC data arrives 
            const auto feedForward = m_k_rate_f * demandDelta * FREQUENCY();

            const auto feedforwardMaxRateLimit =
                feedforwardMaxRate * FEEDFORWARD_MAX_RATE_LIMIT * 0.01f;

            return feedforwardMaxRateLimit != 0 ?
                applyFeedforwardLimit(
                        feedForward,
                        currentSetpoint,
                        feedforwardMaxRateLimit) :
                feedForward;
        }

        float computeDMinFactor(
                cyclicAxis_t & cyclicAxis,
                const float dMinPercent,
                const float demandDelta,
                const float delta)
        {
            const auto d_min_gyro_gain =
                D_MIN_GAIN * D_MIN_GAIN_FACTOR / D_MIN_LOWPASS_HZ;

            const auto dMinGyroFactor = 
                fabsf(cyclicAxis.dMinRange.apply(delta)) * d_min_gyro_gain;

            const auto d_min_setpoint_gain =
                D_MIN_GAIN * D_MIN_SETPOINT_GAIN_FACTOR *
                D_MIN_ADVANCE * FREQUENCY() /
                (100 * D_MIN_LOWPASS_HZ);

            const auto dMinSetpointFactor =
                (fabsf(demandDelta)) * d_min_setpoint_gain;

            const auto dMinFactor = dMinPercent + (1.0f - dMinPercent) *
                fmaxf(dMinGyroFactor, dMinSetpointFactor);

            const auto dMinFactorFiltered = cyclicAxis.dMinLpf.apply(dMinFactor);

            return fminf(dMinFactorFiltered, 1.0f);
        }

        float computeDerivative(
                cyclicAxis_t & cyclicAxis,
                const float demandDelta,
                const float dterm)
        {
            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with
            // dynamically calculated deltaT whenever another task causes
            // the PID loop execution to be delayed.
            const float delta = -(dterm - cyclicAxis.previousDterm) * FREQUENCY();

            const auto preTpaD = m_k_rate_d * delta;

            const auto dMinPercent = 
                D_MIN > 0 && D_MIN < m_k_rate_d ?
                D_MIN / m_k_rate_d :
                0.0f;

            const auto dMinFactor =
                dMinPercent > 0 ?
                computeDMinFactor(cyclicAxis, dMinPercent, demandDelta, delta) :
                1.0f;

            // Apply the dMinFactor
            return preTpaD * dMinFactor;
        }

        float updateCyclic(
                const float demand,
                const float angle,
                const float angvel,
                cyclicAxis_t & cyclicAxis)
        {
            const auto maxVelocity = MAX_VELOCITY_CYCLIC();

            axis_t * axis = &cyclicAxis.axis;

            const auto currentSetpoint = 
                maxVelocity ?
                accelerationLimit(axis, demand, maxVelocity) :
                demand;

            const auto newSetpoint = levelPid(currentSetpoint, angle);

            // -----calculate error rate
            const auto errorRate = newSetpoint - angvel;

            const auto itermErrorRate = applyItermRelax(
                    cyclicAxis,
                    axis->I,
                    newSetpoint,
                    errorRate);

            // -----calculate P component
            const auto P = m_k_rate_p * errorRate;

            // -----calculate I component
            axis->I =
                constrain_f(axis->I + (m_k_rate_i * DT) * itermErrorRate,
                    -ITERM_LIMIT, +ITERM_LIMIT);

            // -----calculate D component
            const auto dterm = cyclicAxis.dtermLpf2.apply(cyclicAxis.dtermLpf1.apply(angvel));
            const auto D =
                m_k_rate_d > 0 ?
                computeDerivative(cyclicAxis, 0, dterm) :
                0;

            cyclicAxis.previousDterm = dterm;

            // -----calculate feedforward component
            const auto F =
                m_k_rate_f > 0 ?
                computeFeedforward(newSetpoint, 670, 0) :
                0;

            return P + axis->I + D + F;
        }

        float updateYaw(const float demand, const float angvel)
        {
            // gradually scale back integration when above windup point
            const auto itermWindupPointInv =
                1 / (1 - (ITERM_WINDUP_POINT_PERCENT / 100));

            const auto dynCi = DT * 
                (itermWindupPointInv > 1 ?
                 constrain_f(itermWindupPointInv, 0, 1) :
                 1);

            const auto maxVelocity = MAX_VELOCITY_YAW();

            const auto currentSetpoint = 
                maxVelocity ?
                accelerationLimit(&m_yaw, demand, maxVelocity) :
                demand;

            const auto errorRate = currentSetpoint - angvel;

            // -----calculate P component
            const auto P = m_ptermYawLpf.apply(m_k_rate_p * errorRate);

            // -----calculate I component, constraining windup
            m_yaw.I = constrain_f(
                    m_yaw.I + (m_k_rate_i * dynCi) * errorRate,
                    -ITERM_LIMIT, ITERM_LIMIT);

            return P + m_yaw.I; 
        }

        static float constrainOutput(const float demand, const float limit)
        {
            return constrain_f(demand, -limit, +limit) / OUTPUT_SCALING;
        }

        // [-1,+1] => [-670,+670] with nonlinearity
        static float rescale(const float command)
        {
            static constexpr float CTR = 0.104;

            const auto expof = command * fabsf(command);
            const auto angleRate = command * CTR + (1-CTR) * expof;
            return 670 * angleRate;
        }

    public:

        AnglePidController(
                const float k_rate_p = 1.441305,
                const float k_rate_i = 48.8762 ,
                const float k_rate_d = 0.021160,
                const float k_rate_f = 0.0165048, 
                const float k_level_p = 0.0) // 3.0
        {
            m_k_rate_p = k_rate_p;
            m_k_rate_i = k_rate_i;
            m_k_rate_d = k_rate_d;
            m_k_rate_f = k_rate_f;
            m_k_level_p = k_level_p;

            // to allow an initial zero throttle to set the filter cutoff
            m_dynLpfPreviousQuantizedThrottle = -1;  
        }

        virtual void modifyDemands(
                Demands & demands,
                const int32_t dusec,
                const VehicleState & vstate,
                const bool reset) override
        {
            const auto rollDemand  = rescale(demands.roll);
            const auto pitchDemand = rescale(demands.pitch);
            const auto yawDemand   = rescale(demands.yaw);

            const auto roll=
                updateCyclic(rollDemand, vstate.phi, vstate.dphi, m_roll);

            const auto pitch =
                updateCyclic(pitchDemand, vstate.theta, vstate.dtheta, m_pitch);

            const auto yaw = updateYaw(yawDemand, vstate.dpsi);

            if (reset) {
                m_roll.axis.I = 0;
                m_pitch.axis.I = 0;
                m_yaw.I = 0;
            }

            if (dusec >= DYN_LPF_THROTTLE_UPDATE_DELAY_US) {

                // quantize the throttle reduce the number of filter updates
                const int32_t quantizedThrottle =
                    lrintf(demands.throttle * DYN_LPF_THROTTLE_STEPS); 

                if (quantizedThrottle != m_dynLpfPreviousQuantizedThrottle) {

                    // scale the quantized value back to the throttle range so the
                    // filter cutoff steps are repeatable
                    const auto dynLpfThrottle =
                        (float)quantizedThrottle / DYN_LPF_THROTTLE_STEPS;
                    pidDynLpfDTermUpdate(dynLpfThrottle);
                    m_dynLpfPreviousQuantizedThrottle = quantizedThrottle;
                }
            }

            demands.roll = constrainOutput(roll, LIMIT);
            demands.pitch = constrainOutput(pitch, LIMIT),
            demands.yaw = -constrainOutput(yaw, LIMIT_YAW);

        } // modifyDemands

}; // class AnglePidController
