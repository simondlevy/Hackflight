/*
   nCopyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <string.h>
#include <math.h>

#include "datatypes.h"
#include "debug.h"
#include "core_dt.h"
#include "pt1_filter.h"
#include "pt3_filter.h"
#include "rx_rate.h"

// minimum of 5ms between updates
static const uint16_t DYN_LPF_THROTTLE_UPDATE_DELAY_US = 5000; 

static const uint16_t DYN_LPF_THROTTLE_STEPS = 100;

// The constant scale factor to replace the Kd component of the feedforward
// calculation.  This value gives the same "feel" as the previous Kd default of
// 26 (26 * DTERM_SCALE)
static const float FEEDFORWARD_SCALE = 0.013754;

// Full iterm suppression in setpoint mode at high-passed setpoint rate >
// 40deg/sec
static const float   ITERM_RELAX_SETPOINT_THRESHOLD = 40;
static const uint8_t ITERM_RELAX_CUTOFF     = 15;

static const uint16_t DTERM_LPF1_DYN_MIN_HZ = 75;
static const uint16_t DTERM_LPF1_DYN_MAX_HZ = 150;
static const uint16_t DTERM_LPF2_HZ         = 150;
static const uint16_t YAW_LOWPASS_HZ     = 100;
static const bool     USE_INTEGRATED_YAW = false;  // XXX try true?
static const uint8_t  ITERM_WINDUP_POINT_PERCENT = 85;        

// How much integrated yaw should be reduced to offset the drag based yaw component
static const uint8_t INTEGRATED_YAW_RELAX = 200;  

static const uint8_t D_MIN = 30;
static const uint8_t D_MIN_GAIN = 37;
static const uint8_t D_MIN_ADVANCE = 20;

// Amount of lowpass type smoothing for feedforward steps
static const float FEEDFORWARD_SMOOTH_FACTOR = 0.75;      

static const uint8_t FEEDFORWARD_JITTER_FACTOR = 7;
static const uint8_t FEEDFORWARD_BOOST_FACTOR  = 15;
static const uint8_t FEEDFORWARD_MAX_RATE_LIMIT = 90;
static const uint8_t DYN_LPF_CURVE_EXPO = 5;


// Scale factors to make best use of range with D_LPF debugging, aiming for max
// +/-16K as debug values are 16 bit
static const float D_LPF_FILT_SCALE = 22;

// PT2 lowpass input cutoff to peak D around propwash frequencies
static const float D_MIN_RANGE_HZ   = 85;  

// PT2 lowpass cutoff to smooth the boost effect
static const float D_MIN_LOWPASS_HZ = 35;  

static const float D_MIN_GAIN_FACTOR          = 0.00008;
static const float D_MIN_SETPOINT_GAIN_FACTOR = 0.00008f;

static const uint16_t ITERM_LIMIT = 400;

static const float LEVEL_ANGLE_LIMIT = 45;

#if defined(__cplusplus)
extern "C" {
#endif

    static float pt2FilterApply(pt2Filter_t *filter, float input)
    {
        filter->state1 = filter->state1 + filter->k * (input - filter->state1);
        filter->state =
            filter->state + filter->k * (filter->state1 - filter->state);
        return filter->state;
    }

    static float pt2FilterGain(float f_cut, float dT)
    {
        const float order = 2;
        const float orderCutoffCorrection = 1 / sqrtf(powf(2, 1 / order) - 1);
        float RC = 1 / (2 * orderCutoffCorrection * M_PI * f_cut);
        // float RC = 1 / (2 * 1.553773974f * M_PI * f_cut);
        // where 1.553773974 = 1 / sqrt( (2^(1 / order) - 1) ) and order is 2
        return dT / (RC + dT);
    }

    static void pt2FilterInit(pt2Filter_t *filter, float k)
    {
        filter->state = 0;
        filter->state1 = 0;
        filter->k = k;
    }

    static float applyFeedforwardLimit(
            anglePidConstants_t * constants,
            float value,
            float currentPidSetpoint,
            float maxRateLimit) {

        if (value * currentPidSetpoint > 0) {
            if (fabsf(currentPidSetpoint) <= maxRateLimit) {
                value = constrain_f(value, (-maxRateLimit -
                            currentPidSetpoint) * constants->k_rate_p,
                        (maxRateLimit - currentPidSetpoint) *
                        constants->k_rate_p);
            } else {
                value = 0;
            }
        }

        return value;
    }

    static void cyclicItermRelax(
            cyclicPid_t * cyclic,
            const float iterm,
            float *itermErrorRate,
            float *currentPidSetpoint)
    {
        float setpointLpf =
            pt1FilterApply(&cyclic->windupLpf, *currentPidSetpoint);

        float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);

        const float itermRelaxFactor =
            fmaxf(0, 1 - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD);
        const bool isDecreasingI = ((iterm > 0) && (*itermErrorRate < 0)) ||
            ((iterm < 0) && (*itermErrorRate > 0));
        if (isDecreasingI) {
            // Do Nothing, use the precalculed itermErrorRate
        } else {
            *itermErrorRate *= itermRelaxFactor;
        } 
    }

    static float applySmoothingFeedforwardFilter(
            anglePid_t * pid,
            uint8_t index,
            float pidSetpointDelta)
    {
        float ret = pidSetpointDelta;
        if (pid->feedforwardLpfInitialized) {
            ret = pt3FilterApply(&pid->feedforwardPt3[index], pidSetpointDelta);
        }
        return ret;
    }

    static float dynLpfCutoffFreq(
            float throttle,
            uint16_t dynLpfMin,
            uint16_t dynLpfMax,
            uint8_t expo) {
        const float expof = expo / 10;
        static float curve;
        curve = throttle * (1 - throttle) * expof + throttle;
        return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
    }

    static void pidDynLpfDTermUpdateAxis(
            anglePid_t * pid,
            float cutoffFreq,
            uint8_t index)
    {
        pid->dtermLowpass[index].pt1Filter.k =
            pt1FilterGain(cutoffFreq, CORE_DT());
    }

    static void pidDynLpfDTermUpdate(anglePid_t * pid, float throttle)
    {
        float cutoffFreq =
            dynLpfCutoffFreq(
                    throttle,
                    DTERM_LPF1_DYN_MIN_HZ,
                    DTERM_LPF1_DYN_MAX_HZ,
                    DYN_LPF_CURVE_EXPO);

        pidDynLpfDTermUpdateAxis(pid, cutoffFreq, 0);
        pidDynLpfDTermUpdateAxis(pid, cutoffFreq, 1);
        pidDynLpfDTermUpdateAxis(pid, cutoffFreq, 2);
    }

    static void updateDynLpfCutoffs(
            anglePid_t * pid,
            uint32_t currentTimeUs,
            float throttle)
    {
        if (cmpTimeUs(currentTimeUs, pid->lastDynLpfUpdateUs) >=
                DYN_LPF_THROTTLE_UPDATE_DELAY_US) {

            // quantize the throttle reduce the number of filter updates
            int32_t quantizedThrottle =
                lrintf(throttle * DYN_LPF_THROTTLE_STEPS); 

            if (quantizedThrottle != pid->dynLpfPreviousQuantizedThrottle) {

                // scale the quantized value back to the throttle range so the
                // filter cutoff steps are repeatable
                float dynLpfThrottle =
                    (float)quantizedThrottle / DYN_LPF_THROTTLE_STEPS;
                pidDynLpfDTermUpdate(pid, dynLpfThrottle);
                pid->dynLpfPreviousQuantizedThrottle = quantizedThrottle;
                pid->lastDynLpfUpdateUs = currentTimeUs;
            }
        }
    }

    static float levelPid(
            anglePidConstants_t * constants,
            float currentSetpoint,
            float currentAngle)
    {
        // calculate error angle and limit the angle to the max inclination
        // rcDeflection in [-1.0, 1.0]
        float angle = LEVEL_ANGLE_LIMIT * currentSetpoint;
        angle = constrain_f(angle, -LEVEL_ANGLE_LIMIT, LEVEL_ANGLE_LIMIT);
        float errorAngle = angle - (currentAngle / 10);
        return constants->k_level_p > 0 ?
            errorAngle * constants->k_level_p :
            currentSetpoint;
    }

    static void pt1FilterInitAxis(anglePid_t * pid, uint8_t index)
    {
        pt1FilterInit(&pid->dtermLowpass[index].pt1Filter,
                pt1FilterGain(DTERM_LPF1_DYN_MIN_HZ, CORE_DT()));

        pt1FilterInit(&pid->dtermLowpass2[index].pt1Filter,
                pt1FilterGain(DTERM_LPF2_HZ, CORE_DT()));
    }

    static void pt1FilterInitWindupAxis(cyclicPid_t * cyclic)
    {
        pt1FilterInit(&cyclic->windupLpf,
                pt1FilterGain(ITERM_RELAX_CUTOFF, CORE_DT()));
    }

    static void pt2FilterInitAxis(anglePid_t * pid, uint8_t index)
    {
        pt2FilterInit(&pid->dMinRange[index],
                pt2FilterGain(D_MIN_RANGE_HZ, CORE_DT()));
        pt2FilterInit(&pid->dMinLowpass[index],
                pt2FilterGain(D_MIN_LOWPASS_HZ, CORE_DT()));
    }

    static float computeDtermAxis(
            anglePid_t * pid,
            float gyroRate,
            uint8_t index)
    {
        filterApplyFnPtr dtermLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;

        float dterm = dtermLowpassApplyFn(
                (filter_t *) &pid->dtermLowpass[index], gyroRate);

        filterApplyFnPtr dtermLowpass2ApplyFn = (filterApplyFnPtr)pt1FilterApply;
        
        return dtermLowpass2ApplyFn(
                (filter_t *) &pid->dtermLowpass2[index], dterm);
    }

    static void resetItermAxis(anglePid_t * pid, uint8_t index)
    {
        pid->data[index].I = 0;
    }

    static float computeFeedforwardAndSum(
            anglePid_t *pid,
            anglePidConstants_t * constants,
            bool shouldApplyFeedforwardLimits,
            float pidSetpointDelta,
            float setpointCorrection,
            float feedforwardMaxRate,
            float currentPidSetpoint,
            uint8_t index)
    {
        pidSetpointDelta += setpointCorrection -
            pid->previousSetpointCorrection[index];

        pid->previousSetpointCorrection[index] = setpointCorrection;

        float feedforwardGain = constants->k_rate_f;

        if (feedforwardGain > 0) {
            // halve feedforward in Level mode since stick sensitivity is
            // weaker by about half transition now calculated in
            // feedforward.c when new RC data arrives 
            float feedForward = feedforwardGain * pidSetpointDelta / CORE_DT();

            float feedforwardMaxRateLimit =
                feedforwardMaxRate * FEEDFORWARD_MAX_RATE_LIMIT * 0.01f;

            shouldApplyFeedforwardLimits = shouldApplyFeedforwardLimits && 
                (feedforwardMaxRateLimit != 0);

            pid->data[index].F = shouldApplyFeedforwardLimits ?
                applyFeedforwardLimit(
                        constants,
                        feedForward,
                        currentPidSetpoint,
                        feedforwardMaxRateLimit) :
                feedForward;
            pid->data[index].F =
                applySmoothingFeedforwardFilter(pid, index, pid->data[index].F);

        }

        else {
            pid->data[index].F = 0;
        }

        return
            pid->data[index].P +
            pid->data[index].I +
            pid->data[index].D +
            pid->data[index].F;
    }

    static void computeItermAxis(
            anglePid_t * pid,
            float previousIterm,
            float Ki,
            float axisDynCi,
            float itermErrorRate,
            uint8_t index)
    {
        pid->data[index].I =
            constrain_f(previousIterm + (Ki * axisDynCi) * itermErrorRate,
                    -ITERM_LIMIT, ITERM_LIMIT);
    }

    static void computeCyclic(
            anglePid_t * pid,
            cyclicPid_t * cyclic,
            anglePidConstants_t * constants,
            float pidSetpoint,
            float currentAngle,
            float gyroRate,
            uint8_t index)
    {
        float dterm = computeDtermAxis(pid, gyroRate, index);

        // Apply optional level PID to get initial setpoint
        float currentPidSetpoint =
            levelPid(constants, pidSetpoint, currentAngle);

        // -----calculate error rate
        float errorRate = currentPidSetpoint - gyroRate; // r - y
        const float previousIterm = pid->data[index].I;
        float itermErrorRate = errorRate;
        float uncorrectedSetpoint = currentPidSetpoint;

        cyclicItermRelax(cyclic, previousIterm, &itermErrorRate,
                &currentPidSetpoint);

        errorRate = currentPidSetpoint - gyroRate;
        float setpointCorrection = currentPidSetpoint - uncorrectedSetpoint;

        // -----calculate P component
        pid->data[index].P = constants->k_rate_p * errorRate;

        // -----calculate I component
        // if launch control is active override the iterm gains and apply
        // iterm windup protection to all axes
        float Ki = constants->k_rate_i;
        computeItermAxis(pid, previousIterm, Ki, CORE_DT(), itermErrorRate,
                index);

        float pidSetpointDelta = 0;
        float feedforwardMaxRate = rxApplyRates(1, 1);

        // -----calculate D component
        if (constants->k_rate_d > 0) {

            float freq = 1 / CORE_DT();

            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with
            // dynamically calculated deltaT whenever another task causes
            // the PID loop execution to be delayed.
            const float delta = -(dterm - cyclic->previousDterm) * freq;

            float dMinFactor = 1;

            float dMinPercent = 
                D_MIN > 0 && D_MIN < constants->k_rate_d ?
                D_MIN / constants->k_rate_d :
                0;

            if (dMinPercent > 0) {
                const float d_min_gyro_gain =
                    D_MIN_GAIN * D_MIN_GAIN_FACTOR / D_MIN_LOWPASS_HZ;
                float dMinGyroFactor =
                    pt2FilterApply(&pid->dMinRange[index], delta);
                dMinGyroFactor = fabsf(dMinGyroFactor) * d_min_gyro_gain;
                const float d_min_setpoint_gain =
                    D_MIN_GAIN * D_MIN_SETPOINT_GAIN_FACTOR *
                    D_MIN_ADVANCE * freq / (100 * D_MIN_LOWPASS_HZ);
                const float dMinSetpointFactor =
                    (fabsf(pidSetpointDelta)) * d_min_setpoint_gain;
                dMinFactor = fmaxf(dMinGyroFactor, dMinSetpointFactor);
                dMinFactor =
                    dMinPercent + (1 - dMinPercent) * dMinFactor;
                dMinFactor =
                    pt2FilterApply(&pid->dMinLowpass[index], dMinFactor);
                dMinFactor = fminf(dMinFactor, 1);
            }

            // Apply the dMinFactor
            pid->data[index].D = constants->k_rate_d * delta * dMinFactor;

        } else {
            pid->data[index].D = 0;
        }

        cyclic->previousDterm = dterm;

        pid->data[index].Sum = 
            computeFeedforwardAndSum(pid, constants, true,
                pidSetpointDelta, setpointCorrection, feedforwardMaxRate,
                currentPidSetpoint, index);

    } // computeCyclic

    static void computeYaw(
            anglePid_t * pid,
            anglePidConstants_t * constants,
            float pidSetpoint,
            float gyroRate,
            float dynCi)
    {
        float currentPidSetpoint = pidSetpoint;

        // -----calculate error rate
        float errorRate = currentPidSetpoint - gyroRate; // r - y
        const float previousIterm = pid->data[2].I;
        float itermErrorRate = errorRate;
        float uncorrectedSetpoint = currentPidSetpoint;

        errorRate = currentPidSetpoint - gyroRate;
        float setpointCorrection = currentPidSetpoint - uncorrectedSetpoint;

        // -----calculate P component
        pid->data[2].P = constants->k_rate_p * errorRate;

        // Apply LPF
        filterApplyFnPtr ptermLowpass= ((filterApplyFnPtr)pt1FilterApply);
        pid->data[2].P = ptermLowpass(
                (filter_t *)&pid->ptermYawLowpass, pid->data[2].P);

        // -----calculate I component
        // if launch control is active override the iterm gains and apply
        // iterm windup protection to all axes
        float Ki = constants->k_rate_i * ((!USE_INTEGRATED_YAW) ?  2.5 : 1);
        computeItermAxis(pid, previousIterm, Ki, dynCi, itermErrorRate, 2);

        float pidSetpointDelta = 0;
        float feedforwardMaxRate = rxApplyRates(1, 1);

        // D component is zero for yaw
        pid->data[2].D = 0;

        float pidSum =
            computeFeedforwardAndSum(pid, constants, false,
                pidSetpointDelta, setpointCorrection, feedforwardMaxRate,
                currentPidSetpoint, 2);

        if (USE_INTEGRATED_YAW) {
            pid->data[2].Sum += pidSum * CORE_DT() * 100;
            pid->data[2].Sum -= pid->data[2].Sum *
                INTEGRATED_YAW_RELAX / 100000 * CORE_DT() / 0.000125f;
        } else {
            pid->data[2].Sum = pidSum;
        }    

    } // computeYaw

    static void initAxis(anglePid_t * pid, uint8_t index) 
    {
        pt1FilterInitAxis(pid, index);
        pt2FilterInitAxis(pid, index);
    }

    static void initCyclicAxis(anglePid_t * pid, cyclicPid_t * cyclic,
            uint8_t index) 
    {
        initAxis(pid, index);
        pt1FilterInitWindupAxis(cyclic);
    }

    // =========================================================================

    static void anglePidInit(anglePid_t * pid, anglePidConstants_t * constants)
    {
        // set constants
        memcpy(&pid->constants, constants, sizeof(anglePidConstants_t));

        // to allow an initial zero throttle to set the filter cutoff
        pid->dynLpfPreviousQuantizedThrottle = -1;  

        pt1FilterInit(&pid->ptermYawLowpass,
                pt1FilterGain(YAW_LOWPASS_HZ, CORE_DT()));

        initCyclicAxis(pid, &pid->x, 0);
        initCyclicAxis(pid, &pid->y, 1);
        initAxis(pid, 2);
    }

    static void anglePidUpdate(
            uint32_t currentTimeUs,
            demands_t * demands,
            void * data,
            vehicleState_t * vstate,
            bool reset
            )
    {
        anglePid_t * pid = (anglePid_t *)data;
        anglePidConstants_t * constants = &pid->constants;

        // gradually scale back integration when above windup point
        float dynCi = CORE_DT();
        const float itermWindupPointInv =
            1 / (1 - (ITERM_WINDUP_POINT_PERCENT / 100));
        if (itermWindupPointInv > 1) {
            dynCi *= constrain_f(itermWindupPointInv, 0, 1);
        }

        axes_t rpy = demands->rpy;

        computeCyclic(pid, &pid->x,
                constants, rpy.x, vstate->phi, vstate->dphi, 0);

        computeCyclic(pid, &pid->y,
                constants, rpy.y, vstate->theta, vstate->dtheta, 1);

        computeYaw(pid, constants, rpy.z, vstate->dpsi, dynCi);

        // Disable PID control if at zero throttle or if gyro overflow detected
        // This may look very innefficient, but it is done on purpose to always
        // show real CPU usage as in flight
        if (reset) {
            resetItermAxis(pid, 0);
            resetItermAxis(pid, 1);
            resetItermAxis(pid, 2);
        }

        demands->rpy.x = pid->data[0].Sum;
        demands->rpy.y = pid->data[1].Sum;
        demands->rpy.z = pid->data[2].Sum;

        updateDynLpfCutoffs(pid, currentTimeUs, demands->throttle);
    }

#if defined(__cplusplus)
}
#endif
