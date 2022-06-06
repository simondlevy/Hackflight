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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "accel.h"
#include "arming.h"
#include "const.h"
#include "datatypes.h"
#include "debug.h"
#include "dt.h"
#include "filter.h"
#include "filter_impl.h"
#include "filter_pt1.h"
#include "filter_pt3.h"
#include "gyro.h"
#include "maths.h"
#include "angle_struct.h"
#include "rx_rates.h"
#include "sensors.h"
#include "time.h"

// minimum of 5ms between updates
static const uint16_t DYN_LPF_THROTTLE_UPDATE_DELAY_US = 5000; 

static const uint16_t DYN_LPF_THROTTLE_STEPS = 100;

// The constant scale factor to replace the Kd component of the feedforward calculation.
// This value gives the same "feel" as the previous Kd default of 26 (26 * DTERM_SCALE)
static const float FEEDFORWARD_SCALE = 0.013754;

// Full iterm suppression in setpoint mode at high-passed setpoint rate > 40deg/sec
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


static CONST float FREQUENCY() {return 1.0f / DT(); }

// Scale factors to make best use of range with D_LPF debugging, aiming for max
// +/-16K as debug values are 16 bit
static const float D_LPF_FILT_SCALE = 22;

// PT2 lowpass input cutoff to peak D around propwash frequencies
static const float D_MIN_RANGE_HZ   = 85;  

// PT2 lowpass cutoff to smooth the boost effect
static const float D_MIN_LOWPASS_HZ = 35;  

static const float D_MIN_GAIN_FACTOR          = 0.00008;
static const float D_MIN_SETPOINT_GAIN_FACTOR = 0.00008f;

static const uint16_t RATE_ACCEL_LIMIT = 0;
static const uint16_t YAW_RATE_ACCEL_LIMIT = 0;
static const uint16_t ITERM_LIMIT = 400;

static const float LEVEL_ANGLE_LIMIT = 45;

#if defined(__cplusplus)
extern "C" {
#endif

    static CONST float MAX_VELOCITY_CYCLIC() { return RATE_ACCEL_LIMIT * 100 * DT(); }
    static CONST float MAX_VELOCITY_YAW()    { return YAW_RATE_ACCEL_LIMIT * 100 * DT(); }

    static float pt2FilterApply(pt2Filter_t *filter, float input)
    {
        filter->state1 = filter->state1 + filter->k * (input - filter->state1);
        filter->state = filter->state + filter->k * (filter->state1 - filter->state);
        return filter->state;
    }

    static float pt2FilterGain(float f_cut, float dT)
    {
        const float order = 2.0f;
        const float orderCutoffCorrection = 1 / sqrtf(powf(2, 1.0f / order) - 1);
        float RC = 1 / (2 * orderCutoffCorrection * M_PI * f_cut);
        // float RC = 1 / (2 * 1.553773974f * M_PI * f_cut);
        // where 1.553773974 = 1 / sqrt( (2^(1 / order) - 1) ) and order is 2
        return dT / (RC + dT);
    }

    static void pt2FilterInit(pt2Filter_t *filter, float k)
    {
        filter->state = 0.0f;
        filter->state1 = 0.0f;
        filter->k = k;
    }

    static float applyFeedforwardLimit(
            angle_pid_t * pid,
            float value,
            float currentPidSetpoint,
            float maxRateLimit) {

        if (value * currentPidSetpoint > 0.0f) {
            if (fabsf(currentPidSetpoint) <= maxRateLimit) {
                value = constrainf(value, (-maxRateLimit -
                            currentPidSetpoint) * pid->k_rate_p,
                        (maxRateLimit - currentPidSetpoint) * pid->k_rate_p);
            } else {
                value = 0;
            }
        }

        return value;
    }

    static float accelerationLimit(angle_pid_t * pid, uint8_t axis,
            float currentPidSetpoint)
    {
        const float currentVelocity = currentPidSetpoint - pid->previousSetpoint[axis];

        float maxVelocity = axis == 2 ? MAX_VELOCITY_YAW() : MAX_VELOCITY_CYCLIC();

        if (fabsf(currentVelocity) > maxVelocity) {
            currentPidSetpoint = (currentVelocity > 0) ?
                pid->previousSetpoint[axis] + maxVelocity :
                pid->previousSetpoint[axis] - maxVelocity;
        }

        pid->previousSetpoint[axis] = currentPidSetpoint;
        return currentPidSetpoint;
    }

    static void applyItermRelax(
            angle_pid_t * pid,
            const int axis,
            const float iterm,
            float *itermErrorRate,
            float *currentPidSetpoint)
    {
        const float setpointLpf =
            pt1FilterApply(&pid->windupLpf[axis], *currentPidSetpoint);

        const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);

        if (axis < 2) {

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
    }

    static float applyRcSmoothingFeedforwardFilter(
            angle_pid_t * pid, int axis, float pidSetpointDelta)
    {
        float ret = pidSetpointDelta;
        if (pid->feedforwardLpfInitialized) {
            ret = pt3FilterApply(&pid->feedforwardPt3[axis], pidSetpointDelta);
        }
        return ret;
    }

    static float dynLpfCutoffFreq(
            float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo) {
        const float expof = expo / 10.0f;
        static float curve;
        curve = throttle * (1 - throttle) * expof + throttle;
        return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
    }

    static void pidDynLpfDTermUpdate(angle_pid_t * pid, float throttle)
    {
        const uint16_t dyn_lpf_min = DTERM_LPF1_DYN_MIN_HZ;
        const uint16_t dyn_lpf_max = DTERM_LPF1_DYN_MAX_HZ;
        float cutoffFreq =
            dynLpfCutoffFreq(throttle, dyn_lpf_min, dyn_lpf_max, DYN_LPF_CURVE_EXPO);

        for (uint8_t axis = 0; axis < 3; axis++) {
            pid->dtermLowpass[axis].pt1Filter.k = pt1FilterGain(cutoffFreq, DT());

        }
    }

    static void updateDynLpfCutoffs(
            angle_pid_t * pid,
            timeUs_t currentTimeUs,
            float throttle)
    {
        if (cmpTimeUs(currentTimeUs, pid->lastDynLpfUpdateUs) >=
                DYN_LPF_THROTTLE_UPDATE_DELAY_US) {

            // quantize the throttle reduce the number of filter updates
            int32_t quantizedThrottle = lrintf(throttle * DYN_LPF_THROTTLE_STEPS); 

            if (quantizedThrottle != pid->dynLpfPreviousQuantizedThrottle) {

                // scale the quantized value back to the throttle range so the
                // filter cutoff steps are repeatable
                float dynLpfThrottle = (float)quantizedThrottle / DYN_LPF_THROTTLE_STEPS;
                pidDynLpfDTermUpdate(pid, dynLpfThrottle);
                pid->dynLpfPreviousQuantizedThrottle = quantizedThrottle;
                pid->lastDynLpfUpdateUs = currentTimeUs;
            }
        }
    }

    static float levelPid(angle_pid_t * pid, float currentSetpoint, float currentAngle)
    {
        // calculate error angle and limit the angle to the max inclination
        // rcDeflection in [-1.0, 1.0]
        float angle = LEVEL_ANGLE_LIMIT * currentSetpoint;
        angle = constrainf(angle, -LEVEL_ANGLE_LIMIT, LEVEL_ANGLE_LIMIT);
        float errorAngle = angle - (currentAngle / 10);
        return pid->k_level_p > 0 ? errorAngle * pid->k_level_p : currentSetpoint;
    }

    // ==================================================================================

    static void anglePidInit(
            angle_pid_t * pid,
            float rate_p,
            float rate_i,
            float rate_d,
            float rate_f,
            float level_p
            )
    {
        // set constants
        pid->k_rate_p = rate_p;
        pid->k_rate_i = rate_i;
        pid->k_rate_d = rate_d;
        pid->k_rate_f = rate_f;
        pid->k_level_p = level_p;

        // to allow an initial zero throttle to set the filter cutoff
        pid->dynLpfPreviousQuantizedThrottle = -1;  

        // 1st Dterm Lowpass Filter
        uint16_t dterm_lpf1_init_hz = DTERM_LPF1_DYN_MIN_HZ;

        dterm_lpf1_init_hz = DTERM_LPF1_DYN_MIN_HZ;

        for (uint8_t axis = 0; axis <= 2; axis++) {
            pt1FilterInit(&pid->dtermLowpass[axis].pt1Filter,
                    pt1FilterGain(dterm_lpf1_init_hz, DT()));
        }

        // 2nd Dterm Lowpass Filter
        for (uint8_t axis = 0; axis <= 2; axis++) {
            pt1FilterInit(&pid->dtermLowpass2[axis].pt1Filter,
                    pt1FilterGain(DTERM_LPF2_HZ, DT()));
        }

        pt1FilterInit(&pid->ptermYawLowpass, pt1FilterGain(YAW_LOWPASS_HZ, DT()));

        for (int i = 0; i < 3; i++) {
            pt1FilterInit(&pid->windupLpf[i], pt1FilterGain(ITERM_RELAX_CUTOFF, DT()));
        }

        // Initialize the filters for all axis even if the d_min[axis] value is 0
        // Otherwise if the pidProfile.d_min_xxx parameters are ever added to
        // in-flight adjustments and transition from 0 to > 0 in flight the feature
        // won't work because the filter wasn't initialized.
        for (uint8_t axis = 0; axis <= 2; axis++) {
            pt2FilterInit(&pid->dMinRange[axis], pt2FilterGain(D_MIN_RANGE_HZ, DT()));
            pt2FilterInit(&pid->dMinLowpass[axis],
                    pt2FilterGain(D_MIN_LOWPASS_HZ, DT()));
        }
    }

    static void anglePidUpdate(
            void * ignore,
            uint32_t currentTimeUs,
            demands_t * demands,
            void * data,
            vehicle_state_t * vstate,
            bool reset
            )
    {
        (void)ignore;

        angle_pid_t * pid = (angle_pid_t *)data;

        // gradually scale back integration when above windup point
        float dynCi = DT();
        const float itermWindupPointInv = 1 / (1 - (ITERM_WINDUP_POINT_PERCENT / 100));
        if (itermWindupPointInv > 1.0f) {
            dynCi *= constrainf(itermWindupPointInv, 0.0f, 1.0f);
        }

        float gyroRates[3] = {vstate->dphi, vstate->dtheta, vstate->dpsi};

        // Precalculate gyro deta for D-term here, this allows loop unrolling
        float gyroRateDterm[3];
        for (uint8_t axis = 0; axis <= 2; ++axis) {

            gyroRateDterm[axis] = gyroRates[axis];

            filterApplyFnPtr dtermLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
            gyroRateDterm[axis] =
                dtermLowpassApplyFn((filter_t *) &pid->dtermLowpass[axis],
                        gyroRateDterm[axis]);

            filterApplyFnPtr dtermLowpass2ApplyFn = (filterApplyFnPtr)pt1FilterApply;
            gyroRateDterm[axis] =
                dtermLowpass2ApplyFn((filter_t *) &pid->dtermLowpass2[axis],
                        gyroRateDterm[axis]);
        }

        float pidSetpoints[3] = { demands->roll, demands->pitch, demands->yaw };
        float currentAngles[3] = { vstate->phi, vstate->theta, vstate->psi };

        // ----------PID controller----------
        for (uint8_t axis = 0; axis <= 2; ++axis) {

            float currentPidSetpoint = pidSetpoints[axis];

            float maxVelocity = axis == 2 ? MAX_VELOCITY_YAW() : MAX_VELOCITY_CYCLIC();
            if (maxVelocity) {
                currentPidSetpoint = accelerationLimit(pid, axis, currentPidSetpoint);
            }

            if (axis != 2) {
                currentPidSetpoint =
                    levelPid(pid, currentPidSetpoint, currentAngles[axis]);
            }

            // Handle yaw spin recovery - zero the setpoint on yaw to aid in
            // recovery It's not necessary to zero the set points for R/P because
            // the PIDs will be zeroed below

            // -----calculate error rate
            const float gyroRate = gyroRates[axis]; // gyro output in deg/sec
            float errorRate = currentPidSetpoint - gyroRate; // r - y
            const float previousIterm = pid->data[axis].I;
            float itermErrorRate = errorRate;
            float uncorrectedSetpoint = currentPidSetpoint;

            applyItermRelax(pid, axis, previousIterm, &itermErrorRate,
                    &currentPidSetpoint);
            errorRate = currentPidSetpoint - gyroRate;
            float setpointCorrection = currentPidSetpoint - uncorrectedSetpoint;

            // --------low-level gyro-based PID based on 2DOF PID controller.
            // ---------- 2-DOF PID controller with optional filter on derivative
            // term.  b = 1 and only c (feedforward weight) can be tuned (amount
            // derivative on measurement or error).

            // -----calculate P component
            filterApplyFnPtr ptermYawLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
            pid->data[axis].P = pid->k_rate_p * errorRate;
            if (axis == 2) {
                pid->data[axis].P = ptermYawLowpassApplyFn((filter_t *)
                        &pid->ptermYawLowpass, pid->data[axis].P);
            }

            // -----calculate I component
            // if launch control is active override the iterm gains and apply iterm
            // windup protection to all axes
            float Ki = pid->k_rate_i * ((axis == 2 && !USE_INTEGRATED_YAW) ? 2.5 : 1);
            float axisDynCi = (axis == 2) ? dynCi : DT(); // check windup for yaw only

            pid->data[axis].I =
                constrainf(previousIterm + (Ki * axisDynCi) * itermErrorRate,
                        -ITERM_LIMIT, ITERM_LIMIT);

            // -----calculate pidSetpointDelta
            float pidSetpointDelta = 0;
            float feedforwardMaxRate = rxApplyRates(1, 1);

            // -----calculate D component
            if ((axis < 2 && pid->k_rate_d > 0)) {

                // Divide rate change by dT to get differential (ie dr/dt).
                // dT is fixed and calculated from the target PID loop time
                // This is done to avoid DTerm spikes that occur with dynamically
                // calculated deltaT whenever another task causes the PID
                // loop execution to be delayed.
                const float delta = -(gyroRateDterm[axis] -
                        pid->previousGyroRateDterm[axis]) * FREQUENCY();

                float preTpaD = pid->k_rate_d * delta;

                float dMinFactor = 1.0f;

                float dMinPercent = axis == 2 ?
                    0 :
                    D_MIN > 0 && D_MIN < pid->k_rate_d ?
                    D_MIN / pid->k_rate_d :
                    0;

                if (dMinPercent > 0) {
                    const float d_min_gyro_gain =
                        D_MIN_GAIN * D_MIN_GAIN_FACTOR / D_MIN_LOWPASS_HZ;
                    float dMinGyroFactor = pt2FilterApply(&pid->dMinRange[axis], delta);
                    dMinGyroFactor = fabsf(dMinGyroFactor) * d_min_gyro_gain;
                    const float d_min_setpoint_gain =
                        D_MIN_GAIN * D_MIN_SETPOINT_GAIN_FACTOR *
                        D_MIN_ADVANCE * FREQUENCY() / (100 * D_MIN_LOWPASS_HZ);
                    const float dMinSetpointFactor =
                        (fabsf(pidSetpointDelta)) * d_min_setpoint_gain;
                    dMinFactor = fmaxf(dMinGyroFactor, dMinSetpointFactor);
                    dMinFactor = dMinPercent + (1.0f - dMinPercent) * dMinFactor;
                    dMinFactor = pt2FilterApply(&pid->dMinLowpass[axis], dMinFactor);
                    dMinFactor = fminf(dMinFactor, 1.0f);
                }

                // Apply the dMinFactor
                preTpaD *= dMinFactor;
                pid->data[axis].D = preTpaD;

                // Log the value of D pre application of TPA
                preTpaD *= D_LPF_FILT_SCALE;

            } else {
                pid->data[axis].D = 0;
            }

            pid->previousGyroRateDterm[axis] = gyroRateDterm[axis];

            // -----calculate feedforward component
            // include abs control correction in feedforward
            pidSetpointDelta += setpointCorrection -
                pid->previousSetpointCorrection[axis];
            pid->previousSetpointCorrection[axis] = setpointCorrection;

            // no feedforward in launch control
            float feedforwardGain = pid->k_rate_f;
            if (feedforwardGain > 0) {
                // halve feedforward in Level mode since stick sensitivity is
                // weaker by about half transition now calculated in feedforward.c
                // when new RC data arrives 
                float feedForward = feedforwardGain * pidSetpointDelta * FREQUENCY();

                float feedforwardMaxRateLimit =
                    feedforwardMaxRate * FEEDFORWARD_MAX_RATE_LIMIT * 0.01f;

                bool shouldApplyFeedforwardLimits =
                    feedforwardMaxRateLimit != 0.0f && axis < 2;

                pid->data[axis].F = shouldApplyFeedforwardLimits ?
                    applyFeedforwardLimit(pid, feedForward, currentPidSetpoint,
                            feedforwardMaxRateLimit) :
                    feedForward;
                pid->data[axis].F =
                    applyRcSmoothingFeedforwardFilter(pid, axis, pid->data[axis].F);
            } else {
                pid->data[axis].F = 0;
            }

            // calculating the PID sum
            const float pidSum =
                pid->data[axis].P +
                pid->data[axis].I +
                pid->data[axis].D +
                pid->data[axis].F;
            if (axis == 2 && USE_INTEGRATED_YAW) {
                pid->data[axis].Sum += pidSum * DT() * 100.0f;
                pid->data[axis].Sum -= pid->data[axis].Sum *
                    INTEGRATED_YAW_RELAX / 100000.0f * DT() / 0.000125f;
            } else {
                pid->data[axis].Sum = pidSum;
            }
        }

        // Disable PID control if at zero throttle or if gyro overflow detected
        // This may look very innefficient, but it is done on purpose to always
        // show real CPU usage as in flight
        if (reset) {
            for (uint8_t axis = 0; axis < 3; axis++) {
                pid->data[axis].I = 0.0f;
            }
        }

        demands->roll  = pid->data[0].Sum;
        demands->pitch = pid->data[1].Sum;
        demands->yaw   = pid->data[2].Sum;

        updateDynLpfCutoffs(pid, currentTimeUs, demands->throttle);
    }

#if defined(__cplusplus)
}
#endif
