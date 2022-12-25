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

#include "datatypes.h"
#include "debug.h"
#include "core_dt.h"
#include "../filters/pt1.h"
#include "../filters/pt3.h"

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


static float FREQUENCY() {return 1.0f / CORE_DT(); }

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

    static float MAX_VELOCITY_CYCLIC() 
    {
        return RATE_ACCEL_LIMIT * 100 * CORE_DT();
    }

    static float MAX_VELOCITY_YAW() 
    {
        return YAW_RATE_ACCEL_LIMIT * 100 * CORE_DT(); 
    }

    static float pt2FilterApply(pt2Filter_t *filter, float input)
    {
        filter->state1 = filter->state1 + filter->k * (input - filter->state1);
        filter->state =
            filter->state + filter->k * (filter->state1 - filter->state);
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
            anglePidConstants_t * constants,
            float value,
            float currentPidSetpoint,
            float maxRateLimit) {

        if (value * currentPidSetpoint > 0.0f) {
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

    static float accelerationLimit(anglePid_t * pid, uint8_t axis,
            float currentPidSetpoint)
    {
        const float currentVelocity =
            currentPidSetpoint - pid->previousSetpoint[axis];

        float maxVelocity =
            axis == 2 ? MAX_VELOCITY_YAW() : MAX_VELOCITY_CYCLIC();

        if (fabsf(currentVelocity) > maxVelocity) {
            currentPidSetpoint = (currentVelocity > 0) ?
                pid->previousSetpoint[axis] + maxVelocity :
                pid->previousSetpoint[axis] - maxVelocity;
        }

        pid->previousSetpoint[axis] = currentPidSetpoint;
        return currentPidSetpoint;
    }

    static void applyItermRelax(
            anglePid_t * pid,
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
            anglePid_t * pid, int axis, float pidSetpointDelta)
    {
        float ret = pidSetpointDelta;
        if (pid->feedforwardLpfInitialized) {
            ret = pt3FilterApply(&pid->feedforwardPt3[axis], pidSetpointDelta);
        }
        return ret;
    }

    static float dynLpfCutoffFreq(
            float throttle,
            uint16_t dynLpfMin,
            uint16_t dynLpfMax,
            uint8_t expo) {
        const float expof = expo / 10.0f;
        static float curve;
        curve = throttle * (1 - throttle) * expof + throttle;
        return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
    }

    static void pidDynLpfDTermUpdate(anglePid_t * pid, float throttle)
    {
        const uint16_t dyn_lpf_min = DTERM_LPF1_DYN_MIN_HZ;
        const uint16_t dyn_lpf_max = DTERM_LPF1_DYN_MAX_HZ;
        float cutoffFreq =
            dynLpfCutoffFreq(throttle, dyn_lpf_min, dyn_lpf_max,
                    DYN_LPF_CURVE_EXPO);

        for (uint8_t axis = 0; axis < 3; axis++) {
            pid->dtermLowpass[axis].pt1Filter.k =
                pt1FilterGain(cutoffFreq, CORE_DT());

        }
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

    // =========================================================================

    static void anglePidInit(anglePid_t * pid, anglePidConstants_t * constants)
    {
        // set constants
        memcpy(&pid->constants, constants, sizeof(anglePidConstants_t));

        // to allow an initial zero throttle to set the filter cutoff
        pid->dynLpfPreviousQuantizedThrottle = -1;  

        // 1st Dterm Lowpass Filter
        uint16_t dterm_lpf1_init_hz = DTERM_LPF1_DYN_MIN_HZ;

        dterm_lpf1_init_hz = DTERM_LPF1_DYN_MIN_HZ;

        for (uint8_t axis = 0; axis <= 2; axis++) {
            pt1FilterInit(&pid->dtermLowpass[axis].pt1Filter,
                    pt1FilterGain(dterm_lpf1_init_hz, CORE_DT()));
        }

        // 2nd Dterm Lowpass Filter
        for (uint8_t axis = 0; axis <= 2; axis++) {
            pt1FilterInit(&pid->dtermLowpass2[axis].pt1Filter,
                    pt1FilterGain(DTERM_LPF2_HZ, CORE_DT()));
        }

        pt1FilterInit(&pid->ptermYawLowpass,
                pt1FilterGain(YAW_LOWPASS_HZ, CORE_DT()));

        for (int i = 0; i < 3; i++) {
            pt1FilterInit(&pid->windupLpf[i],
                    pt1FilterGain(ITERM_RELAX_CUTOFF, CORE_DT()));
        }

        // Initialize the filters for all axis even if the d_min[axis] value is
        // 0 Otherwise if the pidProfile.d_min_xxx parameters are ever added to
        // in-flight adjustments and transition from 0 to > 0 in flight the
        // feature won't work because the filter wasn't initialized.
        for (uint8_t axis = 0; axis <= 2; axis++) {
            pt2FilterInit(&pid->dMinRange[axis],
                    pt2FilterGain(D_MIN_RANGE_HZ, CORE_DT()));
            pt2FilterInit(&pid->dMinLowpass[axis],
                    pt2FilterGain(D_MIN_LOWPASS_HZ, CORE_DT()));
        }
    }

    static void anglePidUpdate(
            uint32_t currentTimeUs,
            demands_t * demands,
            void * data,
            vehicle_state_t * vstate,
            bool reset
            )
    {
    }

#if defined(__cplusplus)
}
#endif
