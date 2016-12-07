/*
   stabilize.cpp : PID-based stability class implementation

   Adapted from 

     https://github.com/multiwii/baseflight/blob/master/src/mw.c

     https://github.com/cleanflight/cleanflight/blob/master/src/main/flight/pid_luxfloat.c

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef __arm__
extern "C" {
#else
#include <stdio.h>
#endif

#include "hackflight.hpp"
#include "pidvals.hpp"

/*
extern uint8_t PIDweight[3];
extern float lastITermf[3], ITermLimitf[3];

extern pt1Filter_t deltaFilter[3];
extern pt1Filter_t yawFilter;

extern biquadFilter_t dtermFilterNotch[3];
extern biquadFilter_t dtermFilterLpf[3];

// constants to scale pidLuxFloat so output is same as pidMultiWiiRewrite
static const float luxPTermScale = 1.0f / 128;
static const float luxITermScale = 1000000.0f / 0x1000000;
static const float luxDTermScale = (0.000001f * (float)0xFFFF) / 512;
static const float luxGyroScale = 16.4f / 4; // the 16.4 is needed because mwrewrite does not scale according to the gyro model gyro.scale

static int16_t pidLuxFloatCore(int axis, const pidProfile_t *pidProfile, float gyroRate, float angleRate)
{
    static float lastRateForDelta[3];

    const float rateError = angleRate - gyroRate;

    // -----calculate P component
    float PTerm = luxPTermScale * rateError * pidProfile->P8[axis] * PIDweight[axis] / 100;
    // Constrain YAW by yaw_p_limit value if not servo driven, in that case servolimits apply
    if (axis == YAW) {
        if (pidProfile->yaw_lpf_hz) {
            PTerm = pt1FilterApply4(&yawFilter, PTerm, pidProfile->yaw_lpf_hz, getdT());
        }
        if (pidProfile->yaw_p_limit) {
            PTerm = constrainf(PTerm, -pidProfile->yaw_p_limit, pidProfile->yaw_p_limit);
        }
    }

    // -----calculate I component
    float ITerm = lastITermf[axis] + luxITermScale * rateError * getdT() * pidProfile->I8[axis];
    // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
    // I coefficient (I8) moved before integration to make limiting independent from PID settings
    ITerm = constrainf(ITerm, -PID_MAX_I, PID_MAX_I);
    // Anti windup protection
    if (rcModeIsActive(BOXAIRMODE)) {
        if (STATE(ANTI_WINDUP) || motorLimitReached) {
            ITerm = constrainf(ITerm, -ITermLimitf[axis], ITermLimitf[axis]);
        } else {
            ITermLimitf[axis] = ABS(ITerm);
        }
    }
    lastITermf[axis] = ITerm;

    // -----calculate D component
    float DTerm;
    if (pidProfile->D8[axis] == 0) {
        // optimisation for when D8 is zero, often used by YAW axis
        DTerm = 0;
    } else {
        float delta;
        if (pidProfile->deltaMethod == PID_DELTA_FROM_MEASUREMENT) {
            delta = -(gyroRate - lastRateForDelta[axis]);
            lastRateForDelta[axis] = gyroRate;
        } else {
            delta = rateError - lastRateForDelta[axis];
            lastRateForDelta[axis] = rateError;
        }
        // Divide delta by targetLooptime to get differential (ie dr/dt)
        delta /= getdT();

        // Filter delta
        if (pidProfile->dterm_notch_hz) {
            delta = biquadFilterApply(&dtermFilterNotch[axis], delta);
        }

        if (pidProfile->dterm_lpf_hz) {
            if (pidProfile->dterm_filter_type == FILTER_BIQUAD) {
                delta = biquadFilterApply(&dtermFilterLpf[axis], delta);
            } else {
                // DTerm delta low pass filter
                delta = pt1FilterApply4(&deltaFilter[axis], delta, pidProfile->dterm_lpf_hz, getdT());
            }
        }

        DTerm = luxDTermScale * delta * pidProfile->D8[axis] * PIDweight[axis] / 100;
        DTerm = constrainf(DTerm, -PID_MAX_D, PID_MAX_D);
    }

    // -----calculate total PID output
    return lrintf(PTerm + ITerm + DTerm);
}

void pidLuxFloat(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig)
{
    float horizonLevelStrength = 0.0f;
    // (convert 0-100 range to 0.0-1.0 range)
    horizonLevelStrength = (float)calcHorizonLevelStrength(rxConfig->midrc, pidProfile->horizon_tilt_effect,
            pidProfile->horizon_tilt_mode, pidProfile->D8[PIDLEVEL]) / 100.0f;

    // ----------PID controller----------
    for (int axis = 0; axis < 3; axis++) {
        const uint8_t rate = controlRateConfig->rates[axis];

        // -----Get the desired angle rate depending on flight mode
        float angleRate;
        if (axis == FD_YAW) {
            // YAW is always gyro-controlled (MAG correction is applied to rcCommand) 100dps to 1100dps max yaw rate
            angleRate = (float)((rate + 27) * rcCommand[YAW]) / 32.0f;
        } else {
            // control is GYRO based for ACRO and HORIZON - direct sticks control is applied to rate PID
            angleRate = (float)((rate + 27) * rcCommand[axis]) / 16.0f; // 200dps to 1200dps max roll/pitch rate
            // calculate error angle and limit the angle to the max inclination
            // multiplication of rcCommand corresponds to changing the sticks scaling here
            const float errorAngle = constrain(2 * rcCommand[axis], -((int)max_angle_inclination), max_angle_inclination)
                - attitude.raw[axis] + angleTrim->raw[axis];
            // mix in errorAngle to desired angleRate to add a little auto-level feel.
            // horizonLevelStrength has been scaled to the stick input
            angleRate += errorAngle * pidProfile->I8[PIDLEVEL] * horizonLevelStrength / 16.0f;
        }

        // --------low-level gyro-based PID. ----------
        const float gyroRate = luxGyroScale * gyroADCf[axis] * gyro.scale;
        axisPID[axis] = pidLuxFloatCore(axis, pidProfile, gyroRate, angleRate);
    }
}

*/
void Stabilize::init(class RC * _rc, class IMU * _imu)
{
    this->rc = _rc;
    this->imu = _imu;

    for (uint8_t axis=0; axis<3; ++axis) {
        this->lastGyroError[axis] = 0;
        this->delta1[axis] = 0;
        this->delta2[axis] = 0;
    }

    this->rate_p[0] = CONFIG_RATE_PITCHROLL_P;
    this->rate_p[1] = CONFIG_RATE_PITCHROLL_P;
    this->rate_p[2] = CONFIG_YAW_P;

    this->rate_i[0] = CONFIG_RATE_PITCHROLL_I;
    this->rate_i[1] = CONFIG_RATE_PITCHROLL_I;
    this->rate_i[2] = CONFIG_YAW_I;

    this->rate_d[0] = CONFIG_RATE_PITCHROLL_D;
    this->rate_d[1] = CONFIG_RATE_PITCHROLL_D;
    this->rate_d[2] = 0;

    this->resetIntegral();
}

void Stabilize::update(void)
{
    for (uint8_t axis = 0; axis < 3; axis++) {

        int32_t gyroError = this->imu->gyroADC[axis] / 4;

        int32_t error = (int32_t)this->rc->command[axis] * 10 * 8 / this->rate_p[axis] - gyroError;

        int32_t PTermGYRO = this->rc->command[axis];

        this->errorGyroI[axis] = constrain(this->errorGyroI[axis] + error, -16000, +16000); // WindUp
        if ((abs(gyroError) > 640) || ((axis == AXIS_YAW) && (abs(this->rc->command[axis]) > 100)))
            this->errorGyroI[axis] = 0;
        int32_t ITermGYRO = (this->errorGyroI[axis] / 125 * this->rate_i[axis]) >> 6;

        int32_t PTerm = PTermGYRO;
        int32_t ITerm = ITermGYRO;

        if (axis < 2) {

            // 50 degrees max inclination
            int32_t errorAngle = constrain(2 * this->rc->command[axis], 
                                           -((int)CONFIG_MAX_ANGLE_INCLINATION), 
                                           + CONFIG_MAX_ANGLE_INCLINATION) 
                                 - this->imu->angle[axis];

            int32_t PTermACC = errorAngle * CONFIG_LEVEL_P / 100; 

            this->errorAngleI[axis] = constrain(this->errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp
            int32_t ITermACC = (this->errorAngleI[axis] * CONFIG_LEVEL_I) >> 12;

            int32_t prop = max(abs(this->rc->command[DEMAND_PITCH]), 
                    abs(this->rc->command[DEMAND_ROLL])); // range [0;500]

            PTerm = (PTermACC * (500 - prop) + PTermGYRO * prop) / 500;
            ITerm = (ITermACC * (500 - prop) + ITermGYRO * prop) / 500;
        } 

        PTerm -= gyroError * this->rate_p[axis] / 10 / 8; // 32 bits is needed for calculation
        int32_t delta = gyroError - this->lastGyroError[axis];
        this->lastGyroError[axis] = gyroError;
        int32_t deltaSum = this->delta1[axis] + this->delta2[axis] + delta;
        this->delta2[axis] = this->delta1[axis];
        this->delta1[axis] = delta;
        int32_t DTerm = (deltaSum * this->rate_d[axis]) / 32;
        this->axisPID[axis] = PTerm + ITerm - DTerm;
    }

    // prevent "yaw jump" during yaw correction
    this->axisPID[AXIS_YAW] = constrain(this->axisPID[AXIS_YAW], 
            -100 - abs(this->rc->command[DEMAND_YAW]), +100 + abs(this->rc->command[DEMAND_YAW]));
}

void Stabilize::resetIntegral(void)
{
    this->errorGyroI[AXIS_ROLL] = 0;
    this->errorGyroI[AXIS_PITCH] = 0;
    this->errorGyroI[AXIS_YAW] = 0;
    this->errorAngleI[AXIS_ROLL] = 0;
    this->errorAngleI[AXIS_PITCH] = 0;
}

#ifdef __arm__
} // extern "C"
#endif
