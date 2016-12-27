/*
   stabilize_luxfloat.cpp : LuxFloat PID-based stability class implementation

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

#include <strings.h>

#define PID_DTERM_LPF_HZ     70
#define PID_H_SENSITIVITY    100
#define PID_A_LEVEL          3.0f
#define PID_H_LEVEL          3.0f
#define PID_YAW_PTERM_CUT_HZ 30

#define MAX_ANGLE_INCLINATION 700

#define ANGLE_MODE           false
#define HORIZON_MODE         true

#define ROLL              0
#define PITCH             1
#define YAW               2
#define THROTTLE          3

// XXX These should eventually go in class declaration
static const uint8_t  ESCWriteDenominator = 1; // ESC Write at 1khz
static const uint16_t gyroSamplePeriod = 125; // XXX estimated
static const uint32_t targetESCwritetime = gyroSamplePeriod*ESCWriteDenominator;
static const float    dT = (float)targetESCwritetime * 0.000001f;
static const float    KD_ATTENUATION_BREAK = 0.25f;
static const float    PID_P_f[3] = {5.0f, 6.5f, 9.3f}; 
static const float    PID_I_f[3] = {1.0f, 1.5f, 1.75f};
static const float    PID_D_f[3] = {0.11f, 0.14f, 0.0f};
static const uint8_t  PID_WEIGHT[3] = {2, 2, 2};
static const uint8_t  PID_CONTROL_RATES[3] = {90, 90, 90};
static const uint8_t  PID_ANGLE_TRIMS_RAW[3] = {0, 0, 0};
 
void StabilizeLuxFloat::init(class RC * _rc, class IMU * _imu)
{
    Stabilize::init(_rc, _imu);

    deltaStateIsSet = false;
    fullKiLatched = false;
    bzero(&yawPTermState, sizeof(filterStatePt1_t));
    for (int axis=0; axis<3; ++axis) {
        bzero(&deltaBiQuadState[axis], sizeof(biquad_t));
        errorGyroI[axis] = 0;
        errorGyroIf[axis] = 0;
        lastError[axis] = 0;
    }
}

void StabilizeLuxFloat::update(bool armed)
{
    //debug("%d\n", (int)(1000*dT));

    float throttleP = constrain( ((float)rc->command[THROTTLE] - rc->minrc) / 
            (rc->maxrc - rc->minrc), 0, 100);

    if ((throttleP > 0.1f) && armed) {
    	fullKiLatched = true;
    }

    if (!deltaStateIsSet && PID_DTERM_LPF_HZ) {
    	for (int axis = 0; axis < 3; axis++) BiQuadNewLpf(PID_DTERM_LPF_HZ, &deltaBiQuadState[axis], 0);
        deltaStateIsSet = true;
    }

    // Figure out the raw stick positions
    const int32_t stickPosAil = abs(getRcStickDeflection(rc->data, ROLL, rc->midrc));
    const int32_t stickPosEle = abs(getRcStickDeflection(rc->data, PITCH, rc->midrc));
    const int32_t mostDeflectedPos = max(stickPosAil, stickPosEle);

    // Progressively turn off the horizon self level strength as the stick is banged over
    float horizonLevelStrength = (float)(500 - mostDeflectedPos) / 500;  // 1 at centre stick, 0 = max stick deflection
    horizonLevelStrength = (PID_H_SENSITIVITY == 0) ? 
        0 :
        constrain(((horizonLevelStrength - 1) * (100 / PID_H_SENSITIVITY)) + 1, 0, 1);

    // ----------PID controller----------
    for (int axis = 0; axis < 3; axis++) {

        // -----Get the desired angle rate depending on flight mode
        uint8_t rate = PID_CONTROL_RATES[axis];

        float AngleRate = 0;
        
        if (axis == YAW) {
            // YAW is always gyro-controlled (MAG correction is applied to RC command) 100dps to 1100dps max yaw rate
            AngleRate = (float)((rate + 10) * rc->command[YAW]) / 50.0f;
         } else {
        	 int16_t factor = rc->command[axis]; // 200dps to 1200dps max roll/pitch rate
    		 AngleRate = (float)((rate + 20) * factor) / 50.0f; // 200dps to 1200dps max roll/pitch rate

             // calculate error angle and limit the angle to the max inclination
             const float errorAngle = (constrain(rc->command[axis], -((int) MAX_ANGLE_INCLINATION),
                         +MAX_ANGLE_INCLINATION) - imu->angle[axis] + PID_ANGLE_TRIMS_RAW[axis]) / 10.0f; // 16 bits is ok here
             AngleRate += errorAngle * PID_H_LEVEL * horizonLevelStrength;
         }

        float gyroRate = imu->gyroADC[axis] * imu->gyroScaleDeg; // gyro output scaled to dps

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rc->command corresponds to changing the sticks scaling here
        float RateError = AngleRate - gyroRate;

        // -----calculate P component
        float PTerm = RateError * (PID_P_f[axis]/4) * PID_WEIGHT[axis] / 100;

        if (!fullKiLatched) { PTerm = PTerm / 2; }

        if (axis == YAW && PID_YAW_PTERM_CUT_HZ) {
            PTerm = filterApplyPt1(PTerm, &yawPTermState, PID_YAW_PTERM_CUT_HZ, dT);
        }

        // -----calculate I component.
        if (fullKiLatched) {
            errorGyroIf[axis] = constrain(errorGyroIf[axis] + RateError * dT * (PID_I_f[axis]/2)  * 10, -250.0f, 250.0f);
        } else {
            errorGyroIf[axis] = constrain(errorGyroIf[axis] + RateError * dT * (PID_I_f[axis]/2)  * 10, -20.0f, 20.0f);
        }

        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        // I coefficient (I8) moved before integration to make limiting independent from PID settings

        float ITerm = errorGyroIf[axis];

        //-----calculate D-term
        float delta = RateError - lastError[axis];
        lastError[axis] = RateError;

        if (deltaStateIsSet) {
        	delta = applyBiQuadFilter(delta, &deltaBiQuadState[axis]);
        }

        // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta *= (1.0f / dT);

        float D_f = PID_D_f[axis];
        if (throttleP < KD_ATTENUATION_BREAK) {
        	float Kd_attenuation = constrain((throttleP / KD_ATTENUATION_BREAK) + 0.50f, 0, 1);
        	D_f = Kd_attenuation * D_f;
        }
        float DTerm = constrain(delta * (D_f/10) * PID_WEIGHT[axis] / 100, -300.0f, 300.0f);

        // -----calculate total PID output
        axisPID[axis] = constrain(lrintf(PTerm + ITerm + DTerm), -1000, 1000);
    }
}

void StabilizeLuxFloat::resetIntegral(void)
{
    errorGyroI[ROLL] = 0;
    errorGyroI[PITCH] = 0;
    errorGyroI[YAW] = 0;

    errorGyroIf[ROLL] = 0.0f;
    errorGyroIf[PITCH] = 0.0f;
    errorGyroIf[YAW] = 0.0f;
}

int32_t StabilizeLuxFloat::getRcStickDeflection(int16_t * rcData, int32_t axis, uint16_t midrc) 
{
    return min(abs(rcData[axis] - midrc), 500);
}


#ifdef __arm__
} // extern "C"
#endif
