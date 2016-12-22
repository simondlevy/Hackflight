#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <common/filter.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define ABS(x) ((x) > 0 ? (x) : -(x))

#define PID_DTERM_LPF_HZ     70
#define PID_H_SENSITIVITY    100
#define PID_A_LEVEL          3.0f
#define PID_H_LEVEL          3.0f
#define PID_YAW_PTERM_CUT_HZ 30

#define MAX_ANGLE_INCLINATION 700

#define RX_MIDRC             1500
#define RX_MINCHECK          1100
#define RX_MAXCHECK          1900

#define ANGLE_MODE           false
#define HORIZON_MODE         true

#define ROLL              0
#define PITCH             1
#define YAW               2
#define THROTTLE          3

int16_t axisPID[3];

// indexed as roll / pitch / yaw
static const float PID_P_f[3] = {5.0f, 6.5f, 9.3f}; 
static const float PID_I_f[3] = {1.0f, 1.5f, 1.75f};
static const float PID_D_f[3] = {0.11f, 0.14f, 0.0f};
static const uint8_t PID_WEIGHT[3] = {100, 100, 100};
static const uint8_t PID_CONTROL_RATES[3] = {90, 90, 90};
static const uint8_t PID_ANGLE_TRIMS_RAW[3] = {0, 0, 0};
static const float KD_ATTENUATION_BREAK = 0.25f;

static bool deltaStateIsSet;
static biquad_t deltaBiQuadState[3];
static filterStatePt1_t yawPTermState;
static int32_t errorGyroI[3];
static float   errorGyroIf[3];

static int32_t getRcStickDeflection(int16_t * rcData, int32_t axis, uint16_t midrc) {
    return MIN(ABS(rcData[axis] - midrc), 500);
}

void pidResetErrorGyro(void)
{
    errorGyroI[ROLL] = 0;
    errorGyroI[PITCH] = 0;
    errorGyroI[YAW] = 0;

    errorGyroIf[ROLL] = 0.0f;
    errorGyroIf[PITCH] = 0.0f;
    errorGyroIf[YAW] = 0.0f;
}

void myPidLuxFloat(
        int16_t gyroADC[3], 
        int16_t attitudeRaw[3], 
        int16_t * rcCommand, 
        int16_t * rcData, 
        float gyroScale, 
        float dT, 
        bool armed)
{    
    static bool fullKiLatched;
    static float lastError[3];

    float throttleP = constrainf( ((float)rcCommand[THROTTLE] - RX_MINCHECK) / (RX_MAXCHECK - RX_MINCHECK), 0, 100);

    if ( (throttleP > 0.1f) && armed) {
    	fullKiLatched = true;
    }

    float RateError, AngleRate, gyroRate;
    float ITerm,PTerm,DTerm;
    float delta;
    int axis;
    float horizonLevelStrength = 1;

    if (!deltaStateIsSet && PID_DTERM_LPF_HZ) {
    	for (axis = 0; axis < 3; axis++) BiQuadNewLpf(PID_DTERM_LPF_HZ, &deltaBiQuadState[axis], 0);
        deltaStateIsSet = true;
    }

    if (HORIZON_MODE) {
        // Figure out the raw stick positions
        const int32_t stickPosAil = ABS(getRcStickDeflection(rcData, ROLL, RX_MIDRC));
        const int32_t stickPosEle = ABS(getRcStickDeflection(rcData, PITCH, RX_MIDRC));
        const int32_t mostDeflectedPos = MAX(stickPosAil, stickPosEle);
        // Progressively turn off the horizon self level strength as the stick is banged over
        horizonLevelStrength = (float)(500 - mostDeflectedPos) / 500;  // 1 at centre stick, 0 = max stick deflection
        if(PID_H_SENSITIVITY == 0){
            horizonLevelStrength = 0;
        } else {
            horizonLevelStrength = constrainf(((horizonLevelStrength - 1) * (100 / PID_H_SENSITIVITY)) + 1, 0, 1);
        }
    }

    // ----------PID controller----------
    for (axis = 0; axis < 3; axis++) {
        // -----Get the desired angle rate depending on flight mode
        uint8_t rate = PID_CONTROL_RATES[axis];

        if (axis == YAW) {
            // YAW is always gyro-controlled (MAG correction is applied to rcCommand) 100dps to 1100dps max yaw rate
            AngleRate = (float)((rate + 10) * rcCommand[YAW]) / 50.0f;
         } else {
        	 int16_t factor = rcCommand[axis]; // 200dps to 1200dps max roll/pitch rate
    		 AngleRate = (float)((rate + 20) * factor) / 50.0f; // 200dps to 1200dps max roll/pitch rate

        	 //25 wf + 650

             if (ANGLE_MODE || HORIZON_MODE) {
                // calculate error angle and limit the angle to the max inclination
                const float errorAngle = (constrain(rcCommand[axis], -((int) MAX_ANGLE_INCLINATION),
                    +MAX_ANGLE_INCLINATION) - attitudeRaw[axis] + PID_ANGLE_TRIMS_RAW[axis]) / 10.0f; // 16 bits is ok here
                if (ANGLE_MODE) {
                    // ANGLE mode - control is angle based, so control loop is needed
                    AngleRate = errorAngle * PID_A_LEVEL;
                } else {
                    // HORIZON mode - direct sticks control is applied to rate PID
                    // mix up angle error to desired AngleRate to add a little auto-level feel
                    AngleRate += errorAngle * PID_H_LEVEL * horizonLevelStrength;
                }
            }
        }

        gyroRate = gyroADC[axis] * gyroScale; // gyro output scaled to dps

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rcCommand corresponds to changing the sticks scaling here
        RateError = AngleRate - gyroRate;

        // -----calculate P component
        PTerm = RateError * (PID_P_f[axis]/4) * PID_WEIGHT[axis] / 100;
        if (!fullKiLatched) { PTerm = PTerm / 2; }

        if (axis == YAW && PID_YAW_PTERM_CUT_HZ) {
            PTerm = filterApplyPt1(PTerm, &yawPTermState, PID_YAW_PTERM_CUT_HZ, dT);
        }

        // -----calculate I component.
        if (fullKiLatched) {
            errorGyroIf[axis] = constrainf(errorGyroIf[axis] + RateError * dT * (PID_I_f[axis]/2)  * 10, -250.0f, 250.0f);
        } else {
            errorGyroIf[axis] = constrainf(errorGyroIf[axis] + RateError * dT * (PID_I_f[axis]/2)  * 10, -20.0f, 20.0f);
        }


        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        // I coefficient (I8) moved before integration to make limiting independent from PID settings

        ITerm = errorGyroIf[axis];

        //-----calculate D-term
        delta = RateError - lastError[axis];
        lastError[axis] = RateError;

        if (deltaStateIsSet) {
        	delta = applyBiQuadFilter(delta, &deltaBiQuadState[axis]);
        }

        // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta *= (1.0f / dT);

        float D_f = PID_D_f[axis];
        if (throttleP < KD_ATTENUATION_BREAK) {
        	float Kd_attenuation = constrainf((throttleP / KD_ATTENUATION_BREAK) + 0.50f, 0, 1);
        	D_f = Kd_attenuation * D_f;
        }
        DTerm = constrainf(delta * (D_f/10) * PID_WEIGHT[axis] / 100, -300.0f, 300.0f);

        // -----calculate total PID output
        axisPID[axis] = constrain(lrintf(PTerm + ITerm + DTerm), -1000, 1000);
    }
}


