/* 
   altitude.hpp: Altitude estimator using baro + accelerometer fusion

   Adapted from

    https://github.com/multiwii/baseflight/blob/master/src/imu.c
    https://github.com/multiwii/baseflight/blob/master/src/sensors.c

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
   along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
 */

// XXX we should put these in config.hpp

#define ACCEL_1G             2048
#define ACCEL_Z_LPF_CUTOFF   5.0f
#define ACCEL_Z_DEADBAND     40
#define ACCEL_LPF_FACTOR     4

#define BARO_TAB_SIZE        48
#define BARO_CALIBRATION_SEC 8
#define BARO_NOISE_LPF       0.5f
#define BARO_CF_ALT          0.965f
#define BARO_CF_VEL          0.985f

#define PID_ALT_P            50

#define PID_VEL_P            120
#define PID_VEL_I            45
#define PID_VEL_D            1

#include "config.hpp"

namespace hf {
 
class Altitude {

    public:

        void     init(const AltitudeConfig & _config);
        uint16_t getThrottle(uint16_t throttleDemand, int16_t accelRaw[3], float eulerAnglesRadians[3], uint32_t currentTimeUsec, bool armed);
        void     computePid(float pressure, float eulerAnglesDegrees[3], uint32_t currentTimeUsec);
        void     handleAuxSwitch(uint8_t auxState, uint16_t throttleDemand);

    private:

        AltitudeConfig config;

        // IMU

        float     accelAlt;
        float     accelLpf[3];
        int32_t   accelZSum;
        float     accelVelScale;
        int32_t   accelZOffset;
        float     accelZSmooth;
        int16_t   accelSmooth[3];
        uint32_t  accelTimeSum;
        int32_t   accelSumCount;
        float     accelFc;
        float     accelZOld;

        static int32_t deadbandFilter(int32_t value, int32_t deadband);
        static void    rotateV(float v[3], float *delta);

        // Baro

        uint32_t baroPressureSum;
        int32_t  baroHistTab[BARO_TAB_SIZE];
        int      baroHistIdx;
        int32_t  baroGroundPressure;
        int32_t  baroGroundAltitude;
        int32_t  BaroAlt;
        uint32_t baroCalibrationStart;
        int32_t  lastBaroAlt;

        static float paToCm(uint32_t pa);

        // Fused
        int32_t  AltHold;
        int32_t  BaroPID;
        int32_t  EstAlt;
        float    vel;
        bool     holdingAltitude;
        int32_t  errorVelocityI;
        int32_t  setVelocity;      
        bool     velocityControl; 
        int16_t  initialThrottleHold;
        bool     isAltHoldChanged;
};

/********************************************* CPP ********************************************************/

/*
static void dump(const char * label, int32_t value, const char * end)
{
    Serial.print(label);
    Serial.print(": ");
    if (value>0) 
        Serial.print("+"); 
    Serial.print(value);
    Serial.print(end);
}
*/

void Altitude::init(const AltitudeConfig & _config)
{
    memcpy(&config, &_config, sizeof(AltitudeConfig));

    accelVelScale = 9.80665f / ACCEL_1G / 10000.0f;

    // Calculate RC time constant used in the accelZ lpf    
    accelFc = (float)(0.5f / (M_PI * ACCEL_Z_LPF_CUTOFF)); 

    for (int k=0; k<3; ++k) {
        accelSmooth[k] = 0;
        accelLpf[k] = 0;
    }

    accelAlt = 0;
    accelZOffset = 0;
    accelTimeSum = 0;
    accelSumCount = 0;
    accelZSmooth = 0;
    accelZSum = 0;
    accelZOld = 0;

    baroPressureSum = 0;
    baroHistIdx = 0;
    baroGroundPressure = 0;
    baroGroundAltitude = 0;
    BaroAlt = 0;
    baroCalibrationStart = 0;
    lastBaroAlt = 0;

    for (int k=0; k<BARO_TAB_SIZE; ++k) {
        baroHistTab[k] = 0;
    }

    initialThrottleHold = 0;
    setVelocity = 0;      
    velocityControl = false; 
    BaroPID = 0;
    vel = 0;
    errorVelocityI = 0;
    holdingAltitude = false;
    isAltHoldChanged = false;
}


uint16_t Altitude::getThrottle(uint16_t throttleDemand, int16_t accelRaw[3], float eulerAnglesRadians[3], uint32_t currentTimeUsec, bool armed)
{
    // Track delta time
    static uint32_t previousTimeUsec;
    uint32_t dT_usec = currentTimeUsec - previousTimeUsec;
    previousTimeUsec = currentTimeUsec;

    for (uint8_t k=0; k<3; k++) {
        if (ACCEL_LPF_FACTOR > 0) {
            accelLpf[k] = accelLpf[k] * (1.0f - (1.0f / ACCEL_LPF_FACTOR)) + accelRaw[k] * (1.0f / ACCEL_LPF_FACTOR);
            accelSmooth[k] = accelLpf[k];
        } else {
            accelSmooth[k] = accelRaw[k];
        }
    }

    // Rotate accel values into the earth frame

    float rpy[3];
    rpy[0] = -(float)eulerAnglesRadians[0];
    rpy[1] = -(float)eulerAnglesRadians[1];
    rpy[2] = -(float)eulerAnglesRadians[2];

    float       accelNed[3];
    accelNed[0] = accelSmooth[0];
    accelNed[1] = accelSmooth[1];
    accelNed[2] = accelSmooth[2];
    rotateV(accelNed, rpy);

    // Compute vertical acceleration offset at rest
    if (!armed) {
        accelZOffset -= accelZOffset / 64;
        accelZOffset += (int32_t)accelNed[2];
    }

    // Compute smoothed vertical acceleration
    accelNed[2] -= accelZOffset / 64;  // compensate for gravitation on z-axis
    float dT_sec = dT_usec * 1e-6f;
    accelZSmooth = accelZSmooth + (dT_sec / (accelFc + dT_sec)) * (accelNed[2] - accelZSmooth); // low pass filter

    // Apply Deadband to reduce integration drift and vibration influence and
    // sum up Values for later integration to get velocity and distance
    accelZSum += deadbandFilter((int32_t)lrintf(accelZSmooth),  ACCEL_Z_DEADBAND);

    // Accumulate time and count for integrating accelerometer values
    accelTimeSum += dT_usec;
    accelSumCount++;

    if (holdingAltitude) {
        if (config.fastChange) {
            // rapid alt changes
            if (abs(throttleDemand - initialThrottleHold) > config.throttleNeutral) {
                errorVelocityI = 0;
                isAltHoldChanged = true;
                throttleDemand += (throttleDemand > initialThrottleHold) ? -config.throttleNeutral : config.throttleNeutral;
            } else {
                if (isAltHoldChanged) {
                    AltHold = EstAlt;
                    isAltHoldChanged = false;
                }
                throttleDemand = constrain(initialThrottleHold + BaroPID, config.throttleMin, config.throttleMax);
            }
        } else {
            // slow alt changes
            if (abs(throttleDemand - initialThrottleHold) > config.throttleNeutral) {
                // set velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
                setVelocity = (throttleDemand - initialThrottleHold) / 2;
                velocityControl = true;
                isAltHoldChanged = true;
            } else if (isAltHoldChanged) {
                AltHold = EstAlt;
                velocityControl = false;
                isAltHoldChanged = false;
            }
            throttleDemand = constrain(initialThrottleHold + BaroPID, config.throttleMin, config.throttleMax);
        }
    }

    return throttleDemand;

} // getThrottle

void Altitude::computePid(float pressure, float eulerAnglesDegrees[3], uint32_t currentTimeUsec)
{  
    static uint32_t previousTimeUsec;
    uint32_t dT_usec = currentTimeUsec - previousTimeUsec;
    previousTimeUsec = currentTimeUsec;

    // Start baro calibration if not yet started
    if (!baroCalibrationStart) 
        baroCalibrationStart = millis();

    // Smoothe baro pressure using history
    uint8_t indexplus1 = (baroHistIdx + 1) % BARO_TAB_SIZE;
    baroHistTab[baroHistIdx] = (int32_t)pressure;
    baroPressureSum += baroHistTab[baroHistIdx];
    baroPressureSum -= baroHistTab[indexplus1];
    baroHistIdx = indexplus1;

    // Compute baro ground altitude during calibration
    if (millis() - baroCalibrationStart < 1000*BARO_CALIBRATION_SEC) {
        baroGroundPressure -= baroGroundPressure / 8;
        baroGroundPressure += baroPressureSum / (BARO_TAB_SIZE - 1);
        baroGroundAltitude = paToCm(baroGroundPressure/8);
        vel = 0;
        accelAlt = 0;
    }

    int32_t BaroAlt_tmp = paToCm((float)baroPressureSum/(BARO_TAB_SIZE-1)); 
    BaroAlt_tmp -= baroGroundAltitude;
    BaroAlt = lrintf((float)BaroAlt * BARO_NOISE_LPF + (float)BaroAlt_tmp * (1.0f - BARO_NOISE_LPF)); // additional LPF to reduce baro noise

    float dt = accelTimeSum * 1e-6f; // delta acc reading time in seconds

    // Integrate acceleration to get velocity in cm/sec
    float accelZTmp = (float)accelZSum / (float)accelSumCount;
    float vel_acc = accelZTmp * accelVelScale * (float)accelTimeSum;

    // Integrate velocity to get altitude in cm: x= a/2 * t^2
    accelAlt += (vel_acc * 0.5f) * dt + vel * dt;                                         

    // Apply complementary filter to fuse baro and accel
    accelAlt = accelAlt * BARO_CF_ALT + (float)BaroAlt * (1.0f - BARO_CF_ALT);      

    EstAlt = BaroAlt; 

    /*
       if (sonarAlt > 0 && sonarAlt < 200)
       EstAlt = BaroAlt;
       else
       EstAlt = accAlt;
     */

    vel += vel_acc;

    // Now that computed acceleration, reset it for next time
    accelZSum = 0;
    accelSumCount = 0;
    accelTimeSum = 0;

    // Compute velocity from barometer
    int32_t baroVel = (BaroAlt - lastBaroAlt) * 1000000.0f / dT_usec;
    lastBaroAlt = BaroAlt;
    baroVel = constrain(baroVel, -1500, 1500);    // constrain baro velocity +/- 1500cm/s
    baroVel = deadbandFilter(baroVel, 10);         // to reduce noise near zero

    // Apply complementary filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * BARO_CF_VEL + baroVel * (1 - BARO_CF_VEL);
    int32_t vel_tmp = lrintf(vel);

    // only calculate pid if the copters thrust is facing downwards(<80deg)
    BaroPID = 0;
    uint16_t tiltAngle = (abs(eulerAnglesDegrees[0]) >  abs(eulerAnglesDegrees[1])) ? abs(eulerAnglesDegrees[0]) : abs(eulerAnglesDegrees[1]);
    if (tiltAngle < 800) { 

        int32_t setVel = setVelocity;
        int32_t error = 0;

        // Altitude P-Controller
        if (!velocityControl) {
            error = constrain(AltHold - EstAlt, -500, 500);
            error = deadbandFilter(error, 10);       // remove small P parametr to reduce noise near zero position
            setVel = constrain((PID_ALT_P * error / 128), -300, +300); // limit velocity to +/- 3 m/s
        } 

        // Velocity PID-Controller
        // P
        error = setVel - vel_tmp;
        BaroPID = constrain((PID_VEL_P * error / 32), -300, +300);

        // I
        errorVelocityI += (PID_VEL_I * error);
        errorVelocityI = constrain(errorVelocityI, -(8196 * 200), (8196 * 200));
        BaroPID += errorVelocityI / 8196;     // I in the range of +/-200

        // D
        BaroPID -= constrain(PID_VEL_D * (accelZTmp + accelZOld) / 512, -150, 150);

    } 

    /*
    dump("AltHold", AltHold, "    ");
    dump("setVelocity", setVelocity, "    ");
    dump("velocityControl", velocityControl, "    ");
    dump("BaroPID", BaroPID, "\n");
    */

    accelZOld = accelZTmp;

} // computePid

void Altitude::handleAuxSwitch(uint8_t auxState, uint16_t throttleDemand)
{
    if (auxState > 0) {
        holdingAltitude = true;
        initialThrottleHold = throttleDemand;
        AltHold = EstAlt;
        errorVelocityI = 0;
        BaroPID = 0;
    }
    else {
        holdingAltitude = false;
    }
}


void Altitude::rotateV(float v[3], float *delta)
{
    float * v_tmp = v;

    // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(delta[0]);
    sinx = sinf(delta[0]);
    cosy = cosf(delta[1]);
    siny = sinf(delta[1]);
    cosz = cosf(delta[2]);
    sinz = sinf(delta[2]);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = cosz * cosy;
    mat[0][1] = -cosy * sinz;
    mat[0][2] = siny;
    mat[1][0] = sinzcosx + (coszsinx * siny);
    mat[1][1] = coszcosx - (sinzsinx * siny);
    mat[1][2] = -sinx * cosy;
    mat[2][0] = (sinzsinx) - (coszcosx * siny);
    mat[2][1] = (coszsinx) + (sinzcosx * siny);
    mat[2][2] = cosy * cosx;

    v[0] = v_tmp[0] * mat[0][0] + v_tmp[1] * mat[1][0] + v_tmp[2] * mat[2][0];
    v[1] = v_tmp[0] * mat[0][1] + v_tmp[1] * mat[1][1] + v_tmp[2] * mat[2][1];
    v[2] = v_tmp[0] * mat[0][2] + v_tmp[1] * mat[1][2] + v_tmp[2] * mat[2][2];
}


int32_t Altitude::deadbandFilter(int32_t value, int32_t deadband)
{
    if (abs(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

// Pressure in Pascals to altitude in centimeters
float Altitude::paToCm(uint32_t pa)
{
    return (1.0f - powf(pa / 101325.0f, 0.190295f)) * 4433000.0f;
}

} // namespace hf
