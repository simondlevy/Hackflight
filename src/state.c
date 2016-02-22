#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <breezystm32.h>

#include "vitals.h"
#include "axes.h"
#include "state.h"
#include "config.h"
#include "sensors.h"
#include "chans.h"
#include "utils.h"

static float    accVelScale;
static float    fcAcc;
static float    throttleAngleScale;
static int16_t  smallAngle;
static uint32_t accTimeSum;
static int32_t  accSumCount;
static int32_t  accSum[3];
static uint16_t s_acc_1G;


// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

#define INV_GYR_CMPF_FACTOR   (1.0f / ((float)CONFIG_GYRO_CMPF_FACTOR + 1.0f))

typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

t_fp_vector EstG;

// Normalize a vector
static void normalizeV(struct fp_vector *src, struct fp_vector *dest)
{
    float length;

    length = sqrtf(src->X * src->X + src->Y * src->Y + src->Z * src->Z);
    if (length != 0) {
        dest->X = src->X / length;
        dest->Y = src->Y / length;
        dest->Z = src->Z / length;
    }
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
static void rotateV(struct fp_vector *v, float *delta)
{
    struct fp_vector v_tmp = *v;

    // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(delta[ROLL]);
    sinx = sinf(delta[ROLL]);
    cosy = cosf(delta[PITCH]);
    siny = sinf(delta[PITCH]);
    cosz = cosf(delta[YAW]);
    sinz = sinf(delta[YAW]);

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

    v->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
}

static int32_t applyDeadband(int32_t value, int32_t deadband)
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

// rotate acc into Earth frame and calculate acceleration in it
static void calculateAcceleration(int16_t * accSmooth, uint32_t deltaT, int16_t heading, bool armed, float * anglerad)
{
    static int32_t accZoffset = 0;
    static float accz_smooth = 0;
    float dT = 0;
    float rpy[3];
    t_fp_vector accel_ned;

    // deltaT is measured in us ticks
    dT = (float)deltaT * 1e-6f;

    // the accel values have to be rotated into the earth frame
    rpy[0] = -(float)anglerad[ROLL];
    rpy[1] = -(float)anglerad[PITCH];
    rpy[2] = -(float)heading * RAD;

    accel_ned.V.X = accSmooth[0];
    accel_ned.V.Y = accSmooth[1];
    accel_ned.V.Z = accSmooth[2];

    rotateV(&accel_ned.V, rpy);

    if (CONFIG_ACC_UNARMEDCAL == 1) {
        if (!armed) {
            accZoffset -= accZoffset / 64;
            accZoffset += accel_ned.V.Z;
        }
        accel_ned.V.Z -= accZoffset / 64;  // compensate for gravitation on z-axis
    } else
        accel_ned.V.Z -= s_acc_1G;

    accz_smooth = accz_smooth + (dT / (fcAcc + dT)) * (accel_ned.V.Z - accz_smooth); // low pass filter

    // apply Deadband to reduce integration drift and vibration influence and
    // sum up Values for later integration to get velocity and distance
    accSum[X] += applyDeadband(lrintf(accel_ned.V.X), CONFIG_ACCXY_DEADBAND);
    accSum[Y] += applyDeadband(lrintf(accel_ned.V.Y), CONFIG_ACCXY_DEADBAND);
    accSum[Z] += applyDeadband(lrintf(accz_smooth), CONFIG_ACCZ_DEADBAND);

    accTimeSum += deltaT;
    accSumCount++;
}

// baseflight calculation by Luggi09 originates from arducopter
static int16_t calculateHeading(t_fp_vector *vec, float * anglerad)
{
    int16_t head;

    float cosineRoll = cosf(anglerad[ROLL]);
    float sineRoll = sinf(anglerad[ROLL]);
    float cosinePitch = cosf(anglerad[PITCH]);
    float sinePitch = sinf(anglerad[PITCH]);
    float Xh = vec->A[X] * cosinePitch + vec->A[Y] * sineRoll * sinePitch + vec->A[Z] * sinePitch * cosineRoll;
    float Yh = vec->A[Y] * cosineRoll - vec->A[Z] * sineRoll;
    float hd = (atan2f(Yh, Xh) * 1800.0f / M_PI + CONFIG_MAGNETIC_DECLINATION) / 10.0f;
    head = lrintf(hd);
    if (head < 0)
        head += 360;

    return head;
}

static bool sonarInRange(int32_t sonarAlt)
{
    return sonarAlt > 20 && sonarAlt < 765;
}

// complementary filter
static float cfilter(float a, float b, float c) 
{
    return a * c + b * (1 - c);
}

static void resetAcc(void)
{
    accSum[0] = 0;
    accSum[1] = 0;
    accSum[2] = 0;
    accSumCount = 0;
    accTimeSum = 0;
}

// ==================================================================================================

void stateInit(uint16_t acc_1G)
{
    smallAngle = lrintf(acc_1G * cosf(RAD * CONFIG_SMALL_ANGLE));
    accVelScale = 9.80665f / acc_1G / 10000.0f;
    throttleAngleScale = (1800.0f / M_PI) * (900.0f / CONFIG_THROTTLE_CORRECTION_ANGLE);

    fcAcc = 0.5f / (M_PI * CONFIG_ACCZ_LPF_CUTOFF); // calculate RC time constant used in the accZ lpf

    s_acc_1G = acc_1G;

    resetAcc();
}

void stateEstimateAltitude(vitals_t * vitals, int32_t * AltPID, int32_t * AltHold, int32_t * setVelocity, int32_t * errorVelocityI, bool velocityControl)
{
    static uint32_t previousT;
    static float accZ_old;
    static float accelVel;
    static int32_t  FusedBarosonarAlt;
    static int32_t lastFusedBarosonarAlt;
    static int32_t baroAltBaseline;
    static float   accelAlt;
    static bool wasArmed;
    static int32_t  BaroAlt;
    static int32_t  baroAlt_offset;

    uint32_t currentT = micros();
    int16_t tiltAngle = max(abs(vitals->imuAngle[ROLL]), abs(vitals->imuAngle[PITCH]));
    uint32_t dTime = currentT - previousT;

    if (dTime < CONFIG_ALT_UPDATE_USEC)
        return;
    previousT = currentT;

    // Calculates height from ground in cm via baro pressure
    // See: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140
    int32_t baroAltRaw = lrintf((1.0f - powf((float)(vitals->baroPressureSum / (CONFIG_BARO_TAB_SIZE - 1)) 
                    / 101325.0f, 0.190295f)) * 4433000.0f);

    // Grab baro baseline on arming
    if (vitals->armed) {
        if (!wasArmed) {
            baroAltBaseline = baroAltRaw;
            accelVel = 0;
            accelAlt = 0;
        }
        BaroAlt = baroAltRaw - baroAltBaseline;
    }
    else {
        BaroAlt = 0;
    }
    wasArmed = vitals->armed;

    // Calculate sonar altitude only if the sonar is facing downwards(<25deg)
    vitals->sonarAlt = (tiltAngle > 250) ? -1 : vitals->sonarAlt * (900.0f - tiltAngle) / 900.0f;

    // Fuse sonarAlt and BaroAlt
    if (sonarInRange(vitals->sonarAlt)) {
        baroAlt_offset = BaroAlt - vitals->sonarAlt;
        FusedBarosonarAlt = vitals->sonarAlt;
    } else {
        BaroAlt = BaroAlt - baroAlt_offset;
        if (vitals->sonarAlt > 0) {
            float sonarTransition = (300 - vitals->sonarAlt) / 100.0f;
            FusedBarosonarAlt = cfilter(vitals->sonarAlt, BaroAlt, sonarTransition); 
        }
    }

    // delta acc reading time in seconds
    float dt = accTimeSum * 1e-6f; 

    // Integrator - velocity, cm/sec
    float accZ_tmp = (float)accSum[2] / (float)accSumCount;
    float vel_acc = accZ_tmp * accVelScale * (float)accTimeSum;

    // integrate accelerometer velocity to get distance (x= a/2 * t^2)
    accelAlt += (vel_acc * 0.5f) * dt + accelVel * dt;                                         
    accelVel += vel_acc;

    vitals->estAlt = sonarInRange(vitals->sonarAlt) ? FusedBarosonarAlt : accelAlt;

    resetAcc();

    int32_t fusedBaroSonarVel = (FusedBarosonarAlt - lastFusedBarosonarAlt) * 1000000.0f / dTime;
    lastFusedBarosonarAlt = FusedBarosonarAlt;

    fusedBaroSonarVel = constrain(fusedBaroSonarVel, -1500, 1500);    // constrain baro velocity +/- 1500cm/s
    fusedBaroSonarVel = applyDeadband(fusedBaroSonarVel, 10);         // to reduce noise near zero

    // Apply complementary filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, 
    // i.e without delay
    accelVel = cfilter(accelVel, fusedBaroSonarVel, CONFIG_BARO_CF_VEL);
    int32_t vel_tmp = lrintf(accelVel);

    // set vario
    vitals->vario = applyDeadband(vel_tmp, 5);

    if (tiltAngle < 800) { // only calculate pid if the copters thrust is facing downwards(<80deg)

        int32_t setVel = *setVelocity;

        // Altitude P-Controller
        if (!velocityControl) {
            int32_t error = constrain(*AltHold - vitals->estAlt, -500, 500);
            error = applyDeadband(error, 10);       // remove small P parametr to reduce noise near zero position
            setVel = constrain((CONFIG_ALT_P * error / 128), -300, +300); // limit velocity to +/- 3 m/s
        } 

        // Velocity PID-Controller
        // P
        int32_t error = setVel - vel_tmp;
        *AltPID = constrain((CONFIG_VEL_P * error / 32), -300, +300);

        // I
        *errorVelocityI = *errorVelocityI + (CONFIG_VEL_I * error);
        *errorVelocityI = *errorVelocityI + constrain(*errorVelocityI, -(8196 * 200), (8196 * 200));
        *AltPID = *AltPID + *errorVelocityI / 8196;     // I in the range of +/-200

        // D
        *AltPID -= constrain(CONFIG_VEL_D * (accZ_tmp + accZ_old) / 512, -150, 150);

    } else {
        *AltPID = 0;
    }

    accZ_old = accZ_tmp;
}

bool stateEstimateAttitude(vitals_t * vitals, sensor_t * acc, sensor_t * gyro, int16_t * throttleAngleCorrection)
{
    static float       anglerad[2];    // absolute angle inclination in radians
    static t_fp_vector EstN;
    static float       accLPF[3];
    static uint32_t    previousT;

    uint32_t currentT = micros();
    uint32_t deltaT;
    float scale, deltaGyroAngle[3];
    int16_t accADC[3];
    int16_t gyroADC[3];
    int32_t axis;
    int32_t accMag = 0;


    deltaT = currentT - previousT;
    scale = deltaT * gyro->scale;
    previousT = currentT;
    EstN.A[0] = 1.0f;
    EstN.A[1] = 0.0f;
    EstN.A[2] = 0.0f;

    sensorsGetGyro(gyro, gyroADC);
    sensorsGetAccel(acc, accADC);

    // Initialization
    for (axis = 0; axis < 3; axis++) {
        deltaGyroAngle[axis] = gyroADC[axis] * scale;
        if (CONFIG_ACC_LPF_FACTOR > 0) {
            accLPF[axis] = accLPF[axis] * (1.0f - (1.0f / CONFIG_ACC_LPF_FACTOR)) + accADC[axis] * 
                (1.0f / CONFIG_ACC_LPF_FACTOR);
            vitals->accSmooth[axis] = accLPF[axis];
        } else {
            vitals->accSmooth[axis] = accADC[axis];
        }
        accMag += (int32_t)vitals->accSmooth[axis] * vitals->accSmooth[axis];
    }
    accMag = accMag * 100 / ((int32_t)s_acc_1G * s_acc_1G);

    rotateV(&EstG.V, deltaGyroAngle);

    // Apply complementary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of 
    // accelerometers in the angle estimation.  To do that, we just skip filter, as EstV already rotated by Gyro.
    if (72 < (uint16_t)accMag && (uint16_t)accMag < 133) {
        for (axis = 0; axis < 3; axis++)
            EstG.A[axis] = (EstG.A[axis] * (float)CONFIG_GYRO_CMPF_FACTOR + vitals->accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    }

    bool useSmallAngle = (EstG.A[Z] > smallAngle);

    // Attitude of the estimated vector
    anglerad[ROLL] = atan2f(EstG.V.Y, EstG.V.Z);
    anglerad[PITCH] = atan2f(-EstG.V.X, sqrtf(EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z));
    vitals->imuAngle[ROLL] = lrintf(anglerad[ROLL] * (1800.0f / M_PI));
    vitals->imuAngle[PITCH] = lrintf(anglerad[PITCH] * (1800.0f / M_PI));

    rotateV(&EstN.V, deltaGyroAngle);
    normalizeV(&EstN.V, &EstN.V);
    vitals->heading = calculateHeading(&EstN, anglerad);

    calculateAcceleration(vitals->accSmooth, deltaT, vitals->heading, vitals->armed, anglerad);

    if (CONFIG_THROTTLE_CORRECTION_VALUE) {

        float cosZ = EstG.V.Z / sqrtf(EstG.V.X * EstG.V.X + EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z);

        if (cosZ <= 0.015f) { // we are inverted, vertical or with a small angle < 0.86 deg
            *throttleAngleCorrection = 0;
        } else {
            int deg = lrintf(acosf(cosZ) * throttleAngleScale);
            if (deg > 900)
                deg = 900;
            *throttleAngleCorrection = lrintf(CONFIG_THROTTLE_CORRECTION_VALUE * sinf(deg / (900.0f * M_PI / 2.0f)));
        }
    }

    vitals->gyroData[YAW] = gyroADC[YAW];
    vitals->gyroData[ROLL] = gyroADC[ROLL];
    vitals->gyroData[PITCH] = gyroADC[PITCH];

    return useSmallAngle;
}


