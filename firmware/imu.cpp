/*
   imu.cpp : IMU class implementation

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/imu.c

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

#include <math.h>

#include "mw.hpp"

#define INV_GYR_CMPF_FACTOR   (1.0f / ((float)CONFIG_GYRO_CMPF_FACTOR + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / ((float)CONFIG_GYRO_CMPFM_FACTOR + 1.0f))


typedef struct stdev_t {
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

static void devClear(struct stdev_t *dev)
{
    dev->m_n = 0;
}

static void devPush(stdev_t *dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1) {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    } else {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

static float devVariance(stdev_t *dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

static float devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}


// Normalize a vector
static void normalizeV(float src[3], float dest[3])
{
    float length = sqrtf(src[X] * src[X] + src[Y] * src[Y] + src[Z] * src[Z]);

    if (length != 0) {
        dest[X] = src[X] / length;
        dest[Y] = src[Y] / length;
        dest[Z] = src[Z] / length;
    }
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
static void rotateV(float v[3], float *delta)
{
    float * v_tmp = v;

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

    v[X] = v_tmp[X] * mat[0][0] + v_tmp[Y] * mat[1][0] + v_tmp[Z] * mat[2][0];
    v[Y] = v_tmp[X] * mat[0][1] + v_tmp[Y] * mat[1][1] + v_tmp[Z] * mat[2][1];
    v[Z] = v_tmp[X] * mat[0][2] + v_tmp[Y] * mat[1][2] + v_tmp[Z] * mat[2][2];
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

void IMU::init(Board * board, uint16_t _calibratingGyroCycles, uint16_t _calibratingAccCycles) 
{
    this->_board = board;

    this->_board->imuInit(this->acc1G, this->gyroScale);

    // calculate RC time constant used in the accZ lpf    
    this->fcAcc = 0.5f / (M_PI * CONFIG_ACCZ_LPF_CUTOFF); 

    this->gyroADC[0] = 0;
    this->gyroADC[1] = 0;
    this->gyroADC[2] = 0;

    this->calibratingGyroCycles = _calibratingGyroCycles;
    this->calibratingAccCycles = _calibratingAccCycles;
}

void IMU::update(uint32_t currentTime, bool armed, uint16_t & calibratingA, uint16_t & calibratingG)
{
    static float    accelLPF[3];
    static int32_t  accelZoffset;
    static float    accz_smooth;
    static int16_t  accelZero[3];
    static int32_t  a[3];
    static int16_t  accelSmooth[3];
    static int32_t  accelSum[3];
    static uint32_t accelTimeSum;        // keep track for integration of acc
    static float    EstG[3];
    static float    EstN[3] = { 1.0f, 0.0f, 0.0f };
    static int16_t  gyroZero[3];
    static uint32_t previousTime;

    int32_t accMag = 0;
    float dT = 0;
    float rpy[3];
    float accel_ned[3];
    float deltaGyroAngle[3];
    uint32_t deltaT = currentTime - previousTime;
    float scale = deltaT * this->gyroScale;
    int16_t  accelADC[3];
    float anglerad[3];

    previousTime = currentTime;

    this->_board->imuRead(accelADC, this->gyroADC);

    //printf("%d %d %d\n", accelADC[0], accelADC[1], accelADC[2]);

    if (calibratingA > 0) {

        for (uint8_t axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (calibratingA == this->calibratingAccCycles)
                a[axis] = 0;
            // Sum up this->calibratingAccCycles readings
            a[axis] += accelADC[axis];
            // Clear global variables for next reading
            accelADC[axis] = 0;
            accelZero[axis] = 0;
        }
        // Calculate average, shift Z down by acc1G
        if (calibratingA == 1) {
            accelZero[ROLL] = (a[ROLL] + (this->calibratingAccCycles / 2)) / this->calibratingAccCycles;
            accelZero[PITCH] = (a[PITCH] + (this->calibratingAccCycles / 2)) / this->calibratingAccCycles;
            accelZero[YAW] = (a[YAW] + (this->calibratingAccCycles / 2)) / this->calibratingAccCycles - this->acc1G;
        }
        calibratingA--;
    }

    accelADC[ROLL]  -= accelZero[ROLL];
    accelADC[PITCH] -= accelZero[PITCH];
    accelADC[YAW]   -= accelZero[YAW];

    // range: +/- 8192; +/- 2000 deg/sec

    static int32_t g[3];
    static stdev_t var[3];

    if (calibratingG > 0) {
        for (uint8_t axis = 0; axis < 3; axis++) {
            // Reset g[axis] at start of calibration
            if (calibratingG == this->calibratingGyroCycles) {
                g[axis] = 0;
                devClear(&var[axis]);
            }
            // Sum up 1000 readings
            g[axis] += this->gyroADC[axis];
            devPush(&var[axis], this->gyroADC[axis]);
            // Clear global variables for next reading
            this->gyroADC[axis] = 0;
            gyroZero[axis] = 0;
            if (calibratingG == 1) {
                float dev = devStandardDeviation(&var[axis]);
                // check deviation and startover if idiot was moving the model
                if (CONFIG_MORON_THRESHOLD && dev > CONFIG_MORON_THRESHOLD) {
                    calibratingG = this->calibratingGyroCycles;
                    devClear(&var[0]);
                    devClear(&var[1]);
                    devClear(&var[2]);
                    g[0] = g[1] = g[2] = 0;
                    continue;
                }
                gyroZero[axis] = (g[axis] + (this->calibratingGyroCycles / 2)) / this->calibratingGyroCycles;
            }
        }
        calibratingG--;
    }

    for (uint8_t axis = 0; axis < 3; axis++)
        this->gyroADC[axis] -= gyroZero[axis];

    // Initialization
    for (uint8_t axis = 0; axis < 3; axis++) {
        deltaGyroAngle[axis] = this->gyroADC[axis] * scale;
        if (CONFIG_ACC_LPF_FACTOR > 0) {
            accelLPF[axis] = accelLPF[axis] * (1.0f - (1.0f / CONFIG_ACC_LPF_FACTOR)) + accelADC[axis] * 
                (1.0f / CONFIG_ACC_LPF_FACTOR);
            accelSmooth[axis] = accelLPF[axis];
        } else {
            accelSmooth[axis] = accelADC[axis];
        }
        accMag += (int32_t)accelSmooth[axis] * accelSmooth[axis];
    }
    accMag = accMag * 100 / ((int32_t)this->acc1G * this->acc1G);

    rotateV(EstG, deltaGyroAngle);

    // Apply complementary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit
    // range => we neutralize the effect of accelerometers in the angle
    // estimation.  To do that, we just skip filter, as EstV already rotated by Gyro
    if (72 < (uint16_t)accMag && (uint16_t)accMag < 133) {
        for (uint8_t axis = 0; axis < 3; axis++)
            EstG[axis] = (EstG[axis] * (float)CONFIG_GYRO_CMPF_FACTOR + accelSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    }

    // Attitude of the estimated vector
    anglerad[ROLL] = atan2f(EstG[Y], EstG[Z]);
    anglerad[PITCH] = atan2f(-EstG[X], sqrtf(EstG[Y] * EstG[Y] + EstG[Z] * EstG[Z]));

    rotateV(EstN, deltaGyroAngle);
    normalizeV(EstN, EstN);

    // Calculate heading
    float cosineRoll = cosf(anglerad[ROLL]);
    float sineRoll = sinf(anglerad[ROLL]);
    float cosinePitch = cosf(anglerad[PITCH]);
    float sinePitch = sinf(anglerad[PITCH]);
    float Xh = EstN[X] * cosinePitch + EstN[Y] * sineRoll * sinePitch + EstN[Z] * sinePitch * cosineRoll;
    float Yh = EstN[Y] * cosineRoll - EstN[Z] * sineRoll;
    anglerad[YAW] = atan2f(Yh, Xh); 

    // deltaT is measured in us ticks
    dT = (float)deltaT * 1e-6f;

    // the accel values have to be rotated into the earth frame
    rpy[0] = -(float)anglerad[ROLL];
    rpy[1] = -(float)anglerad[PITCH];
    rpy[2] = -(float)anglerad[YAW];

    accel_ned[X] = accelSmooth[0];
    accel_ned[Y] = accelSmooth[1];
    accel_ned[Z] = accelSmooth[2];

    rotateV(accel_ned, rpy);

    if (!armed) {
        accelZoffset -= accelZoffset / 64;
        accelZoffset += accel_ned[Z];
    }
    accel_ned[Z] -= accelZoffset / 64;  // compensate for gravitation on z-axis

    accz_smooth = accz_smooth + (dT / (fcAcc + dT)) * (accel_ned[Z] - accz_smooth); // low pass filter

    // apply Deadband to reduce integration drift and vibration influence and
    // sum up Values for later integration to get velocity and distance
    accelSum[X] += applyDeadband(lrintf(accel_ned[X]), CONFIG_ACCXY_DEADBAND);
    accelSum[Y] += applyDeadband(lrintf(accel_ned[Y]), CONFIG_ACCXY_DEADBAND);
    accelSum[Z] += applyDeadband(lrintf(accz_smooth), CONFIG_ACCZ_DEADBAND);

    accelTimeSum += deltaT;

    // Convert angles from radians to tenths of a degrees
    this->angle[ROLL]  = lrintf(anglerad[ROLL]  * (1800.0f / M_PI));
    this->angle[PITCH] = lrintf(anglerad[PITCH] * (1800.0f / M_PI));
    this->angle[YAW]   = lrintf(anglerad[YAW]   * 1800.0f / M_PI + CONFIG_MAGNETIC_DECLINATION) / 10.0f;

    // Convert heading from [-180,+180] to [0,360]
    if (this->angle[YAW] < 0)
        this->angle[YAW] += 360;
}

#ifdef __arm__
} // extern "C"
#endif
