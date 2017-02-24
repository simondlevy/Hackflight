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

#include "hackflight.hpp"
#include "filters.hpp"

#define INV_GYR_CMPF_FACTOR   (1.0f / ((float)CONFIG_GYRO_CMPF_FACTOR + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / ((float)CONFIG_GYRO_CMPFM_FACTOR + 1.0f))

enum {
    X = 0,
    Y,
    Z
};


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

    cosx = cosf(delta[AXIS_ROLL]);
    sinx = sinf(delta[AXIS_ROLL]);
    cosy = cosf(delta[AXIS_PITCH]);
    siny = sinf(delta[AXIS_PITCH]);
    cosz = cosf(delta[AXIS_YAW]);
    sinz = sinf(delta[AXIS_YAW]);

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

void IMU::init(uint16_t _calibratingGyroCycles, uint16_t _calibratingAccCycles) 
{
    Board::imuInit(this->acc1G, this->gyroScale);

    // calculate RC time constant used in the this->accelZ lpf    
    this->fcAcc = (float)(0.5f / (M_PI * CONFIG_ACCZ_LPF_CUTOFF)); 

    for (int k=0; k<3; ++k) {
        this->gyroADC[k] = 0;
        this->accelSum[k] = 0;
    }

    this->accelVelScale = 9.80665f / this->acc1G / 10000.0f;

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
    static float    EstG[3];
    static float    EstN[3] = { 1.0f, 0.0f, 0.0f };
    static int16_t  gyroZero[3];
    static uint32_t previousTime;

    int32_t accMag = 0;
    float rpy[3];
    float accel_ned[3];
    float deltaGyroAngle[3];
    uint32_t deltaT = currentTime - previousTime;
    float deltaT_sec = deltaT * 0.000001f; 
    float scale = deltaT_sec* this->gyroScale; 
    int16_t  accelADC[3];
    float anglerad[3];

    previousTime = currentTime;

    Board::imuRead(accelADC, this->gyroADC);

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
            accelZero[AXIS_ROLL] = (a[AXIS_ROLL] + (this->calibratingAccCycles / 2)) / this->calibratingAccCycles;
            accelZero[AXIS_PITCH] = (a[AXIS_PITCH] + (this->calibratingAccCycles / 2)) / this->calibratingAccCycles;
            accelZero[AXIS_YAW] = (a[AXIS_YAW] + (this->calibratingAccCycles / 2)) / this->calibratingAccCycles - this->acc1G;
        }
        calibratingA--;
    }

    accelADC[AXIS_ROLL]  -= accelZero[AXIS_ROLL];
    accelADC[AXIS_PITCH] -= accelZero[AXIS_PITCH];
    accelADC[AXIS_YAW]   -= accelZero[AXIS_YAW];

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
            accelSmooth[axis] = (int16_t)accelLPF[axis];
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
    if (72 < (uint16_t)accMag && (uint16_t)accMag < 133) 
        for (uint8_t axis = 0; axis < 3; axis++)
            EstG[axis] = (EstG[axis] * (float)CONFIG_GYRO_CMPF_FACTOR + accelSmooth[axis]) * INV_GYR_CMPF_FACTOR;

    // Attitude of the estimated vector
    anglerad[AXIS_ROLL] = atan2f(EstG[Y], EstG[Z]);
    anglerad[AXIS_PITCH] = atan2f(-EstG[X], sqrtf(EstG[Y] * EstG[Y] + EstG[Z] * EstG[Z]));

    rotateV(EstN, deltaGyroAngle);
    normalizeV(EstN, EstN);

    // Calculate heading
    float cosineRoll = cosf(anglerad[AXIS_ROLL]);
    float sineRoll = sinf(anglerad[AXIS_ROLL]);
    float cosinePitch = cosf(anglerad[AXIS_PITCH]);
    float sinePitch = sinf(anglerad[AXIS_PITCH]);
    float Xh = EstN[X] * cosinePitch + EstN[Y] * sineRoll * sinePitch + EstN[Z] * sinePitch * cosineRoll;
    float Yh = EstN[Y] * cosineRoll - EstN[Z] * sineRoll;
    anglerad[AXIS_YAW] = atan2f(Yh, Xh); 

    // the accel values have to be rotated into the earth frame
    rpy[0] = -(float)anglerad[AXIS_ROLL];
    rpy[1] = -(float)anglerad[AXIS_PITCH];
    rpy[2] = -(float)anglerad[AXIS_YAW];

    accel_ned[X] = accelSmooth[0];
    accel_ned[Y] = accelSmooth[1];
    accel_ned[Z] = accelSmooth[2];

    rotateV(accel_ned, rpy);

    if (!armed) {
        accelZoffset -= accelZoffset / 64;
        accelZoffset += (int32_t)accel_ned[Z];
    }
    accel_ned[Z] -= accelZoffset / 64;  // compensate for gravitation on z-axis

    accz_smooth = accz_smooth + (deltaT_sec / (fcAcc + deltaT_sec)) * (accel_ned[Z] - accz_smooth); // low pass filter

    // apply Deadband to reduce integration drift and vibration influence and
    // sum up Values for later integration to get velocity and distance
    this->accelSum[X] += deadbandFilter((int32_t)lrintf(accel_ned[X]), CONFIG_ACCXY_DEADBAND);
    this->accelSum[Y] += deadbandFilter((int32_t)lrintf(accel_ned[Y]), CONFIG_ACCXY_DEADBAND);
    this->accelSum[Z] += deadbandFilter((int32_t)lrintf(accz_smooth), CONFIG_ACCZ_DEADBAND);

    this->accelTimeSum += deltaT;
    this->accelSumCount++;

    // Convert angles from radians to tenths of a degrees
    this->angle[AXIS_ROLL]  = (int16_t)lrintf(anglerad[AXIS_ROLL]  * (1800.0f / M_PI));
    this->angle[AXIS_PITCH] = (int16_t)lrintf(anglerad[AXIS_PITCH] * (1800.0f / M_PI));
    this->angle[AXIS_YAW]   = (int16_t)(lrintf(anglerad[AXIS_YAW]   * 1800.0f / M_PI + CONFIG_MAGNETIC_DECLINATION) / 10.0f);

    // Convert heading from [-180,+180] to [0,360]
    if (this->angle[AXIS_YAW] < 0)
        this->angle[AXIS_YAW] += 360;

} // update

float IMU::computeAccelZ(void)
{
    float accelZ = (float)this->accelSum[2] / (float)this->accelSumCount * (9.80665f / 10000.0f / this->acc1G);

    this->accelSum[0] = 0;
    this->accelSum[1] = 0;
    this->accelSum[2] = 0;
    this->accelSumCount = 0;
    this->accelTimeSum = 0;

    return accelZ;
}

