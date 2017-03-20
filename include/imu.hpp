/*
   imu.hpp : IMU class header

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

#pragma once

#include <cstdlib>
#include <cstring>

#include "config.hpp"

namespace hf {

enum {
    AXIS_ROLL = 0,
    AXIS_PITCH,
    AXIS_YAW
};

class IMU {
    
    public: // fields
        int16_t   accelADC[3];   // [-4096,+4096]
        int16_t   gyroADC[3];    // [-4096,+4096]
        ImuConfig config;
        int16_t   angle[3];      // tenths of a degree

    public: // methods
        void init(ImuConfig & imuConfig, uint16_t _calibratingGyroCycles, uint16_t _calibratingAccCycles);
        void update(Board * board, 
                uint32_t currentTimeUsec, bool armed, uint16_t calibratingA=0, uint16_t calibratingG=0);

        // called from Hover
        float computeAccelZ(void);

    private: // types
        enum {
            X = 0,
            Y,
            Z
        };

        typedef struct stdev_t {
            float m_oldM, m_newM, m_oldS, m_newS;
            int m_n;
        } stdev_t;

    private: //methods
        static void devClear(stdev_t *dev);
        static void devPush(stdev_t *dev, float x);
        static float devVariance(stdev_t *dev);
        static float devStandardDeviation(stdev_t *dev);
        static void normalizeV(float src[3], float dest[3]);
        static void rotateV(float v[3], float *delta);

    private: //fields
        int32_t  a[3];
        float    accelLpf[3];
        int16_t  accelSmooth[3];
        int32_t  accelSum[3];
        int32_t  accelSumCount;
        uint32_t accelTimeSum;
        float    accelVelScale;
        int16_t  accelZero[3];
        int32_t  accelZoffset;
        float    accz_smooth;
        uint16_t calibratingGyroCycles;
        uint16_t calibratingAccCycles;
        float    EstG[3];
        float    EstN[3];
        float    fcAcc;
        float    gyroCmpfFactor;
        float    gyroScale;
        int16_t  gyroZero[3];
        uint32_t previousTimeUsec;

};

/********************************************* CPP ********************************************************/

static int32_t deadbandFilter(int32_t value, int32_t deadband)
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

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void IMU::rotateV(float v[3], float *delta)
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

float IMU::computeAccelZ(void)
{
    float accelZ = (float)accelSum[2] / (float)accelSumCount * (9.80665f / 10000.0f / config.acc1G);

    accelSum[0] = 0;
    accelSum[1] = 0;
    accelSum[2] = 0;
    accelSumCount = 0;
    accelTimeSum = 0;

    return accelZ;
}


void IMU::devClear(stdev_t *dev)
{
    dev->m_n = 0;
}

void IMU::devPush(stdev_t *dev, float x)
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

float IMU::devVariance(stdev_t *dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

float IMU::devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}


void IMU::normalizeV(float src[3], float dest[3])
{
    float length = sqrtf(src[X] * src[X] + src[Y] * src[Y] + src[Z] * src[Z]);

    if (length != 0) {
        dest[X] = src[X] / length;
        dest[Y] = src[Y] / length;
        dest[Z] = src[Z] / length;
    }
}

void IMU::init(ImuConfig & imuConfig, uint16_t _calibratingGyroCycles, uint16_t _calibratingAccCycles)
{
    for (int k=0; k<3; ++k) {
        a[k] = 0;
        accelLpf[k] = 0;
        accelSmooth[k] = 0;
        accelSum[k] = 0;
        accelZero[k] = 0;
        EstG[k] = 0;
        gyroZero[k] = 0;
    }

    accelSumCount = 0;
    accelTimeSum = 0;
    accelVelScale = 0;
    accelZoffset = 0;
    accz_smooth = 0;
    calibratingGyroCycles = 0;
    calibratingAccCycles = 0;
    fcAcc = 0;
    previousTimeUsec = 0;

    memcpy(&config, &imuConfig, sizeof(ImuConfig));

    EstN[0] = 1.0f;
    EstN[1] = 1.0f;
    EstN[2] = 0.0f;

    // Convert gyro scale from degrees to radians
    gyroScale = (float)(4.0f / config.gyroScale) * ((float)M_PI / 180.0f);

    // calculate RC time constant used in the accelZ lpf    
    fcAcc = (float)(0.5f / (M_PI * config.accelzLpfCutoff)); 

    for (int k=0; k<3; ++k) {
        accelSum[k] = 0;
    }

    accelVelScale = 9.80665f / config.acc1G / 10000.0f;

    calibratingGyroCycles = _calibratingGyroCycles;
    calibratingAccCycles = _calibratingAccCycles;
}


void IMU::update(Board * board,
        uint32_t currentTimeUsec, bool armed, uint16_t calibratingA, uint16_t calibratingG)
{
    int32_t accMag = 0;
    float rpy[3];
    float accel_ned[3];
    float deltaGyroAngle[3];
    uint32_t dT_usec = currentTimeUsec - previousTimeUsec;
    float dT_sec = dT_usec * 1e-6f;
    float scale = dT_sec* gyroScale; 
    float anglerad[3];

    board->imuRead(accelADC, gyroADC);

    previousTimeUsec = currentTimeUsec;

    for (int k=0; k<3; ++k) {
        gyroADC[k] >>= 2;
    }

    if (calibratingA > 0) {

        for (uint8_t axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (calibratingA == calibratingAccCycles)
                a[axis] = 0;
            // Sum up calibratingAccCycles readings
            a[axis] += accelADC[axis];
            // Clear global variables for next reading
            accelADC[axis] = 0;
            accelZero[axis] = 0;
        }
        // Calculate average, shift Z down by acc1G
        if (calibratingA == 1) {
            accelZero[AXIS_ROLL] = (a[AXIS_ROLL] + (calibratingAccCycles / 2)) / calibratingAccCycles;
            accelZero[AXIS_PITCH] = (a[AXIS_PITCH] + (calibratingAccCycles / 2)) / calibratingAccCycles;
            accelZero[AXIS_YAW] = (a[AXIS_YAW] + (calibratingAccCycles / 2)) / calibratingAccCycles - config.acc1G;
        }
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
            if (calibratingG == calibratingGyroCycles) {
                g[axis] = 0;
                devClear(&var[axis]);
            }
            // Sum up 1000 readings
            g[axis] += gyroADC[axis];
            devPush(&var[axis], gyroADC[axis]);
            // Clear global variables for next reading
            gyroADC[axis] = 0;
            gyroZero[axis] = 0;
            if (calibratingG == 1) {
                float dev = devStandardDeviation(&var[axis]);
                // check deviation and startover if idiot was moving the model
                if (config.moronThreshold && dev > config.moronThreshold) {
                    calibratingG = calibratingGyroCycles;
                    devClear(&var[0]);
                    devClear(&var[1]);
                    devClear(&var[2]);
                    g[0] = g[1] = g[2] = 0;
                    continue;
                }
                gyroZero[axis] = (g[axis] + (calibratingGyroCycles / 2)) / calibratingGyroCycles;
            }
        }
    }

    for (uint8_t axis = 0; axis < 3; axis++)
        gyroADC[axis] -= gyroZero[axis];

    // Initialization
    for (uint8_t axis = 0; axis < 3; axis++) {
        deltaGyroAngle[axis] = gyroADC[axis] * scale;
        if (config.accelLpfFactor > 0) {
            accelLpf[axis] = accelLpf[axis] * (1.0f - (1.0f / config.accelLpfFactor)) + accelADC[axis] * 
                (1.0f / config.accelLpfFactor);
            accelSmooth[axis] = (int16_t)accelLpf[axis];
        } else {
            accelSmooth[axis] = accelADC[axis];
        }
        accMag += (int32_t)accelSmooth[axis] * accelSmooth[axis];
    }

    accMag = accMag * 100 / ((int32_t)config.acc1G * config.acc1G);

    rotateV(EstG, deltaGyroAngle);

    // Apply complementary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit
    // range => we neutralize the effect of accelerometers in the angle
    // estimation.  To do that, we just skip filter, as EstV already rotated by Gyro
    if (72 < (uint16_t)accMag && (uint16_t)accMag < 133) 
        for (uint8_t axis = 0; axis < 3; axis++)
            EstG[axis] = (EstG[axis] * config.gyroCmpfFactor + accelSmooth[axis]) * 
                (1.0f / (config.gyroCmpfFactor + 1.0f));

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

    accz_smooth = accz_smooth + (dT_sec / (fcAcc + dT_sec)) * (accel_ned[Z] - accz_smooth); // low pass filter

    // apply Deadband to reduce integration drift and vibration influence and
    // sum up Values for later integration to get velocity and distance
    accelSum[X] += deadbandFilter((int32_t)lrintf(accel_ned[X]), config.accelXyDeadband);
    accelSum[Y] += deadbandFilter((int32_t)lrintf(accel_ned[Y]), config.accelXyDeadband);
    accelSum[Z] += deadbandFilter((int32_t)lrintf(accz_smooth),  config.accelZDeadband);

    accelTimeSum += dT_usec;
    accelSumCount++;

    // Convert angles from radians to tenths of a degrees
    angle[AXIS_ROLL]  = (int16_t)lrintf(anglerad[AXIS_ROLL]  * (1800.0f / M_PI));
    angle[AXIS_PITCH] = (int16_t)lrintf(anglerad[AXIS_PITCH] * (1800.0f / M_PI));
    angle[AXIS_YAW]   = (int16_t)(lrintf(anglerad[AXIS_YAW]   * 1800.0f / M_PI) / 10.0f);

    // Convert heading from [-180,+180] to [0,360]
    if (angle[AXIS_YAW] < 0)
        angle[AXIS_YAW] += 360;
} // update

} // namespace
