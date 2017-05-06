/*
   mw32.hpp : IMU routines for Multiwii32-derived boards (Naze, Beef, AlienflightF3)

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

#include "imu.hpp"
#include "board.hpp"
#include "config.hpp"

namespace hf {

class MW32 : public Board {

    public: // methods

        virtual void imuInit(void) override; 
        virtual void imuCalibrate(int16_t accelRaw[3], int16_t gyroRaw[3]) override;
        virtual void imuRestartCalibration(void) override; 
        virtual bool imuAccelCalibrated(void) override; 
        virtual bool imuGyroCalibrated(void) override; 
        virtual void imuUpdateFast(void) override; 
        virtual void imuGetEulerAngles(float dT_sec, int16_t accelSmooth[3], int16_t gyroRaw[3], float eulerAnglesRadians[3]) override; 

    protected:

        virtual void imuReadRaw(int16_t _accelRaw[3], int16_t _gyroRaw[3]) = 0;

    private: // types

        typedef struct stdev_t {
            float m_oldM, m_newM, m_oldS, m_newS;
            int m_n;
        } stdev_t;

    private: //methods

        static void devClear(stdev_t *dev);
        static void devPush(stdev_t *dev, float x);
        static void normalizeV(float src[3], float dest[3]);

    private: // fields

        int32_t     accelAdcSum[3];
        uint16_t    accelCalibrationCountdown;
        int16_t     accelZero[3];
        uint16_t    calibratingAccelCycles;
        uint16_t    calibratingGyroCycles;
        float       EstG[3];
        float       EstN[3];
        uint16_t    gyroCalibrationCountdown;
        float       gyroCmpfFactor;
        float       gyroScale;
        int16_t     gyroZero[3];

}; // class MW32


/********************************************* CPP ********************************************************/


void MW32::devClear(stdev_t *dev)
{
    dev->m_n = 0;
}

void MW32::devPush(stdev_t *dev, float x)
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

void MW32::normalizeV(float src[3], float dest[3])
{
    float length = sqrtf(src[X] * src[X] + src[Y] * src[Y] + src[Z] * src[Z]);

    if (length != 0) {
        dest[X] = src[X] / length;
        dest[Y] = src[Y] / length;
        dest[Z] = src[Z] / length;
    }
}


void MW32::imuInit(void)
{
    for (int k=0; k<3; ++k) {
        accelAdcSum[k] = 0;
        accelZero[k] = 0;
        EstG[k] = 0;
        gyroZero[k] = 0;
    }


    EstN[0] = 1.0f;
    EstN[1] = 1.0f;
    EstN[2] = 0.0f;

    // Convert gyro scale from degrees to radians
    // Config is available because MW32 is a subclass of Board
    gyroScale = (float)(4.0f / config.imu.gyroScale) * ((float)M_PI / 180.0f);

    // Compute loop times based on config from board
    calibratingGyroCycles   = (uint16_t)(1000. * config.imu.calibratingGyroMilli  / config.imu.loopMicro);
    calibratingAccelCycles  = (uint16_t)(1000. * config.imu.calibratingAccelMilli / config.imu.loopMicro);

    // Always calibrate gyro at startup
    gyroCalibrationCountdown = calibratingGyroCycles;
}

void MW32::imuRestartCalibration(void) 
{
    gyroCalibrationCountdown = calibratingGyroCycles;
    accelCalibrationCountdown = calibratingAccelCycles;
}

bool MW32::imuAccelCalibrated(void) 
{
    return gyroCalibrationCountdown == 0;
}


bool MW32::imuGyroCalibrated(void)
{
    return gyroCalibrationCountdown == 0;
}

void MW32::imuUpdateFast(void)
{
}

void MW32::imuCalibrate(int16_t accelRaw[3], int16_t gyroRaw[3]) 
{
    for (int k=0; k<3; ++k) {
        gyroRaw[k] >>= 2;
    }

    if (accelCalibrationCountdown > 0) {

        for (uint8_t axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (accelCalibrationCountdown == calibratingAccelCycles)
                accelAdcSum[axis] = 0;
            // Sum up accel readings
            accelAdcSum[axis] += accelRaw[axis];
            // Clear global variables for next reading
            accelRaw[axis] = 0;
            accelZero[axis] = 0;
        }
        // Calculate average, shift Z down by acc1G
        if (accelCalibrationCountdown == 1) {
            accelZero[AXIS_ROLL] = (accelAdcSum[AXIS_ROLL] + (calibratingAccelCycles / 2)) / calibratingAccelCycles;
            accelZero[AXIS_PITCH] = (accelAdcSum[AXIS_PITCH] + (calibratingAccelCycles / 2)) / calibratingAccelCycles;
            accelZero[AXIS_YAW] = (accelAdcSum[AXIS_YAW] + (calibratingAccelCycles / 2)) / calibratingAccelCycles - config.imu.acc1G;
        }
    }

    accelRaw[AXIS_ROLL]  -= accelZero[AXIS_ROLL];
    accelRaw[AXIS_PITCH] -= accelZero[AXIS_PITCH];
    accelRaw[AXIS_YAW]   -= accelZero[AXIS_YAW];

    // range: +/- 8192; +/- 2000 deg/sec

    static int32_t g[3];
    static stdev_t var[3];

    if (gyroCalibrationCountdown > 0) {
        for (uint8_t axis = 0; axis < 3; axis++) {
            // Reset g[axis] at start of calibration
            if (gyroCalibrationCountdown == calibratingGyroCycles) {
                g[axis] = 0;
                devClear(&var[axis]);
            }
            // Sum up 1000 readings
            g[axis] += gyroRaw[axis];
            devPush(&var[axis], gyroRaw[axis]);
            // Clear global variables for next reading
            gyroRaw[axis] = 0;
            gyroZero[axis] = 0;
            if (gyroCalibrationCountdown == 1) {
                gyroZero[axis] = (g[axis] + (calibratingGyroCycles / 2)) / calibratingGyroCycles;
            }
        }
    }

    // Decrement calibration countdowns
    if (accelCalibrationCountdown > 0)
        accelCalibrationCountdown--;
    if (gyroCalibrationCountdown > 0)
        gyroCalibrationCountdown--;

    for (uint8_t axis = 0; axis < 3; axis++)
        gyroRaw[axis] -= gyroZero[axis];

}

void MW32::imuGetEulerAngles(float dT_sec, int16_t accelSmooth[3], int16_t gyroRaw[3], float eulerAnglesRadians[3]) 
{
    float deltaGyroAngle[3];
    float scale = dT_sec* gyroScale; 

    int32_t accMag = 0;

    for (uint8_t axis = 0; axis < 3; axis++) {
        deltaGyroAngle[axis] = gyroRaw[axis] * scale;
        accMag += (int32_t)accelSmooth[axis] * accelSmooth[axis];
    }

    accMag = accMag * 100 / ((int32_t)config.imu.acc1G * config.imu.acc1G);

    IMU::rotateV(EstG, deltaGyroAngle);

    // Apply complementary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit
    // range => we neutralize the effect of accelerometers in the angle
    // estimation.  To do that, we just skip filter, as EstV already rotated by Gyro
    if (72 < (uint16_t)accMag && (uint16_t)accMag < 133) 
        for (uint8_t axis = 0; axis < 3; axis++)
            EstG[axis] = (EstG[axis] * config.imu.gyroCmpfFactor + accelSmooth[axis]) * (1.0f / (config.imu.gyroCmpfFactor + 1.0f));

    // Attitude of the estimated vector
    eulerAnglesRadians[AXIS_ROLL] = atan2f(EstG[Y], EstG[Z]);
    eulerAnglesRadians[AXIS_PITCH] = atan2f(-EstG[X], sqrtf(EstG[Y] * EstG[Y] + EstG[Z] * EstG[Z]));

    IMU::rotateV(EstN, deltaGyroAngle);
    normalizeV(EstN, EstN);

    // Calculate heading
    float cosineRoll = cosf(eulerAnglesRadians[AXIS_ROLL]);
    float sineRoll = sinf(eulerAnglesRadians[AXIS_ROLL]);
    float cosinePitch = cosf(eulerAnglesRadians[AXIS_PITCH]);
    float sinePitch = sinf(eulerAnglesRadians[AXIS_PITCH]);
    float Xh = EstN[X] * cosinePitch + EstN[Y] * sineRoll * sinePitch + EstN[Z] * sinePitch * cosineRoll;
    float Yh = EstN[Y] * cosineRoll - EstN[Z] * sineRoll;
    eulerAnglesRadians[AXIS_YAW] = atan2f(Yh, Xh); 

} // imuUpdateSlow

} // namespace hf
