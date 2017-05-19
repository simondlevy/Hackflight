/*
   fake.hpp : IMU routines for temporary "fake" board for simulation

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

class Fake : public Board {

    public: // methods

        virtual void imuInit(void) override; 
        virtual void imuCalibrate(int16_t accelRaw[3], int16_t gyroRaw[3]) override;
        virtual void imuRestartCalibration(void) override; 
        virtual bool imuAccelCalibrated(void) override; 
        virtual bool imuGyroCalibrated(void) override; 
        virtual void imuGetEulerAngles(float dT_sec, int16_t accelSmooth[3], int16_t gyroRaw[3], float eulerAnglesRadians[3]) override; 

    protected:

        virtual void imuReadRaw(int16_t _accelRaw[3], int16_t _gyroRaw[3]) = 0;

    private: // types

        typedef struct stdev_t {
            float m_oldM, m_newM, m_oldS, m_newS;
            int m_n;
        } stdev_t;

    private: //methods

        static void normalizeV(float src[3], float dest[3]);

    private: // fields

        float       EstG[3];
        float       EstN[3];
        uint16_t    gyroCalibrationCountdown;
        float       gyroCmpfFactor;
        float       gyroScale;

}; // class Fake


/********************************************* CPP ********************************************************/


void Fake::normalizeV(float src[3], float dest[3])
{
    float length = sqrtf(src[X] * src[X] + src[Y] * src[Y] + src[Z] * src[Z]);

    if (length != 0) {
        dest[X] = src[X] / length;
        dest[Y] = src[Y] / length;
        dest[Z] = src[Z] / length;
    }
}


void Fake::imuInit(void)
{
    for (int k=0; k<3; ++k) {
        EstG[k] = 0;
    }

    EstN[0] = 1.0f;
    EstN[1] = 1.0f;
    EstN[2] = 0.0f;

    // Convert gyro scale from degrees to radians
    // Config is available because Fake is a subclass of Board
    gyroScale = (float)(4.0f / config.imu.gyroScale) * ((float)M_PI / 180.0f);
}

void Fake::imuRestartCalibration(void) 
{
}

bool Fake::imuAccelCalibrated(void) 
{
    return true;
}


bool Fake::imuGyroCalibrated(void)
{
    return true;
}

void Fake::imuCalibrate(int16_t accelRaw[3], int16_t gyroRaw[3]) 
{
    (void)accelRaw;
    (void)gyroRaw;
}

void Fake::imuGetEulerAngles(float dT_sec, int16_t accelSmooth[3], int16_t gyroRaw[3], float eulerAnglesRadians[3]) 
{
    float deltaGyroAngle[3];
    float scale = dT_sec* gyroScale; 

    int32_t accelMag = 0;

    for (uint8_t axis = 0; axis < 3; axis++) {
        deltaGyroAngle[axis] = gyroRaw[axis] * scale;
        accelMag += (int32_t)accelSmooth[axis] * accelSmooth[axis];
    }

    accelMag = accelMag * 100 / ((int32_t)config.imu.accel1G * config.imu.accel1G);

    IMU::rotateV(EstG, deltaGyroAngle);

    // Apply complementary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit
    // range => we neutralize the effect of accelerometers in the angle
    // estimation.  To do that, we just skip filter, as EstV already rotated by Gyro
    if (72 < (uint16_t)accelMag && (uint16_t)accelMag < 133) 
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
