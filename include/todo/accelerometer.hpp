/* 
   accelerometer.hpp: Altitude estimation via accelerometer Z-axis integration

   Adapted from

    https://github.com/multiwii/baseflight/blob/master/src/imu.c

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

namespace hf {
 
class Accelerometer {

    public:

        void  init(const AccelerometerConfig & _config);
        void  update(int16_t accelRaw[3], float eulerAnglesRadians[3], uint32_t currentTimeUsec, bool armed);
        float getAltitude(void);
        float getVelocity(void);
        void  adjustVelocity(float fusedVelocity);
        float getAcceleration(void);
        void  reset(void);
        void  integrate(void);

    private:

        float     altitude;
        float     velocity;
        float     acceleration;

        float     fc;
        float     lpf[3];
        int16_t   smooth[3];
        int32_t   sumCount;
        uint32_t  timeSum;
        float     velScale;
        int32_t   zOffset;
        float     zSmooth;
        int32_t   zSum;
        float     zOld;

        void        resetIntegral(void);
        static void rotateV(float v[3], float *delta);

        AccelerometerConfig config;
};

/********************************************* CPP ********************************************************/

void Accelerometer::init(const AccelerometerConfig & _config)
{
    memcpy(&config, &_config, sizeof(AccelerometerConfig));

    velScale = 9.80665f / config.oneG / 10000.0f;

    // Calculate RC time constant used in the low-pass filte
    fc = (float)(0.5f / (M_PI * config.lpfCutoff)); 

    for (int k=0; k<3; ++k) {
        smooth[k] = 0;
        lpf[k] = 0;
    }

    velocity = 0;
    altitude = 0;
    acceleration = 0;

    zOffset = 0;
    timeSum = 0;
    sumCount = 0;
    zSmooth = 0;
    zSum = 0;
    zOld = 0;
}

void Accelerometer::update(int16_t accelRaw[3], float eulerAnglesRadians[3], uint32_t currentTimeUsec, bool armed)
{
    // Track delta time
    static uint32_t previousTimeUsec;
    uint32_t dT_usec = currentTimeUsec - previousTimeUsec;
    previousTimeUsec = currentTimeUsec;

    // Smooth the raw values if indicated
    for (uint8_t k=0; k<3; k++) {
        if (config.lpfFactor > 0) {
            // XXX should use Filter::lpf()
            lpf[k] = lpf[k] * (1.0f - (1.0f / config.lpfFactor)) + accelRaw[k] * (1.0f / config.lpfFactor);
            smooth[k] = lpf[k];
        } else {
            smooth[k] = accelRaw[k];
        }
    }

    // Rotate accel values into the earth frame

    float rpy[3];
    rpy[0] = -(float)eulerAnglesRadians[0];
    rpy[1] = -(float)eulerAnglesRadians[1];
    rpy[2] = -(float)eulerAnglesRadians[2];

    float ned[3];
    ned[0] = smooth[0];
    ned[1] = smooth[1];
    ned[2] = smooth[2];
    Accelerometer::rotateV(ned, rpy);

    // Compute vertical acceleration offset at rest
    if (!armed) {
        zOffset -= zOffset / 64;
        zOffset += (int32_t)ned[2];
    }
    ned[2] -= zOffset / 64;

    // Compute smoothed vertical acceleration
    float dT_sec = dT_usec * 1e-6f;
    zSmooth = zSmooth + (dT_sec / (fc + dT_sec)) * (ned[2] - zSmooth); // XXX Should user Filter::____

    // Apply Deadband to reduce integration drift and vibration influence and
    // sum up Values for later integration to get velocity and distance
    zSum += Filter::deadband((int32_t)lrintf(zSmooth),  config.deadband);

    // Accumulate time and count for integrating accelerometer values
    timeSum += dT_usec;
    sumCount++;

} // update

void Accelerometer::reset()
{
    velocity = 0;
    altitude = 0;

    resetIntegral();
}

void Accelerometer::integrate(void)
{
    // Compute dt for acceleration
    float dt = timeSum * 1e-6f;

    // Integrate acceleration to get velocity in cm/sec
    float zTmp = (float)zSum / (float)sumCount;
    float velAcc = zTmp * velScale * (float)timeSum;
    acceleration = zTmp + zOld;
    zOld = zTmp;

    // Integrate velocity to get altitude in cm: x= a/2 * t^2
    altitude += (velAcc * 0.5f) * dt + velocity * dt;                                         
    velocity += velAcc;

    // Now that computed acceleration, reset it for next time
    resetIntegral();
}

float Accelerometer::getAltitude(void)
{
    return altitude;
}

float Accelerometer::getVelocity(void)
{
    return velocity;
}

void Accelerometer::adjustVelocity(float fusedVelocity)
{
    velocity = fusedVelocity;
}

float Accelerometer::getAcceleration(void)
{
    return acceleration;
}

void Accelerometer::resetIntegral(void)
{
    zSum = 0;
    sumCount = 0;
    timeSum = 0;
}

void Accelerometer::rotateV(float v[3], float *delta)
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

} // namespace hf
