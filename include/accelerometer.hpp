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
        float getAccZ(void);

    private:

        float     accZ;
        float     fc;
        float     lpf[3];
        int16_t   smooth[3];
        int32_t   zOffset;

        static void rotateV(float v[3], float *delta);

        AccelerometerConfig config;
};

/********************************************* CPP ********************************************************/

void Accelerometer::init(const AccelerometerConfig & _config)
{
    memcpy(&config, &_config, sizeof(AccelerometerConfig));

    // Calculate RC time constant used in the low-pass filte
    fc = (float)(0.5f / (M_PI * config.lpfCutoff)); 

    for (int k=0; k<3; ++k) {
        smooth[k] = 0;
        lpf[k] = 0;
    }

    zOffset = 0;
    accZ = 0;
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
    accZ = accZ + (dT_sec / (fc + dT_sec)) * (ned[2] - accZ); // XXX Should user Filter::____

} // update


float Accelerometer::getAccZ(void)
{
    return accZ;
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
