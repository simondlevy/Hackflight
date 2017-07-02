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

        static float rotateV(int16_t ned[3], float * angles);

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
    float rotatedZ = Accelerometer::rotateV(smooth, eulerAnglesRadians);

    // Compute vertical acceleration offset at rest
    if (!armed) {
        zOffset -= zOffset / 64;
        zOffset += (int32_t)rotatedZ;
    }
    rotatedZ -= zOffset / 64;

    // Compute smoothed vertical acceleration
    float dT_sec = dT_usec * 1e-6f;
    accZ = accZ + (dT_sec / (fc + dT_sec)) * (rotatedZ - accZ); // XXX Should user Filter::____

} // update


float Accelerometer::getAccZ(void)
{
    Serial.println(accZ);
    return accZ;
}

float Accelerometer::rotateV(int16_t ned[3], float * angles)
{
    float cosx = cosf(-angles[0]);
    float sinx = sinf(-angles[0]);
    float cosy = cosf(-angles[1]);
    float siny = sinf(-angles[1]);

    return ned[0] * siny + ned[1] * (-sinx * cosy) + ned[2] * (cosy * cosx);
}

} // namespace hf
