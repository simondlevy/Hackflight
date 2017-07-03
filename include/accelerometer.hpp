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

        void  init(const AccelerometerConfig & _config, Board * _board);
        void  update(float eulerAnglesRadians[3], bool armed);
        float getAccZ(void);

    private:

        float     accZ;
        float     fc;
        float     lpf[3];
        uint32_t  previousTimeUsec;
        int16_t   smooth[3];
        float     velScale;
        int32_t   zOffset;

        static float rotate(int16_t ned[3], float * angles);

        AccelerometerConfig config;

        Board * board;
};

/********************************************* CPP ********************************************************/

void Accelerometer::init(const AccelerometerConfig & _config, Board * _board)
{
    memcpy(&config, &_config, sizeof(AccelerometerConfig));

    board = _board;

    velScale = (9.80665f / 10000.0f / config.oneG);

    // Calculate RC time constant used in the low-pass filte
    fc = (float)(0.5f / (M_PI * config.lpfCutoff)); 

    for (int k=0; k<3; ++k) {
        smooth[k] = 0;
        lpf[k] = 0;
    }

    previousTimeUsec = 0;
    zOffset = 0;
    accZ = 0;
}

void Accelerometer::update(float eulerAnglesRadians[3], bool armed)
{
    // Get current time in microseconds
    uint32_t currentTimeUsec = board->getMicros();

    // Get raw accelerometer values    
    int16_t accelRaw[3];
    board->extrasImuGetAccel(accelRaw);

    // Track delta time
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
    float rotatedZ = Accelerometer::rotate(smooth, eulerAnglesRadians);

    // Compute vertical acceleration offset at rest
    if (!armed) {
        zOffset -= zOffset / config.zOffsetDiv;
        zOffset += (int32_t)rotatedZ;
    }
    rotatedZ -= zOffset / config.zOffsetDiv;

    // Compute smoothed vertical acceleration
    float dT_sec = dT_usec * 1e-6f;
    accZ = accZ + (dT_sec / (fc + dT_sec)) * (rotatedZ - accZ); // XXX Should user Filter::____

} // update


float Accelerometer::getAccZ(void)
{
    return Filter::deadband(accZ, config.deadband) * velScale;
}

float Accelerometer::rotate(int16_t ned[3], float * angles)
{
    float cosx = cosf(-angles[0]);
    float sinx = sinf(-angles[0]);
    float cosy = cosf(-angles[1]);
    float siny = sinf(-angles[1]);

    return ned[0] * siny + ned[1] * (-sinx * cosy) + ned[2] * (cosy * cosx);
}

} // namespace hf
