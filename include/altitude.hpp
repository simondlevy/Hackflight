/* 
   altitude.hpp: Altitude hold via barometer/accelerometer fusion

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

#include "filter.hpp"
#include "config.hpp"
#include "barometer.hpp"
#include "accelerometer.hpp"

namespace hf {
 
class Altitude {

    public:

        void init(const AltitudeConfig & _config, Board * _board);
        void start(uint16_t throttleDemand);
        void stop(void);
        void computePid(bool armed);
        void updateAccelerometer(float eulerAnglesRadians[3], bool armed);
        void modifyThrottleDemand(int16_t & throttleDemand);

    private:

        AltitudeConfig config;

        Board * board;

        // Barometer
        Barometer baro;
        uint32_t  baroCalibrationStart;

        // Accelerometer
        Accelerometer accel;

        // Fused
        int32_t  altHold;
        int16_t  errorAltitudeI;
        int32_t  baroAlt;               // cm
        bool     holdingAltitude;
        int16_t  initialThrottleHold; 
        int32_t  pid;
        float    velocity;             // cm/sec

        // Microsecond dt for velocity computations
        uint32_t updateTime(void);
};

/********************************************* CPP ********************************************************/

void Altitude::init(const AltitudeConfig & _config, Board * _board)
{
    memcpy(&config, &_config, sizeof(AltitudeConfig));

    board = _board;

    baro.init(config.baro, _board);
    baroCalibrationStart = 0;

    accel.init(config.accel, _board);

    initialThrottleHold = 0;
    pid = 0;
    holdingAltitude = false;
    errorAltitudeI = 0;
    velocity = 0;
}

void Altitude::start(uint16_t throttleDemand)
{
    holdingAltitude = true;
    initialThrottleHold = throttleDemand;
    altHold = baroAlt;
    pid = 0;
    errorAltitudeI = 0;
}

void Altitude::stop(void)
{
    holdingAltitude = false;
}

void Altitude::modifyThrottleDemand(int16_t & throttleDemand)
{
    if (holdingAltitude) {

        throttleDemand = Filter::constrainMinMax(initialThrottleHold + pid, config.throttleMin, config.throttleMax);
    }
}

void Altitude::computePid(bool armed)
{  
    // Refresh the timer
    uint32_t dTimeMicros = updateTime();
 
    // Update the baro with the current pressure reading
    baro.update();

    // Calibrate baro AGL while not armed
    if (!armed) {
        baro.calibrate();
        return;
    }

    // Get estimated altitude from baro
    baroAlt = baro.getAltitude();

    // Integrate vertical acceleration to compute IMU velocity in cm/sec
    float accZ = accel.getAccZ();
    velocity += accZ * dTimeMicros;

    // Apply complementary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, 
    // i.e without delay.
    int32_t baroVel = baro.getVelocity(dTimeMicros);
    velocity = Filter::complementary(velocity, baroVel, config.cfVel);

    // P
    int16_t error16 = Filter::constrainAbs(altHold - baroAlt, config.pErrorMax);
    error16 = Filter::deadband(error16, config.pDeadband); //remove small P parametr to reduce noise near zero position
    pid = Filter::constrainAbs((config.pidP * error16 >>7), config.pidMax);

    // I
    errorAltitudeI += config.pidI * error16 >>6;
    errorAltitudeI = Filter::constrainAbs(errorAltitudeI, config.iErrorMax);
    pid += errorAltitudeI>>9; //I in range +/-60

    // D
    int32_t vario = Filter::deadband(velocity, config.dDeadband) >> 4;
    pid -= Filter::constrainAbs(config.pidD * vario, config.pidMax);

} // computePid

void Altitude::updateAccelerometer(float eulerAnglesRadians[3], bool armed)
{
    // Throttle modification is synched to aquisition of new IMU data
    accel.update(eulerAnglesRadians, armed);
}

uint32_t Altitude::updateTime(void)
{
    static uint32_t previousT;
    uint32_t currentT = board->getMicros();
    uint32_t dTimeMicros = currentT - previousT;
    previousT = currentT;
    return dTimeMicros;
}

} // namespace hf
