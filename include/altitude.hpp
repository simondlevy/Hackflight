/* 
   altitude.hpp: Altitude hold

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
        int32_t   getBaroVelocity(int32_t BaroAlt, uint32_t dTimeMicros);

        // Accelerometer
        Accelerometer accel;
        float ACC_VelScale;

        // Fused
        int32_t  AltHold;
        int32_t  BaroPID;
        int32_t  EstAlt;
        int16_t  errorAltitudeI;
        bool     holdingAltitude;
        int16_t  initialThrottleHold; 

        // Microsecond dt for velocity computations
        uint32_t updateTime(void);
};

/********************************************* CPP ********************************************************/

void Altitude::init(const AltitudeConfig & _config, Board * _board)
{
    memcpy(&config, &_config, sizeof(AltitudeConfig));

    board = _board;

    baro.init(config.baro);
    baroCalibrationStart = 0;

    accel.init(config.accel);

    ACC_VelScale = (9.80665f / 10000.0f / config.accel.oneG);

    initialThrottleHold = 0;
    BaroPID = 0;
    holdingAltitude = false;
    errorAltitudeI = 0;
}

void Altitude::start(uint16_t throttleDemand)
{
    holdingAltitude = true;
    initialThrottleHold = throttleDemand;
    AltHold = EstAlt;
    BaroPID = 0;
    errorAltitudeI = 0;
}

void Altitude::stop(void)
{
    holdingAltitude = false;
}

void Altitude::modifyThrottleDemand(int16_t & throttleDemand)
{
    if (holdingAltitude) {

        throttleDemand = Filter::constrainMinMax(initialThrottleHold + BaroPID, config.throttleMin, config.throttleMax);
    }
}

void Altitude::computePid(bool armed)
{  
    // Refresh the timer
    uint32_t dTimeMicros = updateTime();
 
    // Update the baro with the current pressure reading
    baro.update(board->extrasGetBaroPressure());

    // Calibrate baro AGL while not armed
    if (!armed) {
        baro.calibrate();
        return;
    }

    // Get estimated altitude from baro
    int32_t BaroAlt = baro.getAltitude();
    EstAlt = BaroAlt;

    // Integrate vertical acceleration to compute IMU velocity in cm/sec
    static float vel;
    float accZ = Filter::deadband(accel.getAccZ(), config.accel.deadband);
    vel += accZ * ACC_VelScale * dTimeMicros;

    // Apply complementary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay.
    int32_t baroVel = getBaroVelocity(BaroAlt, dTimeMicros);
    vel = Filter::complementary(vel, baroVel, config.cfVel);

    // P
    int16_t error16 = Filter::constrainAbs(AltHold - EstAlt, config.pErrorMax);
    error16 = Filter::deadband(error16, 10); //remove small P parametr to reduce noise near zero position
    BaroPID = Filter::constrainAbs((config.pidP * error16 >>7), config.pidMax);

    // I
    errorAltitudeI += config.pidI * error16 >>6;
    errorAltitudeI = Filter::constrainAbs(errorAltitudeI, config.iErrorMax);
    BaroPID += errorAltitudeI>>9; //I in range +/-60

    // D
    int32_t vario = Filter::deadband(vel, 5) >> 4;
    BaroPID -= Filter::constrainAbs(config.pidD * vario, config.pidMax);
 
} // computePid

void Altitude::updateAccelerometer(float eulerAnglesRadians[3], bool armed)
{
    // Throttle modification is synched to aquisition of new IMU data
    int16_t accelRaw[3];
    board->extrasImuGetAccel(accelRaw);
    accel.update(accelRaw, eulerAnglesRadians, board->getMicros(), armed);
}

uint32_t Altitude::updateTime(void)
{
    static uint32_t previousT;
    uint32_t currentT = board->getMicros();
    uint32_t dTimeMicros = currentT - previousT;
    previousT = currentT;
    return dTimeMicros;
}

int32_t Altitude::getBaroVelocity(int32_t BaroAlt, uint32_t dTimeMicros)
{
    static int32_t lastBaroAlt;
    int32_t baroVel = (BaroAlt - lastBaroAlt) * 1000000.0f / dTimeMicros;
    lastBaroAlt = EstAlt;
    baroVel = Filter::constrainAbs(baroVel, 300); 
    return Filter::deadband(baroVel, 10);
}

} // namespace hf
