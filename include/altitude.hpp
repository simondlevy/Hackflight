/* 
   altitude.hpp: Altitude estimator

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
        void handleAuxSwitch(uint8_t auxState, uint16_t throttleDemand);
        void computePid(bool armed);
        void updateAccelerometer(float eulerAnglesRadians[3], bool armed);
        void modifyThrottleDemand(int16_t & throttleDemand);

    private:

        AltitudeConfig config;

        Board * board;

        // Barometer
        Barometer baro;
        uint32_t  baroCalibrationStart;
        int32_t   lastBaroAlt;

        // IMU
        Accelerometer accel;

        // Fused
        int32_t  altHold;
        int32_t  altPid;
        int32_t  errorVelocityI;
        int32_t  estimAlt;
        bool     holdingAltitude;
        int16_t  initialThrottleHold;
        bool     isAltHoldChanged;

        int32_t computeBaroVelocity(int32_t baroAltitude);
};

/********************************************* CPP ********************************************************/

void Altitude::init(const AltitudeConfig & _config, Board * _board)
{
    memcpy(&config, &_config, sizeof(AltitudeConfig));

    board = _board;

    baro.init(config.baro);
    baroCalibrationStart = 0;
    lastBaroAlt = 0;

    accel.init(config.accel);

    initialThrottleHold = 0;
    altPid = 0;
    errorVelocityI = 0;
    holdingAltitude = false;
    isAltHoldChanged = false;
}

void Altitude::handleAuxSwitch(uint8_t auxState, uint16_t throttleDemand)
{
    if (auxState > 0) {
        holdingAltitude = true;
        initialThrottleHold = throttleDemand;
        altHold = estimAlt;
        errorVelocityI = 0;
        altPid = 0;
    }
    else {
        holdingAltitude = false;
    }
}

void Altitude::modifyThrottleDemand(int16_t & throttleDemand)
{
    if (holdingAltitude) {

        throttleDemand = constrain(initialThrottleHold + altPid, config.throttleMin, config.throttleMax);
    }
}

void Altitude::computePid(bool armed)
{  
    // Update the baro with the current pressure reading
    baro.update(board->extrasGetBaroPressure());

    // Calibrate baro while not armed
    if (!armed) {
        baro.calibrate();
        return;
    }

    // Get estimated altitude from baro
    int32_t baroAltitude = baro.getAltitude();

    float accZ = Filter::deadband(accel.accZ, config.accel.deadband);

    Serial.println(accZ);

} // computePid

void Altitude::updateAccelerometer(float eulerAnglesRadians[3], bool armed)
{
    // Throttle modification is synched to aquisition of new IMU data
    int16_t accelRaw[3];
    board->extrasImuGetAccel(accelRaw);
    accel.update(accelRaw, eulerAnglesRadians, board->getMicros(), armed);
}

int32_t Altitude::computeBaroVelocity(int32_t baroAltitude)
{
    static uint32_t previousTimeUsec;
    uint32_t currentTimeUsec = board->getMicros();
    uint32_t dT_usec = currentTimeUsec - previousTimeUsec;
    previousTimeUsec = currentTimeUsec;
    int32_t baroVelocity = (baroAltitude - lastBaroAlt) * 1000000.0f / dT_usec;
    lastBaroAlt = baroAltitude;
    baroVelocity = constrain(baroVelocity, -1500, 1500);    // constrain baro velocity +/- 1500cm/s
    return Filter::deadband(baroVelocity, 10);         // to reduce noise near zero
}

} // namespace hf
