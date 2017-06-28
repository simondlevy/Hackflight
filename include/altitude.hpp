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
        void computePid(float eulerAnglesDegrees[3], bool armed);
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
        int32_t  setVelocity;      
        bool     velocityControl; 

        int32_t computeBaroVelocity(int32_t baroAltitude);
        int32_t updatePid(float estimVel);
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
    setVelocity = 0;      
    velocityControl = false; 
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

        if (abs(throttleDemand - initialThrottleHold) > config.throttleNeutral) {
            // set velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
            setVelocity = (throttleDemand - initialThrottleHold) / 2;
            velocityControl = true;
            isAltHoldChanged = true;
        } else if (isAltHoldChanged) {
            altHold = estimAlt;
            velocityControl = false;
            isAltHoldChanged = false;
        }
        throttleDemand = constrain(initialThrottleHold + altPid, config.throttleMin, config.throttleMax);
    }
}

void Altitude::computePid(float eulerAnglesDegrees[3], bool armed)
{  
    // Update the baro with the current pressure reading
    baro.update(board->extrasGetBaroPressure());

    // Calibrate baro while not armed
    if (!armed) {
        baro.calibrate();
        accel.reset();
        return;
    }

    // Get estimated altitude from baro
    int32_t baroAltitude = baro.getAltitude();

    // Integrate accelerometer to get altitude and velocity
    accel.integrate();

    // Apply complementary filter to fuse baro and accel altitude
    estimAlt = Filter::complementary(accel.getAltitude(), baroAltitude, config.cfAlt);

    // Compute velocity from barometer
    int32_t baroVelocity = computeBaroVelocity(baroAltitude);

    // Apply complementary filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    float estimVel = Filter::complementary(accel.getVelocity(), baroVelocity, config.cfVel);
    accel.adjustVelocity(estimVel);

    // only calculate pid if the copters thrust is facing downwards(<80deg)
    altPid = (Filter::max(abs(eulerAnglesDegrees[0]), abs(eulerAnglesDegrees[1])) < config.maxTiltAngle) ?  updatePid(estimVel) : 0;

} // computePid

void Altitude::updateAccelerometer(float eulerAnglesRadians[3], bool armed)
{
    // Throttle modification is synched to aquisition of new IMU data
    int16_t accelRaw[3];
    board->extrasImuGetAccel(accelRaw);
    accel.update(accelRaw, eulerAnglesRadians, board->getMicros(), armed);
}

int32_t Altitude::updatePid(float estimVel)
{
    int32_t setVel = setVelocity;
    int32_t error = 0;

    // Altitude P-Controller
    if (!velocityControl) {
        error = constrain(altHold - estimAlt, -500, 500);
        error = Filter::deadband(error, 10);       // remove small P parametr to reduce noise near zero position
        setVel = constrain(config.pidAltP * error, -300, +300); // limit velocity to +/- 3 m/s
    } 

    // Velocity PID-Controller
    // P
    error = setVel - (int32_t)lrintf(estimVel);
    int32_t newPid = constrain(config.pidVelP * error, -300, +300);

    // I
    errorVelocityI += (config.pidVelI * error);
    errorVelocityI = constrain(errorVelocityI, -200, +200);
    newPid += errorVelocityI;

    // D
    newPid -= constrain(config.pidVelD * accel.getAcceleration(), -150, 150);

    return newPid;
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
